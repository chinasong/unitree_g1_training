"""Remote workflows exercised through recording consumers, never a physical robot."""
import asyncio
import base64
import json
import threading
import time
from types import SimpleNamespace
from uuid import uuid4

import pytest

from g1_console.adapters import DemoAdapter, UnitreeAdapter
from g1_console.capabilities import CATALOG, READ_OPERATIONS, validate_params
from g1_console.config import ConsoleError, Settings
from g1_console.contracts import Command
from g1_console.service import ConsoleService

OWNER = 'remote-test-owner-0000001'


def command(op, **extra):
    return Command(owner=OWNER, capability=op, sequence=extra.pop('sequence', 1),
                   issued_at=time.time(), request_id=uuid4().hex, **extra)


@pytest.mark.parametrize('op,value', [('balance', True), ('volume', '20'), ('task', 1.5), ('fsm', False)])
def test_numeric_controls_reject_coercion(op, value):
    with pytest.raises(ConsoleError):
        validate_params(op, {'value': value})


@pytest.mark.parametrize('fsm', [2, 4])
def test_documented_squat_and_prepare_parameters_reach_demo_consumer(tmp_path, fsm):
    async def check():
        s = ConsoleService(Settings(data_dir=tmp_path))
        await s.connect('demo', 'eth0')
        await s.acquire(OWNER, True)
        result = await s.command(command('fsm', params={'value': fsm}, acknowledge_risk=True))
        assert result['status'] == 'accepted'
        readback = await s.command(command('get_fsm', sequence=2))
        assert readback['data'] == fsm
    asyncio.run(check())


def test_audio_stop_cannot_miss_an_arbitrary_application():
    with pytest.raises(ConsoleError):
        validate_params('play_pcm', {'app_name': 'other-player', 'pcm_base64': 'AAA='})
    for raw in (b'', b'\0', b'\0' * 48002):
        with pytest.raises(ConsoleError):
            validate_params('play_pcm', {'pcm_base64': base64.b64encode(raw).decode()})


@pytest.mark.parametrize('op,value', [('get_balance', 1), ('get_swing_height', .123),
                                    ('get_stand_height', .456), ('get_phase', [.2, .8]),
                                    ('get_services', [{'name':'sport','status':0,'protect':1}]),
                                    ('get_versions', {'packageVersion':'test','moduleVersionMap':{}}),
                                    ('get_loco_api_version', {'service':'sport','api_version':'1.0.0.0','firmware_version':None}),
                                    ('get_motion_api_version', {'service':'motion_switcher','api_version':'1.0.0.1','firmware_version':None}),
                                    ('get_state_api_version', {'service':'robot_state','api_version':'1.0.0.2','firmware_version':None}),
                                    ('get_volume', {'volume':41}),('get_actions', [{'id':99,'name':'release_arm'}])])
def test_motion_queries_use_isolated_reader_without_legacy_sdk_fallback(op, value):
    calls = []
    a = object.__new__(UnitreeAdapter)
    a.read_only = a.verify_only = a.accessory_test = a.damping_test = False
    a.read_only = True
    a.rpc_lock = threading.Lock()
    def read(name):
        calls.append(name)
        return 0, {'data': value}
    a.mode_reader = SimpleNamespace(read=read)
    assert a.execute(op, {}) == (0, value)
    assert calls == [op]
    a.mode_reader = SimpleNamespace(read=lambda _: (7301, {'data': 0}))
    assert a.execute(op, {}) == (7301, None)


def test_all_catalog_workflows_and_synthetic_parameter_roundtrip(tmp_path):
    async def check():
        s = ConsoleService(Settings(data_dir=tmp_path))
        await s.connect('demo', 'eth0')
        await s.acquire(OWNER, True)
        examples = {'none': {}, 'velocity': {'vx': 0., 'vy': 0., 'omega': 0.},
                    'text': {'text': 'test', 'speaker_id': 1}, 'rgb': {'r': 1, 'g': 2, 'b': 3},
                    'pcm': {'pcm_base64': 'AAA='}, 'height': {'value': .4}, 'action': {'value': 17}}
        for seq, cap in enumerate(CATALOG, 1):
            s.adapter.fsm = 500
            await s._sample()
            result = await s.command(command(cap['id'], sequence=seq, params=examples.get(cap['schema'], {'value': 0}), acknowledge_risk=True))
            assert result['status'] == 'accepted', cap['id']
        for seq, op in enumerate(('get_stand_height', 'get_swing_height', 'get_balance'), 100):
            result = await s.command(command(op, sequence=seq))
            assert result['data'] == (0 if op == 'get_balance' else .4)
        rows = s.snapshot()['command_history']
        assert all(r['mode'] == 'demo' for r in rows)
        assert not any(r['status'] == 'completed' for r in rows)
        assert all(r['status'] == 'read_result' for r in rows[:3])
    asyncio.run(check())


@pytest.mark.parametrize('code,http_status,application_code', [(7301,502,'SDK_REJECTED'),
    (3203,502,'SDK_REJECTED'),(3104,504,'SDK_TIMEOUT'),('MODE_WORKER_TIMEOUT',502,'SDK_REJECTED')])
def test_read_failures_preserve_code_and_never_surface_placeholder_or_resend(tmp_path, code, http_status, application_code):
    from fastapi.testclient import TestClient
    from g1_console.app import create_app
    class FailedRead(DemoAdapter):
        calls = 0
        def execute(self, op, p):
            self.calls += 1
            assert op == 'get_balance'
            return code, 0  # Even plausible raw data must be ignored on failure.
        def close(self):
            self.closed = True
    app = create_app(Settings(data_dir=tmp_path))
    with TestClient(app) as http:
        http.get('/api/session')
        http.headers['X-G1-Intent'] = 'operator'
        http.post('/api/connect',json={'mode':'demo'})
        service = app.state.service
        service.adapter = FailedRead()
        c = command('get_balance').model_dump()
        for _ in range(2):
            result = http.post('/api/command',json=c)
            assert result.status_code == http_status
            assert result.json()['error']['code'] == application_code
            assert result.json()['error']['sdk_code'] == code
        assert service.adapter.calls == 1
        row = http.get('/api/state').json()['command_history'][0]
        assert row['status'] == 'read_unavailable' and row['sdk_code'] == code
        assert row.get('data') is None and service.owner is None


def test_ack_fsm_and_timeout_are_distinct_and_correlated(tmp_path):
    async def check():
        s = ConsoleService(Settings(data_dir=tmp_path))
        await s.connect('demo', 'eth0')
        await s.acquire(OWNER, True)
        c = command('start')
        await s.command(c)
        assert s.command_history[c.request_id]['status'] == 'awaiting_feedback'
        await s.tick()
        assert s.command_history[c.request_id]['status'] == 'state_observed'
        c = command('start', sequence=2)
        await s.command(c)
        s.adapter.fsm = 0
        s.pending['until'] = time.monotonic() - 1
        await s.tick()
        assert s.command_history[c.request_id]['status'] == 'feedback_timeout'
        assert next(e for e in s.events if e['kind'] == 'feedback_timeout')['request_id'] == c.request_id
    asyncio.run(check())


def test_timeout_retry_and_concurrent_duplicate_never_resend_write(tmp_path):
    class TimeoutAdapter(DemoAdapter):
        calls = 0
        def execute(self, op, p):
            self.calls += 1
            time.sleep(.02)
            return 3104, None
    async def check():
        s = ConsoleService(Settings(data_dir=tmp_path))
        s.adapter = TimeoutAdapter()
        await s._sample()
        await s.acquire(OWNER, True)
        c = command('volume', params={'value': 20})
        result = await asyncio.gather(s.command(c), s.command(c), return_exceptions=True)
        assert all(isinstance(e, ConsoleError) and e.code == 'SDK_TIMEOUT' for e in result)
        assert s.adapter.calls == 1
        with pytest.raises(ConsoleError, match='3104'):
            await s.command(c)
        assert s.adapter.calls == 1
        assert s.command_history[c.request_id]['status'] == 'outcome_unknown'
        assert len(s.command_history) == 1
        assert next(e for e in s.events if e['kind'] == 'sdk_timeout')['sdk_code'] == 3104
        assert not any(e['kind'] == 'sdk_rejected' for e in s.events)
    asyncio.run(check())
