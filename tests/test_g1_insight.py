"""Observation boundaries: real source, auth, worker leases, stale input and zero actuation."""
import asyncio
import base64
import time
from io import StringIO
from uuid import uuid4
import pytest
from fastapi.testclient import TestClient
from g1_console.adapters import DemoAdapter
from g1_console.app import create_app
from g1_console.config import ConsoleError, Settings
from g1_console.insight import Insights, Report, Submission, Worker
from scripts.observe_g1 import Client, command, tool_event, failure_code, rate_limit_delay


def state():
    return dict(mode='robot', read_only=True, connected=True, at=time.time(), run_id='run-a',
                camera=dict(status='live', backend='unitree', source='camera-a', sequence=10, age_ms=120),
                telemetry_age_ms=20, telemetry=dict(tick=100, joints=[{}]*23), hardware=dict(dof=23, hands=False),
                capabilities=[dict(id='get_fsm', label='状态', availability='sdk_defined')],
                world=dict(source=None, age_ms=None, pose=None, scan=[]))


def report():
    return Report(summary='近处地板可见，完整通路未知', scene_facts=[dict(statement='地板', evidence='image', confidence='high')],
                  body_findings=[], uncertainties=['无深度'], suggestions=[dict(title='继续观察', rationale='验证环境',
                  requirements=['补充深度与定位'], capability_ids=['get_fsm'])], change_summary='首次采样')


def setup():
    manager = Insights()
    worker = Worker(worker=uuid4().hex, model='configured-model')
    manager.heartbeat(worker)
    manager.control(True)
    return manager, worker


def test_tickets_pause_unknown_capabilities_and_order():
    manager, worker = setup()
    data = state()
    bundle = manager.capture(worker, data, b'jpeg')
    assert base64.b64decode(bundle['jpeg_base64']) == b'jpeg'
    with pytest.raises(ConsoleError, match='正在分析'):
        manager.capture(worker, data, b'other')
    bad = report(); bad.suggestions[0].capability_ids = ['invented_grasp']
    with pytest.raises(ConsoleError, match='未知能力'):
        manager.submit(Submission(**worker.model_dump(), ticket=bundle['ticket'], report=bad, error=''), data)
    manager.control(False)
    with pytest.raises(ConsoleError, match='暂停'):
        manager.submit(Submission(**worker.model_dump(), ticket=bundle['ticket'], report=report(), error=''), data)
    assert manager.report is None


@pytest.mark.parametrize('change', ['disconnect', 'camera_stale', 'source', 'run', 'timeout'])
def test_report_rejected_after_input_invalidated(change):
    manager, worker = setup(); data = state()
    bundle = manager.capture(worker, data, b'jpeg')
    if change == 'disconnect': data['connected'] = False
    elif change == 'camera_stale': data['camera']['status'] = 'stale'
    elif change == 'source': data['camera']['source'] = 'camera-b'
    elif change == 'run': data['run_id'] = 'new-run'
    elif change == 'timeout': manager.job['started'] -= 151
    # A snapshot sent over HTTP is independent; imitate that boundary for nested camera metadata.
    if change in ('source', 'camera_stale'):
        manager.job['evidence']['camera'] = dict(status='live', source='camera-a')
    with pytest.raises(ConsoleError, match='断流'):
        manager.submit(Submission(**worker.model_dump(), ticket=bundle['ticket'], report=report(), error=''), data)
    assert manager.report is None


def test_leases_expiry_and_historical_report():
    manager, worker = setup(); data = state()
    with pytest.raises(ConsoleError, match='另一台'):
        manager.heartbeat(Worker(worker=uuid4().hex, model=worker.model))
    bundle = manager.capture(worker, data, b'jpeg')
    submission = Submission(**worker.model_dump(), ticket=bundle['ticket'], report=report(), error='')
    manager.submit(submission, data)
    assert manager.snapshot(True, data['camera'])['current']
    assert manager.completed == 1
    with pytest.raises(ConsoleError, match='失效'):
        manager.submit(submission, data)
    assert not manager.snapshot(False, data['camera'])['current']
    manager.report['evidence']['captured_at'] -= 121
    assert not manager.snapshot(True, data['camera'])['current']
    manager.seen -= 21
    assert manager.snapshot(True, data['camera'])['status'] == 'waiting_worker'


def test_http_observer_never_writes_robot_and_disconnect_invalidates(monkeypatch, tmp_path):
    class Robot(DemoAdapter):
        mode = 'robot'
        def __init__(self, *args, **kwargs): super().__init__()
        def execute(self, *args): raise AssertionError('observation must never actuate')
        def close(self): pass  # Match UnitreeAdapter(read_only=True), not DemoAdapter.stop().
    monkeypatch.setattr('g1_console.service.UnitreeAdapter', Robot)
    app = create_app(Settings(data_dir=tmp_path, read_only=True, camera_backend='unitree'))
    service = app.state.service
    service.adapter = Robot()
    async def sample_only(): service.telemetry = service.adapter.sample()
    monkeypatch.setattr(service, '_sample', sample_only)
    identity = dict(worker=uuid4().hex, model='test-model')
    with TestClient(app) as client:
        path = '/api/insight/heartbeat'
        headers = {'X-G1-Intent': 'operator'}
        assert client.post(path, json=identity, headers=headers).status_code == 401
        client.get('/api/session')
        assert client.post(path, json=identity).status_code == 403
        assert client.post(path, json=identity, headers=headers).status_code == 200
        client.post('/api/insight/control', json={'enabled': True}, headers=headers)
        world = service.world
        world.camera, world.camera_source = b'\xff\xd8test\xff\xd9', 'official-source'
        world.camera_at, world.camera_captured_at = time.monotonic(), time.time()
        bundle = client.post('/api/insight/capture', json=identity, headers=headers).json()
        assert bundle['evidence']['joints'] == 23
        assert len(bundle['foundation']['motors']) == 23
        assert bundle['evidence']['foundation_id'] == bundle['foundation']['foundation_id']
        assert bundle['assessment']['current'] is True
        assert bundle['capabilities'][0]['compatibility']['execution_allowed'] is False
        # Use no capability references so this test is independent of catalogue membership.
        r = report().model_dump(); r['suggestions'][0]['capability_ids'] = []
        assert client.post('/api/insight/result', json=dict(**identity, ticket=bundle['ticket'], report=r, error=''), headers=headers).status_code == 200
        snapshot = client.get('/api/state').json()
        assert snapshot['insight']['completed'] == 1 and snapshot['control']['owner'] is None
        assert snapshot['navigation']['status'] == 'idle'
        assert not any(e['kind'] == 'command_accepted' for e in snapshot['events'])
        client.post('/api/disconnect', headers=headers)
        assert client.get('/api/state').json()['insight']['report'] is None


def test_worker_has_no_robot_api_or_model_tools(tmp_path):
    client = object.__new__(Client)
    for path in ('/api/command', '/api/goal', '/api/connect', '/api/control/acquire'):
        with pytest.raises(ValueError, match='不能访问'):
            client.request(path, {})
    args = command('codex', tmp_path, 'configured-model', 'xhigh')
    assert args[args.index('--sandbox')+1] == 'read-only'
    assert '--ignore-user-config' in args and '--ephemeral' in args
    for feature in ('shell_tool', 'apps', 'plugins', 'multi_agent', 'browser_use'):
        assert args[args.index(feature)-1] == '--disable'


def test_cli_startup_warning_is_not_a_tool_but_execution_is_rejected():
    assert not tool_event({'type': 'item.completed', 'item': {'type': 'error', 'message': 'Ignoring malformed agent role definition'}})
    assert not tool_event({'type': 'item.completed', 'item': {'type': 'agent_message', 'text': '{}'}})
    for kind in ('command_execution', 'mcp_tool_call', 'web_search', 'file_change'):
        assert tool_event({'type': 'item.started', 'item': {'type': kind}})


def test_real_provider_rate_limit_is_classified_and_backs_off():
    events = StringIO('{"type":"error","message":"exceeded retry limit, last status: 429 Too Many Requests"}\n')
    assert failure_code(events) == 'CODEX_RATE_LIMITED'
    assert [rate_limit_delay(n, 30) for n in range(1, 7)] == [60, 120, 240, 300, 300, 300]
    assert rate_limit_delay(1, 180) == 180
    assert failure_code(StringIO('{"type":"turn.failed","error":{"message":"rate_limit_exceeded"}}\n')) == 'CODEX_RATE_LIMITED'
    # A model report mentioning 429 is data; it must not trigger the provider path.
    assert failure_code(StringIO('{"type":"item.completed","item":{"type":"agent_message","text":"429"}}\n')) == 'CODEX_FAILED'


def test_rate_limit_keeps_prior_report_without_refreshing_its_age():
    manager, worker = setup(); data = state()
    bundle = manager.capture(worker, data, b'jpeg')
    manager.submit(Submission(**worker.model_dump(), ticket=bundle['ticket'], report=report(), error=''), data)
    previous = manager.report
    bundle = manager.capture(worker, data, b'jpeg')
    manager.submit(Submission(**worker.model_dump(), ticket=bundle['ticket'], report=None, error='CODEX_RATE_LIMITED'), data)
    assert manager.report is previous and manager.completed == 1 and manager.job is None
    assert manager.snapshot(True, data['camera'])['error'] == 'CODEX_RATE_LIMITED'
    previous['evidence']['captured_at'] -= 121
    assert not manager.snapshot(True, data['camera'])['current']


def test_worker_keeps_rate_backoff_when_stale_input_rejects_result(monkeypatch, tmp_path):
    import scripts.observe_g1 as worker
    from urllib.error import HTTPError
    clock = [0.]
    callbacks, captures = [], []
    class StaleInputClient:
        def __init__(self, url): pass
        def request(self, path, data=None):
            if path.endswith('heartbeat'):
                if clock[0] >= 35:
                    callbacks[0]()
                return {'enabled': True}
            if path.endswith('capture'):
                captures.append(clock[0])
                return dict(ticket=uuid4().hex, evidence={'camera': {'sequence': 1}})
            raise HTTPError('http://127.0.0.1/api/insight/result', 409, 'camera went stale', None, None)
    monkeypatch.setattr(worker, '__file__', str(tmp_path/'scripts/observe_g1.py'))
    monkeypatch.setattr(worker, 'Client', StaleInputClient)
    monkeypatch.setattr(worker, 'model_config', lambda _: ('test-model', 'medium'))
    monkeypatch.setattr(worker, 'analyze', lambda *args: (None, 'CODEX_RATE_LIMITED'))
    monkeypatch.setattr(worker.signal, 'signal', lambda _, callback: callbacks.append(callback))
    monkeypatch.setattr(worker.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(worker.time, 'sleep', lambda seconds: clock.__setitem__(0, clock[0]+seconds))
    monkeypatch.setattr(worker.sys, 'argv', ['observe_g1.py'])
    worker.main()
    assert len(captures) == 1 and clock[0] < 60
