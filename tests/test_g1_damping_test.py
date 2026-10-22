"""Single passive motor-mode dispatch with recording fixtures, never real DDS."""
import asyncio
import json
import sys
import threading
import time
from types import ModuleType, SimpleNamespace

import pytest

from g1_console.adapters import UnitreeAdapter
from g1_console.capabilities import CATALOG, READ_OPERATIONS
from g1_console.config import ConsoleError, Settings
from g1_console.state_learning import history_sample
from g1_console.service import ConsoleService
from scripts.connect_orin_console import verify_configuration
from scripts.orin_console_recovery import restore
from test_g1_manual_remote import setup, OWNER, OTHER, command, arm


async def ready(tmp_path):
    s = await setup(tmp_path, damping_test=True)
    s.adapter.fsm = 0
    await s._sample()
    now = time.monotonic()
    s.body_history.clear()
    for offset in (2.3, 2., 1.7, 1.4, 1.1, .8, .5, .2, 0):
        s.body_history.append(history_sample(s.telemetry, now-offset))
    s.world.camera = b'recording fixture only'
    s.world.camera_at = now
    s.world.camera_captured_at = time.time()
    return s


def test_scope_exact_in_settings_connector_and_recovery(tmp_path):
    Settings(manual_remote=True, damping_test=True).validate()
    for changes in ({}, {'manual_remote':True,'read_only':True}, {'manual_remote':True,'verify_only':True}, {'manual_remote':True,'accessory_test':True}):
        with pytest.raises(ValueError): Settings(damping_test=True, **changes).validate()
    state = dict(read_only=False, manual_remote=dict(enabled=True, verify_only=False,
                 accessory_test=False, damping_test=True, allowed_write_operations=['damp']))
    verify_configuration(state, True, damping_test=True)
    for change in ({'damping_test':False}, {'accessory_test':True}, {'verify_only':True}, {'allowed_write_operations':['damp','prepare']}, {'allowed_write_operations':None}):
        with pytest.raises(RuntimeError):
            verify_configuration({**state,'manual_remote':{**state['manual_remote'],**change}}, True, damping_test=True)
    for changes in ({}, {'manual':True,'verify_only':True}, {'manual':True,'accessory_test':True}):
        with pytest.raises(ValueError): restore(tmp_path, damping_test=True, **changes)


def test_one_dispatch_ack_and_fsm_receipt_without_standing_or_retry(tmp_path):
    async def check():
        s = await ready(tmp_path)
        assert s.execution_gate('damp')['allowed']
        with pytest.raises(ConsoleError) as exc: await s.command(command('damp'))
        assert exc.value.code == 'RISK_ACK_REQUIRED' and s.damping_attempt is None
        c = command('damp', seq=2, acknowledge_risk=True)
        result = await s.command(c)
        assert result['status'] == 'accepted'
        assert await s.command(c) == result
        assert s.adapter.calls == [('damp', {})]
        await s.tick()
        assert s.telemetry['fsm'] == 1
        assert s.damping_attempt['status'] == 'fsm_verified'
        assert s.damping_attempt['physical_support'] == 'unknown'
        assert not s.manual_status()['armed']
        assert s.execution_gate('damp')['code'] == 'DAMPING_TEST_SPENT'
        with pytest.raises(ConsoleError): await s.command(command('damp', seq=3, acknowledge_risk=True))
        await s.release(OWNER)
        assert s.adapter.calls == [('damp', {})]
    asyncio.run(check())


@pytest.mark.parametrize('fault', ['fsm','stale_fsm','mode','missing_joint','bad_status','temperature','speed','short_window','camera','lease','stale_body','imu'])
def test_live_evidence_filters_prevent_dispatch(tmp_path, fault):
    async def check():
        s = await ready(tmp_path)
        if fault == 'fsm': s.telemetry['fsm'] = 500
        if fault == 'stale_fsm': s.telemetry['fsm_received'] = time.monotonic()-2
        if fault == 'mode': s.telemetry['safety_feedback']['fsm_mode']['raw']['data'] = 1
        if fault == 'missing_joint': s.telemetry['joints'].pop()
        if fault == 'bad_status': s.telemetry['joints'][0]['status'] = 1
        if fault == 'temperature': s.telemetry['joints'][0]['temperature'] = [70,40]
        if fault == 'speed': s.body_history[-1]['max_dq'] = .2
        if fault == 'short_window': s.body_history.popleft();s.body_history.popleft()
        if fault == 'camera': s.world.camera_at -= 2
        if fault == 'lease': s.lease_until = time.monotonic()-1
        if fault == 'stale_body': s.telemetry['received'] = time.monotonic()-1
        if fault == 'imu': s.body_history[-1]['rpy'] = [float('nan'),0,0]
        assert not s.execution_gate('damp')['allowed']
        with pytest.raises(ConsoleError): await s.command(command('damp', acknowledge_risk=True))
        assert s.adapter.calls == [] and s.damping_attempt is None
    asyncio.run(check())


def test_other_writes_and_autonomous_routes_cannot_borrow_scope(tmp_path):
    async def check():
        s = await ready(tmp_path)
        with pytest.raises(ConsoleError): await arm(s)
        for cap in CATALOG:
            op = cap['id']
            if op in READ_OPERATIONS or op == 'damp': continue
            assert s.execution_gate(op)['code'] == 'DAMPING_TEST_ONLY'
            for route in (s.command(command(op)), s._rpc(op, {}, manual_owner=OWNER)):
                with pytest.raises(ConsoleError) as exc: await route
                assert exc.value.code == 'DAMPING_TEST_ONLY'
        for owner in (None, OTHER):
            with pytest.raises(ConsoleError): await s._rpc('damp', {}, manual_owner=owner)
        assert not s.adapter.calls
        await s._rpc('get_fsm', {})
        assert s.adapter.calls == [('get_fsm', {})]
    asyncio.run(check())


@pytest.mark.parametrize('result', [3104,7301,'exception'])
def test_uncertain_or_rejected_attempt_consumed_before_dispatch(tmp_path, result):
    async def check():
        s = await ready(tmp_path)
        calls = []
        def execute(op, params):
            calls.append(op)
            if result == 'exception': raise RuntimeError('recording fixture')
            return result, None
        s.adapter.execute = execute
        c = command('damp', acknowledge_risk=True)
        for _ in range(2):
            with pytest.raises(ConsoleError): await s.command(c)
        with pytest.raises(ConsoleError): await s.command(command('damp', seq=2, acknowledge_risk=True))
        assert calls == ['damp']
        assert s.damping_attempt['status'] != 'fsm_verified'
    asyncio.run(check())


def test_sdk_consumer_exact_fsm_and_one_attempt(monkeypatch):
    fake = ModuleType('unitree_sdk2py.g1.loco.g1_loco_api')
    fake.InternalFsmMode = SimpleNamespace()
    monkeypatch.setitem(sys.modules, fake.__name__, fake)
    calls = []
    adapter = object.__new__(UnitreeAdapter)
    adapter.read_only = adapter.verify_only = adapter.accessory_test = adapter.damping_test = False
    adapter.damping_test = True
    adapter.damping_attempted = False
    adapter.rpc_lock = threading.Lock()
    adapter.loco = SimpleNamespace(SetFsmId=lambda fsm: calls.append(fsm) or (0,None))
    for cap in CATALOG:
        if cap['id'] not in READ_OPERATIONS and cap['id'] != 'damp':
            with pytest.raises(ConsoleError): adapter.execute(cap['id'], {})
    assert adapter.execute('damp', {}) == (0,None)
    with pytest.raises(ConsoleError): adapter.execute('damp', {})
    assert calls == [1]


def test_attempt_receipt_survives_service_restart(tmp_path):
    async def check():
        s = await ready(tmp_path)
        await s.command(command('damp', acknowledge_risk=True))
        await s.tick()
        restored = ConsoleService(s.settings)
        assert restored.damping_attempt == s.damping_attempt
        assert restored.damping_attempt['status'] == 'fsm_verified'
        s2 = await ready(tmp_path)
        with pytest.raises(ConsoleError) as exc:
            await s2.command(command('damp', acknowledge_risk=True))
        assert exc.value.code == 'DAMPING_TEST_SPENT'
        assert not s2.adapter.calls
    asyncio.run(check())


@pytest.mark.parametrize('receipt', ['{}', 'invalid', '{"status":"outcome_unknown"}'])
def test_existing_or_invalid_receipt_cannot_reset_test(tmp_path, receipt):
    (tmp_path/'damping-test.json').write_text(receipt)
    async def check():
        s = await ready(tmp_path)
        with pytest.raises(ConsoleError) as exc:
            await s.command(command('damp', acknowledge_risk=True))
        assert exc.value.code == 'DAMPING_TEST_SPENT'
        assert not s.adapter.calls
    asyncio.run(check())


def test_concurrent_receipt_creation_blocks_sdk_dispatch(tmp_path):
    async def check():
        s = await ready(tmp_path)
        s.damping_file.write_text('{"status":"outcome_unknown"}')
        with pytest.raises(ConsoleError) as exc:
            await s.command(command('damp', acknowledge_risk=True))
        assert exc.value.code == 'DAMPING_TEST_SPENT'
        assert not s.adapter.calls
    asyncio.run(check())


def feedback_rows(s, values=(1, 1)):
    for key, field, value in (('fsm_id', 'data', values[0]), ('sport', 'fsm_id', values[1])):
        s.telemetry['safety_feedback'][key] = dict(current=True, code=0, raw={field: value},
            received_at=time.time(), received_monotonic=time.monotonic())


def test_timeout_with_dual_state_change_does_not_rewrite_ack_or_retry(tmp_path):
    async def check():
        s = await ready(tmp_path)
        calls = []
        s.adapter.execute = lambda op, params: (calls.append(op) or (3104, None))
        with pytest.raises(ConsoleError):
            await s.command(command('damp', acknowledge_risk=True))
        s.adapter.fsm = 1
        await s._sample()
        feedback_rows(s)
        status = s.manual_status()
        assert status['damping_feedback']['status'] == 'target_observed'
        assert status['damping_feedback']['physical_support'] == 'unknown'
        assert not status['damping_feedback']['motion_authority_verified']
        assert not status['armed'] and not status['robot_authority_verified']
        assert status['damping_attempt']['sdk_code'] == 3104
        assert status['damping_attempt']['status'] == 'outcome_unknown'
        assert json.loads(s.damping_file.read_text()) == status['damping_attempt']
        with pytest.raises(ConsoleError):
            await s.command(command('damp', seq=2, acknowledge_risk=True))
        assert calls == ['damp']
    asyncio.run(check())


@pytest.mark.parametrize('fault', ['stale_row', 'missing_raw', 'error', 'before_attempt',
                                  'future_row', 'conflict', 'other_fsm', 'stale_body',
                                  'invalid_receipt_time', 'demo', 'disconnected'])
def test_current_feedback_cannot_borrow_stale_missing_or_conflicting_evidence(tmp_path, fault):
    async def check():
        s = await ready(tmp_path)
        s.damping_attempt = dict(at=time.time()-.5, status='outcome_unknown', sdk_code=3104)
        feedback_rows(s)
        row = s.telemetry['safety_feedback']['sport']
        if fault == 'stale_row': row['received_monotonic'] -= 2
        if fault == 'missing_raw': row['raw'] = None
        if fault == 'error': row['code'] = 7301
        if fault == 'before_attempt': row['received_at'] = s.damping_attempt['at']-.1
        if fault == 'future_row': row['received_monotonic'] += 2
        if fault == 'conflict': row['raw']['fsm_id'] = 0
        if fault == 'other_fsm': feedback_rows(s, (0, 0))
        if fault == 'stale_body': s.telemetry['received'] -= 2
        if fault == 'invalid_receipt_time': s.damping_attempt['at'] = None
        if fault == 'demo': s.adapter.mode = 'demo'
        if fault == 'disconnected': s.adapter = None
        result = s.damping_feedback()
        expected = 'conflict' if fault == 'conflict' else 'other_state_observed' if fault == 'other_fsm' else 'unknown'
        assert result['status'] == expected
        assert not result['motion_authority_verified'] and result['physical_support'] == 'unknown'
        assert s.damping_attempt['status'] == 'outcome_unknown'
    asyncio.run(check())
