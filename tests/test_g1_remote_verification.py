"""Verification leases and real SDK payloads without a hardware write or DDS runtime."""
import ast
import asyncio
import json
from pathlib import Path
import time
from types import MethodType, SimpleNamespace

import pytest
from fastapi.testclient import TestClient

from g1_console.adapters import UnitreeAdapter
from g1_console.app import create_app
from g1_console.capabilities import CATALOG
from g1_console.config import ConsoleError, Settings
from g1_console.contracts import Preflight
from g1_console.remote_preflight import describe_request
from scripts.connect_orin_console import verify_configuration
from test_g1_manual_remote import setup, OWNER, OTHER, command, arm


def test_lease_preflight_heartbeat_release_do_not_dispatch(tmp_path):
    async def check():
        s = await setup(tmp_path, verify_only=True)
        # No fabricated standing confirmation is required for a project lease.
        s.adapter.fsm = 0
        await s._sample()
        await s.acquire(OWNER, False)
        with pytest.raises(ConsoleError) as exc:
            await s.acquire(OTHER, False)
        assert exc.value.code == 'CONTROL_BUSY'
        await s.heartbeat(OWNER)
        for op, params in [('squat_to_stand', {}), ('lie_to_stand', {}), ('prepare', {}),
                           ('balance', {'value': 0}), ('balance', {'value': 1}), ('stand_height', {'value': .5})]:
            result = await s.preflight(Preflight(owner=OWNER, capability=op, params=params))
            assert result['dispatched'] is False and result['params_valid'] is True
            assert result['current_fsm'] == 0 and result['firmware_acceptance'] == 'unknown'
            assert result['execution_gate']['code'] == 'VERIFICATION_ONLY'
        snap = s.snapshot()
        assert snap['manual_remote']['enabled'] and snap['manual_remote']['verify_only']
        assert not snap['manual_remote']['armed'] and not snap['control']['floor_verified']
        assert snap['manual_remote']['robot_authority_verified'] is False
        assert len(snap['manual_remote']['preflights']) == 6
        await s.release(OWNER)
        assert s.owner is None and s.adapter.calls == []
        await s.acquire(OTHER, False)
        s.lease_until = time.monotonic() - 1
        with pytest.raises(ConsoleError) as exc:
            await s.preflight(Preflight(owner=OTHER, capability='prepare'))
        assert exc.value.code == 'NO_CONTROL_LEASE' and not s.adapter.calls
    asyncio.run(check())


def test_every_physical_route_blocked_before_adapter_even_if_forced_armed(tmp_path):
    async def check():
        s = await setup(tmp_path, verify_only=True)
        with pytest.raises(ConsoleError) as exc:
            await arm(s)
        assert exc.value.code == 'VERIFICATION_ONLY'
        s.manual_permit = dict(owner=OWNER, until=time.monotonic()+60, observed_at=time.time())
        for cap in CATALOG:
            if cap['risk'] == 'read':
                continue
            assert s.execution_gate(cap['id'])['code'] == 'VERIFICATION_ONLY'
            for call in (s.command(command(cap['id'])), s._rpc(cap['id'], {}, manual_owner=OWNER)):
                with pytest.raises(ConsoleError) as exc:
                    await call
                assert exc.value.code == 'VERIFICATION_ONLY'
        assert s.adapter.calls == []
        await s._rpc('get_fsm', {})
        assert s.adapter.calls == [('get_fsm', {})]  # Positive reachable read control.
        s.motion_active = True
        await s._stop('fixture forced active')
        assert s.adapter.calls == [('get_fsm', {})]
    asyncio.run(check())


def test_adapter_guard_blocks_all_writes_without_even_loading_sdk():
    adapter = object.__new__(UnitreeAdapter)
    adapter.read_only = adapter.verify_only = adapter.accessory_test = adapter.damping_test = False
    adapter.verify_only = True
    for cap in CATALOG:
        if cap['risk'] != 'read':
            with pytest.raises(ConsoleError) as exc:
                adapter.execute(cap['id'], {})
            assert exc.value.code == 'VERIFICATION_ONLY'


def test_preflight_http_checks_owner_params_and_leaves_reads_live(tmp_path):
    app = create_app(Settings(manual_remote=True, verify_only=True, data_dir=tmp_path))
    with TestClient(app) as client:
        client.get('/api/session')
        headers = {'X-G1-Intent': 'operator'}
        assert client.post('/api/connect', json={'mode': 'demo'}, headers=headers).status_code == 200
        data = dict(owner=OWNER, capability='balance', params={'value': 1})
        assert client.post('/api/control/preflight', json=data, headers=headers).json()['error']['code'] == 'NO_CONTROL_LEASE'
        assert client.post('/api/control/acquire', json={'owner': OWNER}, headers=headers).status_code == 200
        assert client.post('/api/control/preflight', json=data, headers=headers).json()['dispatched'] is False
        assert client.post('/api/control/preflight', json={**data, 'params': {'value': 2}}, headers=headers).status_code == 422
        assert client.post('/api/control/preflight', json={**data, 'owner': OTHER}, headers=headers).status_code == 409
        assert client.post('/api/control/preflight', json=data).status_code == 403
        assert client.post('/api/control/preflight', json=data, headers={**headers, 'Origin': 'http://evil.test'}).status_code == 403
        assert client.post('/api/command', json=command('stop').model_dump(), headers=headers).json()['error']['code'] == 'VERIFICATION_ONLY'
        assert client.post('/api/command', json=command('get_fsm').model_dump(), headers=headers).status_code == 200
        assert not client.get('/api/state').json()['manual_remote']['armed']


@pytest.mark.parametrize('op,params,method,args', [
    ('squat_to_stand', {}, 'SetFsmId', (706,)), ('stand_to_squat', {}, 'SetFsmId', (706,)),
    ('lie_to_stand', {}, 'SetFsmId', (702,)), ('prepare', {}, 'SetFsmId', (4,)),
    ('start', {}, 'SetFsmId', (500,)), ('damp', {}, 'SetFsmId', (1,)),
    ('balance', {'value': 0}, 'SetBalanceMode', (0,)), ('balance', {'value': 1}, 'SetBalanceMode', (1,)),
    ('stand_height', {'value': .5}, 'SetStandHeight', (.5,)),
    ('high_stand', {}, 'SetStandHeight', (4294967295,)), ('low_stand', {}, 'SetStandHeight', (0,)),
    ('stop', {}, 'SetVelocity', (0., 0., 0., .25)),
    ('velocity', {'vx': .1, 'vy': -.1, 'omega': .2}, 'SetVelocity', (.1, -.1, .2, .25)),
    ('user_control', {}, 'SwitchToUserCtrl', ()), ('internal_control', {'value': 2}, 'SwitchToInternalCtrl', (2,)),
])
def test_preflight_payload_matches_pinned_official_sdk_serialization(op, params, method, args):
    # Load actual official functions via AST, replacing only transport. No DDS Init.
    sdk = Path(__file__).resolve().parents[1] / 'externals/unitree_sdk2_python/unitree_sdk2py/g1/loco'
    namespace = {'json': json}
    exec(compile((sdk/'g1_loco_api.py').read_text(), str(sdk/'g1_loco_api.py'), 'exec'), namespace)
    tree = ast.parse((sdk/'g1_loco_client.py').read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
    func = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == method)
    exec(compile(ast.Module(body=[func], type_ignores=[]), '<official-sdk-method>', 'exec'), namespace)
    calls = []
    client = SimpleNamespace(_Call=lambda api, payload: (calls.append((api, json.loads(payload))) or (0, '')))
    assert MethodType(namespace[method], client)(*args) == 0
    request = describe_request(op, params)
    assert calls == [(request['api_id'], request['parameter'])]


def test_reconnector_refuses_loss_of_verification_guard():
    manual = dict(read_only=False, manual_remote=dict(enabled=True, verify_only=True))
    verify_configuration(manual, manual_remote=True, verify_only=True)
    with pytest.raises(RuntimeError):
        verify_configuration({**manual, 'manual_remote': {'enabled': True}}, True, True)


def test_verification_settings_cannot_silently_become_readonly_or_unscoped():
    Settings(manual_remote=True, verify_only=True).validate()
    for settings in (Settings(verify_only=True), Settings(manual_remote=True, verify_only=True, read_only=True)):
        with pytest.raises(ValueError):
            settings.validate()
