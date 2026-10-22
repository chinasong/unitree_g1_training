"""Manual authorization and final dispatch proof; every actuator call is a recording fake."""
import asyncio
from dataclasses import replace
import time
from uuid import uuid4

import pytest
from fastapi.testclient import TestClient
from pydantic import ValidationError

from g1_console.adapters import DemoAdapter
from g1_console.app import create_app
from g1_console.capabilities import ARM_ACTIONS, CATALOG
from g1_console.config import ConsoleError, Settings
from g1_console.contracts import Command, Goal, ManualArm
from g1_console.navigation import World
from g1_console.service import ConsoleService
from scripts.connect_orin_console import verify_service

OWNER = 'manual-owner-00000000001'
OTHER = 'manual-owner-00000000002'
CONFIRM = dict(standing_observed=True, unsupported_observed=True, area_clear_observed=True, floor_verified=True)


def test_tunnel_requires_explicit_manual_selection_and_real_robot():
    readonly = dict(mode='robot', read_only=True, manual_remote=dict(enabled=False))
    manual = dict(mode='robot', read_only=False, manual_remote=dict(enabled=True, verify_only=False,
                                                                    accessory_test=False, damping_test=False))
    verify_service(readonly)
    verify_service(manual, True)
    restricted = [{**manual, 'manual_remote': {**manual['manual_remote'], flag: True}}
                  for flag in ('verify_only', 'accessory_test', 'damping_test')]
    for state, selected in ((manual, False), (readonly, True),
                            ({**manual, 'manual_remote': {}}, True),
                            ({**manual, 'manual_remote': {'enabled': True}}, True),
                            ({**manual, 'mode': 'demo'}, True), *((s, True) for s in restricted)):
        with pytest.raises(RuntimeError):
            verify_service(state, selected)


class RecordingRobot(DemoAdapter):
    mode = 'robot'

    def __init__(self):
        super().__init__()
        self.calls = []
        self.action_data = [dict(id=i, name=name, fsm=[500], mode_machine=[4]) for i, name in ARM_ACTIONS.items()]

    def sample(self):
        data = super().sample()
        data['mode_machine'] = 4
        data['safety_feedback'] = {
            'fsm_mode': {'current': True, 'raw': {'data': 0}},
            'sport': {'current': True, 'raw': {'fsm_id': self.fsm, 'fsm_mode': 0}},
        }
        return data

    def execute(self, op, params):
        self.calls.append((op, params))
        return super().execute(op, params)


def run(coro):
    return asyncio.run(coro)


def command(op, seq=1, owner=OWNER, **extra):
    return Command(owner=owner, capability=op, sequence=seq, issued_at=time.time(), request_id=uuid4().hex, **extra)


async def setup(tmp_path, **settings):
    service = ConsoleService(Settings(data_dir=tmp_path, manual_remote=True, **settings))
    service.adapter = RecordingRobot()
    await service._sample()
    await service.acquire(OWNER, True)
    service.world = World()  # No scan; manual visual confirmation is distinct from autonomous sensing.
    return service


async def arm(service):
    await service.arm_manual(ManualArm(owner=OWNER, **CONFIRM))


def test_arm_contract_rejects_missing_false_and_coerced_attestations():
    for key in CONFIRM:
        for value in (False, 1, 'true', None):
            with pytest.raises(ValidationError):
                ManualArm(owner=OWNER, **{**CONFIRM, key: value})
    with pytest.raises(ValidationError):
        ManualArm(owner=OWNER)


def test_authorize_and_query_have_no_motor_writes(tmp_path):
    async def check():
        s = await setup(tmp_path)
        await arm(s)
        assert s.adapter.calls == []
        snap = s.snapshot()
        assert snap['manual_remote']['armed'] and snap['manual_remote']['full_remote_equivalence'] is False
        assert snap['assessment']['state_learning']['safety']['execution_allowed'] is False
        assert next(c for c in snap['capabilities'] if c['id'] == 'velocity')['execution_gate']['allowed']
        assert not next(c for c in snap['capabilities'] if c['id'] == 'lie_to_stand')['execution_gate']['allowed']
        await s.command(command('get_fsm'))
        assert s.adapter.calls == [('get_fsm', {})]
    run(check())


def test_current_seated_fsm_cannot_arm_even_with_all_boxes_true(tmp_path):
    async def check():
        s = await setup(tmp_path)
        s.adapter.fsm = 0
        await s._sample()
        with pytest.raises(ConsoleError, match='行走控制') as exc:
            await arm(s)
        assert exc.value.code == 'NOT_WALKING_MODE' and s.manual_permit is None
        assert s.adapter.calls == []
    run(check())


def test_final_boundary_rechecks_manual_owner_and_every_nonpermitted_operation(tmp_path):
    async def check():
        s = await setup(tmp_path)
        await arm(s)
        for cap in CATALOG:
            if cap['risk'] not in ('motion', 'guarded'):
                continue
            if cap['id'] in ('velocity', 'damp') or cap['id'].startswith('arm_'):
                continue  # damp is the manual's protection item; see test_g1_mode_sequence.py
            with pytest.raises(ConsoleError) as exc:
                await s._rpc(cap['id'], {}, manual_owner=OWNER)
            assert exc.value.code == 'SAFETY_ENVELOPE_UNVERIFIED'
        for supplied in (None, OTHER):
            with pytest.raises(ConsoleError):
                await s._rpc('velocity', dict(vx=.1, vy=0, omega=0), manual_owner=supplied)
        assert s.adapter.calls == []
        await s.command(command('velocity', params=dict(vx=.1)))
        assert s.adapter.calls[-1] == ('velocity', dict(vx=.1, vy=0., omega=0.))
        # A later stop sequence prevents a delayed held-input packet restarting motion.
        await s.command(command('stop', seq=3))
        writes = len(s.adapter.calls)
        with pytest.raises(ConsoleError) as exc:
            await s.command(command('velocity', seq=2, params=dict(vx=.1)))
        assert exc.value.code == 'OLD_COMMAND' and len(s.adapter.calls) == writes
    run(check())


@pytest.mark.parametrize('fsm', [2, 4])
def test_documented_fsm_params_do_not_unlock_unverified_real_robot_modes(tmp_path, fsm):
    async def check():
        s = await setup(tmp_path)
        await arm(s)
        with pytest.raises(ConsoleError) as exc:
            await s.command(command('fsm', params={'value': fsm}, acknowledge_risk=True))
        assert exc.value.code == 'SAFETY_ENVELOPE_UNVERIFIED'
        assert s.adapter.calls == []
    run(check())


def test_manual_cannot_lend_authority_to_autonomy(tmp_path):
    async def check():
        s = await setup(tmp_path)
        await arm(s)
        with pytest.raises(ConsoleError) as exc:
            await s.goal(Goal(owner=OWNER, x=0, y=0, request_id=uuid4().hex, sequence=1, issued_at=time.time()))
        assert exc.value.code == 'SAFETY_ENVELOPE_UNVERIFIED'
        assert s.navigation['status'] == 'idle' and s.adapter.calls == []
    run(check())


@pytest.mark.parametrize('condition', ['disabled', 'expired', 'lease', 'fsm', 'mode_conflict', 'joint_missing', 'temp_missing', 'raw_status_unknown', 'imu_invalid', 'hardware'])
def test_loss_of_conditions_blocks_final_dispatch(tmp_path, condition):
    async def check():
        s = await setup(tmp_path)
        await arm(s)
        if condition == 'disabled': s.settings = replace(s.settings, manual_remote=False)
        if condition == 'expired': s.manual_permit['until'] = time.monotonic() - 1
        if condition == 'lease': s.lease_until = time.monotonic() - 1
        if condition == 'fsm': s.telemetry['fsm_received'] = time.monotonic() - 2
        if condition == 'mode_conflict': s.telemetry['safety_feedback']['fsm_mode']['raw']['data'] = 1
        if condition == 'joint_missing': s.telemetry['joints'].pop()
        if condition == 'temp_missing': s.telemetry['joints'][0]['temperature'] = None
        if condition == 'raw_status_unknown': s.telemetry['joints'][0]['status'] = None
        if condition == 'imu_invalid': s.telemetry['imu']['rpy'][0] = float('nan')
        if condition == 'hardware': s.telemetry['mode_machine'] = 0
        with pytest.raises(ConsoleError):
            await s._rpc('velocity', dict(vx=.1, vy=0, omega=0), manual_owner=OWNER)
        assert s.adapter.calls == []
    run(check())


@pytest.mark.parametrize('condition', ['different_fields', 'id_conflict', 'mode_conflict', 'stale_sport', 'stale_mode', 'missing_sport', 'invalid_id', 'invalid_mode'])
def test_official_modes_compare_same_fields_before_final_dispatch(tmp_path, condition):
    async def check():
        s = await setup(tmp_path)
        await arm(s)
        feedback = s.telemetry['safety_feedback']
        if condition == 'id_conflict': feedback['sport']['raw']['fsm_id'] = 1
        if condition == 'mode_conflict': feedback['sport']['raw']['fsm_mode'] = 1
        if condition == 'stale_sport': feedback['sport']['current'] = False
        if condition == 'stale_mode': feedback['fsm_mode']['current'] = False
        if condition == 'missing_sport': feedback.pop('sport')
        if condition == 'invalid_id': feedback['sport']['raw']['fsm_id'] = True
        if condition == 'invalid_mode': feedback['fsm_mode']['raw']['data'] = None
        if condition == 'different_fields':
            assert s.telemetry['fsm'] == 500 and feedback['fsm_mode']['raw']['data'] == 0
            await s._rpc('velocity', dict(vx=.1, vy=0, omega=0), manual_owner=OWNER)
            assert s.adapter.calls == [('velocity', dict(vx=.1, vy=0, omega=0))]
        else:
            with pytest.raises(ConsoleError) as exc:
                await s._rpc('velocity', dict(vx=.1, vy=0, omega=0), manual_owner=OWNER)
            assert exc.value.code == ('MODE_CONFLICT' if condition.endswith('conflict') else 'MODE_FEEDBACK_UNVERIFIED')
            assert s.adapter.calls == []
    run(check())


def test_expiry_revokes_and_stops_without_resuming(tmp_path):
    async def check():
        s = await setup(tmp_path)
        await arm(s)
        await s.command(command('velocity', params=dict(vx=.1)))
        s.manual_permit['until'] = time.monotonic() - 1
        await s.tick()
        assert s.manual_permit is None and not s.motion_active
        assert s.adapter.calls[-1][0] == 'stop'
        writes = len(s.adapter.calls)
        await s.tick()
        assert len(s.adapter.calls) == writes
        with pytest.raises(ConsoleError):
            await s.command(command('velocity', seq=2, params=dict(vx=.1)))
        assert len(s.adapter.calls) == writes
    run(check())


def test_unknown_action_or_inapplicable_declaration_never_reaches_sdk(tmp_path):
    async def check():
        s = await setup(tmp_path)
        await arm(s)
        for row in (dict(id=500, name='unmapped', fsm=[500]), dict(id=11, name='missing-fsm'), dict(id=11, name='wrong-machine', fsm=[500], mode_machine=[0])):
            s.adapter.action_ids = {row['id']}
            s.adapter.action_data = [row]
            with pytest.raises(ConsoleError):
                await s._rpc('arm_action', {'value': row['id']}, manual_owner=OWNER)
        assert s.adapter.calls == []
        s.adapter.action_ids = {11}
        s.adapter.action_data = [dict(id=11, name='both-kiss', fsm=[500], mode_machine=[4])]
        await s.command(command('arm_action', params={'value': 11}))
        assert s.adapter.calls == [('arm_action', {'value': 11})]
    run(check())


def test_http_readonly_blocks_all_manual_routes_and_page_load_never_arms(tmp_path):
    app = create_app(Settings(data_dir=tmp_path, read_only=True, manual_remote=True))
    s = app.state.service
    s.adapter = RecordingRobot()
    s.telemetry = s.adapter.sample()
    s.owner, s.lease_until = OWNER, time.monotonic() + 60
    with TestClient(app) as client:
        client.get('/api/session')
        for _ in range(3):
            assert client.get('/api/state').json()['manual_remote']['armed'] is False
        assert s.adapter.calls == []
        for path, data in (('arm', {'owner': OWNER, **CONFIRM}), ('disarm', {'owner': OWNER})):
            response = client.post('/api/control/manual/' + path, json=data, headers={'x-g1-intent': 'operator'})
            assert response.status_code == 403 and response.json()['error']['code'] == 'READ_ONLY'
        assert s.adapter.calls == []
