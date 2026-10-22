"""One-click startup / shutdown: every step waits for its own dual readback; nothing is retried or substituted."""
import asyncio
import time
from uuid import uuid4

import pytest
from pydantic import ValidationError

from g1_console import mode_sequence
from g1_console.adapters import FSM_COMMANDS
from g1_console.config import ConsoleError, Settings
from g1_console.contracts import Command, SequenceContinue, SequenceStart
from g1_console.service import ConsoleService
from tests.test_g1_manual_remote import OWNER, OTHER, RecordingRobot, arm

CONFIRM = dict(posture_observed=True, protection_ready=True, area_clear_observed=True, remote_in_hand=True)
MODE_OPS = set(FSM_COMMANDS)


class Robot(RecordingRobot):
    """SetFsmId lands only when `obeys`; `after` maps a reached FSM to the one the firmware settles in."""

    def __init__(self, fsm, obeys=True, code=0, after=None):
        super().__init__()
        self.fsm, self.obeys, self.code, self.after = fsm, obeys, code, after or {}

    def execute(self, op, params):
        self.calls.append((op, params))
        if op in FSM_COMMANDS:
            if self.obeys:
                self.fsm = self.after.get(FSM_COMMANDS[op], FSM_COMMANDS[op])
            return self.code, None
        return super(RecordingRobot, self).execute(op, params)


EVIDENCE = dict(mode_sequence.EVIDENCE)


@pytest.fixture(autouse=True)
def fast(monkeypatch):
    monkeypatch.setattr(mode_sequence, 'SETTLE_S', .05)
    # Chain mechanics are tested on the seat route too; the evidence gate has its own tests below.
    monkeypatch.setattr(mode_sequence, 'EVIDENCE', {op: (True, why) for op, (_, why) in EVIDENCE.items()})
    for name, spec in mode_sequence.ROUTES.items():
        short = {k: tuple((op, target, .4 if timeout else timeout) for op, target, timeout in spec[k]) for k in ('start', 'stop')}
        monkeypatch.setitem(mode_sequence.ROUTES, name, {**spec, **short})


async def setup(tmp_path, robot):
    s = ConsoleService(Settings(data_dir=tmp_path, manual_remote=True))
    s.adapter = robot
    await s._sample()
    await s.acquire(OWNER, True)
    return s


async def ticking(s, until, limit=5.):
    deadline = time.monotonic() + limit
    while not until() and time.monotonic() < deadline:
        await s.tick()
        await s.heartbeat(OWNER) if s.owner == OWNER else None
        await asyncio.sleep(.02)


def modes(s):
    return [op for op, _ in s.adapter.calls if op in MODE_OPS or op == 'stop']


def start(route, kind='start', owner=OWNER):
    return SequenceStart(owner=owner, route=route, kind=kind, **CONFIRM)


def command(op, seq, **extra):
    return Command(owner=OWNER, capability=op, sequence=seq, issued_at=time.time(), request_id=uuid4().hex, **extra)


def test_seat_route_chains_and_pauses_only_where_the_manual_needs_a_human(tmp_path):
    async def check():
        s = await setup(tmp_path, Robot(fsm=0))
        await s.start_sequence(start('seat'))
        await ticking(s, lambda: s.mode_sequence.status != 'running')
        assert s.mode_sequence.status == 'waiting_operator' and modes(s) == ['damp', 'prepare']
        with pytest.raises(ConsoleError) as exc:  # Walking stays shut while the chain owns the robot.
            await s.command(command('velocity', 1, params=dict(vx=.1)))
        await s.continue_sequence(SequenceContinue(owner=OWNER, upright_held=True))
        await ticking(s, lambda: not s.mode_sequence.live)
        view = s.snapshot()['manual_remote']['sequence']
        assert view['status'] == 'done' and [x['status'] for x in view['steps']] == ['done'] * 4
        assert modes(s) == ['damp', 'prepare', 'start'] and s.telemetry['fsm'] == 500
        assert s.manual_permit is None  # Walking still needs its own on-site confirmation.
    asyncio.run(check())


@pytest.mark.parametrize('after, expected', [({}, ['squat_to_stand', 'start']), ({706: 500}, ['squat_to_stand'])])
def test_squat_route_from_damping_skips_reached_steps(tmp_path, after, expected):
    async def check():
        s = await setup(tmp_path, Robot(fsm=1, after=after))
        await s.start_sequence(start('squat'))
        await ticking(s, lambda: not s.mode_sequence.live)
        assert s.mode_sequence.status == 'done', s.mode_sequence.message
        assert modes(s) == expected and s.telemetry['fsm'] == 500
    asyncio.run(check())


def test_missing_readback_aborts_without_retry_or_fallback(tmp_path):
    async def check():
        s = await setup(tmp_path, Robot(fsm=0, obeys=False))
        await s.start_sequence(start('seat'))
        await ticking(s, lambda: not s.mode_sequence.live)
        assert s.mode_sequence.status == 'aborted' and '未回读到 FSM 1' in s.mode_sequence.message
        assert modes(s) == ['damp']
    asyncio.run(check())


def test_3104_is_settled_by_readback_not_resent(tmp_path):
    async def check():
        s = await setup(tmp_path, Robot(fsm=1, code=3104))
        await s.start_sequence(start('squat'))
        await ticking(s, lambda: not s.mode_sequence.live)
        assert s.mode_sequence.status == 'done' and modes(s) == ['squat_to_stand', 'start']
    asyncio.run(check())


@pytest.mark.parametrize('interrupt', ['stop', 'release', 'abort'])
def test_stop_release_or_abort_ends_the_chain_with_no_further_mode_write(tmp_path, interrupt):
    async def check():
        s = await setup(tmp_path, Robot(fsm=0))
        await s.start_sequence(start('seat'))
        await ticking(s, lambda: s.mode_sequence.status == 'waiting_operator')
        if interrupt == 'stop':
            await s.command(command('stop', 1))
        elif interrupt == 'release':
            await s.release(OWNER)
        else:
            await s.abort_sequence(OWNER)
        await ticking(s, lambda: s.mode_sequence.task.done(), 1.)
        assert s.mode_sequence.status == 'aborted'
        assert [op for op in modes(s) if op != 'stop'] == ['damp', 'prepare']
    asyncio.run(check())


def test_shutdown_from_walking_stops_then_follows_the_route(tmp_path):
    async def check():
        s = await setup(tmp_path, Robot(fsm=500))
        await s.start_sequence(start('seat', 'stop'))
        await ticking(s, lambda: not s.mode_sequence.live)
        assert s.mode_sequence.status == 'done' and modes(s)[-2:] == ['stop', 'sit'] and s.telemetry['fsm'] == 3
    asyncio.run(check())


@pytest.mark.parametrize('route, kind, fsm', [('seat', 'start', 500), ('seat', 'stop', 0), ('squat', 'stop', 4)])
def test_routes_only_start_from_their_documented_fsm(tmp_path, route, kind, fsm):
    async def check():
        s = await setup(tmp_path, Robot(fsm=fsm))
        with pytest.raises(ConsoleError) as exc:
            await s.start_sequence(start(route, kind))
        assert exc.value.code == 'SEQUENCE_PRECONDITION' and s.adapter.calls == []
    asyncio.run(check())


def test_sequence_needs_the_controller_and_honest_confirmations(tmp_path):
    for key in CONFIRM:
        with pytest.raises(ValidationError):
            SequenceStart(owner=OWNER, route='seat', kind='start', **{**CONFIRM, key: False})
    with pytest.raises(ValidationError):
        SequenceStart(owner=OWNER, route='lie', kind='start', **CONFIRM)

    async def check():
        s = await setup(tmp_path, Robot(fsm=0))
        with pytest.raises(ConsoleError):
            await s.start_sequence(start('seat', owner=OTHER))
        assert s.adapter.calls == []
    asyncio.run(check())


def test_single_mode_ops_stay_locked_but_damping_protection_is_open_to_the_controller(tmp_path):
    async def check():
        s = await setup(tmp_path, Robot(fsm=500))
        await arm(s)  # Even an armed walking permit never unlocks a single mode switch.
        for seq, op in enumerate(('prepare', 'start', 'sit', 'squat_to_stand', 'zero_torque'), 1):
            with pytest.raises(ConsoleError) as exc:
                await s.command(command(op, seq, acknowledge_risk=True))
            assert exc.value.code == 'SAFETY_ENVELOPE_UNVERIFIED'
        with pytest.raises(ConsoleError) as exc:
            await s.command(command('damp', 10))
        assert exc.value.code == 'RISK_ACK_REQUIRED' and modes(s) == []
        await s.command(command('damp', 11, acknowledge_risk=True))
        assert modes(s) == ['damp']
    asyncio.run(check())


class SlowRobot(Robot):
    """Field case 2026-10-22 20:29: SetFsmId blocked ~0.7 s then returned 3104 while the mode did change."""

    def execute(self, op, params):
        if op in FSM_COMMANDS:
            time.sleep(.8)  # Longer than telemetry_ttl; sampling is paused behind the service lock.
        return super().execute(op, params)


def test_slow_3104_dispatch_is_not_mistaken_for_stale_feedback(tmp_path):
    async def check():
        s = await setup(tmp_path, SlowRobot(fsm=0, code=3104))
        await s.start_sequence(start('seat'))
        await ticking(s, lambda: s.mode_sequence.status != 'running', 8.)
        assert s.mode_sequence.status == 'waiting_operator', s.mode_sequence.message
        assert modes(s) == ['damp', 'prepare']
    asyncio.run(check())


class GappyRobot(Robot):
    """Field case 2026-10-22 20:35: the low-rate HG sport topic dropped out of its window in ~30% of reads."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.reads = 0

    def sample(self):
        data = super().sample()
        self.reads += 1
        if self.reads % 4 != 0:  # Mostly out of window, so every dispatch meets a gap first.
            data['safety_feedback']['sport']['current'] = False
        return data


def test_sport_readback_gaps_delay_dispatch_instead_of_aborting(tmp_path):
    async def check():
        s = await setup(tmp_path, GappyRobot(fsm=1))
        await s.start_sequence(start('seat'))
        await ticking(s, lambda: s.mode_sequence.status != 'running', 8.)
        assert s.mode_sequence.status == 'waiting_operator', s.mode_sequence.message
        assert modes(s) == ['prepare']
    asyncio.run(check())


def test_feedback_that_never_completes_aborts_before_any_write(tmp_path, monkeypatch):
    monkeypatch.setattr(mode_sequence, 'READY_WAIT', .3)

    async def check():
        s = await setup(tmp_path, Robot(fsm=1))
        await s.start_sequence(start('seat'))
        s.adapter.sample = lambda: {**Robot.sample(s.adapter), 'safety_feedback': {}}
        await ticking(s, lambda: not s.mode_sequence.live, 3.)
        assert s.mode_sequence.status == 'aborted' and modes(s) == []
    asyncio.run(check())


class TippingRobot(Robot):
    """Field case 2026-10-22 20:55: lock-stand (FSM 4) from the chair read back fine, then the body went over."""

    def __init__(self, *args, tip_on='prepare', roll=1.46, **kwargs):
        super().__init__(*args, **kwargs)
        self.tip_on, self.roll, self.tipped = tip_on, roll, False

    def execute(self, op, params):
        self.tipped = self.tipped or op == self.tip_on
        return super().execute(op, params)

    def sample(self):
        data = super().sample()
        if self.tipped:
            data['imu']['rpy'] = [self.roll, -.6, .1]
        return data


@pytest.mark.parametrize('roll, reason', [(1.46, '疑似倾倒'), (.5, '未直立')])
def test_body_tipping_after_lock_stand_aborts_before_the_hold_prompt(tmp_path, roll, reason):
    async def check():
        s = await setup(tmp_path, TippingRobot(fsm=1, roll=roll))
        await s.start_sequence(start('seat'))
        await ticking(s, lambda: not s.mode_sequence.live)
        assert s.mode_sequence.status == 'aborted' and reason in s.mode_sequence.message
        assert modes(s) == ['prepare'] and [x['status'] for x in s.mode_sequence.steps][1:] == ['pending', 'pending']
    asyncio.run(check())


def test_squat_stand_up_must_end_upright(tmp_path):
    async def check():
        s = await setup(tmp_path, TippingRobot(fsm=1, tip_on='squat_to_stand', roll=.5))
        await s.start_sequence(start('squat'))
        await ticking(s, lambda: not s.mode_sequence.live)
        assert s.mode_sequence.status == 'aborted' and '未直立' in s.mode_sequence.message
        assert modes(s) == ['squat_to_stand']
    asyncio.run(check())


@pytest.mark.parametrize('route, kind, fsm, roll', [('squat', 'start', 1, 1.2), ('seat', 'stop', 4, .84), ('squat', 'stop', 500, .5)])
def test_posture_outside_the_route_refuses_before_any_write(tmp_path, route, kind, fsm, roll):
    async def check():
        robot = TippingRobot(fsm=fsm, roll=roll)
        robot.tipped = True
        s = await setup(tmp_path, robot)
        with pytest.raises(ConsoleError) as exc:
            await s.start_sequence(start(route, kind))
        assert exc.value.code == 'SEQUENCE_PRECONDITION' and modes(s) == []
    asyncio.run(check())


def test_seat_start_is_suspended_after_the_field_tip_over(tmp_path, monkeypatch):
    monkeypatch.setattr(mode_sequence, 'EVIDENCE', EVIDENCE)

    async def check():
        s = await setup(tmp_path, Robot(fsm=1))
        for kind in ('start', 'stop'):
            with pytest.raises(ConsoleError) as exc:
                await s.start_sequence(start('seat', kind))
            assert exc.value.code == 'ROUTE_SUSPENDED' and modes(s) == []
        routes = s.snapshot()['manual_remote']['routes']
        assert routes['seat']['start'] and routes['seat']['stop'] and routes['squat'] == dict(start=None, stop=None)
    asyncio.run(check())


def test_every_chained_op_has_evidence_and_only_documented_ops_are_open():
    ops = {op for spec in mode_sequence.ROUTES.values() for kind in ('start', 'stop') for op, _, _ in spec[kind]} | {'stop'}
    assert ops <= set(EVIDENCE)
    assert {op for op, (ok, _) in EVIDENCE.items() if not ok} == {'prepare', 'sit'}


class HotRobot(Robot):
    def __init__(self, *args, heat=72, heat_on=None, **kwargs):
        super().__init__(*args, **kwargs)
        self.heat, self.heat_on, self.hot = heat, heat_on, heat_on is None

    def execute(self, op, params):
        self.hot = self.hot or op == self.heat_on
        return super().execute(op, params)

    def sample(self):
        data = super().sample()
        if self.hot:
            data['joints'][0]['temperature'] = [47, self.heat]
        return data


def test_hot_motor_refuses_start_but_not_stop(tmp_path):
    async def check():
        s = await setup(tmp_path, HotRobot(fsm=1))
        with pytest.raises(ConsoleError) as exc:
            await s.start_sequence(start('squat'))
        assert exc.value.code == 'MOTOR_HOT' and modes(s) == []
        thermal = s.snapshot()['manual_remote']['thermal']
        assert thermal == dict(limit_c=70, max_c=72, joint=0, hot=True)
        s.adapter.fsm = 500
        await s._sample()
        await s.start_sequence(start('squat', 'stop'))
        await ticking(s, lambda: not s.mode_sequence.live)
        assert s.mode_sequence.status == 'done' and modes(s) == ['stop', 'stand_to_squat']
    asyncio.run(check())


def test_motor_heating_during_start_aborts_with_no_further_write(tmp_path):
    async def check():
        s = await setup(tmp_path, HotRobot(fsm=1, heat_on='squat_to_stand'))
        await s.start_sequence(start('squat'))
        await ticking(s, lambda: not s.mode_sequence.live)
        assert s.mode_sequence.status == 'aborted' and '温度' in s.mode_sequence.message
        assert modes(s) == ['squat_to_stand']
    asyncio.run(check())
