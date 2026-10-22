"""One-click startup / shutdown along the official remote-manual routes (23 DOF edition 2026-04-07).

Each step is a single official SetFsmId. The next step starts only after GetFsmId and the HG sport
field both read the target *after* the request was issued and every joint has settled. 3104 is an
unknown outcome, so the readback decides. Nothing is retried, and an aborted sequence never issues a
fallback mode: damping an unsupported robot can drop it, so the operator takes over with the
physical remote. The lying route needs adaptive stand-up (manual item 10), which has no verified SDK
entry, so it is not offered.
"""
import asyncio
import time

from .config import ConsoleError
from .state_learning import PROJECT_MOTOR_TEMP_C, PROJECT_TILT_RAD

# (op, target FSM, readback timeout s); 'hold' pauses for the operator, as the manual requires.
ROUTES = {
    'seat': dict(label='座椅路线（官方推荐）', posture='机器人端坐在椅子上',
                 start=(('damp', 1, 8.), ('prepare', 4, 15.), ('hold', None, 60.), ('start', 500, 8.)),
                 stop=(('sit', 3, 15.),), stop_from=(500, 4),
                 hold='锁定站立后请人工扶直机器人，扶稳后点“已扶直，进入行走”'),
    'squat': dict(label='蹲姿路线', posture='机器人双脚着地蹲姿',
                  start=(('damp', 1, 8.), ('squat_to_stand', 706, 20.), ('start', 500, 8.)),
                  stop=(('stand_to_squat', 706, 20.),), stop_from=(500, 706)),
}
START_FROM = (0, 1)
# Posture guard (IMU roll / pitch). Field 2026-10-22 20:55: after FSM 4 from the chair the body rolled to
# 1.46 rad and ended at pitch -1.52 while every joint was still, and the chain went on to the hold prompt.
FALL_RAD = .8                                   # any time, any step: the body is going over
STANDING = ('prepare', 'squat_to_stand', 'start')  # these must end upright
# Evidence per mode op: (allowed in an automatic chain, basis). Only an op whose SDK entry is used the same way in
# an official G1 example, or that was read back on this robot, may be chained; a name match or a candidate mapping
# may not (2026-10-22: FSM 4 was chained as manual item 2 on a candidate mapping and the robot tipped).
# Sources: doc/G1_STANDUP_AND_DAMP_MAPPING_20261022.md, assets/safety_evidence_sources/official_remote_manual_20261022_v34.json.
EVIDENCE = dict(
    stop=(True, 'StopMove / 零速度，只撤销速度请求'),
    hold=(True, '等待现场人员确认，不发送指令'),
    damp=(True, 'SDK Damp → FSM 1；本机 13:26、20:29 两次双路回读'),
    squat_to_stand=(True, '官方 G1 示例 Damp → Squat2StandUp（FSM 706）；本机从坐地发送时未改变 FSM，蹲姿起点未实机验证'),
    start=(True, 'G1 Python / C++ SDK 均为 Start → FSM 500；只在直立门控通过后发送'),
    stand_to_squat=(True, '官方 G1 示例 StandUp2Squat（FSM 706）'),
    prepare=(False, 'FSM 4 仅为 C++ StandUp() 的候选对应，与手册 ② 锁定站立的等价性未核定；2026-10-22 实测发送后倾倒'),
    sit=(False, 'FSM 3 为 SDK Sit()，官方 G1 示例未使用；手册 ④ 的座椅条件无法由传感器确认，本机未核定'),
)
# Readbacks that also complete a step: stand-up may hand over to walking control before 706 is sampled.
ALSO_REACHED = dict(squat_to_stand=(500,))
SETTLE_DQ = .5        # rad/s on every joint
SETTLE_S = 1.
SETTLE_TIMEOUT = 10.
# rt/lf/sportmodestate is a low-rate topic; field 2026-10-22 it was outside its 1.5 s window in 9/30 reads.
READY_WAIT = 3.
LABELS = dict(damp='阻尼', prepare='锁定站立', hold='人工扶直', start='进入行走', sit='落座',
              squat_to_stand='蹲 → 站', stand_to_squat='站 → 蹲', stop='停止行走')


def blocked(route, kind):
    """Why this route direction may not run as a chain, or None."""
    reasons = [EVIDENCE[op][1] for op, _, _ in ROUTES[route][kind] if not EVIDENCE[op][0]]
    return f'{ROUTES[route]["label"]}一键{"启动" if kind == "start" else "停止"}未开放：' + '；'.join(reasons) + '。请用实体遥控器' if reasons else None


def hottest(joints):
    temps = [t for j in joints or [] for t in j.get('temperature') or []]
    return max(temps) if temps else None


def tilt(rpy):
    return max(abs(rpy[0]), abs(rpy[1])) if rpy and len(rpy) == 3 else None


def degrees(angle):
    return '未知' if angle is None else f'{angle:.2f} rad'


def plan(route, kind, telemetry):
    """Steps for this request from the current FSM, posture and temperature; raises if the route does not start here."""
    if route not in ROUTES:
        raise ConsoleError('ROUTE_UNKNOWN', '未知的启停路线', 422)
    reason = blocked(route, kind)
    if reason:
        raise ConsoleError('ROUTE_SUSPENDED', reason, 409)
    spec = ROUTES[route]
    fsm = telemetry.get('fsm')
    angle = tilt((telemetry.get('imu') or {}).get('rpy'))
    heat = hottest(telemetry.get('joints'))
    # Start only: a stop moves toward rest, and refusing it would leave a hot robot under load.
    if kind == 'start' and (heat is None or heat >= PROJECT_MOTOR_TEMP_C):
        raise ConsoleError('MOTOR_HOT', f'电机温度 {heat} ℃ 达到项目拦截线 {PROJECT_MOTOR_TEMP_C} ℃（或缺失）；降温后再启动', 409)
    if angle is None:
        raise ConsoleError('INCOMPLETE_BODY', '需要完整有效的 IMU 姿态')
    if kind == 'start' and angle > FALL_RAD:
        raise ConsoleError('SEQUENCE_PRECONDITION', f'躯干倾斜 {angle:.2f} rad，疑似倒地；请人工扶正到路线起始姿态', 409)
    if kind == 'stop' and angle > PROJECT_TILT_RAD:
        raise ConsoleError('SEQUENCE_PRECONDITION', f'躯干倾斜 {angle:.2f} rad，不是直立姿态；一键停止不适用，请用实体遥控器阻尼', 409)
    if kind == 'start':
        if fsm not in START_FROM:
            raise ConsoleError('SEQUENCE_PRECONDITION', f'一键启动需从零力矩或阻尼（FSM 0/1）开始，当前 {fsm}', 409)
        steps = [s for s in spec['start'] if not (s[0] == 'damp' and fsm == 1)]
    else:
        if fsm not in spec['stop_from']:
            raise ConsoleError('SEQUENCE_PRECONDITION', f'{spec["label"]}的一键停止需从 FSM {spec["stop_from"]} 开始，当前 {fsm}', 409)
        steps = ([('stop', None, None)] if fsm == 500 else []) + list(spec['stop'])
    return [dict(op=op, label=LABELS[op], target=target, timeout=timeout, status='pending') for op, target, timeout in steps]


class Sequence:
    def __init__(self, service, owner, route, kind, steps):
        self.service, self.owner, self.route, self.kind, self.steps = service, owner, route, kind, steps
        self.status, self.message, self.index, self.dispatching = 'running', '', 0, None
        self.started_at, self.finished_at = time.time(), None
        self.continued = asyncio.Event()
        self.task = None

    def view(self):
        return dict(route=self.route, route_label=ROUTES[self.route]['label'], kind=self.kind, status=self.status,
                    message=self.message, current=self.index if self.status in ('running', 'waiting_operator') else None,
                    steps=[{k: s[k] for k in ('op', 'label', 'target', 'status')} for s in self.steps],
                    started_at=self.started_at, finished_at=self.finished_at)

    def finish(self, status, message):
        if self.status in ('running', 'waiting_operator'):
            self.status, self.message, self.finished_at = status, message, time.time()
            if status != 'done' and self.index < len(self.steps):
                self.steps[self.index]['status'] = 'aborted'
            self.continued.set()
            self.service.event(f'sequence_{status}', f'{ROUTES[self.route]["label"]} · {"启动" if self.kind == "start" else "停止"}：{message}',
                               'info' if status == 'done' else 'warning')

    @property
    def live(self):
        return self.status in ('running', 'waiting_operator')

    def _check(self, fresh=True):
        s = self.service
        if not self.live:
            raise asyncio.CancelledError
        if s.owner != self.owner:
            raise ConsoleError('NO_CONTROL_LEASE', '控制权已失去')
        if fresh and not s.fresh():
            raise ConsoleError('STALE_TELEMETRY', '身体反馈过期')
        angle = tilt((s.telemetry.get('imu') or {}).get('rpy'))
        if fresh and (angle is None or angle > FALL_RAD):
            raise ConsoleError('BODY_FALLING', f'躯干倾斜 {degrees(angle)}，疑似倾倒；立即中止')
        heat = hottest(s.telemetry.get('joints'))
        if fresh and self.kind == 'start' and (heat is None or heat >= PROJECT_MOTOR_TEMP_C):
            raise ConsoleError('MOTOR_HOT', f'电机温度 {heat} ℃ 达到项目拦截线 {PROJECT_MOTOR_TEMP_C} ℃（或缺失）；立即中止')

    async def _wait(self, condition, timeout, message, fresh=True):
        deadline = time.monotonic() + timeout
        while True:
            self._check(fresh)
            if condition():
                return
            if time.monotonic() > deadline:
                raise ConsoleError('SEQUENCE_TIMEOUT', message)
            await asyncio.sleep(.05)

    def _reads(self, op, target, issued):
        t = self.service.telemetry
        sport = ((t.get('safety_feedback') or {}).get('sport') or {})
        fsm = t.get('fsm')
        return (fsm in (target, *ALSO_REACHED.get(op, ())) and t.get('fsm_received', 0) > issued and sport.get('current')
                and (sport.get('raw') or {}).get('fsm_id') == fsm)

    async def _settle(self):
        still_since = None
        deadline = time.monotonic() + SETTLE_TIMEOUT
        while True:
            self._check()
            joints = self.service.telemetry.get('joints') or []
            still = len(joints) == 23 and all(abs(j.get('dq') or 0) < SETTLE_DQ for j in joints)
            now = time.monotonic()
            still_since = (still_since or now) if still else None
            if still_since and now - still_since >= SETTLE_S:
                return
            if now > deadline:
                raise ConsoleError('NOT_SETTLED', f'{SETTLE_TIMEOUT:.0f} 秒内关节未静止')
            await asyncio.sleep(.05)

    async def _step(self, step):
        s = self.service
        if step['op'] == 'hold':
            self.status, self.message = 'waiting_operator', ROUTES[self.route]['hold']
            deadline = time.monotonic() + step['timeout']
            while not self.continued.is_set():  # Keep watching the posture while the operator works.
                self._check()
                if time.monotonic() > deadline:
                    raise ConsoleError('SEQUENCE_TIMEOUT', '等待人工扶直超时；保持锁定站立，请用实体遥控器接管')
                await asyncio.sleep(.05)
            self._check()
            self.status = 'running'
            return
        if step['target'] is not None and s.telemetry.get('fsm') == step['target'] and step['op'] == 'start':
            return  # Squat stand-up may already end in walking control.
        # Dispatch at the first moment the full body evidence is current; nothing is sent while waiting.
        deadline = time.monotonic() + READY_WAIT
        while True:
            async with s.lock:
                self._check()
                try:
                    s._body_ready()
                except ConsoleError:
                    if time.monotonic() > deadline:
                        raise
                else:
                    issued = time.monotonic()
                    self.dispatching = step['op']
                    try:
                        if step['op'] == 'stop':
                            await s._stop('一键停止', 'cancelled')
                        await s._rpc(step['op'], {}, manual_owner=self.owner)
                    except ConsoleError as exc:
                        if exc.code != 'SDK_TIMEOUT':
                            raise
                        step['sdk'] = '3104'  # Unknown outcome: the readback below decides, never a resend.
                    finally:
                        self.dispatching = None
                    break
            await asyncio.sleep(.05)
        # The service lock was held for the whole SDK call (3104 alone takes ~0.7 s), so sampling paused.
        # Judge freshness only after the loop has had the chance to take a new sample.
        returned = time.monotonic()
        await self._wait(lambda: s.telemetry.get('received', 0) > returned, 1.5,
                         '请求返回后 1.5 秒内没有新的身体反馈', fresh=False)
        if step['target'] is None:
            await asyncio.sleep(.3)
            return
        await self._wait(lambda: self._reads(step['op'], step['target'], issued), step['timeout'],
                         f'{step["timeout"]:.0f} 秒内未回读到 FSM {step["target"]}')
        await self._settle()
        angle = tilt((s.telemetry.get('imu') or {}).get('rpy'))
        if step['op'] in STANDING and (angle is None or angle > PROJECT_TILT_RAD):
            raise ConsoleError('NOT_UPRIGHT', f'{step["label"]}后躯干倾斜 {degrees(angle)}，未直立')

    async def run(self):
        try:
            for self.index, step in enumerate(self.steps):
                step['status'] = 'active'
                await self._step(step)
                step['status'] = 'done'
            self.index = len(self.steps)
            self.finish('done', '已到达 FSM 500，可开始驾驶' if self.kind == 'start' else '已到达路线终点姿态')
        except asyncio.CancelledError:
            self.finish('aborted', '已中止')
        except ConsoleError as exc:
            self.finish('aborted', f'{exc.message}；未自动回退其他模式，请用实体遥控器接管')
        except Exception as exc:  # The loop must always land in a terminal state.
            self.finish('aborted', f'序列异常 {type(exc).__name__}；未自动回退，请用实体遥控器接管')
