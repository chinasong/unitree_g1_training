import asyncio
import base64
from collections import deque, OrderedDict
import json
import math
import os
import time
from uuid import uuid4
from .adapters import DemoAdapter, UnitreeAdapter, FSM_COMMANDS
from .body import BODY, KNOWLEDGE, JOINTS, empty_joints
from .state_learning import interpret, history_sample, PROJECT_TILT_RAD, PROJECT_MOTOR_TEMP_C, finite
from .assessment import assess
from .capabilities import CATALOG, BY_ID, READ_OPERATIONS, ACCESSORY_TEST_OPERATIONS, ARM_ACTIONS, firmware_action_catalog, validate_params, require_write_scope
from .config import ConsoleError, Settings
from .camera_process import IsolatedCamera as OfficialCamera
from .insight import Insights
from .contracts import Grid, Observation, CameraFrame, Command, Goal
from .navigation import World, plan, wrap, local_clear
from .remote_preflight import describe_request
from .mode_sequence import ROUTES as SEQUENCE_ROUTES, Sequence, blocked as sequence_blocked, hottest, plan as sequence_plan


class ConsoleService:
    def __init__(self, settings: Settings):
        self.settings = settings
        self.lock = asyncio.Lock()
        self.adapter = None
        self.camera_reader = None
        self.world = World()
        self.telemetry = dict(joints=empty_joints(), imu=None, received=0, fsm=None, fsm_received=0)
        self.owner, self.lease_until = None, 0.
        self.floor_verified = False
        self.manual_permit = None
        self.preflight_history = deque(maxlen=20)
        self.motion_until = 0.
        self.motion_active = False
        self.events = deque(maxlen=160)
        self.results = OrderedDict()
        self.command_history = OrderedDict()
        self.failed_commands = OrderedDict()
        self.sequences = OrderedDict()
        self.last_velocity = dict(vx=0., vy=0., omega=0.)
        self.navigation = dict(status="idle", phase="等待目标", path=[], goal=None, corrections=0)
        self.pending = None
        self.mode_sequence = None
        self.next_nav = 0.
        self.closed = False
        self.run_id = uuid4().hex[:12]
        self.insights = Insights()
        self.body_history = deque(maxlen=240)
        self.last_state_observation = 0.
        self.damping_attempt = None
        settings.data_dir.mkdir(parents=True, exist_ok=True)
        self.damping_file = settings.data_dir / 'damping-test.json'
        if settings.damping_test and self.damping_file.exists():
            try:
                self.damping_attempt = json.loads(self.damping_file.read_text())
                if not isinstance(self.damping_attempt, dict) or not self.damping_attempt:
                    raise ValueError('Invalid attempt receipt')
            except (ValueError, OSError):
                self.damping_attempt = dict(status='outcome_unknown', source_fsm=0, target_fsm=1)
        self.journal = settings.data_dir / "events.jsonl"
        self.memory_file = settings.data_dir / "memory.json"
        self.memories = []
        if self.memory_file.exists():
            try:
                self.memories = json.loads(self.memory_file.read_text()).get("visits", [])[-40:]
            except (ValueError, OSError):
                self.event("memory_error", "历史空间记录读取失败，将使用本次观察", "warning")

    def event(self, kind, message, level="info", **extra):
        event = dict(id=uuid4().hex, at=time.time(), run=self.run_id, kind=kind, message=message, level=level, **extra)
        self.events.appendleft(event)
        # Bounded, local-only audit; no frames, tokens or private credentials.
        if self.journal.exists() and self.journal.stat().st_size > 4_000_000:
            self.journal.replace(self.journal.with_suffix(".previous.jsonl"))
        with self.journal.open("a") as file:
            file.write(json.dumps(event, ensure_ascii=False, allow_nan=False) + "\n")
        return event

    def _persist_damping_receipt(self):
        # An older outcome_unknown receipt is still a spent test if persistence fails.
        temporary = self.damping_file.with_suffix('.tmp')
        try:
            temporary.write_text(json.dumps(self.damping_attempt, ensure_ascii=False))
            temporary.replace(self.damping_file)
        except OSError:
            self.event('damping_receipt_error', '阻尼结果记录更新失败；保留已占用状态，不重发', 'warning')

    async def connect(self, mode, interface):
        if self.settings.read_only and mode == "demo":
            raise ConsoleError("READ_ONLY", "此服务用于真实机器人只读观察，不能切换到合成演示", 403)
        async with self.lock:
            if self.owner and time.monotonic() < self.lease_until:
                raise ConsoleError("CONTROL_BUSY", "请先释放控制权，再切换连接")
            await self._disconnect()
            self.world = World()
            self.body_history.clear()
            self.telemetry = dict(joints=empty_joints(), imu=None, received=0, fsm=None, fsm_received=0)
            try:
                guards = dict(read_only=self.settings.read_only, verify_only=self.settings.verify_only)
                if self.settings.accessory_test:
                    guards['accessory_test'] = True
                if self.settings.damping_test:
                    guards['damping_test'] = True
                self.adapter = DemoAdapter() if mode == "demo" else await asyncio.to_thread(UnitreeAdapter, interface, **guards)
            except ConsoleError:
                raise
            except Exception as exc:
                self.event("connect_error", f"连接失败：{type(exc).__name__}", "error")
                raise ConsoleError("SDK_CONNECT_FAILED", "官方 SDK 初始化失败，请检查 Orin 环境、DDS 网卡和服务日志", 503) from exc
            self.event("connected", "演示环境已启动，全部遥测为合成数据" if mode == "demo" else "DDS 已初始化，等待真实 rt/lowstate", mode=mode)
            if mode == "robot" and self.settings.camera_backend == "unitree":
                self.camera_reader = OfficialCamera(self.settings.camera_width, self.settings.camera_fps, interface)
                self.event("camera_connected", "只读订阅官方 GetImageSample，相机设备由官方服务管理")
            await self._sample()

    async def _disconnect(self):
        self.insights.invalidate()
        await self._stop("连接关闭", nav_status="cancelled")
        if self.camera_reader:
            await asyncio.to_thread(self.camera_reader.close)
            self.camera_reader = None
        if self.adapter:
            await asyncio.to_thread(self.adapter.close)
        self.adapter = None
        self.body_history.clear()
        self.owner, self.lease_until = None, 0.
        self.floor_verified = False
        self.manual_permit = None

    async def disconnect(self):
        async with self.lock:
            await self._disconnect()
            self.world = World()
            self.telemetry = dict(joints=empty_joints(), imu=None, received=0, fsm=None, fsm_received=0)
            self.event("disconnected", "机器人连接已关闭")

    def fresh(self):
        return self.adapter is not None and time.monotonic() - self.telemetry["received"] < self.settings.telemetry_ttl

    def require_connection(self):
        if not self.adapter:
            raise ConsoleError("DISCONNECTED", "请先连接机器人或启动演示环境")
        if not self.fresh():
            raise ConsoleError("STALE_TELEMETRY", "身体反馈已过期，拒绝继续执行")

    def require_owner(self, owner):
        if self.owner != owner or time.monotonic() >= self.lease_until:
            raise ConsoleError("NO_CONTROL_LEASE", "请先取得控制权；控制权只授予一个窗口")

    def require_writable(self):
        if self.settings.read_only:
            raise ConsoleError("READ_ONLY", "只读观察模式：禁止动作、声光和控制模式写入", 403)

    def require_dispatch(self, op=None):
        require_write_scope(op, self.settings)

    def _damping_ready(self):
        """A narrow passive-mode test, never a standing or contact inference."""
        self.require_dispatch('damp')
        self.require_connection()
        if self.damping_attempt is not None:
            raise ConsoleError('DAMPING_TEST_SPENT', '已有阻尼测试记录；结果不明或服务重启也不会自动重发', 403)
        now = time.monotonic()
        if self.telemetry.get('fsm') != 0 or not 0 <= now-self.telemetry.get('fsm_received', 0) < 1.5:
            raise ConsoleError('DAMPING_START_STATE', '此测试只允许新鲜 FSM 0→1，不允许从站立或其他模式卸力', 403)
        row = self.telemetry.get('safety_feedback', {}).get('fsm_mode', {})
        if not row.get('current') or (row.get('raw') or {}).get('data') != 0:
            raise ConsoleError('DAMPING_START_STATE', '需要 GetFsmMode 同时回读 0；未知或冲突时不下发', 403)
        if self.telemetry.get('mode_machine') != 4 or self.telemetry.get('mode_pr') != 0:
            raise ConsoleError('HARDWARE_UNVERIFIED', '本机型号或关节坐标模式与已核验的 EDU 23 档案不符')
        joints = self.telemetry.get('joints', [])
        if len(joints) != 23 or {j.get('index') for j in joints} != {j['index'] for j in JOINTS}:
            raise ConsoleError('INCOMPLETE_BODY', '需要完整的 23 关节实测反馈')
        if any(not all(finite(j.get(k)) for k in ('q', 'dq')) or j.get('status') != 0 or
               len(j.get('temperature') or []) != 2 or not all(finite(t) and t < PROJECT_MOTOR_TEMP_C for t in j['temperature']) for j in joints):
            raise ConsoleError('INCOMPLETE_BODY', '关节反馈缺失、状态码异常或达到项目温度拦截线')
        window = [h for h in self.body_history if now-h['at'] <= 2.5]
        if len(window) < 2 or window[-1]['at']-window[0]['at'] < 2 or any(
            len(h.get('q', {})) != 23 or h.get('mode_machine') != 4 or h.get('mode_pr') != 0 or
            not finite(h.get('max_dq')) or h['max_dq'] > .1 or not h.get('rpy') or
            len(h['rpy']) != 3 or not all(finite(v) for v in h['rpy']) or
            max(abs(v) for v in h['rpy'][:2]) > PROJECT_TILT_RAD for h in window):
            raise ConsoleError('DAMPING_OBSERVATION', '需连续至少 2 秒低速完整反馈；0.1 rad/s、0.35 rad 为项目测试筛选线，不是防摔边界')
        camera = self.camera_status()
        if camera.get('status') != 'live':
            raise ConsoleError('DAMPING_CAMERA', '阻尼测试需要持续更新的官方相机；画面不证明足底或外部支撑')

    async def preflight(self, data):
        # Pure request construction: never call _rpc, adapter.execute or a mode setter.
        async with self.lock:
            self.require_writable()
            self.require_owner(data.owner)
            self.require_connection()
            if not self.settings.manual_remote:
                raise ConsoleError("MANUAL_REMOTE_DISABLED", "尚未启用人工遥控入口", 403)
            params = validate_params(data.capability, data.params)
            request = describe_request(data.capability, params)
            fsm_current = time.monotonic() - self.telemetry.get("fsm_received", 0) < 1.5
            result = dict(capability=data.capability, label=BY_ID[data.capability]["label"],
                          at=time.time(), run_id=self.run_id, dispatched=False,
                          params_valid=True, request=request,
                          telemetry_tick=self.telemetry.get("tick"),
                          current_fsm=self.telemetry.get("fsm") if fsm_current else None,
                          execution_gate=self.execution_gate(data.capability, params),
                          firmware_acceptance="unknown", physical_result="not_tested",
                          conditions=["SDK 请求映射已核对，不代表本机固件接受该写接口",
                                      "历史人工报告坐地靠柜不代表当前姿势；足底与外部支撑承重仍未知",
                                      "起身、姿态保持和平衡切换的本机姿势前提与运动固件版本仍待确认"])
            self.preflight_history.appendleft(result)
            self.event("remote_preflight", f"{result['label']}：控制权、参数与接口映射核对通过；未下发实机指令", capability=data.capability, dispatched=False)
            return result

    async def acquire(self, owner, floor_verified):
        self.require_writable()
        async with self.lock:
            self.require_connection()
            if self.owner and self.owner != owner and time.monotonic() < self.lease_until:
                raise ConsoleError("CONTROL_BUSY", "另一个窗口持有控制权")
            self.owner, self.lease_until = owner, time.monotonic() + self.settings.lease_ttl
            self.manual_permit = None
            self.floor_verified = floor_verified
            self.event("control_acquired", "已取得控制权；保持页面心跳有效", mode=self.adapter.mode)

    async def heartbeat(self, owner):
        self.require_writable()
        async with self.lock:
            self.require_owner(owner)
            self.require_connection()
            self.lease_until = time.monotonic() + self.settings.lease_ttl

    async def release(self, owner):
        self.require_writable()
        async with self.lock:
            self.require_owner(owner)
            await self._stop("控制权已释放", "cancelled")
            self.manual_permit = None
            self.owner, self.lease_until = None, 0.

    def _manual_ready(self):
        self.require_dispatch()
        if not self.settings.manual_remote:
            raise ConsoleError("MANUAL_REMOTE_DISABLED", "此服务尚未启用人工遥控入口", 403)
        self._walking_ready()
        self._body_ready()

    def _body_ready(self):
        """Complete, fresh and self-consistent body feedback; independent of the current FSM."""
        self.require_connection()
        if not self.fresh():
            raise ConsoleError("STALE_TELEMETRY", "身体反馈过期")
        if self.adapter.mode != "robot":
            return
        imu = self.telemetry.get("imu") or {}
        if len(imu.get("rpy", [])) != 3 or not all(finite(v) for v in imu["rpy"]):
            raise ConsoleError("INCOMPLETE_BODY", "需要完整有效的 IMU 实测姿态")
        if self.telemetry.get("mode_machine") != KNOWLEDGE["documented_hardware"]["mode_machine"] or self.telemetry.get("mode_pr") != 0:
            raise ConsoleError("HARDWARE_UNVERIFIED", "机型或关节坐标模式与本机档案不符")
        joints = self.telemetry["joints"]
        if {j["index"] for j in joints} != {j["index"] for j in JOINTS} or len(joints) != 23:
            raise ConsoleError("INCOMPLETE_BODY", "需要完整的 23 关节实测反馈")
        if any(not finite(j.get("q")) or not finite(j.get("dq")) or j.get("status") != 0 or
               not j.get("temperature") or not all(finite(t) for t in j["temperature"]) for j in joints):
            raise ConsoleError("INCOMPLETE_BODY", "关节角度、速度、温度或原始状态码有缺失或异常")
        # HG carries two distinct fields. Compare each getter to its own field;
        # GetFsmMode=0 and GetFsmId=500 are not a disagreement.
        feedback = self.telemetry.get("safety_feedback", {})
        mode_row, sport_row = feedback.get("fsm_mode", {}), feedback.get("sport", {})
        mode, sport = mode_row.get("raw") or {}, sport_row.get("raw") or {}
        if not (mode_row.get("current") and sport_row.get("current") and
                type(mode.get("data")) is int and type(sport.get("fsm_id")) is int and
                type(sport.get("fsm_mode")) is int):
            raise ConsoleError("MODE_FEEDBACK_UNVERIFIED", "需要新鲜完整的 GetFsmMode 与 HG 官方状态交叉回读")
        if sport["fsm_id"] != self.telemetry["fsm"] or sport["fsm_mode"] != mode["data"]:
            raise ConsoleError("MODE_CONFLICT", "官方状态同字段回读不一致：FSM ID 或 FSM Mode")

    async def arm_manual(self, data):
        async with self.lock:
            self.require_owner(data.owner)
            if self.mode_sequence and self.mode_sequence.live:
                raise ConsoleError("SEQUENCE_RUNNING", "一键启停进行中；完成或中止后再操作", 409)
            self._manual_ready()
            if not all(getattr(data, k) is True for k in ("standing_observed", "unsupported_observed", "area_clear_observed", "floor_verified")):
                raise ConsoleError("OPERATOR_OBSERVATION_REQUIRED", "请逐项确认现场站立、无外部支撑、净空和平整地面", 422)
            await self._stop("人工遥控准备", "cancelled")
            self.manual_permit = dict(owner=data.owner, until=time.monotonic() + 60, observed_at=time.time())
            self.event("manual_armed", "人工确认已站立、无外部支撑、场地净空且平整；授权最长 60 秒，不代表测得足底承重或防摔保证", observation_source="operator")

    async def disarm_manual(self, owner):
        self.require_writable()
        async with self.lock:
            self.require_owner(owner)
            self.manual_permit = None
            await self._stop("人工遥控已锁定", "cancelled")
            self.event("manual_disarmed", "人工遥控已锁定；重新启用需要再次核对现场")

    def _require_manual(self, owner, op, params):
        self.require_owner(owner)
        seq = self.mode_sequence
        if seq and seq.live and seq.dispatching == op and seq.owner == owner:
            self._body_ready()  # Every chained step re-checks complete, consistent feedback.
            return
        if op == "damp":
            # Manual protection item (L2+B): always open to the controller; never a power cut.
            self.require_connection()
            return
        if seq and seq.live:
            raise ConsoleError("SEQUENCE_RUNNING", "一键启停进行中；完成或中止后再操作", 409)
        self._manual_ready()
        permit = self.manual_permit
        if not permit or permit["owner"] != owner or time.monotonic() >= permit["until"]:
            raise ConsoleError("MANUAL_NOT_ARMED", "人工遥控尚未启用或已过期；请重新确认现场条件", 403)
        if op == "velocity":
            validate_params(op, params)
            return
        if op.startswith("arm_"):
            action = params["value"] if op == "arm_action" else int(op[4:])
            if action not in ARM_ACTIONS:
                raise ConsoleError("ACTION_MAPPING_UNVERIFIED", "该动作 ID 缺少已核查的 SDK 手势映射", 403)
            rows = [r for r in firmware_action_catalog(self.adapter.action_data) if r["id"] == action]
            if action not in self.adapter.action_ids or len(rows) != 1:
                raise ConsoleError("ACTION_UNVERIFIED", "固件未唯一声明该动作 ID", 403)
            row = rows[0]
            if not row.get("fsm") or self.telemetry["fsm"] not in row["fsm"]:
                raise ConsoleError("ACTION_CONDITION_UNVERIFIED", "固件动作缺少当前 FSM 的适用声明", 403)
            if "mode_machine" in row and self.telemetry.get("mode_machine") not in row["mode_machine"]:
                raise ConsoleError("ACTION_CONDITION_UNVERIFIED", "固件动作不适用于当前机型", 403)
            return
        raise ConsoleError("SAFETY_ENVELOPE_UNVERIFIED", "起身、模式切换、舞蹈等入口尚未核实本机适用条件，继续锁定", 403)

    def damping_feedback(self):
        """Current dual readback is independent of the durable command acknowledgement."""
        result = dict(status='not_attempted' if self.damping_attempt is None else 'unknown',
                      fsm_id=None, sport_fsm_id=None, physical_support='unknown',
                      motion_authority_verified=False, scope='current_readback_only')
        if self.damping_attempt is None or not self.fresh() or self.adapter.mode != 'robot':
            return result
        attempted_at = self.damping_attempt.get('at')
        if not finite(attempted_at):
            return result
        now, wall = time.monotonic(), time.time()
        for key, field in (('fsm_id', 'fsm_id'), ('sport', 'sport_fsm_id')):
            row = self.telemetry.get('safety_feedback', {}).get(key) or {}
            raw = row.get('raw') or {}
            value = raw.get('data' if key == 'fsm_id' else 'fsm_id')
            received = row.get('received_at')
            mono = row.get('received_monotonic')
            age = (now-mono)*1000 if finite(mono) else (wall-received)*1000 if finite(received) else None
            if (row.get('current') is True and row.get('code') == 0 and type(value) is int
                    and finite(age) and 0 <= age < 1500 and finite(received)
                    and received >= attempted_at):
                result[field] = value
        values = (result['fsm_id'], result['sport_fsm_id'])
        if None not in values:
            result['status'] = ('conflict' if values[0] != values[1] else
                                'target_observed' if values[0] == 1 else 'other_state_observed')
        return result

    def manual_status(self):
        blocker = None
        try:
            self._damping_ready() if self.settings.damping_test else self._manual_ready()
        except ConsoleError as exc:
            blocker = dict(code=exc.code, message=exc.message)
        permit = self.manual_permit
        active = bool(permit and not blocker and permit["owner"] == self.owner and time.monotonic() < min(permit["until"], self.lease_until))
        return dict(enabled=self.settings.manual_remote and not self.settings.read_only, armed=active,
                    verify_only=self.settings.verify_only, accessory_test=self.settings.accessory_test,
                    damping_test=self.settings.damping_test, damping_attempt=self.damping_attempt,
                    damping_feedback=self.damping_feedback(),
                    allowed_write_operations=['damp'] if self.settings.damping_test else sorted(ACCESSORY_TEST_OPERATIONS) if self.settings.accessory_test else [],
                    preflights=list(self.preflight_history),
                    control_scope="project_console_lease", robot_authority_verified=False,
                    owner=permit["owner"] if active else None, expires_in=max(0, permit["until"] - time.monotonic()) if active else 0,
                    observed_at=permit["observed_at"] if permit else None, blocker=blocker,
                    authority="人工现场确认与官方高层控制；不修改基础模型或授权自主任务", full_remote_equivalence=False,
                    sequence=self.mode_sequence.view() if self.mode_sequence else None,
                    routes={r: {k: sequence_blocked(r, k) for k in ('start', 'stop')} for r in SEQUENCE_ROUTES},
                    thermal=self._thermal())

    def _thermal(self):
        joints = self.telemetry.get("joints") or []
        heat = hottest(joints)
        joint = next((j["index"] for j in joints if heat is not None and heat in (j.get("temperature") or [])), None)
        return dict(limit_c=PROJECT_MOTOR_TEMP_C, max_c=heat, joint=joint, hot=heat is not None and heat >= PROJECT_MOTOR_TEMP_C)

    async def start_sequence(self, data):
        self.require_writable()
        async with self.lock:
            self.require_dispatch()
            if not self.settings.manual_remote:
                raise ConsoleError("MANUAL_REMOTE_DISABLED", "此服务尚未启用人工遥控入口", 403)
            self.require_owner(data.owner)
            if self.mode_sequence and self.mode_sequence.live:
                raise ConsoleError("SEQUENCE_RUNNING", "一键启停进行中；完成或中止后再操作", 409)
            self.require_connection()
            if not self.fresh():
                raise ConsoleError("STALE_TELEMETRY", "身体反馈过期")
            # Full body evidence is required at every dispatch, where the sequence waits for it.
            steps = sequence_plan(data.route, data.kind, self.telemetry)
            self.manual_permit = None
            await self._stop("一键启停接管", "cancelled")
            self.mode_sequence = Sequence(self, data.owner, data.route, data.kind, steps)
            self.event("sequence_started", f"一键{'启动' if data.kind == 'start' else '停止'}：{' → '.join(s['label'] for s in steps)}", observation_source="operator")
            self.mode_sequence.task = asyncio.create_task(self.mode_sequence.run())

    async def continue_sequence(self, data):
        self.require_writable()
        async with self.lock:
            self.require_owner(data.owner)
            seq = self.mode_sequence
            if not seq or seq.status != "waiting_operator" or seq.owner != data.owner:
                raise ConsoleError("SEQUENCE_NOT_WAITING", "当前没有等待人工确认的启停步骤", 409)
            self.event("sequence_operator", "操作者确认已人工扶直", observation_source="operator")
            seq.continued.set()

    async def abort_sequence(self, owner):
        self.require_writable()
        async with self.lock:
            self.require_owner(owner)
            if self.mode_sequence and self.mode_sequence.live:
                self.mode_sequence.finish("aborted", "操作者中止")

    def execution_gate(self, op, params=None):
        try:
            if op not in READ_OPERATIONS:
                self.require_dispatch(op)
            self.require_connection()
            if self.settings.damping_test and op == 'damp':
                self.require_owner(self.owner)
                self._damping_ready()
                return dict(allowed=True, code=None, message='仅允许一次 FSM 0→1；不设关节目标，不证明站稳')
            if self.settings.accessory_test and op in ACCESSORY_TEST_OPERATIONS:
                self.require_owner(self.owner)
            if self.adapter.mode == "robot" and BY_ID[op]["risk"] in ("motion", "guarded"):
                if params is None and op in ("velocity", "arm_action"):
                    params = dict(vx=0., vy=0., omega=0.) if op == "velocity" else dict(value=-1)
                self._require_manual(self.owner, op, params or {})
            return dict(allowed=True, code=None, message="允许提交；接口应答不代表动作完成")
        except ConsoleError as exc:
            return dict(allowed=False, code=exc.code, message=exc.message)

    async def _rpc(self, op, params, *, manual_owner=None):
        if op not in READ_OPERATIONS:
            self.require_dispatch(op)
        if self.settings.accessory_test and op in ACCESSORY_TEST_OPERATIONS:
            self.require_owner(manual_owner)
            self.require_connection()
            params = validate_params(op, params)
        if not self.adapter:
            raise ConsoleError("DISCONNECTED", "机器人未连接")
        damping = self.settings.damping_test and op == 'damp'
        if damping:
            self.require_owner(manual_owner)
            self._damping_ready()
            self.damping_attempt = dict(at=time.time(), run_id=self.run_id, status='outcome_unknown', target_fsm=1, source_fsm=0)
            try:
                with self.damping_file.open('x') as receipt:
                    receipt.write(json.dumps(self.damping_attempt))
                    receipt.flush()
                    os.fsync(receipt.fileno())
            except OSError as exc:
                raise ConsoleError('DAMPING_TEST_SPENT', '无法独占持久化单次测试凭据，不下发；先核对已有记录', 403) from exc
            self.event('damping_test_attempt', '单次官方阻尼测试已占用；不自动重发、不串联起身或回退模式', source_fsm=0, target_fsm=1)
        if not damping and self.adapter.mode == 'robot' and op not in READ_OPERATIONS and op != 'stop' and BY_ID[op]['risk'] != 'accessory':
            # Autonomous calls cannot borrow the separate, owner-bound manual permit.
            if manual_owner is None or not self.settings.manual_remote:
                raise ConsoleError('SAFETY_ENVELOPE_UNVERIFIED', '尚无经核定的自主运动防摔边界；人工操作需要独立现场授权', 403)
            self._require_manual(manual_owner, op, params)
            if op == "velocity":
                self._manual_clearance(params)
        try:
            code, data = await asyncio.to_thread(self.adapter.execute, op, params)
        except Exception as exc:
            self.event("sdk_error", f"{op} 调用异常：{type(exc).__name__}", "error")
            message = ("只读查询异常；未取得有效值，保持未知，未自动重试" if op in READ_OPERATIONS else
                       "官方接口调用异常；是否执行未知，未自动重发。请核对实时状态与日志")
            raise ConsoleError("SDK_ERROR", message, 503) from exc
        if damping:
            self.damping_attempt.update(sdk_code=code, status='accepted' if code == 0 else 'outcome_unknown' if code == 3104 else 'rejected')
            self._persist_damping_receipt()
        if code != 0:
            if code == 3104:
                message = ('官方只读查询 3104 接收超时；未取得有效值，保持未知，未自动重试' if op in READ_OPERATIONS else
                           '官方接口 3104 超时；是否执行未知，未自动重发。请先核对实时状态')
                self.event("sdk_timeout", f"{op}：{message}", "warning", sdk_code=code)
                raise ConsoleError('SDK_TIMEOUT', message, 504, sdk_code=code)
            self.event("sdk_rejected", f"{op} 被官方服务拒绝，返回码 {code}", "warning", sdk_code=code)
            if op in READ_OPERATIONS:
                if code == 7301:
                    message = '官方服务返回 7301：LocoState 不可用；模式/高度/相位保持未知，忽略返回中的占位零值。不会自动切换模式'
                elif code == 'MODE_WORKER_TIMEOUT':
                    message = '只读进程通信超时；未收到查询结果，不能等同于机器人返回 3104'
                elif code == 3203:
                    message = '官方服务返回 3203：不支持此 API；未取得有效值'
                else:
                    message = f'只读查询返回 {code}；未取得有效值，保持未知'
            else:
                message = f"官方服务返回 {code}；不代表机器人已完成操作"
            raise ConsoleError("SDK_REJECTED", message, 502, sdk_code=code)
        return data

    async def _stop(self, reason, nav_status="cancelled"):
        was_active = self.motion_active or self.navigation["status"] == "running"
        self.motion_until, self.motion_active = 0., False
        if self.pending:
            self._finish_feedback("feedback_interrupted", "状态机核验已被后续操作中断；机器人动作是否结束未知")
        if self.navigation["status"] == "running":
            self.navigation.update(status=nav_status, phase=reason, finished_at=time.time())
            self._remember()
        if self.adapter and was_active:
            try:
                await self._rpc("stop", {})
            except ConsoleError as exc:
                self.event("stop_unconfirmed", "停止命令未得到确认；运动指令自身的 250 ms 有效期仍适用", "error", reason=exc.code)
                return False
        if was_active:
            self.event("stopped", reason)
        return True

    def _remember(self):
        if self.navigation.get("goal"):
            self.memories.append(dict(at=time.time(), mode=self.adapter.mode if self.adapter else None,
                frame=self.world.observation.frame if self.world.observation else None,
                goal=self.navigation["goal"], status=self.navigation["status"],
                pose=self.world.observation.pose.model_dump() if self.world.observation else None))
            self.memories = self.memories[-40:]
            temp = self.memory_file.with_suffix(".tmp")
            temp.write_text(json.dumps(dict(version=1, visits=self.memories), ensure_ascii=False, indent=2))
            temp.replace(self.memory_file)

    def _dedupe(self, request_id, fingerprint):
        if request_id in self.results:
            old = self.results[request_id]
            if old["fingerprint"] != fingerprint:
                raise ConsoleError("IDEMPOTENCY_CONFLICT", "同一请求 ID 不能用于不同操作")
            return old["result"]
        return None

    def _save_result(self, request_id, fingerprint, result):
        self.results[request_id] = dict(fingerprint=fingerprint, result=result)
        if len(self.results) > 200:
            self.results.popitem(last=False)
        return result

    def _admit(self, command, ttl):
        # A stop is a sequence barrier: delayed packets must never restart motion.
        if command.sequence <= self.sequences.get(command.owner, -1):
            raise ConsoleError("OLD_COMMAND", "拒绝倒序操作；停止后到达的旧请求不会重新启动运动")
        self.sequences[command.owner] = command.sequence
        if len(self.sequences) > 200:
            self.sequences.popitem(last=False)
        if not -.1 <= time.time() - command.issued_at <= ttl:
            raise ConsoleError("EXPIRED_COMMAND", "操作已超时；请校准浏览器与 Orin 时钟后重新操作")

    def _finish_feedback(self, status, message):
        if self.pending:
            row = self.command_history.get(self.pending['request_id'])
            if row:
                row.update(status=status, message=message, updated_at=time.time())
            self.pending = None

    async def command(self, command: Command):
        # Failed write outcomes are also idempotent: a transport error must not
        # silently resend a physical operation with the same request ID.
        fingerprint = json.dumps(command.model_dump(exclude={'issued_at', 'sequence'}), sort_keys=True)
        failed = self.failed_commands.get(command.request_id)
        if failed:
            if failed[0] != fingerprint:
                raise ConsoleError('IDEMPOTENCY_CONFLICT', '同一请求 ID 不能用于不同操作')
            raise failed[1]
        try:
            result = await self._command(command)
        except ConsoleError as exc:
            if command.request_id not in self.command_history:
                self.failed_commands[command.request_id] = (fingerprint, exc)
                if len(self.failed_commands) > 200:
                    self.failed_commands.popitem(last=False)
                status = ('read_unavailable' if command.capability in READ_OPERATIONS and exc.code.startswith('SDK_') else
                          'outcome_unknown' if exc.code in ('SDK_ERROR', 'SDK_TIMEOUT') else 'rejected')
                self._record_command(command, status, exc.message, error_code=exc.code, sdk_code=exc.sdk_code)
            raise
        if command.request_id not in self.command_history and command.capability != 'velocity':
            waiting = bool(self.pending and self.pending['request_id'] == command.request_id)
            self._record_command(command, 'read_result' if command.capability in READ_OPERATIONS else 'awaiting_feedback' if waiting else 'sdk_ack', result['message'], data=result.get('data'))
        return result

    def _record_command(self, command, status, message, **extra):
        self.command_history[command.request_id] = dict(request_id=command.request_id,
            capability=command.capability, label=BY_ID.get(command.capability, {}).get('label', command.capability),
            at=time.time(), updated_at=time.time(), mode=self.adapter.mode if self.adapter else 'disconnected',
            status=status, message=message, **extra)
        if len(self.command_history) > 60:
            self.command_history.popitem(last=False)

    async def _command(self, command: Command):
        op = command.capability
        if op not in READ_OPERATIONS:
            self.require_dispatch(op)
        params = validate_params(op, command.params)
        cap = BY_ID[op]
        fingerprint = json.dumps([command.owner, op, params], sort_keys=True)
        async with self.lock:
            failed = self.failed_commands.get(command.request_id)
            if failed:
                raw_fingerprint = json.dumps(command.model_dump(exclude={'issued_at', 'sequence'}), sort_keys=True)
                if failed[0] != raw_fingerprint:
                    raise ConsoleError('IDEMPOTENCY_CONFLICT', '同一请求 ID 不能用于不同操作')
                raise failed[1]
            previous = self._dedupe(command.request_id, fingerprint)
            if previous:
                return previous
            if op in ("stop", "damp") and self.mode_sequence and self.mode_sequence.live:
                self.mode_sequence.finish("aborted", "操作者停止" if op == "stop" else "操作者执行阻尼保护")
            if op == "stop":
                if not self.adapter:
                    raise ConsoleError("DISCONNECTED", "机器人未连接")
                self.sequences[command.owner] = max(command.sequence, self.sequences.get(command.owner, -1))
                await self._stop("操作者停止", "cancelled")
                await self._rpc("stop", {})  # Always send, including after stale telemetry / lease loss.
                return self._save_result(command.request_id, fingerprint, dict(status="accepted", message="已确认停止行走命令；仍需观察身体反馈"))
            self.require_connection()
            if cap["risk"] != "read":
                self.require_owner(command.owner)
                self._admit(command, .25 if op == "velocity" else 3.)
            if cap["risk"] == "guarded" and not command.acknowledge_risk:
                raise ConsoleError("RISK_ACK_REQUIRED", "该操作可能改变支撑或控制模式，请在界面确认现场条件")
            if op.startswith("arm_"):
                action = params["value"] if op == "arm_action" else int(op[4:])
                if action not in self.adapter.action_ids:
                    raise ConsoleError("ACTION_UNVERIFIED", "固件尚未声明支持该动作；先查询动作清单")
            if cap["risk"] not in ("read", "accessory") and (self.navigation["status"] == "running" or op != "velocity"):
                await self._stop("人工操作接管导航", "cancelled")
            if op == "velocity":
                try:
                    self._walking_ready()
                    if self.adapter.mode == "robot" and self.settings.manual_remote:
                        self._require_manual(command.owner, op, params)
                        self._manual_clearance(params)
                    else:
                        self._check_velocity(params)
                except ConsoleError:
                    await self._stop("速度操作被反馈检查阻止", "failed")
                    raise
            issued = time.monotonic()
            data = await self._rpc(op, params, manual_owner=command.owner)
            if op == "velocity":
                self.last_velocity = params
                self.motion_active = any(params.values())
                self.motion_until = time.monotonic() + self.settings.velocity_ttl
            if op in FSM_COMMANDS or op == "fsm":
                self.pending = dict(target=FSM_COMMANDS.get(op, params.get("value")), until=time.monotonic() + 6,
                                    issued=issued, request_id=command.request_id)
            is_read = cap["risk"] == "read"
            event = self.event("query_result" if is_read else "command_accepted",
                               f"{cap['label']}：已读取官方返回值" if is_read else f"{cap['label']}：官方接口已应答，等待反馈验证", capability=op,
                               request_id=command.request_id, mode=self.adapter.mode)
            return self._save_result(command.request_id, fingerprint, dict(status="accepted", data=data, event_id=event["id"],
                message="演示接口应答" if self.adapter.mode == "demo" else "已读取官方返回值" if is_read else "官方接口应答成功；动作完成状态尚未确认"))

    def _walking_ready(self):
        self.require_connection()
        if self.telemetry.get("fsm") != 500 or time.monotonic() - self.telemetry.get("fsm_received", 0) > 1.5:
            raise ConsoleError("NOT_WALKING_MODE", "请先进入行走控制，并等待 GetFsmId 返回 500")
        imu = self.telemetry.get("imu")
        if not imu or max(abs(v) for v in imu["rpy"][:2]) > PROJECT_TILT_RAD:
            raise ConsoleError("BODY_TILT", "姿态偏离行走范围，停止发出速度指令")
        for j in self.telemetry["joints"]:
            if j["temperature"] and max(j["temperature"]) >= PROJECT_MOTOR_TEMP_C:
                raise ConsoleError("MOTOR_HOT", "电机温度达到本项目保守阈值 70°C；停止继续移动")
            if j["status"] not in (0, None):
                raise ConsoleError("MOTOR_STATUS", "检测到未解析的电机状态码，等待人工检查")

    def _check_velocity(self, params):
        if self.world.observation and self.world.fresh(self.settings.sensor_ttl):
            if self.world.observation.coverage_radians < 6.0:
                raise ConsoleError("SCAN_COVERAGE", "需要接近 360° 的新鲜扫描；局部视野不能确认身体周围净空")
            if not local_clear(self.world.observation.scan, params["vx"], params["vy"], params["omega"], self.settings.radius):
                raise ConsoleError("OBSTACLE_NEAR", "运动方向净空不足，已阻止速度指令")
        elif self.adapter.mode == "robot":
            raise ConsoleError("NO_OBSTACLE_FEEDBACK", "需要实时障碍物反馈后才能使用网页速度控制")

    def _manual_clearance(self, params):
        # The operator supplies visual coverage for manual teleoperation only.
        # Still honor any available fresh scan; never reinterpret it as 360° coverage.
        if self.world.observation and self.world.fresh(self.settings.sensor_ttl):
            if not local_clear(self.world.observation.scan, **params, radius=self.settings.radius):
                raise ConsoleError("OBSTACLE_NEAR", "新鲜扫描发现运动方向净空不足")

    async def goal(self, goal: Goal):
        self.require_writable()
        if self.settings.damping_test:
            self.require_dispatch('velocity')
        if self.adapter and self.adapter.mode == "robot":
            raise ConsoleError("SAFETY_ENVELOPE_UNVERIFIED", "人工遥控授权不能用于自主导航；自主任务防摔边界尚未核定", 403)
        fingerprint = json.dumps([goal.owner, goal.x, goal.y])
        async with self.lock:
            previous = self._dedupe(goal.request_id, fingerprint)
            if previous:
                return previous
            self.require_owner(goal.owner)
            self._admit(goal, 1.)
            self._walking_ready()
            if self.adapter.mode == "robot" and not self.floor_verified:
                raise ConsoleError("FLOOR_NOT_VERIFIED", "自主导航限已检查的平整地面；在取得控制权时确认场地")
            if not self.world.fresh(self.settings.sensor_ttl) or not self.world.grid:
                raise ConsoleError("NO_LOCALIZATION", "需要实时定位、扫描和占用地图；相机画面不能替代定位反馈")
            if self.world.observation.coverage_radians < 6.0:
                raise ConsoleError("SCAN_COVERAGE", "自主导航需要接近 360° 的扫描覆盖")
            pose = self.world.observation.pose
            path = plan(self.world.grid, (pose.x, pose.y), (goal.x, goal.y), self.settings.radius)
            await self._stop("新目标替换上一个任务", "cancelled")
            self.navigation = dict(status="running", phase="观察 → 路径规划", path=path,
                goal=[goal.x, goal.y], frame=self.world.observation.frame, corrections=0, waypoint=0,
                revision=self.world.grid.revision, started_at=time.time(), last_progress=time.monotonic(),
                best_distance=math.inf, request_id=goal.request_id)
            self.event("navigation_started", "已从当前观察规划路线，开始执行并校验位置反馈", goal=[goal.x, goal.y])
            return self._save_result(goal.request_id, fingerprint, dict(status="running", path=path))

    async def observation(self, observation: Observation):
        async with self.lock:
            if not self.adapter or self.adapter.mode != "robot":
                raise ConsoleError("ROBOT_SOURCE_REQUIRED", "真实传感器只可接入真实连接模式")
            self.world.update(observation)

    async def camera(self, frame: CameraFrame):
        if self.settings.camera_backend == "unitree":
            raise ConsoleError("CAMERA_SOURCE_RESERVED", "当前相机由官方只读接口提供，不能混入外部帧", 409)
        try:
            raw = base64.b64decode(frame.jpeg_base64, validate=True)
            if not raw.startswith(b"\xff\xd8") or not raw.endswith(b"\xff\xd9") or len(raw) > 500000:
                raise ValueError()
        except ValueError as exc:
            raise ConsoleError("INVALID_JPEG", "相机输入必须为 ≤500 KB 的 JPEG", 422) from exc
        async with self.lock:
            if not self.adapter or self.adapter.mode != "robot":
                raise ConsoleError("ROBOT_SOURCE_REQUIRED", "相机帧只可接入真实连接模式")
            if not -0.1 <= time.time() - frame.captured_at <= .8 or frame.sequence <= self.world.camera_sequence:
                raise ConsoleError("STALE_CAMERA", "拒绝过期、未来或重复的相机帧")
            if self.world.camera_source is not None and self.world.camera_source != frame.source:
                raise ConsoleError("SOURCE_CONFLICT", "已有活动相机源；断开机器人后才能切换")
            self.world.camera, self.world.camera_at, self.world.camera_sequence = raw, time.monotonic(), frame.sequence
            self.world.camera_captured_at, self.world.camera_source = frame.captured_at, frame.source
            self.world.camera_metadata = dict(timestamp_basis="external_captured_at", sensor_timestamp=frame.captured_at)

    def camera_status(self):
        world = self.world
        age = round(world.camera_age() * 1000) if world.camera else None
        status, error = "waiting", None
        if self.camera_reader:
            _, status, error = self.camera_reader.latest()
        if world.camera:
            status = "live" if age < 1000 else "stale"
        return dict(backend=self.settings.camera_backend, status=status, error=error,
                    diagnostics=self.camera_reader.diagnostics() if self.camera_reader and hasattr(self.camera_reader, 'diagnostics') else None,
                    source=world.camera_source, sequence=world.camera_sequence, age_ms=age,
                    **{k: v for k, v in world.camera_metadata.items() if k not in ("source", "sequence")})

    async def _sample(self):
        if self.adapter:
            self.telemetry = self.adapter.sample()
            stamp = self.telemetry.get('received', 0)
            now = time.monotonic()
            if self.manual_permit:
                try:
                    self._require_manual(self.manual_permit["owner"], "velocity", dict(vx=0., vy=0., omega=0.))
                except ConsoleError as exc:
                    self.manual_permit = None
                    await self._stop("人工授权失效：" + exc.message, "failed")
                    self.event("manual_revoked", exc.message, "warning", reason=exc.code)
            if not self.fresh():
                self.body_history.clear()
            elif stamp and (not self.body_history or stamp > self.body_history[-1]['at']):
                if self.body_history and stamp - self.body_history[-1]['at'] > self.settings.telemetry_ttl:
                    self.body_history.clear()
                if self.body_history and any(self.body_history[-1].get(k) != self.telemetry.get(k) for k in ('mode_pr', 'mode_machine')):
                    self.body_history.clear()
                speeds = [abs(j['dq']) for j in self.telemetry['joints'] if finite(j.get('dq'))]
                if speeds:
                    self.body_history.append(history_sample(self.telemetry, stamp))
                while self.body_history and now - self.body_history[0]['at'] > 12:
                    self.body_history.popleft()
            if self.adapter.mode == 'robot' and self.fresh() and now - self.last_state_observation >= 30:
                fsm_at = self.telemetry.get('fsm_received', 0)
                learned = interpret(self.telemetry, True, bool(fsm_at and 0 <= now-fsm_at < 1.5), KNOWLEDGE, self.body_history, 'robot')
                self.event('state_observed', '只读身体状态已记录；不产生安全范围或动作权限',
                    current=False, foundation_id=KNOWLEDGE['foundation_id'], tick=self.telemetry.get('tick'),
                    telemetry_age_ms=round((now-stamp)*1000), control_mode=learned['control_mode'],
                    posture=learned['posture'], observation_window=learned['observation_window'],
                    safety_status='unverified', execution_allowed=False)
                self.last_state_observation = now
            if self.camera_reader:
                frame, _, _ = self.camera_reader.latest()
                if frame:
                    raw, started, wall, metadata = frame
                    if metadata["sequence"] > self.world.camera_sequence and time.monotonic() - started < 1:
                        self.world.camera, self.world.camera_at = raw, started
                        self.world.camera_captured_at, self.world.camera_source = wall, metadata["source"]
                        self.world.camera_sequence, self.world.camera_metadata = metadata["sequence"], metadata
            if self.adapter.mode == "demo":
                from .simulation import observe
                self.world.update(observe(self.adapter.pose, self.world.sequence + 1, include_grid=self.world.grid is None))

    async def tick(self):
        async with self.lock:
            await self._sample()
            now = time.monotonic()
            if self.owner and (now >= self.lease_until or not self.fresh()):
                await self._stop("控制心跳或身体反馈过期", "failed")
                self.owner, self.lease_until = None, 0.
                self.event("control_expired", "控制权已收回，请检查连接后重新取得控制权", "warning")
            if self.motion_active and now >= self.motion_until and self.navigation["status"] != "running":
                await self._stop("速度指令 250 ms 有效期结束")
            if self.motion_active and self.navigation["status"] != "running":
                try:
                    self._walking_ready()
                    if self.adapter.mode == "robot" and self.settings.manual_remote:
                        self._require_manual(self.owner, "velocity", self.last_velocity)
                        self._manual_clearance(self.last_velocity)
                    else:
                        self._check_velocity(self.last_velocity)
                except ConsoleError as exc:
                    await self._stop(exc.message, "failed")
                    self.event("motion_blocked", exc.message, "warning", reason=exc.code)
            if self.pending:
                if self.telemetry.get("fsm") == self.pending["target"] and self.telemetry.get("fsm_received", 0) > self.pending["issued"]:
                    if self.settings.damping_test and self.damping_attempt and self.pending['target'] == 1:
                        self.damping_attempt.update(status='fsm_verified', verified_at=time.time(), physical_support='unknown')
                        self._persist_damping_receipt()
                    self.event("feedback_verified", f"状态机反馈已验证：{self.pending['target']}", request_id=self.pending["request_id"])
                    self._finish_feedback('state_observed', '已观测到目标 FSM；这不证明站稳、承重或动作完成')
                elif now > self.pending["until"]:
                    message = "已收到接口应答，但未在 6 秒内观测到目标状态机；未确认完成"
                    self.event("feedback_timeout", message, "warning", request_id=self.pending['request_id'])
                    self._finish_feedback('feedback_timeout', message)
            if self.navigation["status"] == "running" and now >= self.next_nav:
                self.next_nav = now + .1
                try:
                    await self._navigate()
                except ConsoleError as exc:
                    await self._stop(exc.message, "failed")
                    self.event("navigation_failed", exc.message, "warning", reason=exc.code)

    async def _navigate(self):
        self.require_owner(self.owner)
        self._walking_ready()
        nav, world = self.navigation, self.world
        if not world.fresh(self.settings.sensor_ttl) or not world.grid:
            raise ConsoleError("STALE_WORLD", "定位或障碍物反馈过期，导航已停止")
        if world.observation.coverage_radians < 6.0:
            raise ConsoleError("SCAN_COVERAGE", "扫描覆盖不足，导航已停止")
        if nav["frame"] != world.observation.frame:
            raise ConsoleError("FRAME_CHANGED", "导航期间坐标系发生变化")
        if time.time() - nav["started_at"] > 90:
            raise ConsoleError("GOAL_TIMEOUT", "90 秒内未到达目标，停止并报告失败")
        pose = world.observation.pose
        distance = math.dist((pose.x, pose.y), nav["goal"])
        nav["distance"] = distance
        if distance < .13:
            await self._rpc("stop", {})
            self.motion_active = False
            nav.update(status="succeeded", phase="位置反馈已验证 · 到达目标", finished_at=time.time())
            self._remember()
            self.event("goal_verified", "已用定位反馈确认到达目标", distance=distance)
            return
        if distance < nav["best_distance"] - .04:
            nav["best_distance"], nav["last_progress"] = distance, time.monotonic()
        if world.grid.revision != nav["revision"] or time.monotonic() - nav["last_progress"] > 5:
            await self._rpc("stop", {})
            if nav["corrections"] >= 3:
                raise ConsoleError("STALLED", "已重规划 3 次仍未获得有效进展")
            nav["path"] = plan(world.grid, (pose.x, pose.y), nav["goal"], self.settings.radius)
            nav.update(waypoint=0, revision=world.grid.revision, corrections=nav["corrections"] + 1,
                       last_progress=time.monotonic(), phase="反馈偏差 → 纠错重规划")
            self.event("replanned", "根据地图变化或进展不足重新规划", corrections=nav["corrections"])
        while nav["waypoint"] < len(nav["path"]) - 1 and math.dist((pose.x, pose.y), nav["path"][nav["waypoint"]]) < .17:
            nav["waypoint"] += 1
        target = nav["path"][nav["waypoint"]]
        heading = wrap(math.atan2(target[1] - pose.y, target[0] - pose.x) - pose.yaw)
        velocity = dict(vx=min(.22, distance * .6) if abs(heading) < .35 else 0., vy=0., omega=max(-.35, min(.35, heading * 1.2)))
        if not local_clear(world.observation.scan, **velocity, radius=self.settings.radius):
            raise ConsoleError("OBSTACLE_NEAR", "局部扫描发现运动净空不足，停止导航并等待新观察")
        await self._rpc("velocity", velocity)
        self.motion_active = True
        self.motion_until = time.monotonic() + .25
        nav["phase"] = "执行 → 检查位移反馈"

    def snapshot(self):
        fresh = self.fresh()
        mode = self.adapter.mode if self.adapter else "disconnected"
        age = round((time.monotonic() - self.telemetry["received"]) * 1000) if self.telemetry["received"] else None
        available = self.adapter.action_ids if self.adapter else set()
        caps = [dict(c, availability="demo" if mode == "demo" else "firmware_declared" if c.get("action_id") in available
                     else "unverified" if c.get("action_id") is not None else "sdk_defined") for c in CATALOG]
        telemetry = {k: v for k, v in self.telemetry.items() if k not in ("received", "fsm_received")}
        telemetry['fsm_age_ms'] = round((time.monotonic()-self.telemetry['fsm_received'])*1000) if self.telemetry.get('fsm_received') else None
        firmware = firmware_action_catalog(getattr(self.adapter, "action_data", None))
        assessment = assess(telemetry, fresh, telemetry['fsm_age_ms'], caps, firmware, self.settings.read_only, self.body_history)
        assessment['state_learning'] = interpret(telemetry, fresh, assessment['fsm_fresh'], KNOWLEDGE, self.body_history, mode, self.camera_status())
        compatibility = {c['id']:c for c in assessment['capabilities']}
        caps = [dict(c, compatibility=compatibility[c['id']], execution_gate=self.execution_gate(c['id'])) for c in caps]
        for row in assessment['firmware_actions']:
            row['execution_gate'] = self.execution_gate('arm_action', dict(value=row['id'])) if row['id'] is not None else dict(allowed=False, code='NO_ACTION_ID', message='仅有名称，调用映射尚未核实')
        nav = {k: v for k, v in self.navigation.items() if k != "best_distance"}
        return dict(at=time.time(), run_id=self.run_id, mode=mode, read_only=self.settings.read_only, connected=fresh, telemetry_age_ms=age,
                    telemetry=telemetry, assessment=assessment, capabilities=caps, control=dict(owner=self.owner, expires_in=max(0, self.lease_until - time.monotonic()),
                    floor_verified=self.floor_verified), world=self.world.snapshot(), camera=self.camera_status(), navigation=nav,
                    insight=self.insights.snapshot(fresh, self.camera_status()),
                    manual_remote=self.manual_status(),
                    command_history=list(reversed(self.command_history.values())),
                    events=list(self.events)[:60], memories=self.memories,
                    firmware_actions=sorted(available), firmware_action_error=self.adapter.action_error if self.adapter else None,
                    firmware_action_catalog=assessment['firmware_actions'],
                    hardware=dict(dof=23, hands=False, battery=None, battery_source="HG BmsState 原始消息独立读取；单位与物理来源尚未核定，电量保留未知"))

    async def run(self):
        if self.settings.demo:
            await self.connect("demo", "eth0")
        while not self.closed:
            try:
                await self.tick()
            except Exception as exc:
                self.event("loop_error", f"控制循环异常：{type(exc).__name__}", "error")
                async with self.lock:
                    await self._stop("控制循环异常", "failed")
                    self.owner, self.lease_until = None, 0.
            await asyncio.sleep(.05)

    async def close(self):
        self.closed = True
        if self.mode_sequence and self.mode_sequence.live:
            self.mode_sequence.finish('aborted', '服务关闭')
        await self.disconnect()
