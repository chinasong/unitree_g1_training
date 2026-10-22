"""Hardware boundary. Demo never instantiates DDS; real never synthesizes telemetry."""
import base64
import copy
import json
import math
import platform
import threading
import time
from .body import JOINTS, empty_joints
from .capabilities import ARM_ACTIONS, READ_OPERATIONS, require_write_scope
from .config import ConsoleError


def decode_lowstate(msg):
    joints = []
    for spec in JOINTS:
        m = msg.motor_state[spec["index"]]
        values = [m.q, m.dq, m.ddq, m.tau_est, m.vol, *m.temperature]
        if not all(math.isfinite(v) for v in values):
            raise ValueError("non-finite motor telemetry")
        joints.append(dict(index=spec["index"], q=float(m.q), dq=float(m.dq), ddq=float(m.ddq),
                           tau=float(m.tau_est), temperature=list(m.temperature), voltage=float(m.vol),
                           status=int(m.motorstate), mode=int(m.mode), sensor=list(m.sensor), reserve=list(m.reserve)))
    imu = msg.imu_state
    if not all(math.isfinite(v) for v in [*imu.rpy, *imu.gyroscope, *imu.accelerometer, *imu.quaternion]):
        raise ValueError("non-finite IMU telemetry")
    return dict(joints=joints, imu=dict(rpy=list(imu.rpy), gyroscope=list(imu.gyroscope),
                accelerometer=list(imu.accelerometer), quaternion=list(imu.quaternion), temperature=imu.temperature),
                tick=msg.tick, mode_pr=int(msg.mode_pr), mode_machine=msg.mode_machine, version=list(msg.version))


class DemoAdapter:
    mode = "demo"

    def __init__(self):
        self.last = time.monotonic()
        self.started = self.last
        self.pose = [-1.4, -1.4, 0.0]
        self.velocity = [0.0, 0.0, 0.0]
        self.deadline = 0.0
        self.fsm = 500
        self.fsm_received = self.last
        self.volume = 50
        self.parameters = dict(balance=0, stand_height=0.75, swing_height=0.1, speed_mode=0, task=0)
        self.action_ids = set(ARM_ACTIONS)
        self.action_data = dict(ARM_ACTIONS)
        self.action_error = None
        self.joints = empty_joints()
        self.closed = False

    def sample(self):
        now = time.monotonic()
        dt = min(now - self.last, 0.1)
        self.last = now
        if now > self.deadline:
            self.velocity = [0., 0., 0.]
        vx, vy, w = self.velocity
        yaw = self.pose[2]
        self.pose[0] += (vx * math.cos(yaw) - vy * math.sin(yaw)) * dt
        self.pose[1] += (vx * math.sin(yaw) + vy * math.cos(yaw)) * dt
        self.pose[2] = math.atan2(math.sin(yaw + w * dt), math.cos(yaw + w * dt))
        t = now - self.started
        for spec, j in zip(JOINTS, self.joints):
            name = spec["name"]
            neutral = 0.25 if "knee" in name else -0.12 if "hip_pitch" in name or "ankle_pitch" in name else 0.1 if "elbow" in name else 0
            j.update(q=neutral + math.sin(t * .7 + j["index"]) * .008,
                     dq=math.cos(t * .7 + j["index"]) * .0056, ddq=0.0,
                     tau=1.5 + abs(math.sin(t + j["index"])) * 2,
                     temperature=[34 + round(math.sin(t / 12 + j["index"]) * 2), 33],
                     voltage=49.3, status=0, mode=1, sensor=[0, 0], reserve=[0, 0, 0, 0])
        self.fsm_received = now
        return dict(joints=copy.deepcopy(self.joints), imu=dict(rpy=[.002 * math.sin(t), .003 * math.cos(t), self.pose[2]],
                    gyroscope=[0, 0, w], accelerometer=[0, 0, 9.81], quaternion=[1, 0, 0, 0], temperature=35),
                    tick=int(t * 500), mode_pr=0, mode_machine=0, version=[1, 0], received=now,
                    fsm=self.fsm, fsm_received=now, bad_packets=0,
                    # Synthetic mirror of both official mode readbacks so chained steps can complete in demo.
                    safety_feedback={'fsm_mode': {'current': True, 'raw': {'data': 0}},
                                     'sport': {'current': True, 'raw': {'fsm_id': self.fsm, 'fsm_mode': 0}}})

    def execute(self, op, p):
        if op == "velocity":
            self.velocity = [p["vx"], p["vy"], p["omega"]]
            self.deadline = time.monotonic() + .25
        elif op == "stop":
            self.velocity = [0., 0., 0.]
        elif op in FSM_COMMANDS:
            self.fsm = FSM_COMMANDS[op]
        elif op == "fsm":
            self.fsm = p["value"]
        elif op in ("get_fsm", "get_fsm_mode"):
            return 0, self.fsm
        elif op in ("balance", "stand_height", "swing_height", "speed_mode", "task"):
            self.parameters[op] = p["value"]
        elif op in ("high_stand", "low_stand"):
            self.parameters["stand_height"] = 4294967295 if op == "high_stand" else 0
        elif op in ("get_balance", "get_stand_height", "get_swing_height"):
            return 0, self.parameters[op[4:]]
        elif op == "get_phase":
            return 0, [0., 0.]
        elif op == "get_silent":
            return 0, {"silent": False}
        elif op == "get_services":
            return 0, [{"name": "demo_sport", "status": 0, "protect": 0}]
        elif op == "get_versions":
            return 0, {"packageVersion": "demo", "moduleVersionMap": {"sport": "demo"}}
        elif op in ("get_loco_api_version", "get_motion_api_version", "get_state_api_version"):
            return 0, {"service": {"get_loco_api_version": "sport", "get_motion_api_version": "motion_switcher",
                                   "get_state_api_version": "robot_state"}[op],
                       "api_version": "demo", "firmware_version": None}
        elif op == "get_actions":
            return 0, self.action_data
        elif op == "volume":
            self.volume = p["value"]
        elif op == "get_volume":
            return 0, {"volume": self.volume}
        return 0, None

    def close(self):
        self.execute("stop", {})
        self.closed = True


FSM_COMMANDS = dict(start=500, prepare=4, squat=2, squat_to_stand=706, stand_to_squat=706, lie_to_stand=702, sit=3, damp=1, zero_torque=0)


class UnitreeAdapter:
    mode = "robot"
    _interface = None

    def __init__(self, interface, read_only=False, verify_only=False, accessory_test=False, damping_test=False):
        self.read_only = read_only
        self.verify_only = verify_only
        self.accessory_test = accessory_test
        self.damping_test = damping_test
        self.damping_attempted = False
        if platform.system() != "Linux":
            raise ConsoleError("LINUX_REQUIRED", "真实控制服务请在 G1 的 Orin / Linux 上运行；Mac 可通过浏览器或 SSH 转发连接")
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
        from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
        from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient
        from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient
        from .safety_feedback import PassiveSafetyFeedback
        from .service_versions import PassiveServiceVersions
        from .mode_readback import IsolatedModeReadback
        from unitree_sdk2py.utils.crc import CRC
        if self._interface and self._interface != interface:
            raise ConsoleError("DDS_INTERFACE_LOCKED", "DDS 已初始化到另一网卡，切换网卡需要重启本服务")
        if not self._interface:
            ChannelFactoryInitialize(0, interface)
            UnitreeAdapter._interface = interface
        self.lock = threading.Lock()
        self.rpc_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.closed = False
        self.low = None
        self.received = 0.
        self.tick = None
        self.bad_packets = 0
        self.fsm, self.fsm_received = None, 0.
        self.action_ids, self.action_data, self.action_error = set(), None, "尚未查询"
        self.crc = CRC()
        self.loco, self.arm, self.audio = LocoClient(), G1ArmActionClient(), AudioClient()
        for client in (self.loco, self.arm, self.audio):
            client.SetTimeout(.7)
            client.Init()
        self.safety_feedback = PassiveSafetyFeedback()
        self.service_versions = PassiveServiceVersions()
        self.subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.subscriber.Init(self._lowstate, 1)
        self.mode_reader = IsolatedModeReadback(interface)
        self.poller = threading.Thread(target=self._poll, daemon=True, name="g1-rpc-readback")
        self.poller.start()

    def _lowstate(self, msg):
        if self.closed:
            return
        try:
            if self.crc.Crc(msg) != msg.crc:
                raise ValueError("CRC mismatch")
            decoded = decode_lowstate(msg)
            with self.lock:
                # A frozen tick does not renew freshness. uint32 wrap is allowed.
                if self.tick is not None:
                    delta = (msg.tick - self.tick) & 0xffffffff
                    if delta == 0 or delta >= 0x80000000:
                        return
                self.low, self.tick, self.received = decoded, msg.tick, time.monotonic()
        except (ValueError, IndexError, AttributeError, TypeError):
            self.bad_packets += 1

    def _poll(self):
        # This one-time getter uses the same isolated, correlated receive path.
        try:
            self.execute("get_actions", {})
        except Exception as exc:
            self.action_error = f"SDK 查询异常：{type(exc).__name__}"

    def sample(self):
        modes = self.mode_reader.snapshot()
        fsm = modes['fsm_id']
        with self.lock:
            value = copy.deepcopy(self.low) if self.low else dict(joints=empty_joints(), imu=None, tick=None)
            value.update(received=self.received, fsm=fsm['raw']['data'] if fsm['current'] else None,
                         fsm_received=fsm['received_monotonic'] if fsm['current'] else 0., bad_packets=self.bad_packets)
            value['safety_feedback'] = self.safety_feedback.snapshot()
            value['safety_feedback'].update(modes)
            value['service_versions'] = self.service_versions.snapshot()
            return value

    def execute(self, op, p):
        # Same rule set as the service gate, re-checked at the SDK boundary.
        require_write_scope(op, self)
        if op == 'damp' and self.damping_test:
            with self.rpc_lock:
                if self.damping_attempted:
                    raise ConsoleError("DAMPING_TEST_SPENT", "阻尼测试已尝试，不允许自动重发或第二次写入", 403)
                self.damping_attempted = True
        from unitree_sdk2py.g1.loco.g1_loco_api import InternalFsmMode
        from .capabilities import action_ids
        with self.rpc_lock:
            if op in FSM_COMMANDS:
                result = self.loco.SetFsmId(FSM_COMMANDS[op])
            elif op == "stop":
                result = self.loco.SetVelocity(0., 0., 0., .25)
            elif op == "velocity":
                result = self.loco.SetVelocity(p["vx"], p["vy"], p["omega"], .25)
            elif op.startswith("arm_") or op == "arm_action":
                result = self.arm.ExecuteAction(p["value"] if op == "arm_action" else int(op[4:]))
            elif op == "get_actions":
                code, raw = self.mode_reader.read(op)
                result = code, raw['data'] if code == 0 else None
                if result[0] == 0:
                    self.action_data, self.action_ids, self.action_error = result[1], action_ids(result[1]), None
                else:
                    self.action_error = f"GetActionList 返回 {result[0]}"
            elif op in ("get_fsm", "get_fsm_mode"):
                row = self.mode_reader.snapshot()['fsm_id' if op == 'get_fsm' else 'fsm_mode']
                result = (0, row['raw']['data']) if row['current'] else (row['code'] or 'STALE_FEEDBACK', None)
            elif op in ("get_balance", "get_swing_height", "get_stand_height", "get_phase", "get_services", "get_versions", "get_volume", "get_silent",
                        "get_loco_api_version", "get_motion_api_version", "get_state_api_version"):
                # The legacy SDK listener sometimes lost a correlated 7301 reply
                # and reported 3104 instead. Reuse our polled, isolated reader.
                code, raw = self.mode_reader.read(op)
                result = code, raw['data'] if code == 0 else None
            elif op in ("wave", "wave_turn", "shake_start", "shake_end"):
                result = self.loco.SetTaskId({"wave": 0, "wave_turn": 1, "shake_start": 2, "shake_end": 3}[op])
            elif op == "fsm":
                result = self.loco.SetFsmId(p["value"])
            elif op == "task":
                result = self.loco.SetTaskId(p["value"])
            elif op in ("high_stand", "low_stand", "stand_height"):
                result = self.loco.SetStandHeight(4294967295 if op == "high_stand" else 0 if op == "low_stand" else p["value"])
            elif op == "balance":
                result = self.loco.SetBalanceMode(p["value"])
            elif op == "swing_height":
                from unitree_sdk2py.g1.loco.g1_loco_api import ROBOT_API_ID_LOCO_SET_SWING_HEIGHT
                result = self.loco._Call(ROBOT_API_ID_LOCO_SET_SWING_HEIGHT, json.dumps({"data": p["value"]}))
            elif op == "speed_mode":
                result = self.loco.SetSpeedMode(p["value"])
            elif op == "user_control":
                result = self.loco.SwitchToUserCtrl()
            elif op == "internal_control":
                modes = [InternalFsmMode.LAST, InternalFsmMode.PASSIVE, InternalFsmMode.WALKRUN]
                result = self.loco.SwitchToInternalCtrl(modes[p["value"]])
            elif op == "tts":
                result = self.audio.TtsMaker(p["text"], p["speaker_id"])
            elif op == "volume":
                result = self.audio.SetVolume(p["value"])
            elif op == "led":
                result = self.audio.LedControl(p["r"], p["g"], p["b"])
            elif op == "play_pcm":
                result = self.audio.PlayStream(p["app_name"], p["stream_id"], base64.b64decode(p["pcm_base64"]))
            elif op == "play_stop":
                # PlayStop in this SDK discards the actual return code. Preserve real ACK.
                from unitree_sdk2py.g1.audio.g1_audio_api import ROBOT_API_ID_AUDIO_STOP_PLAY
                result = self.audio._Call(ROBOT_API_ID_AUDIO_STOP_PLAY, json.dumps({"app_name": "g1-console"}))
            else:
                raise ConsoleError("UNIMPLEMENTED", "该官方操作尚未接线", 501)
            return result if isinstance(result, tuple) else (result, None)

    def close(self):
        self.stop_event.set()
        self.closed = True
        self.subscriber.Close()
        self.safety_feedback.close()
        self.service_versions.close()
        self.mode_reader.close()
        self.poller.join(timeout=2)
