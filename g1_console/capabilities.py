"""Explicit allowlist of the pinned SDK's G1 public operations (not raw RPC)."""
import base64
import math
from pydantic import ValidationError
from .contracts import Contract, Velocity, OneInt, Scalar, Text, RGB, PCM
from .config import ConsoleError

ARM_ACTIONS = {99: "释放手臂", 11: "双臂飞吻", 12: "左臂飞吻", 13: "右臂飞吻", 15: "双臂举起",
               17: "拍手", 18: "击掌", 19: "拥抱", 20: "双臂比心", 21: "右臂比心", 22: "拒绝",
               23: "右臂举起", 24: "交叉双臂", 25: "面前挥手", 26: "高位挥手", 27: "握手姿态"}


def entry(id, label, method, group="运动", risk="motion", schema="none", **extra):
    return dict(id=id, label=label, method=method, group=group, risk=risk, schema=schema, **extra)


CATALOG = [
    entry("stop", "停止行走", "StopMove / SetVelocity(0,0,0)", risk="stop"),
    entry("velocity", "速度控制", "SetVelocity / Move", schema="velocity"),
    entry("start", "进入行走控制", "Start / SetFsmId(500)"),
    entry("prepare", "站立预备 / 姿态保持", "C++ StandUp / SetFsmId(4)", risk="guarded",
          note="与 Python 蹲站转换 706 是不同入口；本机固件和当前坐姿适用性未核定，实机锁定"),
    entry("squat", "蹲姿模式", "C++ Squat / SetFsmId(2)", risk="guarded",
          note="蹲姿无平衡控制；本机切换条件未核定，不能把坐地当成蹲姿，实机锁定"),
    entry("squat_to_stand", "蹲姿起立", "Squat2StandUp / SetFsmId(706)", risk="guarded"),
    entry("stand_to_squat", "站姿下蹲", "StandUp2Squat / SetFsmId(706)", risk="guarded"),
    entry("lie_to_stand", "仰卧起身", "Lie2StandUp / SetFsmId(702)", risk="guarded"),
    entry("sit", "坐下", "Sit / SetFsmId(3)", risk="guarded"),
    entry("damp", "阻尼模式", "Damp / SetFsmId(1)", risk="guarded", note="可能失去站立支撑；不能用作通用急停"),
    entry("zero_torque", "零力矩", "ZeroTorque / SetFsmId(0)", risk="guarded", note="会失去主动支撑，需要吊装或可靠支撑"),
    entry("high_stand", "最高站立高度", "HighStand / SetStandHeight(UINT32_MAX)", risk="guarded"),
    entry("low_stand", "最低站立高度", "LowStand / SetStandHeight(0)", risk="guarded"),
    entry("stand_height", "站立高度参数", "SetStandHeight", risk="guarded", schema="height",
          note="0–1 仅为项目输入限制；本机单位、范围和适用条件未核定，实机锁定"),
    entry("swing_height", "摆腿高度参数", "SetSwingHeight / API 7103", risk="guarded", schema="height",
          note="官方 C++ 协议补齐；0–1 仅为项目输入限制，本机范围未核定，实机锁定"),
    entry("balance", "平衡模式", "SetBalanceMode / BalanceStand", schema="balance"),
    entry("speed_mode", "速度档位", "SetSpeedMode", schema="speed"),
    entry("fsm", "设置状态机", "SetFsmId", risk="guarded", schema="fsm"),
    entry("task", "设置官方臂任务", "SetTaskId", group="手臂", schema="task"),
    entry("wave", "挥手", "WaveHand / SetTaskId(0)", group="手臂"),
    entry("wave_turn", "挥手并转身", "WaveHand(True) / SetTaskId(1)", group="手臂", risk="guarded"),
    entry("shake_start", "握手姿态 · 伸臂", "ShakeHand(0) / SetTaskId(2)", group="手臂"),
    entry("shake_end", "握手姿态 · 收臂", "ShakeHand(1) / SetTaskId(3)", group="手臂"),
    entry("user_control", "切换用户控制", "SwitchToUserCtrl", group="高级", risk="guarded", note="退出内部控制；本软件不提供低层腿部控制器"),
    entry("internal_control", "恢复内部控制", "SwitchToInternalCtrl", group="高级", risk="guarded", schema="internal"),
    entry("get_fsm", "读取状态机", "GetFsmId", group="查询", risk="read"),
    entry("get_fsm_mode", "读取控制模式", "GetFsmMode", group="查询", risk="read"),
    entry("get_silent", "读取模式服务 silent 标志", "GetSilent / motion_switcher 1005", group="查询", risk="read",
          note="只读原始布尔标志；false 不证明运动权限或站稳，不提供 SetSilent 操作"),
    entry("get_balance", "读取平衡模式", "GetBalanceMode / API 7003", group="查询", risk="read"),
    entry("get_swing_height", "读取摆腿高度", "GetSwingHeight / API 7004", group="查询", risk="read"),
    entry("get_stand_height", "读取站立高度", "GetStandHeight / API 7005", group="查询", risk="read"),
    entry("get_phase", "读取步态相位", "GetPhase / API 7006", group="查询", risk="read",
          note="官方已标记弃用；按需读取，错误与缺失保留未知，不作为足底承重"),
    entry("get_actions", "查询固件动作清单", "GetActionList", group="查询", risk="read"),
    entry("get_services", "读取官方服务清单", "ServiceList / robot_state 1003", group="查询", risk="read",
          note="原始 status/protect 不推断为健康或运动权限；本机清单没有版本字段"),
    entry("get_versions", "读取官方包版本", "GetPkgVersion / robot_state 1006", group="查询", risk="read",
          note="本机只读复查返回 3203 不支持；失败保留未知，不用 App 或缓存版本替代运动固件"),
    entry("get_loco_api_version", "读取运动服务协议版本", "GetServerApiVersion / sport 1", group="查询", risk="read",
          note="只读 API 协议标识；不是运动固件，不证明起身或平衡支持"),
    entry("get_motion_api_version", "读取模式服务协议版本", "GetServerApiVersion / motion_switcher 1", group="查询", risk="read",
          note="只读 API 协议标识；不是运动固件，不证明控制归属"),
    entry("get_state_api_version", "读取状态服务协议版本", "GetServerApiVersion / robot_state 1", group="查询", risk="read",
          note="只读 API 协议标识；与 GetPkgVersion 分开记录，不代替模块版本"),
    entry("arm_action", "执行固件动作 ID", "ExecuteAction", group="手臂", schema="action"),
    entry("tts", "机器人语音播报", "TtsMaker", group="声光", risk="accessory", schema="text"),
    entry("get_volume", "读取音量", "GetVolume", group="查询", risk="read"),
    entry("volume", "设置音量", "SetVolume", group="声光", risk="accessory", schema="volume"),
    entry("led", "设置 RGB 灯", "LedControl", group="声光", risk="accessory", schema="rgb"),
    entry("play_pcm", "播放 PCM 音频", "PlayStream", group="声光", risk="accessory", schema="pcm",
          note="上传 16 kHz 单声道 S16LE 原始 PCM；单次 ≤ 48 KB"),
    entry("play_stop", "停止音频播放", "PlayStop", group="声光", risk="accessory"),
] + [entry(f"arm_{i}", name, f"ExecuteAction({i})", group="官方手势", action_id=i,
           note="连接后以固件 GetActionList 为准；未安装手，不包含手指动作") for i, name in ARM_ACTIONS.items()]
BY_ID = {item["id"]: item for item in CATALOG}
READ_OPERATIONS = frozenset(item["id"] for item in CATALOG if item["risk"] == "read")
# Explicitly narrow the first live write test. Never include a motor/mode setter,
# volume mutation or arbitrary audio stream through a risk-label expansion.
ACCESSORY_TEST_OPERATIONS = frozenset(("tts", "led"))


def require_write_scope(op, scope):
    """The one rule set for which writes each run mode permits.

    `scope` exposes read_only / verify_only / accessory_test / damping_test.
    Missing flags raise AttributeError instead of defaulting to allowed.
    op=None means a generic write such as arming manual control.
    """
    if op in READ_OPERATIONS:
        return
    if scope.read_only:
        raise ConsoleError("READ_ONLY", "只读观察模式：禁止动作、声光和控制模式写入", 403)
    if scope.verify_only:
        raise ConsoleError("VERIFICATION_ONLY", "人工遥控接口验证阶段：可取得控制权和核对参数，禁止全部实机动作、声光和模式切换", 403)
    if scope.accessory_test and op not in ACCESSORY_TEST_OPERATIONS:
        raise ConsoleError("ACCESSORY_TEST_ONLY", "声光测试仅开放语音和 RGB 灯；身体动作与控制模式切换继续锁定", 403)
    if scope.damping_test and op != "damp":
        raise ConsoleError("DAMPING_TEST_ONLY", "仅开放一次 FSM 0→1 官方阻尼测试；其余写入全部锁定", 403)


def validate_params(capability: str, params: dict):
    if capability not in BY_ID:
        raise ConsoleError("UNKNOWN_CAPABILITY", "操作不在官方能力白名单中", 404)
    schema = BY_ID[capability]["schema"]
    model = {"none": Contract, "velocity": Velocity, "text": Text, "rgb": RGB, "pcm": PCM,
             "height": Scalar}.get(schema, OneInt)
    try:
        p = model.model_validate(params).model_dump()
    except ValidationError as exc:
        raise ConsoleError("INVALID_PARAMS", str(exc), 422) from exc
    allowed = {"balance": {0, 1}, "speed": {0, 1, 2}, "fsm": {0, 1, 2, 3, 4, 500, 702, 706},
               "task": {0, 1, 2, 3}, "internal": {0, 1, 2}}
    if schema in allowed and p["value"] not in allowed[schema]:
        raise ConsoleError("PARAM_RANGE", "参数不在已核查的官方取值中", 422)
    if schema == "volume" and not 0 <= p["value"] <= 100:
        raise ConsoleError("PARAM_RANGE", "音量必须在 0–100 之间", 422)
    if schema == "height" and not 0 <= p["value"] <= 1:
        raise ConsoleError("PARAM_RANGE", "高度参数限于 0–1；最高站立使用独立官方操作", 422)
    if schema == "pcm":
        try:
            data = base64.b64decode(p["pcm_base64"], validate=True)
            if not 0 < len(data) <= 48000 or len(data) % 2:
                raise ValueError()
        except ValueError as exc:
            raise ConsoleError("INVALID_PCM", "需要 ≤48 KB、完整 16 位采样的 base64 PCM", 422) from exc
    return p


def firmware_action_catalog(data):
    """Normalize declared entries, including G1's [ID actions, named motions] reply."""
    if isinstance(data, dict):
        for key in ("actions", "action_list", "data"):
            if key in data:
                return firmware_action_catalog(data[key])
        action_id = data.get("id", data.get("action_id"))
        has_id = isinstance(action_id, int) and not isinstance(action_id, bool)
        name = data.get("name")
        if has_id or isinstance(name, str) and name:
            row = dict(id=action_id if has_id else None, name=name if isinstance(name, str) else str(action_id))
            for key in ("fsm", "mode_machine"):
                values = data.get(key)
                if isinstance(values, list) and all(isinstance(v, int) and not isinstance(v, bool) for v in values):
                    row[key] = values
            duration = data.get("time")
            if isinstance(duration, (int, float)) and not isinstance(duration, bool) and math.isfinite(duration) and duration >= 0:
                row["duration_s"] = duration
            return [row]
        # Official servers can return the SDK's {name: numeric_id} action map.
        if data and all(isinstance(v, int) and not isinstance(v, bool) for v in data.values()):
            return [dict(id=v, name=k) for k, v in data.items()]
    if isinstance(data, list):
        entries = []
        for row in data:
            if isinstance(row, int) and not isinstance(row, bool):
                entries.append(dict(id=row, name=str(row)))
            elif isinstance(row, (dict, list)):
                entries.extend(firmware_action_catalog(row))
        return entries
    return []


def action_ids(data):
    """Only explicit numeric IDs establish ExecuteAction availability."""
    return {row["id"] for row in firmware_action_catalog(data) if row["id"] is not None}
