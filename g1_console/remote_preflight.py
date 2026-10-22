"""Pure descriptions of pinned official LocoClient requests; no DDS imports or I/O.

Source: unitreerobotics/unitree_sdk2_python@814556d15970dd2ecf1c9984e845ca02ab07e206
The C++ StandUp/Squat mapping is independently documented in the existing catalog.
"""
from .adapters import FSM_COMMANDS
from .config import ConsoleError


def describe_request(op, params):
    if op in FSM_COMMANDS or op == "fsm":
        method, api_id, payload = "SetFsmId", 7101, {"data": params["value"] if op == "fsm" else FSM_COMMANDS[op]}
    elif op == "balance":
        method, api_id, payload = "SetBalanceMode", 7102, {"data": params["value"]}
    elif op in ("high_stand", "low_stand", "stand_height"):
        method, api_id = "SetStandHeight", 7104
        payload = {"data": 4294967295 if op == "high_stand" else 0 if op == "low_stand" else params["value"]}
    elif op in ("velocity", "stop"):
        method, api_id = "SetVelocity", 7105
        payload = {"velocity": [params[k] for k in ("vx", "vy", "omega")] if op == "velocity" else [0., 0., 0.], "duration": .25}
    elif op in ("user_control", "internal_control"):
        method, api_id = ("SwitchToUserCtrl", 7110) if op == "user_control" else ("SwitchToInternalCtrl", 7111)
        payload = {"data": False if op == "user_control" else params["value"]}
    else:
        raise ConsoleError("PREFLIGHT_UNAVAILABLE", "此入口尚无接口预检映射；不会尝试实机调用", 422)
    return dict(service="sport", method=method, api_id=api_id, parameter=payload,
                sdk_commit="814556d15970dd2ecf1c9984e845ca02ab07e206")
