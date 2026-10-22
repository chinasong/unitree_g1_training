"""Bounded, receive-only evidence. Raw transport values never grant safety authority."""
import copy
from dataclasses import asdict
import json
import threading
import time

CHANNELS = {
    'sport': ('rt/lf/sportmodestate', 'unitree_hg::msg::dds_::SportModeState_', 1500),
    'bms': ('rt/lf/bmsstate', 'unitree_hg::msg::dds_::BmsState_', 1500),
    'mainboard': ('rt/lf/mainboardstate', 'unitree_hg::msg::dds_::MainBoardState_', 1500),
    'arm_state': ('rt/arm/action/state', 'std_msgs::msg::dds_::String_', 1500),
    'selftest': ('rt/selftest', 'std_msgs::msg::dds_::String_', 1500),
    'secondary_imu': ('rt/secondary_imu', 'unitree_hg::msg::dds_::IMUState_', 1500),
    'pelvis_odom': ('rt/state_estimator/odom_pelvis', 'nav_msgs::msg::dds_::Odometry_', 1500),
    'torso_odom': ('rt/state_estimator/odom_torso', 'nav_msgs::msg::dds_::Odometry_', 1500),
    'motion_mode': ('motion_switcher.CheckMode / 1001', 'official JSON response', 6000),
    'fsm_mode': ('sport.GetFsmMode / 7002', 'official JSON response', 6000),
}


def read_fsm_mode(client):
    """Pinned C++ GetFsmMode contract: dedicated getter, empty parameter, no lease."""
    code, data = client._Call(7002, '')
    if code != 0:
        return code, None
    try:
        raw = json.loads(data)
        if not isinstance(raw, dict) or type(raw.get('data')) is not int:
            raise ValueError('invalid FSM mode')
        if not 0 <= raw['data'] <= 0xffffffff:
            raise ValueError('out of range FSM mode')
    except (ValueError, TypeError):
        return 'INVALID_FEEDBACK', None
    return 0, {'data': raw['data']}


class FeedbackStore:
    def __init__(self):
        self.lock = threading.Lock()
        self.values = {}
        self.closed = False

    def record(self, key, raw, code=0):
        if code == 0:
            try:
                encoded = json.dumps(raw, allow_nan=False, ensure_ascii=False)
                if not isinstance(raw, dict) or len(encoded.encode()) > 4096:
                    raise ValueError('invalid or oversized feedback')
            except (TypeError, ValueError):
                raw, code = None, 'INVALID_FEEDBACK'
        if code != 0:
            raw = None
        with self.lock:
            if self.closed:
                return
            previous = self.values.get(key, {})
            self.values[key] = dict(raw=copy.deepcopy(raw), code=code, received=time.monotonic(),
                received_at=time.time(), samples=previous.get('samples', 0) + int(raw is not None))

    def snapshot(self, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            values = copy.deepcopy(self.values)
        out = {}
        for key, (source, typename, ttl_ms) in CHANNELS.items():
            value = values.get(key, {})
            age = (now - value['received']) * 1000 if 'received' in value else None
            current = age is not None and 0 <= age < ttl_ms and value.get('code') == 0
            out[key] = dict(source=source, type=typename, current=current,
                age_ms=round(age) if age is not None else None, ttl_ms=ttl_ms,
                received_at=value.get('received_at'), samples=value.get('samples', 0),
                raw=value.get('raw') if current else None, code=value.get('code'),
                physical_publisher_verified=False, hardware_health_verified=False,
                timestamp_basis='local_receive_time', sensor_timestamp=None)
        return out


class PassiveSafetyFeedback(FeedbackStore):
    def __init__(self):
        super().__init__()
        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import BmsState_, MainBoardState_, IMUState_
        from unitree_sdk2py.idl.nav_msgs.msg.dds_ import Odometry_
        from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
        from .hg_sport_state import SportModeState_
        self.subscribers = []
        for key, cls in [('sport', SportModeState_), ('bms', BmsState_), ('mainboard', MainBoardState_),
                         ('arm_state', String_), ('selftest', String_), ('secondary_imu', IMUState_),
                         ('pelvis_odom', Odometry_), ('torso_odom', Odometry_)]:
            try:
                sub = ChannelSubscriber(CHANNELS[key][0], cls)
                sub.Init(lambda sample, key=key: self.record(key, asdict(sample)), 1)
                self.subscribers.append(sub)
            except Exception as exc:
                self.record(key, None, type(exc).__name__)

    def close(self):
        with self.lock:
            self.closed = True
            self.values.clear()
        for subscriber in self.subscribers:
            subscriber.Close()
