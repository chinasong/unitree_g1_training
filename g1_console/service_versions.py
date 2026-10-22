"""Continuously receive service advertisements. They never grant motion authority."""
import copy
import hashlib
import json
import re
import threading
import time

TOPIC = 'rt/servicestate'
NAME = re.compile(r'[A-Za-z0-9_][A-Za-z0-9_.-]{0,95}')
VERSION = re.compile(r'v?[0-9]{1,10}(?:\.[0-9]{1,10}){1,5}(?:[-+][A-Za-z0-9_.-]{1,48})?')
NAME_KEYS = ('name', 'service', 'service_name', 'serviceName', 'module')
VERSION_KEYS = ('version', 'service_version', 'serviceVersion', 'moduleVersion')


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError('duplicate field')
        result[key] = value
    return result


def extract_versions(text):
    """Keep explicit same-object/map bindings only, never adjacency in a string."""
    if not isinstance(text, str) or len(text.encode()) > 262144:
        raise ValueError('oversized frame')
    def invalid_constant(value):
        raise ValueError('nonfinite JSON')
    root = json.loads(text, object_pairs_hook=_unique_object, parse_constant=invalid_constant)
    rows, nodes = [], 0

    def walk(value, path='$', depth=0, map_key=None):
        nonlocal nodes
        nodes += 1
        if nodes > 2048 or depth > 12:
            raise ValueError('excessive structure')
        if isinstance(value, dict):
            names = {value[k] for k in NAME_KEYS if isinstance(value.get(k), str) and NAME.fullmatch(value[k])}
            versions = {value[k] for k in VERSION_KEYS if isinstance(value.get(k), str) and VERSION.fullmatch(value[k])}
            # Conflicting labels/versions remain unbound. A map key is used only
            # when the object has no explicit name; the JSON path is retained.
            if not any(k in value for k in NAME_KEYS) and map_key and NAME.fullmatch(map_key):
                names = {map_key}
                binding = 'map_key'
            else:
                binding = 'object_fields'
            if len(names) == len(versions) == 1:
                rows.append(dict(name=next(iter(names)), version=next(iter(versions)), path=path, binding=binding))
                if len(rows) > 128:
                    raise ValueError('excessive versions')
            for key, item in value.items():
                # Paths disclose field names only; credentials and arbitrary
                # strings are never retained in the API or evidence store.
                safe_key = key if NAME.fullmatch(key) else '?'
                walk(item, f'{path}.{safe_key}', depth+1, key)
        elif isinstance(value, list):
            for index, item in enumerate(value):
                walk(item, f'{path}[{index}]', depth+1)
    walk(root)
    return rows


class ServiceVersionStore:
    def __init__(self):
        self.lock = threading.Lock()
        self.closed, self.listening = False, False
        self.started_at = time.time()
        self.received_at = self.received = None
        self.frames, self.rows, self.sha256 = 0, [], None
        self.status = 'waiting'

    def record(self, text):
        digest = None
        try:
            rows = extract_versions(text)
            digest = hashlib.sha256(text.encode()).hexdigest()
            status = 'advertisement_received' if rows else 'no_matching_fields'
        except (ValueError, TypeError, RecursionError, UnicodeError):
            rows, status = [], 'invalid_frame'
        with self.lock:
            if self.closed:
                return
            self.frames += 1
            self.rows, self.status, self.sha256 = rows, status, digest
            self.received_at, self.received = time.time(), time.monotonic()

    def snapshot(self):
        with self.lock:
            return dict(topic=TOPIC, listening=self.listening and not self.closed, status=self.status,
                started_at=self.started_at, received_at=self.received_at,
                age_ms=round((time.monotonic()-self.received)*1000) if self.received is not None else None,
                frames=self.frames, rows=copy.deepcopy(self.rows), sha256=self.sha256,
                current=False, physical_publisher_verified=False, running_build_verified=False,
                device_motion_firmware=None, execution_allowed=False,
                timestamp_basis='local_receive_time', retention='current_console_connection')


class PassiveServiceVersions(ServiceVersionStore):
    def __init__(self):
        super().__init__()
        self.channel = None
        try:
            from cyclonedds.qos import Qos, Policy
            from unitree_sdk2py.core.channel import ChannelFactory
            from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
            self.channel = ChannelFactory().CreateChannel(TOPIC, String_)
            self.channel.SetReader(Qos(Policy.Reliability.Reliable(100000000),
                Policy.Durability.Volatile, Policy.History.KeepLast(4)),
                lambda sample: self.record(sample.data), 4)
            self.listening = True
        except Exception:
            self.status = 'reader_unavailable'

    def close(self):
        with self.lock:
            self.closed, self.listening = True, False
            self.rows = []
        if self.channel is not None:
            self.channel.CloseReader()
