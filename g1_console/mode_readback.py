"""Fixed official getters in a fresh DDS process; no control API or lease."""
import copy
import json
import math
import multiprocessing
import re
import threading
import time
from collections import deque

READS = {
    'fsm_id': ('sport', 7001, '', 1500),
    'fsm_mode': ('sport', 7002, '', 6000),
    'motion_mode': ('motion_switcher', 1001, '{}', 6000),
}
QUERY_READS = {
    'get_silent': ('motion_switcher', 1005, '', 0),
    'get_loco_api_version': ('sport', 1, '{}', 0),
    'get_motion_api_version': ('motion_switcher', 1, '{}', 0),
    'get_state_api_version': ('robot_state', 1, '{}', 0),
    'get_balance': ('sport', 7003, '', 15000),
    'get_swing_height': ('sport', 7004, '', 15000),
    'get_stand_height': ('sport', 7005, '', 15000),
    'get_phase': ('sport', 7006, '', 15000),
    'get_services': ('robot_state', 1003, '{}', 0),
    'get_versions': ('robot_state', 1006, '{}', 0),
    'get_actions': ('arm', 7107, '{}', 0),
    'get_volume': ('voice', 1005, '{}', 0),
}
LOCO_QUERIES = ('get_balance', 'get_swing_height', 'get_stand_height', 'get_phase')


class ReadOnlyModeClient:
    def __init__(self, timeout=.7):
        from unitree_sdk2py.core.channel import ChannelFactory
        from unitree_sdk2py.idl.unitree_api.msg.dds_ import Request_, Response_
        self.timeout, self.closed, self.channels = timeout, False, {}
        try:
            for service in ('sport', 'motion_switcher', 'robot_state', 'arm', 'voice'):
                request = ChannelFactory().CreateChannel(f'rt/api/{service}/request', Request_)
                response = ChannelFactory().CreateChannel(f'rt/api/{service}/response', Response_)
                self.channels[service] = request, response
                response.SetReader()  # No listener, SDK response queue, or retained futures.
                request.SetWriter()
        except Exception:
            self.close()
            raise

    def read(self, name):
        service, api, parameter, _ = (READS | QUERY_READS)[name]  # Fixed allowlist before writing.
        if self.closed:
            return 3102, None
        from unitree_sdk2py.idl.unitree_api.msg.dds_ import (
            Request_, RequestHeader_, RequestIdentity_, RequestLease_, RequestPolicy_)
        identity = RequestIdentity_(time.monotonic_ns(), api)
        request = Request_(RequestHeader_(identity, RequestLease_(0), RequestPolicy_(0, False)), parameter, [])
        writer, reader = self.channels[service]
        deadline = time.monotonic() + self.timeout
        if not writer.Write(request, self.timeout):
            return 3102, None
        while (remaining := deadline-time.monotonic()) > 0:
            response = reader.Read(remaining)
            if response is None:
                break
            header = getattr(response, 'header', None)
            if header is None or header.identity.id != identity.id:
                continue
            if header.identity.api_id != api:
                return 3105, None
            if header.status.code != 0:
                return header.status.code, None
            try:
                if len(response.data.encode()) > 4096:
                    raise ValueError('oversized feedback')
                if name in ('get_loco_api_version', 'get_motion_api_version', 'get_state_api_version'):
                    # Internal API 1 returns a plain protocol string, not JSON
                    # and not a module/firmware version. Never infer either.
                    if not re.fullmatch(r'[0-9]{1,10}(?:\.[0-9]{1,10}){1,3}', response.data):
                        raise ValueError('invalid API protocol version')
                    return 0, {'data': {'service': service, 'api_version': response.data, 'firmware_version': None}}
                raw = json.loads(response.data)
                if name == 'motion_mode':
                    if (not isinstance(raw, dict) or
                        not all(isinstance(raw.get(k), str) for k in ('form', 'name'))):
                        raise ValueError('invalid CheckMode')
                    raw = {k: raw[k] for k in ('form', 'name')}
                elif name == 'get_silent':
                    if not isinstance(raw, dict) or type(raw.get('silent')) is not bool:
                        raise ValueError('invalid silent flag')
                    raw = {'data': {'silent': raw['silent']}}
                elif name in ('fsm_id', 'fsm_mode', 'get_balance'):
                    if not isinstance(raw, dict) or type(raw.get('data')) is not int or not 0 <= raw['data'] <= 0xffffffff:
                        raise ValueError('invalid FSM integer')
                    raw = {'data': raw['data']}
                elif name == 'get_volume':
                    if not isinstance(raw, dict) or type(raw.get('volume')) is not int or not 0 <= raw['volume'] <= 100:
                        raise ValueError('invalid volume')
                    raw = {'data': {'volume': raw['volume']}}
                elif name == 'get_actions':
                    if not isinstance(raw, (list, dict)):
                        raise ValueError('invalid action catalog')
                    raw = {'data': raw}
                elif name == 'get_services':
                    if (not isinstance(raw, list) or len(raw) > 128 or
                        any(not isinstance(r, dict) or not isinstance(r.get('name'), str) or
                            len(r['name']) > 256 or type(r.get('status')) is not int or
                            type(r.get('protect')) is not int for r in raw)):
                        raise ValueError('invalid service list')
                    raw = {'data': [{k: r[k] for k in ('name', 'status', 'protect')} for r in raw]}
                elif name == 'get_versions':
                    if (not isinstance(raw, dict) or not isinstance(raw.get('packageVersion'), str) or
                        not isinstance(raw.get('moduleVersionMap'), dict) or
                        any(not isinstance(k, str) or not isinstance(v, str)
                            for k, v in raw['moduleVersionMap'].items())):
                        raise ValueError('invalid package versions')
                    raw = {'data': {k: raw[k] for k in ('packageVersion', 'moduleVersionMap')}}
                else:
                    value = raw['data']
                    numbers = value if name == 'get_phase' and isinstance(value, list) else [value]
                    if (name == 'get_phase' and (not isinstance(value, list) or not 1 <= len(value) <= 32) or
                        any(type(v) not in (int, float) or not math.isfinite(v) for v in numbers)):
                        raise ValueError('invalid motion feedback')
                    raw = {'data': value}
            except (TypeError, ValueError, AttributeError, KeyError, OverflowError):
                return 'INVALID_FEEDBACK', None
            return 0, raw
        return 3104, None

    def close(self):
        if self.closed:
            return
        self.closed = True
        for writer, reader in self.channels.values():
            reader.CloseReader()
            writer.CloseWriter()


def _read_modes(send, stop, interface):
    client = None
    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        ChannelFactoryInitialize(0, interface)
        client = ReadOnlyModeClient()
        next_due = {key: 0. for key in READS}
        history = {key: deque(maxlen=256) for key in READS}
        totals = {key: dict(attempts=0, successes=0, timeouts=0, errors=0) for key in READS}
        while not stop.is_set():
            for name in READS:
                if stop.is_set() or time.monotonic() < next_due[name]:
                    continue
                start = time.monotonic()
                try:
                    code, raw = client.read(name)
                except Exception as exc:
                    code, raw = type(exc).__name__, None
                received, received_at = time.monotonic(), time.time()
                outcome = 'successes' if code == 0 else 'timeouts' if code == 3104 else 'errors'
                totals[name]['attempts'] += 1
                totals[name][outcome] += 1
                history[name].append((received, outcome))
                recent = [outcome for stamp, outcome in history[name] if 0 <= received-stamp < 60]
                stats = dict(total=totals[name].copy(), window_s=60, attempts=len(recent),
                             **{key: recent.count(key) for key in ('successes', 'timeouts', 'errors')})
                send.send((name, dict(code=code, raw=raw, received=received, received_at=received_at,
                                     request_ms=round((received-start)*1000, 2), diagnostics=stats)))
                next_due[name] = received + (.5 if name == 'fsm_id' else 3.)
            # These queries are on demand only. One per cycle keeps the
            # periodic FSM readers from being starved by operator queries.
            if send.poll():
                token, name = send.recv()
                if name not in QUERY_READS:
                    send.send(('query_result', (token, 'INVALID_QUERY', None,
                                               time.monotonic(), time.time(), 0.)))
                else:
                    started = time.monotonic()
                    try:
                        code, raw = client.read(name)
                    except Exception as exc:
                        code, raw = type(exc).__name__, None
                    received = time.monotonic()
                    send.send(('query_result', (token, code, raw, received, time.time(),
                                               round((received-started)*1000, 2))))
            stop.wait(.02)
    except (BrokenPipeError, EOFError, OSError):
        pass
    except Exception as exc:
        try:
            send.send(('worker_error', type(exc).__name__))
        except (BrokenPipeError, EOFError, OSError):
            pass
    finally:
        if client:
            client.close()
        send.close()


class IsolatedModeReadback:
    def __init__(self, interface='eth0', *, worker=_read_modes):
        ctx = multiprocessing.get_context('spawn')
        self.receive, send = ctx.Pipe(duplex=True)
        self.stop = ctx.Event()
        self.lock = threading.Lock()
        self.query_lock = threading.Lock()
        self.pending = None
        self.values, self.error, self.received, self.closed = {}, None, 0., False
        self.process = ctx.Process(target=worker, args=(send, self.stop, interface),
                                   name='g1-mode-readonly', daemon=True)
        try:
            self.process.start()
        except Exception:
            self.receive.close()
            send.close()
            raise
        send.close()
        self.thread = threading.Thread(target=self._receive, daemon=True, name='g1-mode-ipc')
        self.thread.start()

    def _receive(self):
        try:
            while True:
                if self.receive.poll(.1):
                    name, row = self.receive.recv()
                    with self.lock:
                        if name == 'query_result':
                            token, code, raw, received, received_at, request_ms = row
                            if self.pending and self.pending['token'] == token:
                                self.pending['result'] = (code, raw if code == 0 else None)
                                query = self.pending['name']
                                if query in LOCO_QUERIES:
                                    self.values[query] = dict(code=code, raw=raw if code == 0 else None,
                                        received=received, received_at=received_at, request_ms=request_ms)
                                    self.received = time.monotonic()
                                self.pending['event'].set()
                        elif name in READS:
                            self.values[name] = row  # Preserve the child's receive time.
                            self.received = time.monotonic()
                        else:
                            self.error = 'MODE_WORKER_ERROR'
                            self.values.clear()
                elif not self.process.is_alive():
                    break
        except (EOFError, OSError):
            pass
        finally:
            with self.lock:
                self.values.clear()
                self.error = 'MODE_WORKER_STOPPED' if self.closed else 'MODE_WORKER_EXITED'
                if self.pending:
                    self.pending['result'] = (self.error, None)
                    self.pending['event'].set()

    def read(self, name):
        if name not in QUERY_READS:
            raise KeyError(name)
        with self.query_lock:
            with self.lock:
                if self.closed or not self.process.is_alive():
                    return 'MODE_WORKER_STOPPED' if self.closed else 'MODE_WORKER_EXITED', None
                # Withdraw an earlier success while its replacement is pending.
                self.values.pop(name, None)
                pending = dict(token=time.monotonic_ns(), name=name, event=threading.Event(), result=None)
                self.pending = pending
                try:
                    self.receive.send((pending['token'], name))
                except (BrokenPipeError, EOFError, OSError):
                    self.pending = None
                    return 'MODE_WORKER_EXITED', None
            pending['event'].wait(4)  # Initial 3 polls + this read, each bounded to .7 s.
            with self.lock:
                self.pending = None  # Late replies cannot satisfy the next request.
                return pending['result'] or ('MODE_WORKER_TIMEOUT', None)

    def snapshot(self, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            values, error, last = copy.deepcopy(self.values), self.error, self.received
        if last and now-last > 4:
            error = 'MODE_WORKER_STALLED'
        out = {}
        visible = READS | {name: QUERY_READS[name] for name in LOCO_QUERIES}
        for name, (service, api, _, ttl) in visible.items():
            row = values.get(name, {})
            age = (now-row['received'])*1000 if 'received' in row else None
            recent = not error and age is not None and 0 <= age < ttl
            current = recent and row.get('code') == 0
            stats = row.get('diagnostics')
            out[name] = dict(source=f'{service} / {api}', type='official JSON response',
                current=bool(current), response_recent=bool(recent), age_ms=round(age) if age is not None else None, ttl_ms=ttl,
                raw=row.get('raw') if current else None, code=error or row.get('code'),
                received_at=row.get('received_at'), received_monotonic=row.get('received'),
                samples=stats['total']['successes'] if stats else int(bool(row) and row.get('code') == 0), request_ms=row.get('request_ms'),
                diagnostics=stats, transport='official_getters_isolated_polled',
                physical_publisher_verified=False, hardware_health_verified=False,
                timestamp_basis='worker_local_receive_time', sensor_timestamp=None)
        return out

    def close(self):
        if self.closed:
            return
        self.closed = True
        with self.lock:
            if self.pending:
                self.pending['result'] = ('MODE_WORKER_STOPPED', None)
                self.pending['event'].set()
        self.stop.set()
        self.process.join(timeout=3)
        if self.process.is_alive():
            self.process.terminate()  # Only our own child; no official process is touched.
            self.process.join(timeout=1)
        self.thread.join(timeout=1)
        self.receive.close()
