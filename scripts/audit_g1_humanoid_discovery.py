"""Orin-only bounded discovery of the actual sport responder's DDS endpoints.

Only two fixed official getters are sent. Only the two official HG state topics
are subscribed; no command topic or arbitrary RPC is used. RTPS capture retains
headers only; decoded HG fields are recorded separately with receipt timing.
"""
from datetime import datetime, timezone
import http.cookiejar
import json
import math
from pathlib import Path
import socket
import sys
import threading
import time
import urllib.request


def discovery_metadata(qos):
    safe = {'__Hostname', '__ProcessName', '__Pid', '__ProcessId', '__ExecName'}
    properties = []
    for policy in qos:
        if type(policy).__qualname__ == 'Policy.Property':
            row = dict(name=policy.key)
            if policy.key in safe and isinstance(policy.value, str) and len(policy.value) <= 256:
                row['value'] = policy.value
            properties.append(row)
    return dict(properties=properties)


def rtps_header(data):
    if len(data) < 20 or data[:4] != b'RTPS' or data[4] != 2:
        return None
    return dict(guid_prefix=data[8:20].hex(), protocol=list(data[4:6]), vendor_id=data[6:8].hex())


def inspect():
    root = Path('/home/unitree/workspace/g1-body-console')
    release = (root/'active-release').read_text().strip()
    sys.path.insert(0, str(root/'releases'/release))
    from cyclonedds.builtin import (BuiltinDataReader, BuiltinTopicDcpsPublication,
                                   BuiltinTopicDcpsSubscription, BuiltinTopicDcpsParticipant)
    from cyclonedds.domain import Domain, DomainParticipant
    from cyclonedds.pub import DataWriter
    from cyclonedds.sub import DataReader
    from cyclonedds.topic import Topic
    from unitree_sdk2py.idl.unitree_api.msg.dds_ import (
        Request_, Response_, RequestHeader_, RequestIdentity_, RequestLease_, RequestPolicy_)
    from g1_console.hg_sport_state import SportModeState_
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}),
        urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar()))
    opener.open('http://127.0.0.1:8765/api/session', timeout=5).read()

    def guard():
        s = json.load(opener.open('http://127.0.0.1:8765/api/state', timeout=5))
        assert s['connected'] and s['mode'] == 'robot'
        assert s['control']['owner'] is None and not s['manual_remote']['armed']
        assert s.get('navigation', {}).get('status') not in ('running', 'executing')
        return dict(owner=s['control']['owner'], armed=s['manual_remote']['armed'],
                    navigation=s.get('navigation', {}).get('status'),
                    fsm=s.get('telemetry', {}).get('fsm'))

    def processes():
        rows = []
        for p in Path('/proc').iterdir():
            if not p.name.isdigit():
                continue
            try:
                args = [v.decode(errors='replace') for v in (p/'cmdline').read_bytes().split(b'\0') if v]
                if args and args[0].startswith(('/unitree/', '/home/unitree/megaflow_sdk/')):
                    rows.append(dict(pid=int(p.name), argv=args,
                        start_ticks=int((p/'stat').read_text().rsplit(')', 1)[1].split()[19])))
            except (OSError, ValueError, IndexError):
                continue
        return sorted(rows, key=lambda row: row['pid'])

    begin, before, official = datetime.now(timezone.utc).isoformat(), guard(), processes()
    seen, errors, stop = {}, [], threading.Event()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)

    def receive():
        while not stop.is_set():
            try:
                data, address = sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError as exc:
                if not stop.is_set():
                    errors.append(type(exc).__name__)
                break
            header = rtps_header(data)
            if header:
                row = seen.setdefault((header['guid_prefix'], address[0]),
                    dict(**header, source_ip=address[0], packets=0))
                row['packets'] += 1

    publications, subscriptions, participants, queries = {}, {}, {}, []
    streams, state_errors = {}, []
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('', 7400))
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
            socket.inet_aton('239.255.0.1')+socket.inet_aton('192.168.123.164'))
        sock.settimeout(.25)
        receiver = threading.Thread(target=receive, daemon=True)
        receiver.start()
        domain = Domain(0, '<CycloneDDS><Domain Id="0"><General><Interfaces>'
            '<NetworkInterface name="eth0"/></Interfaces></General></Domain></CycloneDDS>')
        participant = DomainParticipant(0)
        readers = [(BuiltinDataReader(participant, BuiltinTopicDcpsPublication), publications),
                   (BuiltinDataReader(participant, BuiltinTopicDcpsSubscription), subscriptions)]
        participant_reader = BuiltinDataReader(participant, BuiltinTopicDcpsParticipant)
        writer = DataWriter(participant, Topic(participant, 'rt/api/sport/request', Request_))
        reader = DataReader(participant, Topic(participant, 'rt/api/sport/response', Response_))
        state_readers = [(name, DataReader(participant, Topic(participant, name, SportModeState_)))
            for name in ('rt/sportmodestate', 'rt/lf/sportmodestate')]
        state_started = time.monotonic()

        def receive_states():
            try:
                while not stop.is_set():
                    for name, state_reader in state_readers:
                        for sample in state_reader.take(1000):
                            if not sample.sample_info.valid_data:
                                continue
                            values = {key: getattr(sample, key) for key in ('fsm_id', 'fsm_mode', 'task_id', 'task_time')}
                            if not all(math.isfinite(v) for v in values.values()):
                                state_errors.append('nonfinite HG state')
                                continue
                            elapsed = round(time.monotonic()-state_started, 6)
                            handle = sample.sample_info.publication_handle
                            row = streams.setdefault((name, handle), dict(topic=name,
                                publication_handle=handle, samples=0, first_receipt_s=elapsed,
                                last_receipt_s=elapsed, first=values, last=values, distinct_values=[]))
                            row['samples'] += 1
                            row['last_receipt_s'], row['last'] = elapsed, values
                            if values not in row['distinct_values'] and len(row['distinct_values']) < 20:
                                row['distinct_values'].append(values)
                    time.sleep(.01)
            except Exception as exc:
                state_errors.append(type(exc).__name__)

        state_receiver = threading.Thread(target=receive_states, daemon=True)
        state_receiver.start()
        until = time.monotonic()+16
        time.sleep(1)
        for api in (7001, 7003):
            guard()
            identity = RequestIdentity_(time.monotonic_ns(), api)
            started, replies = time.monotonic(), []
            writer.write(Request_(RequestHeader_(identity, RequestLease_(0),
                RequestPolicy_(0, False)), '', []))
            while time.monotonic()-started < 1:
                for msg in reader.take(50):
                    if msg.sample_info.valid_data and msg.header.identity.id == identity.id:
                        code = msg.header.status.code
                        replies.append(dict(api_id=msg.header.identity.api_id, code=code,
                            raw=msg.data if code == 0 and len(msg.data.encode()) <= 8192 else None,
                            publication_handle=msg.sample_info.publication_handle,
                            elapsed_ms=round((time.monotonic()-started)*1000, 2)))
                time.sleep(.005)
            queries.append(dict(service='sport', api_id=api, request_id=identity.id,
                sent_at_receipt_clock_s=round(started-state_started, 6), replies=replies))
        while time.monotonic() < until:
            for reader_, rows in readers:
                for s in reader_.take(1000):
                    if s.sample_info.valid_data:
                        rows[str(s.key)] = dict(key=str(s.key), participant_key=str(s.participant_key),
                            topic=s.topic_name, type=s.type_name,
                            instance_handle=s.sample_info.instance_handle)
            for s in participant_reader.take(1000):
                if s.sample_info.valid_data:
                    participants[str(s.key)] = dict(key=str(s.key), **discovery_metadata(s.qos))
            time.sleep(.05)
    finally:
        stop.set()
        sock.close()
        if 'receiver' in locals():
            receiver.join(1)
        if 'state_receiver' in locals():
            state_receiver.join(1)
    for row in streams.values():
        row['matching_publications'] = [p for p in publications.values()
            if p['instance_handle'] == row['publication_handle'] and p['topic'] == row['topic']]
    actual = set()
    for query in queries:
        for reply in query['replies']:
            reply['matching_publications'] = [p for p in publications.values()
                if p['instance_handle'] == reply['publication_handle'] and p['topic'] == 'rt/api/sport/response']
            actual.update(p['participant_key'] for p in reply['matching_publications'])
    endpoints = {}
    for name, rows in [('publications', publications), ('subscriptions', subscriptions)]:
        endpoints[name] = sorted([p for p in rows.values() if p['participant_key'] in actual], key=lambda p:p['topic'])
    peer_keys = {key for key in participants if any(
        r['guid_prefix'] == key.replace('-', '')[:24] and r['source_ip'] == '192.168.123.161'
        for r in seen.values())}
    peers = [dict(**participants[key],
        observed_rtps_source_ips=sorted({r['source_ip'] for r in seen.values()
            if r['guid_prefix'] == key.replace('-', '')[:24]}),
        publications=sorted([p for p in publications.values() if p['participant_key'] == key], key=lambda p:p['topic']),
        subscriptions=sorted([p for p in subscriptions.values() if p['participant_key'] == key], key=lambda p:p['topic']))
        for key in sorted(peer_keys-actual)]
    after, official_after = guard(), processes()
    senders = [r for r in seen.values() if r['guid_prefix'] in {k.replace('-', '')[:24] for k in actual}]
    return dict(current=False, checked_at_utc=begin, completed_at_utc=datetime.now(timezone.utc).isoformat(),
        release=release, boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
        before=before, after=after, official_processes_before=official,
        official_processes_after=official_after, official_processes_unchanged=official == official_after,
        queries=queries, response_participants=[participants[k] for k in sorted(actual) if k in participants],
        endpoints=endpoints, other_node161_participants=peers,
        hg_state=dict(streams=list(streams.values()), errors=state_errors,
            clock='Orin monotonic receipt time; not sensor time or latency measurement',
            duration_s=round(time.monotonic()-state_started, 6)),
        total_discovered=dict(publications=len(publications), subscriptions=len(subscriptions)),
        passive_rtps=dict(duration_s=16, payloads_retained=False, errors=errors, senders=senders),
        device_motion_firmware=None, exact_7301_predicate=None, command_writes=0, lease_requests=0,
        limitation='Bounded discovery inventory only. Topic names and DDS self-reports do not prove a public read contract, installed firmware or internal 7301 predicate.')


if __name__ == '__main__':
    print(json.dumps(inspect(), ensure_ascii=False, indent=2))
