"""Bounded Orin audit linking actual service frames and getter replies to DDS peers.

Only fixed official getters are sent, with lease ID zero. No mode, activation,
command, service-management or log-control request is available in this script.
RTPS multicast headers identify observed source IPs; participant metadata remains
a self-report and cannot establish an executable's build or motion authority.
"""
from datetime import datetime, timezone
import hashlib
import http.cookiejar
import json
from pathlib import Path
import socket
import sys
import threading
import time
import urllib.request


def inspect(duration=330):
    if not 1 <= duration <= 360:
        raise ValueError('duration must be between 1 and 360 seconds')
    root = Path('/home/unitree/workspace/g1-body-console')
    release = (root/'active-release').read_text().strip()
    sys.path[:0] = [str(root/'releases'/release),
                   str(root/'releases'/release/'externals/unitree_sdk2_python')]
    from cyclonedds.builtin import (BuiltinDataReader, BuiltinTopicDcpsPublication,
                                   BuiltinTopicDcpsParticipant)
    from cyclonedds.domain import Domain, DomainParticipant
    from cyclonedds.pub import DataWriter
    from cyclonedds.qos import Qos, Policy
    from cyclonedds.sub import DataReader
    from cyclonedds.topic import Topic
    from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
    from unitree_sdk2py.idl.unitree_api.msg.dds_ import (
        Request_, Response_, RequestHeader_, RequestIdentity_, RequestLease_, RequestPolicy_)
    from g1_console.service_versions import extract_versions
    from g1_console.hg_sport_state import SportModeState_

    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}),
        urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar()))
    opener.open('http://127.0.0.1:8765/api/session', timeout=5).read()

    def guard():
        state = json.load(opener.open('http://127.0.0.1:8765/api/state', timeout=5))
        assert state['connected'] and state['mode'] == 'robot'
        assert state['control']['owner'] is None and not state['manual_remote']['armed']
        assert state.get('navigation', {}).get('status') not in ('running', 'executing')
        return dict(run_id=state['run_id'], owner=state['control']['owner'],
            armed=state['manual_remote']['armed'],
            navigation=state.get('navigation', {}).get('status'))

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

    boot_path = Path('/proc/sys/kernel/random/boot_id')
    began = datetime.now(timezone.utc).isoformat()
    boot, before, official = boot_path.read_text().strip(), guard(), processes()
    seen, errors, stop = {}, [], threading.Event()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)

    def receive_headers():
        while not stop.is_set():
            try:
                data, address = sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError as exc:
                if not stop.is_set():
                    errors.append(type(exc).__name__)
                break
            if len(data) >= 20 and data[:4] == b'RTPS' and data[4] == 2:
                prefix = data[8:20].hex()
                row = seen.setdefault((prefix, address[0]),
                    dict(guid_prefix=prefix, source_ip=address[0], packets=0))
                row['packets'] += 1

    publications, participants, frames, queries, states = {}, {}, [], [], {}
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('', 7400))
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
            socket.inet_aton('239.255.0.1')+socket.inet_aton('192.168.123.164'))
        sock.settimeout(.25)
        receiver = threading.Thread(target=receive_headers, daemon=True)
        receiver.start()
        domain = Domain(0, '<CycloneDDS><Domain Id="0"><General><Interfaces>'
            '<NetworkInterface name="eth0"/></Interfaces></General></Domain></CycloneDDS>')
        participant = DomainParticipant(0)
        pub_reader = BuiltinDataReader(participant, BuiltinTopicDcpsPublication)
        peer_reader = BuiltinDataReader(participant, BuiltinTopicDcpsParticipant)
        qos = Qos(Policy.Reliability.Reliable(100000000),
            Policy.Durability.Volatile, Policy.History.KeepLast(4))
        ad_reader = DataReader(participant, Topic(participant, 'rt/servicestate', String_), qos=qos)
        state_reader = DataReader(participant,
            Topic(participant, 'rt/lf/sportmodestate', SportModeState_))
        channels = {service: (
            DataWriter(participant, Topic(participant, f'rt/api/{service}/request', Request_)),
            DataReader(participant, Topic(participant, f'rt/api/{service}/response', Response_)))
            for service in ('sport', 'motion_switcher')}
        started = time.monotonic()

        def drain():
            for sample in pub_reader.take(1000):
                if sample.sample_info.valid_data:
                    publications[str(sample.key)] = dict(key=str(sample.key),
                        participant_key=str(sample.participant_key), topic=sample.topic_name,
                        type=sample.type_name, instance_handle=sample.sample_info.instance_handle)
            safe = {'__Hostname', '__ProcessName', '__Pid', '__ProcessId', '__ExecName'}
            for sample in peer_reader.take(1000):
                if sample.sample_info.valid_data:
                    properties = {p.key: p.value for p in sample.qos
                        if type(p).__qualname__ == 'Policy.Property' and p.key in safe
                        and isinstance(p.value, str) and len(p.value) <= 256}
                    participants[str(sample.key)] = dict(key=str(sample.key), self_report=properties)
            for sample in ad_reader.take(10):
                if sample.sample_info.valid_data:
                    try:
                        rows = extract_versions(sample.data)
                        frames.append(dict(received_at_utc=datetime.now(timezone.utc).isoformat(),
                            elapsed_s=round(time.monotonic()-started, 6), rows=rows,
                            sha256=hashlib.sha256(sample.data.encode()).hexdigest(),
                            publication_handle=sample.sample_info.publication_handle))
                    except (ValueError, TypeError, RecursionError, UnicodeError):
                        errors.append('invalid_service_advertisement')
            for sample in state_reader.take(1000):
                if sample.sample_info.valid_data:
                    handle = sample.sample_info.publication_handle
                    values = {k: getattr(sample, k) for k in ('fsm_id', 'fsm_mode', 'task_id', 'task_time')}
                    row = states.setdefault(handle, dict(publication_handle=handle, samples=0,
                        first=values, last=values, distinct_values=[]))
                    row['samples'] += 1
                    row['last'] = values
                    if values not in row['distinct_values'] and len(row['distinct_values']) < 20:
                        row['distinct_values'].append(values)

        def getters(phase):
            for service, api in (('sport', 7001), ('sport', 7002), ('sport', 7003),
                                 ('sport', 7005), ('motion_switcher', 1005)):
                guard()
                writer, reader = channels[service]
                identity = RequestIdentity_(time.monotonic_ns(), api)
                sent = time.monotonic()
                row = dict(phase=phase, service=service, api_id=api, request_id=identity.id,
                    parameter='', lease_id=0, sent_at_utc=datetime.now(timezone.utc).isoformat(), replies=[])
                writer.write(Request_(RequestHeader_(identity, RequestLease_(0),
                    RequestPolicy_(0, False)), '', []))
                while time.monotonic()-sent < 1:
                    drain()
                    for msg in reader.take(50):
                        if (msg.sample_info.valid_data and msg.header.identity.id == identity.id
                                and msg.header.identity.api_id == api):
                            code = msg.header.status.code
                            row['replies'].append(dict(code=code,
                                data=msg.data if code == 0 and len(msg.data.encode()) <= 8192 else None,
                                publication_handle=msg.sample_info.publication_handle,
                                elapsed_ms=round((time.monotonic()-sent)*1000, 3)))
                    time.sleep(.005)
                queries.append(row)

        time.sleep(1)
        getters('initial')
        after_frame = False
        while time.monotonic()-started < duration:
            drain()
            if frames and not after_frame:
                after_frame = True
                getters('after_service_advertisement')
            time.sleep(.03)
        drain()
    finally:
        stop.set()
        sock.close()
        if 'receiver' in locals():
            receiver.join(1)

    selected = set()
    def bind(row, topic):
        matching = [p for p in publications.values()
            if p['topic'] == topic and p['instance_handle'] == row['publication_handle']]
        row['matching_publications'] = matching
        selected.update(p['participant_key'] for p in matching)
    for row in frames:
        bind(row, 'rt/servicestate')
    for query in queries:
        for row in query['replies']:
            bind(row, f"rt/api/{query['service']}/response")
    for row in states.values():
        bind(row, 'rt/lf/sportmodestate')
    peers = [dict(**participants[key], observed_rtps_source_ips=sorted({r['source_ip']
        for r in seen.values() if r['guid_prefix'] == key.replace('-', '')[:24]}))
        for key in sorted(selected) if key in participants]
    after, official_after = guard(), processes()
    return dict(current=False, checked_at_utc=began,
        completed_at_utc=datetime.now(timezone.utc).isoformat(), release=release,
        boot_id=boot, boot_unchanged=boot == boot_path.read_text().strip(),
        before=before, after=after, official_processes_before=official,
        official_processes_after=official_after, official_processes_unchanged=official == official_after,
        duration_s=round(time.monotonic()-started, 3), frames=frames, queries=queries,
        hg_state=list(states.values()), participants=peers,
        passive_rtps=dict(headers_only=True, raw_payloads_retained=False,
            senders=[r for r in seen.values() if r['guid_prefix'] in
                {key.replace('-', '')[:24] for key in selected}]), errors=errors,
        command_writes=0, lease_requests=0, device_motion_firmware=None,
        exact_7301_predicate=None, motion_authority_verified=False,
        limitations=['DDS self-reports and source IP pairing are evidence of publisher provenance only.',
            'Advertisement versions do not prove the humanoid executable build.',
            'No action or motion lease was requested; accepting getters does not prove motion authority.'])


if __name__ == '__main__':
    print(json.dumps(inspect(), ensure_ascii=False, indent=2))
