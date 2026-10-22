"""Orin-only sport responder attribution; fixed getters and passive RTPS headers.

Run with the console venv on stdin. No application command, control lease,
service switch, report-frequency change, or official configuration write.
RTPS headers identify observed network senders, never a firmware version.
"""
from datetime import datetime, timezone
import hashlib
import http.cookiejar
import json
from pathlib import Path
import socket
import subprocess
import sys
import threading
import time
import urllib.request


def discovery_metadata(qos):
    """Keep only named identity properties; DDS self-reports are not attestation."""
    properties, policy_types = [], []
    safe = {'__Hostname', '__ProcessName', '__Pid', '__ProcessId', '__ExecName'}
    for policy in qos:
        policy_types.append(type(policy).__qualname__)
        if type(policy).__qualname__ != 'Policy.Property':
            continue
        name = policy.key
        row = dict(name=name)
        if name in safe and isinstance(policy.value, str) and len(policy.value) <= 256:
            row['value'] = policy.value
        # Never retain arbitrary properties, userdata or credentials.
        properties.append(row)
    return dict(policy_types=policy_types, properties=properties,
                authority='DDS participant self-reported metadata; executable and component version unverified')


def known_read_paths():
    """Two known ports, no application payload; never run OTA against a peer."""
    checks = []
    for port in (22, 1026):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(2)
        started = time.monotonic()
        try:
            sock.connect(('192.168.123.161', port))
            status, error = 'connected', None
        except OSError as exc:
            status, error = type(exc).__name__, exc.errno
        finally:
            sock.close()
        checks.append(dict(port=port, result=status, errno=error,
                           elapsed_ms=round((time.monotonic()-started)*1000, 2), application_bytes_sent=0))
    binary = Path('/unitree/sbin/ota_pipe_cli')
    return dict(target='192.168.123.161', checked_at_utc=datetime.now(timezone.utc).isoformat(),
        known_ports=checks, ota_hosts=json.loads(Path('/unitree/etc/ota_pipe/ota_pipe_hosts.json').read_text()),
        ota_binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),
        peer_ota_queries=0, ota_mutations=0, configuration_writes=0,
        limitation='Port checks do not enumerate other services or prove the absence of another version interface.')


def rtps_header(data):
    """Keep only a validated RTPS header; never retain application payloads."""
    if len(data) < 20 or data[:4] != b'RTPS' or data[4] != 2:
        return None
    return dict(guid_prefix=data[8:20].hex(), protocol=list(data[4:6]),
                vendor_id=data[6:8].hex())


def inspect(duration=24):
    from cyclonedds.builtin import BuiltinDataReader, BuiltinTopicDcpsPublication, BuiltinTopicDcpsParticipant
    from cyclonedds.domain import Domain, DomainParticipant
    from cyclonedds.pub import DataWriter
    from cyclonedds.sub import DataReader
    from cyclonedds.topic import Topic
    from unitree_sdk2py.idl.unitree_api.msg.dds_ import (
        Request_, Response_, RequestHeader_, RequestIdentity_, RequestLease_, RequestPolicy_)

    if duration != 24:
        raise ValueError('Fixed bounded observation only')
    root = Path('/home/unitree/workspace/g1-body-console')
    release = (root/'active-release').read_text().strip()
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}),
        urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar()))
    base = 'http://127.0.0.1:8765'
    opener.open(base+'/api/session', timeout=5).read()

    def guard():
        s = json.load(opener.open(base+'/api/state', timeout=5))
        assert s['connected'] and s['mode'] == 'robot'
        assert s['control']['owner'] is None and not s['manual_remote']['armed']
        assert s.get('navigation', {}).get('status') not in ('running', 'executing')
        return dict(owner=s['control']['owner'], armed=s['manual_remote']['armed'],
                    fsm=s.get('telemetry', {}).get('fsm'))

    def processes():
        vendor, sport = [], []
        for p in Path('/proc').iterdir():
            if not p.name.isdigit():
                continue
            try:
                first = (p/'cmdline').read_bytes().split(b'\0')[0].decode(errors='replace')
                comm = (p/'comm').read_text().strip()
                ticks = int((p/'stat').read_text().rsplit(')', 1)[1].split()[19])
                row = dict(pid=int(p.name), exe=first, comm=comm, start_ticks=ticks)
                if first.startswith(('/unitree/', '/home/unitree/megaflow_sdk/')):
                    vendor.append(row)
                if 'sport' in comm.lower() or 'ai_sport' in first.lower():
                    sport.append(row)
            except (OSError, ValueError, IndexError):
                continue
        return sorted(vendor, key=lambda r:r['pid']), sport

    begin = datetime.now(timezone.utc).isoformat()
    before, (official, local_sport) = guard(), processes()
    network = {}
    for key, cmd in [('interfaces', ['ip','-br','-4','addr']),
                     ('neighbors', ['ip','neigh','show','dev','eth0'])]:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=4)
        network[key] = dict(code=p.returncode, output=p.stdout.strip())
    # Membership belongs to this socket only; no persistent network changes.
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    seen, capture_error = {}, []
    stop = threading.Event()

    def receive():
        while not stop.is_set():
            try:
                packet, address = sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError as exc:
                if not stop.is_set():
                    capture_error.append(type(exc).__name__)
                break
            header = rtps_header(packet)
            if header:
                key = header['guid_prefix'], address[0]
                row = seen.setdefault(key, dict(**header, source_ip=address[0],
                    packets=0, source_ports=[]))
                row['packets'] += 1
                if address[1] not in row['source_ports']:
                    row['source_ports'].append(address[1])

    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('', 7400))
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                       socket.inet_aton('239.255.0.1')+socket.inet_aton('192.168.123.164'))
        sock.settimeout(.25)
        receiver = threading.Thread(target=receive, daemon=True)
        receiver.start()
        # A fresh participant, using only this process's eth0 configuration.
        domain = Domain(0, '<CycloneDDS><Domain Id="0"><General><Interfaces>'
            '<NetworkInterface name="eth0"/></Interfaces></General></Domain></CycloneDDS>')
        participant = DomainParticipant(0)
        discovery = BuiltinDataReader(participant, BuiltinTopicDcpsPublication)
        participant_discovery = BuiltinDataReader(participant, BuiltinTopicDcpsParticipant)
        channels = {service: (
            DataWriter(participant, Topic(participant, f'rt/api/{service}/request', Request_)),
            DataReader(participant, Topic(participant, f'rt/api/{service}/response', Response_)))
            for service in ('sport', 'motion_switcher', 'robot_state')}
        until = time.monotonic()+duration
        time.sleep(1)
        queries = []
        fixed = [('sport', 1, '{}'), ('sport', 7001, ''), ('sport', 7002, ''),
                 ('sport', 7003, ''), ('sport', 7004, ''), ('sport', 7005, ''),
                 ('sport', 7006, ''), ('motion_switcher', 1001, '{}'),
                 ('robot_state', 1003, '{}'), ('robot_state', 1006, '{}')]
        for service, api, parameter in fixed:
            guard()
            writer, reader = channels[service]
            identity = RequestIdentity_(time.monotonic_ns(), api)
            start = time.monotonic()
            writer.write(Request_(RequestHeader_(identity, RequestLease_(0),
                RequestPolicy_(0, False)), parameter, []))
            replies = []
            # Observe all replies in a bounded window, not just the first one.
            while time.monotonic()-start < 1:
                for msg in reader.take(50):
                    if not msg.sample_info.valid_data or msg.header.identity.id != identity.id:
                        continue
                    code = msg.header.status.code
                    replies.append(dict(api_id=msg.header.identity.api_id, code=code,
                        raw=msg.data if code == 0 and len(msg.data.encode()) <= 8192 else None,
                        publication_handle=msg.sample_info.publication_handle,
                        elapsed_ms=round((time.monotonic()-start)*1000, 2)))
                time.sleep(.005)
            queries.append(dict(service=service, api_id=api, request_id=identity.id, replies=replies))
        publications, participants = {}, {}
        while time.monotonic() < until:
            for sample in participant_discovery.take(1000):
                if sample.sample_info.valid_data:
                    participants[str(sample.key)] = dict(key=str(sample.key), **discovery_metadata(sample.qos))
            for sample in discovery.take(1000):
                if sample.sample_info.valid_data and sample.topic_name in {
                    f'rt/api/{s}/response' for s in channels}:
                    publications[str(sample.key)] = dict(key=str(sample.key), topic=sample.topic_name,
                        participant_key=str(sample.participant_key),
                        instance_handle=sample.sample_info.instance_handle)
            time.sleep(.05)
    finally:
        stop.set()
        sock.close()
        if 'receiver' in locals():
            receiver.join(1)
    after, (official_after, _) = guard(), processes()
    paths = known_read_paths()
    guard()
    mappings = list(seen.values())
    for pub in publications.values():
        prefix = pub['participant_key'].replace('-', '')[:24]
        pub['observed_rtps_source_ips'] = sorted({r['source_ip'] for r in mappings
                                                if r['guid_prefix'] == prefix})
    for query in queries:
        for reply in query['replies']:
            reply['matching_publications'] = [p for p in publications.values()
                if p['instance_handle'] == reply['publication_handle']]
    return dict(current=False, checked_at_utc=begin,
        completed_at_utc=datetime.now(timezone.utc).isoformat(), release=release,
        boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
        before=before, after=after, official_processes_before=official,
        official_processes_after=official_after, official_processes_unchanged=official == official_after,
        local_sport_process_matches=local_sport, network=network,
        passive_rtps=dict(multicast='239.255.0.1:7400', interface='eth0', duration_s=duration,
            payloads_retained=False, errors=capture_error, senders=mappings),
        queries=queries, response_publishers=list(publications.values()),
        response_participants=[p for key, p in participants.items()
                               if key in {pub['participant_key'] for pub in publications.values()}],
        known_read_paths=paths,
        device_motion_firmware=None, command_writes=0, lease_requests=0,
        limitations=['RTPS source IP identifies an observed network sender, not a verified physical board or executable.',
            'Service status 0 means on in the official G1 documentation, not LocoState readiness or balance safety.',
            'Absence from bounded local process enumeration does not exclude renamed or inaccessible processes.',
            'Only successful payloads are retained; error placeholders are not readings.',
            'No mode changes were used to investigate the internal 7301 predicate.'])


if __name__ == '__main__':
    root = Path('/home/unitree/workspace/g1-body-console')
    sys.path.insert(0, str(root/'releases'/(root/'active-release').read_text().strip()))
    print(json.dumps(inspect(), ensure_ascii=False, indent=2))
