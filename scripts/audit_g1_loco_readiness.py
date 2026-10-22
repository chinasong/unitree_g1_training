"""Orin-only fixed read audit. No leases, motion calls, service changes or retries.

Run via the console venv's python stdin. API 1 is GetServerApiVersion in
the pinned official rpc/internal.py; it is an API version, never firmware.
"""
from pathlib import Path
from datetime import datetime, timezone
import http.cookiejar
import json
import sys
import time
import urllib.request

root = Path('/home/unitree/workspace/g1-body-console')
release = root / 'releases' / (root / 'active-release').read_text().strip()
sys.path.insert(0, str(release))
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelFactory
from unitree_sdk2py.idl.unitree_api.msg.dds_ import (
    Request_, Response_, RequestHeader_, RequestIdentity_, RequestLease_, RequestPolicy_)
from cyclonedds.builtin import BuiltinDataReader, BuiltinTopicDcpsPublication
from cyclonedds.domain import DomainParticipant

opener = urllib.request.build_opener(urllib.request.ProxyHandler({}),
    urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar()))
base = 'http://127.0.0.1:8765'
opener.open(base + '/api/session', timeout=5).read()


def guard():
    state = json.load(opener.open(base + '/api/state', timeout=5))
    assert state['connected'] and state['mode'] == 'robot'
    assert state['control']['owner'] is None and not state['manual_remote']['armed']
    assert state.get('navigation', {}).get('status') not in ('running', 'executing')
    return dict(fsm=state.get('telemetry', {}).get('fsm'), control_owner=state['control']['owner'],
                armed=state['manual_remote']['armed'])


def processes():
    rows = []
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            argv = (entry / 'cmdline').read_bytes().split(b'\0')
            exe = argv[0].decode(errors='replace')
            if exe.startswith(('/unitree/', '/home/unitree/megaflow_sdk/')):
                tail = (entry / 'stat').read_text().rsplit(')', 1)[1].split()
                rows.append(dict(pid=int(entry.name), start_ticks=int(tail[19]), exe=exe))
        except (OSError, ValueError, IndexError):
            continue
    return sorted(rows, key=lambda row: row['pid'])


begin = datetime.now(timezone.utc).isoformat()
before, identities = guard(), processes()
ChannelFactoryInitialize(0, 'eth0')
participant = DomainParticipant(0)
discovery = BuiltinDataReader(participant, BuiltinTopicDcpsPublication)
channels = {}
for service in ('sport', 'motion_switcher', 'robot_state'):
    writer = ChannelFactory().CreateChannel(f'rt/api/{service}/request', Request_)
    reader = ChannelFactory().CreateChannel(f'rt/api/{service}/response', Response_)
    reader.SetReader()
    writer.SetWriter()
    channels[service] = writer, reader
time.sleep(1)
rows = []
fixed_reads = [('sport', 1, '{}'), ('motion_switcher', 1, '{}'),
               ('robot_state', 1, '{}'), ('sport', 7001, ''),
               ('sport', 7002, ''), ('sport', 7003, ''), ('sport', 7004, ''),
               ('sport', 7005, ''), ('sport', 7006, ''),
               ('motion_switcher', 1001, '{}'), ('robot_state', 1006, '{}')]
try:
    for trial in range(3):
        for service, api, parameter in fixed_reads:
            guard()
            writer, reader = channels[service]
            identity = RequestIdentity_(time.monotonic_ns(), api)
            request = Request_(RequestHeader_(identity, RequestLease_(0), RequestPolicy_(0, False)), parameter, [])
            start = time.monotonic()
            code, raw = 3104, None
            if not writer.Write(request, 1):
                code = 3102
            else:
                while (left := 1 - (time.monotonic() - start)) > 0:
                    response = reader.Read(left)
                    if response is None:
                        break
                    if response.header.identity.id != identity.id:
                        continue
                    code = response.header.status.code if response.header.identity.api_id == api else 3105
                    # Error placeholders are deliberately not retained as readings.
                    if code == 0:
                        raw = response.data if len(response.data.encode()) <= 8192 else None
                    break
            rows.append(dict(trial=trial, service=service, api_id=api,
                request_id=identity.id, code=code, raw=raw,
                elapsed_ms=round((time.monotonic() - start) * 1000, 2)))
finally:
    for writer, reader in channels.values():
        reader.CloseReader()
        writer.CloseWriter()

found = {}
until = time.monotonic() + 10
while time.monotonic() < until:
    for sample in discovery.take(500):
        if sample.sample_info.valid_data:
            found[str(sample.key)] = dict(topic=sample.topic_name, type=sample.type_name,
                participant_key=str(sample.participant_key))
    time.sleep(.05)
after, identities_after = guard(), processes()
print(json.dumps(dict(current=False, checked_at_utc=begin,
    completed_at_utc=datetime.now(timezone.utc).isoformat(), release=release.name,
    boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
    before=before, after=after, official_processes_before=identities,
    official_processes_after=identities_after, official_processes_unchanged=identities == identities_after,
    reads=rows, publications=sorted(found.values(), key=lambda row: (row['topic'], row['type'])),
    device_motion_firmware=None, command_writes=0, lease_requests=0,
    limits=['API version is a protocol identifier, not motion firmware.',
            'Bounded DDS discovery does not prove an internal LocoState absent.',
            'Participant GUIDs do not establish the physical host or firmware.',
            'No mode was changed to test whether 7301 would disappear.']), ensure_ascii=False, indent=2))
