"""Orin-only audit of installed SDK Init and fixed getters; no motion writes.

Uses the robot's console venv. The RPC observer independently correlates replies
to distinguish a server rejection from an SDK callback timeout. Odometry is a
separate public message and never treated as proof of the private LocoState.
"""
from dataclasses import asdict
from datetime import datetime, timezone
from hashlib import sha256
import http.cookiejar
import inspect
import json
from pathlib import Path
import sys
import time
import urllib.request

root = Path('/home/unitree/workspace/g1-body-console')
release = root / 'releases' / (root / 'active-release').read_text().strip()
sys.path.insert(0, str(release))
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from unitree_sdk2py.rpc.client_base import ClientBase
from unitree_sdk2py.rpc.client_stub import ClientStub
from unitree_sdk2py.idl.unitree_api.msg.dds_ import Response_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_ as GoSport
from g1_console.hg_sport_state import SportModeState_ as HgSport
from cyclonedds.builtin import BuiltinDataReader, BuiltinTopicDcpsPublication
from cyclonedds.domain import DomainParticipant
from cyclonedds.qos import Qos, Policy
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic

opener = urllib.request.build_opener(urllib.request.ProxyHandler({}),
    urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar()))
opener.open('http://127.0.0.1:8765/api/session', timeout=5).read()


def guard():
    state = json.load(opener.open('http://127.0.0.1:8765/api/state', timeout=5))
    assert state['connected'] and state['mode'] == 'robot'
    assert state['control']['owner'] is None and not state['manual_remote']['armed']
    assert state.get('navigation', {}).get('status') not in ('running', 'executing')
    return dict(fsm=state.get('telemetry', {}).get('fsm'),
        owner=state['control']['owner'], armed=state['manual_remote']['armed'])


started = datetime.now(timezone.utc).isoformat()
before = guard()
requests = []
original_send = ClientStub.SendRequest


def guarded_send(self, request, timeout):
    guard()
    assert request.header.identity.api_id in (1, 7001, 7002, 7003, 7004, 7005, 7006)
    assert request.parameter in ('', '{}') and not request.binary
    assert request.header.lease.id == 0
    assert request.header.policy.priority == 0 and not request.header.policy.noreply
    requests.append(dict(id=request.header.identity.id,
        api_id=request.header.identity.api_id, parameter=request.parameter))
    return original_send(self, request, timeout)


ClientStub.SendRequest = guarded_send
ChannelFactoryInitialize(0, 'eth0')
participant = DomainParticipant(0)
discovery = BuiltinDataReader(participant, BuiltinTopicDcpsPublication)
observer = DataReader(participant, Topic(participant, 'rt/api/sport/response', Response_))
client = LocoClient()
constructor_requests = len(requests)
client.SetTimeout(1)
client.Init()
init_requests = len(requests) - constructor_requests
assert constructor_requests == init_requests == 0
sources = []
for cls in (LocoClient, ClientBase, ClientStub):
    path = Path(inspect.getfile(cls))
    sources.append(dict(class_name=cls.__name__, path=str(path),
        sha256=sha256(path.read_bytes()).hexdigest()))

publications = {}
until = time.monotonic() + 3
while time.monotonic() < until:
    for sample in discovery.take(500):
        if sample.sample_info.valid_data:
            publications[str(sample.key)] = dict(topic=sample.topic_name,
                type=sample.type_name, participant_key=str(sample.participant_key))
    time.sleep(.025)

rows = []
for api in (1, 7001, 7002, 7003, 7004, 7005, 7006):
    guard()
    started_call = time.monotonic()
    if api == 1:
        code, value = client.GetServerApiVersion()
    elif api == 7001:
        code, value = client.GetFsmId()
    else:
        code, value = client._Call(api, '')
    request = requests[-1]
    assert request['api_id'] == api
    correlated = []
    until = time.monotonic() + .15
    while time.monotonic() < until:
        for response in observer.take(500):
            if not hasattr(response, 'header'):
                continue
            if response.header.identity.id != request['id']:
                continue
            assert response.header.identity.api_id == api
            server_code = response.header.status.code
            correlated.append(dict(code=server_code,
                data=response.data if server_code == 0 else None))
        time.sleep(.005)
    rows.append(dict(api_id=api, request_id=request['id'], sdk_code=code,
        sdk_value=value if code == 0 else None, replies=correlated,
        elapsed_ms=round((time.monotonic() - started_call) * 1000, 2)))

# Subscribe only to discovered topics with exactly the official public type.
# The Go message's four foot_force slots are not a G1 two-foot load contract.
types = {'unitree_go::msg::dds_::SportModeState_': GoSport,
         'unitree_hg::msg::dds_::SportModeState_': HgSport}
allowed_topics = {'rt/odommodestate', 'rt/lf/odommodestate',
                  'rt/sportmodestate', 'rt/lf/sportmodestate'}
qos = Qos(Policy.Reliability.BestEffort, Policy.Durability.Volatile,
          Policy.History.KeepLast(20))
streams = {}
for pub in publications.values():
    name, typename = pub['topic'], pub['type']
    if name in allowed_topics and typename in types and name not in streams:
        streams[name] = dict(reader=DataReader(participant,
            Topic(participant, name, types[typename]), qos),
            type=typename, samples=0, first=None, last=None)
until = time.monotonic() + 12
while time.monotonic() < until:
    guard()
    for row in streams.values():
        for sample in row['reader'].take(200):
            if not hasattr(sample, '__dataclass_fields__'):
                continue
            value = asdict(sample)
            if 'path_point' in value:
                value.pop('path_point')
            row['samples'] += 1
            row['first'] = row['first'] or value
            row['last'] = value
    time.sleep(.025)
for row in streams.values():
    row.pop('reader')
after = guard()
print(json.dumps(dict(schema_version=1, current=False, checked_at_utc=started,
    completed_at_utc=datetime.now(timezone.utc).isoformat(), release=release.name,
    boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
    before=before, after=after, sources=sources,
    installed_init_source=inspect.getsource(LocoClient.Init),
    constructor_rpc_requests=constructor_requests, init_rpc_requests=init_requests,
    sdk_getters=rows, streams=streams,
    publications=sorted(publications.values(), key=lambda p: (p['topic'], p['type'])),
    command_writes=0, lease_requests=0, device_motion_firmware=None,
    limits=['Only fixed official read APIs 1 and 7001-7006 were sent.',
        'Public odometry does not establish private server LocoState availability.',
        'Four generic foot_force slots are not validated G1 bilateral load readings.',
        'Init registration of setters is local and does not execute them.',
        'No mode transition was attempted to probe server readiness.']), ensure_ascii=False, indent=2))
