"""Collect every correlated reply, plus existing string state topics. Read only."""
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
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
from cyclonedds.builtin import BuiltinDataReader, BuiltinTopicDcpsPublication
from cyclonedds.domain import DomainParticipant
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic

opener = urllib.request.build_opener(urllib.request.ProxyHandler({}),
    urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar()))
base = 'http://127.0.0.1:8765'
opener.open(base + '/api/session', timeout=5).read()


def guard():
    s = json.load(opener.open(base + '/api/state', timeout=5))
    assert s['connected'] and s['mode'] == 'robot'
    assert s['control']['owner'] is None and not s['manual_remote']['armed']
    assert s.get('navigation', {}).get('status') not in ('running', 'executing')


guard()
begin = datetime.now(timezone.utc).isoformat()
ChannelFactoryInitialize(0, 'eth0')
participant = DomainParticipant(0)
discovery = BuiltinDataReader(participant, BuiltinTopicDcpsPublication)
readers = {name: DataReader(participant, Topic(participant, name, String_))
           for name in ('rt/servicestate', 'rt/multiplestate', 'rt/servicestateactivate')}
writer = ChannelFactory().CreateChannel('rt/api/sport/request', Request_)
reader = ChannelFactory().CreateChannel('rt/api/sport/response', Response_)
reader.SetReader()
writer.SetWriter()
time.sleep(1)
queries, publications, states = [], {}, {name: dict(samples=0, values=[]) for name in readers}
try:
    for api in (1, 7001, 7002, 7003, 7004, 7005, 7006):
        guard()
        identity = RequestIdentity_(time.monotonic_ns(), api)
        req = Request_(RequestHeader_(identity, RequestLease_(0), RequestPolicy_(0, False)), '{}' if api == 1 else '', [])
        start = time.monotonic()
        sent = writer.Write(req, 1)
        replies = []
        while (left := 1 - (time.monotonic() - start)) > 0:
            msg = reader.Read(left)
            if msg is None:
                break
            if msg.header.identity.id == identity.id:
                code = msg.header.status.code
                replies.append(dict(api_id=msg.header.identity.api_id, code=code,
                    raw=msg.data if code == 0 and len(msg.data.encode()) <= 8192 else None,
                    publication_handle=msg.sample_info.publication_handle,
                    elapsed_ms=round((time.monotonic() - start) * 1000, 2)))
        queries.append(dict(api_id=api, request_id=identity.id, sent=sent, replies=replies))
    until = time.monotonic() + 8
    while time.monotonic() < until:
        for sample in discovery.take(500):
            if sample.sample_info.valid_data:
                publications[str(sample.key)] = dict(topic=sample.topic_name, type=sample.type_name,
                    participant_key=str(sample.participant_key), instance_handle=sample.sample_info.instance_handle)
        for name, sub in readers.items():
            for sample in sub.take(50):
                if not sample.sample_info.valid_data:
                    continue
                row = states[name]
                row['samples'] += 1
                if len(sample.data.encode()) <= 65536 and sample.data not in row['values'] and len(row['values']) < 4:
                    row['values'].append(sample.data)
        time.sleep(.02)
finally:
    reader.CloseReader()
    writer.CloseWriter()
guard()
print(json.dumps(dict(current=False, checked_at_utc=begin,
    completed_at_utc=datetime.now(timezone.utc).isoformat(),
    boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
    queries=queries, string_states=states,
    response_publishers=[p for p in publications.values() if p['topic'] == 'rt/api/sport/response'],
    command_writes=0, lease_requests=0, device_motion_firmware=None), ensure_ascii=False, indent=2))
