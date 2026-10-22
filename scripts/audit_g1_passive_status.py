"""Bounded passive inspection of advertised String status topics; no RPC or writers."""
from datetime import datetime, timezone
from pathlib import Path
import hashlib
import json
import re
import sys
import time

root = Path('/home/unitree/workspace/g1-body-console')
sys.path.insert(0, str(root/'releases'/(root/'active-release').read_text().strip()))
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
from cyclonedds.domain import DomainParticipant
from cyclonedds.builtin import BuiltinDataReader, BuiltinTopicDcpsPublication
from cyclonedds.topic import Topic
from cyclonedds.sub import DataReader
from cyclonedds.qos import Qos, Policy

TOPICS = ('rt/multiplestate', 'rt/selftest', 'rt/servicestate', 'rt/servicestateactivate',
          'rt/arm/action/state', 'rt/arm_service_outbound')
start_utc = datetime.now(timezone.utc).isoformat()
ChannelFactoryInitialize(0, 'eth0')
participant = DomainParticipant(0)
discovery = BuiltinDataReader(participant, BuiltinTopicDcpsPublication)
found = {}
begin = time.monotonic()
while time.monotonic()-begin < 5:
    for sample in discovery.take(500):
        if sample.sample_info.valid_data and sample.topic_name in TOPICS:
            found[(sample.topic_name, sample.type_name)] = True
    time.sleep(.05)
readers = {}
rows = {}
for topic in TOPICS:
    expected = String_.__idl_typename__.replace('.', '::')
    rows[topic] = dict(samples=0, expected_type=expected,
                       advertised_types=[typ for name, typ in found if name == topic])
    if (topic, expected) in found:
        readers[topic] = DataReader(participant, Topic(participant, topic, String_),
            qos=Qos(Policy.Reliability.BestEffort, Policy.Durability.Volatile, Policy.History.KeepLast(1)))


def describe(value, path='', depth=0):
    """Save schema and narrow evidence fields, not arbitrary personal/status text."""
    if depth > 7:
        return []
    if isinstance(value, dict):
        return [row for key, child in list(value.items())[:120]
                for row in describe(child, f'{path}.{key}', depth+1)]
    if isinstance(value, list):
        return [row for n, child in enumerate(value[:40])
                for row in describe(child, f'{path}[{n}]', depth+1)]
    row = dict(path=path, type=type(value).__name__)
    if re.search(r'version|firmware|contact|foot.?force|load|balance|fsm|task.?id', path, re.I):
        if value is None or isinstance(value, (int, float, bool)):
            row['raw'] = value
        elif isinstance(value, str) and re.fullmatch(r'[\w. +:/-]{0,80}', value):
            row['raw'] = value
    return [row]


until = time.monotonic()+15
while time.monotonic() < until:
    for topic, reader in readers.items():
        for sample in reader.take(8):
            if not sample.sample_info.valid_data:
                continue
            raw = sample.data.encode()
            row = rows[topic]
            row.update(samples=row['samples']+1, bytes=len(raw), sha256=hashlib.sha256(raw).hexdigest(),
                       received_at_utc=datetime.now(timezone.utc).isoformat())
            try:
                row['json_schema_and_selected_fields'] = describe(json.loads(raw))[:300]
                row['valid_json'] = True
            except (ValueError, UnicodeError):
                row['valid_json'] = False
    time.sleep(.05)
print(json.dumps(dict(current=False, checked_at_utc=start_utc,
    completed_at_utc=datetime.now(timezone.utc).isoformat(),
    boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
    observations=rows, rpc_calls=0, command_writes=0, lease_requests=0,
    limitations=['bounded silence does not prove absence or inactivity',
                 'field names and raw values do not establish units, hardware source, or firmware semantics',
                 'receipt time is not sensor acquisition time']), ensure_ascii=False, indent=2))
