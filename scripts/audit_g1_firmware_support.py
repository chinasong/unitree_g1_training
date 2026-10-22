"""Passive, type-matched DDS firmware/support survey, run on Orin via python stdin."""
from pathlib import Path
from dataclasses import asdict
from datetime import datetime, timezone
import json,sys,time
root=Path('/home/unitree/workspace/g1-body-console')
release=root/'releases'/(root/'active-release').read_text().strip()
sys.path.insert(0,str(release))
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_,MainBoardState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from cyclonedds.domain import DomainParticipant
from cyclonedds.builtin import BuiltinDataReader,BuiltinTopicDcpsPublication
from cyclonedds.topic import Topic
from cyclonedds.sub import DataReader
from cyclonedds.qos import Qos,Policy
begin=datetime.now(timezone.utc).isoformat()
ChannelFactoryInitialize(0,'eth0')
participant=DomainParticipant(0)
discovery=BuiltinDataReader(participant,BuiltinTopicDcpsPublication)
found={};start=time.monotonic()
while time.monotonic()-start<10:
 for sample in discovery.take(500):
  if sample.sample_info.valid_data:found[str(sample.key)]=dict(topic=sample.topic_name,type=sample.type_name)
 time.sleep(.05)
readers={};rows={}
for topic,cls in [('rt/lf/lowstate',LowState_),('rt/lf/mainboardstate',MainBoardState_),('rt/odommodestate',SportModeState_)]:
 typ=cls.__idl_typename__.replace('.','::')
 advertised=[r for r in found.values() if r['topic']==topic]
 rows[topic]=dict(type=typ,advertised=advertised,samples=0,last=None,physical_source_verified=False)
 if any(r['type']==typ for r in advertised):
  readers[topic]=DataReader(participant,Topic(participant,topic,cls),qos=Qos(Policy.Reliability.BestEffort,Policy.Durability.Volatile,Policy.History.KeepLast(1)))
until=time.monotonic()+10
while time.monotonic()<until:
 for topic,reader in readers.items():
  for sample in reader.take(4):
   if not sample.sample_info.valid_data:continue
   raw=asdict(sample);row=rows[topic];row['samples']+=1
   fields=('version','mode_pr','mode_machine','tick') if topic=='rt/lf/lowstate' else ('foot_force','foot_position_body','foot_speed_body','body_height') if topic=='rt/odommodestate' else tuple(raw)
   row['last']={k:raw[k] for k in fields if k in raw};row['received_at_utc']=datetime.now(timezone.utc).isoformat()
 time.sleep(.05)
print(json.dumps(dict(current=False,checked_at_utc=begin,completed_at_utc=datetime.now(timezone.utc).isoformat(),boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),duration_s=round(time.monotonic()-start,3),total_publications=len(found),relevant_publications=sorted([r for r in found.values() if any(w in r['topic'].lower() for w in ('foot','force','contact','press','wrench','version','firmware','mainboard','lowstate','odom'))],key=lambda r:r['topic']),observations=rows,rpc_calls=0,command_writes=0,lease_requests=0,motor_configuration_values_obtained=0,limitations=['raw version uint32[2] has no verified decoding to motion firmware','mainboard fields have no firmware version member in pinned HG IDL','GO four-foot arrays have no verified G1 left/right force mapping','bounded discovery does not prove an interface absent','receipt time is not sensor acquisition time']),ensure_ascii=False,indent=2))
