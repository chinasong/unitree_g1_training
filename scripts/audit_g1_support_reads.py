"""Linux-only fixed read-only research probe; see G1_MODE_READBACK_AND_STANDING_AUDIT.md."""
from pathlib import Path
from dataclasses import asdict
import datetime,json,sys,time
root=Path('/home/unitree/workspace/g1-body-console')
release=root/'releases'/(root/'active-release').read_text().strip()
sys.path.insert(0,str(release))
from unitree_sdk2py.core.channel import ChannelFactoryInitialize,ChannelFactory
from unitree_sdk2py.idl.unitree_api.msg.dds_ import Request_,Response_,RequestHeader_,RequestIdentity_,RequestLease_,RequestPolicy_
from cyclonedds.domain import DomainParticipant
from cyclonedds.builtin import BuiltinDataReader,BuiltinTopicDcpsPublication
from cyclonedds.topic import Topic
from cyclonedds.sub import DataReader
from cyclonedds.qos import Qos,Policy
ChannelFactoryInitialize(0,'eth0')
participant=DomainParticipant(0)
discovery=BuiltinDataReader(participant,BuiltinTopicDcpsPublication)
found={};start=time.monotonic()
while time.monotonic()-start<10:
 for s in discovery.take(500):
  if s.sample_info.valid_data:found[str(s.key)]=dict(topic=s.topic_name,type=s.type_name)
 time.sleep(.05)
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_ as GoSport
from g1_console.hg_sport_state import SportModeState_ as HgSport
readers={};observations={}
for topic,cls in [('rt/odommodestate',GoSport),('rt/lf/sportmodestate',HgSport)]:
 typ=cls.__idl_typename__.replace('.','::')
 advertised=[r for r in found.values() if r['topic']==topic]
 observations[topic]=dict(type=typ,advertised=advertised,samples=0,last=None,physical_source_verified=False)
 if any(r['type']==typ for r in advertised):
  readers[topic]=DataReader(participant,Topic(participant,topic,cls),qos=Qos(Policy.Reliability.BestEffort,Policy.Durability.Volatile,Policy.History.KeepLast(1)))
until=time.monotonic()+10
while time.monotonic()<until:
 for topic,reader in readers.items():
  for sample in reader.take(4):
   if sample.sample_info.valid_data:
    row=observations[topic];row['samples']+=1
    raw=asdict(sample)
    row['last']={k:raw[k] for k in ('foot_force','foot_position_body','foot_speed_body','body_height') if k in raw} if topic=='rt/odommodestate' else raw
    row['received_at']=time.time()
 time.sleep(.05)
writer=ChannelFactory().CreateChannel('rt/api/sport/request',Request_)
reader=ChannelFactory().CreateChannel('rt/api/sport/response',Response_)
reader.SetReader();writer.SetWriter();getters=[]
for api,name in [(7003,'GetBalanceMode'),(7004,'GetSwingHeight'),(7005,'GetStandHeight')]:
 identity=RequestIdentity_(time.monotonic_ns(),api)
 req=Request_(RequestHeader_(identity,RequestLease_(0),RequestPolicy_(0,False)),'',[])
 begin=time.monotonic();code=3104;raw=None
 if not writer.Write(req,.7):code=3102
 else:
  while time.monotonic()-begin<.7:
   response=reader.Read(max(.00001,.7-(time.monotonic()-begin)))
   if response is None:break
   if response.header.identity.id!=identity.id:continue
   code=response.header.status.code if response.header.identity.api_id==api else 3105
   raw=response.data if code==0 else None;break
 getters.append(dict(api_id=api,method=name,parameter='',code=code,raw=raw,received_at=time.time()))
reader.CloseReader();writer.CloseWriter()
print(json.dumps(dict(current=False,completed_at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),duration_s=round(time.monotonic()-start,2),discovery_seconds=10,sample_seconds=10,total_publications=len(found),relevant_publications=sorted([r for r in found.values() if any(w in r['topic'].lower() for w in ('foot','contact','force','press','odom','sportmodestate','lowstate','wrench'))],key=lambda r:r['topic']),observations=observations,getters=getters,rpc_calls=3,command_writes=0,lease_requests=0,motor_configuration_values_obtained=0,limitations=['bounded discovery does not prove a sensor/interface is absent','GO four-foot arrays are not verified G1 left/right contact or load','getter errors are not numeric defaults and do not authorize changing state']),ensure_ascii=False,indent=2))
