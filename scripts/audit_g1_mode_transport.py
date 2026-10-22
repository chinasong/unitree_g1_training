"""Linux-only fixed read-only research probe; see G1_MODE_READBACK_AND_STANDING_AUDIT.md."""
from pathlib import Path
import collections, datetime, json, sys, time
from dataclasses import asdict
root=Path('/home/unitree/workspace/g1-body-console')
release=root/'releases'/(root/'active-release').read_text().strip()
sys.path.insert(0,str(release))
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelFactory
from unitree_sdk2py.idl.unitree_api.msg.dds_ import Request_, Response_, RequestHeader_, RequestIdentity_, RequestLease_, RequestPolicy_
from cyclonedds.domain import DomainParticipant
from cyclonedds.topic import Topic
from cyclonedds.sub import DataReader
from cyclonedds.qos import Qos, Policy
ChannelFactoryInitialize(0,'eth0')
participant=DomainParticipant(0)
qos=Qos(Policy.Reliability.BestEffort,Policy.Durability.Volatile,Policy.History.KeepLast(256))
monitors={(service,direction):DataReader(participant,Topic(participant,'rt/api/'+service+'/'+direction,typ),qos=qos) for service in ('sport','motion_switcher') for direction,typ in [('request',Request_),('response',Response_)]}
clients={}
for service in ('sport','motion_switcher'):
 req=ChannelFactory().CreateChannel('rt/api/'+service+'/request',Request_)
 resp=ChannelFactory().CreateChannel('rt/api/'+service+'/response',Response_)
 resp.SetReader();req.SetWriter();clients[service]=(req,resp)
seen_requests={};responses={};own=[];rows=[];start=time.monotonic();next_call=start+2
def observe():
 for (service,direction),reader in monitors.items():
  for msg in reader.take(256):
   if not msg.sample_info.valid_data:continue
   api=msg.header.identity.api_id
   if (service,api) not in {('sport',7001),('sport',7002),('motion_switcher',1001)}:continue
   key=(service,msg.header.identity.id)
   row=dict(service=service,direction=direction,id=key[1],api=api,at=time.time(),monotonic=time.monotonic())
   if direction=='request':
    row.update(lease=msg.header.lease.id,parameter=msg.parameter);seen_requests[key]=row
   else:
    row.update(code=msg.header.status.code,data=msg.data);responses[key]=row
   rows.append(row)
try:
 while time.monotonic()-start<60:
  observe()
  if time.monotonic()>=next_call:
   for service,api,parameter in [('sport',7001,'{}'),('sport',7002,''),('motion_switcher',1001,'{}')]:
    req,resp=clients[service];identity=RequestIdentity_(time.monotonic_ns(),api)
    request=Request_(RequestHeader_(identity,RequestLease_(0),RequestPolicy_(0,False)),parameter,[])
    t=time.monotonic();code=3104;data=None
    if not req.Write(request,.7):code=3102
    else:
     while time.monotonic()-t<.7:
      msg=resp.Read(max(.00001,.7-(time.monotonic()-t)))
      if msg is None:break
      if msg.header.identity.id!=identity.id:continue
      code=msg.header.status.code if msg.header.identity.api_id==api else 3105
      data=msg.data if code==0 else None;break
    own.append(dict(service=service,api=api,id=identity.id,code=code,data=data,request_ms=round((time.monotonic()-t)*1000,2),at=time.time()))
   next_call=time.monotonic()+3
  time.sleep(.005)
 observe()
finally:
 for req,resp in clients.values():resp.CloseReader();req.CloseWriter()
own_ids={(r['service'],r['id']) for r in own}
matched=[]
for key,req in seen_requests.items():
 resp=responses.get(key)
 if resp:matched.append(dict(service=key[0],id=key[1],api=req['api'],own=key in own_ids,code=resp['code'],observed_response_delay_ms=round((resp['monotonic']-req['monotonic'])*1000,2)))
print(json.dumps(dict(started_at_utc=datetime.datetime.fromtimestamp(time.time()-(time.monotonic()-start),datetime.timezone.utc).isoformat(),completed_at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),release=release.name,duration_s=round(time.monotonic()-start,3),command_writes=0,lease_requests=0,own_reads=own,passive=rows,matched=matched,unmatched_request_count=len(seen_requests)-len(matched),limitation='Passive arrival is polling time, not wire latency; a server reply does not prove the other client consumed it.'),ensure_ascii=False,indent=2))
