"""Read Orin package metadata and bounded version logs; no vendor execution or RPC."""
from datetime import datetime, timezone
from pathlib import Path
import hashlib,json,os,re

out=dict(checked_at_utc=datetime.now(timezone.utc).isoformat(),boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),command_writes=0,rpc_calls=0,lease_requests=0,vendor_program_executions=0)
def receipt(path):
 p=Path(path)
 if not p.is_file():return dict(path=path,exists=False)
 raw=p.read_bytes()
 return dict(path=path,exists=True,bytes=len(raw),sha256=hashlib.sha256(raw).hexdigest())
out['metadata']=[]
for name in ['/unitree/robot/pkg/version/version.json','/unitree/robot/pkg/module/master_service_pc4/module.json','/unitree/robot/pkg/module/video_hub_pc4/module.json','/unitree/robot/pkg/module/unitree_patch_pc4/module.json']:
 row=receipt(name)
 if row['exists'] and row['bytes']<524288:
  try:
   d=json.loads(Path(name).read_bytes())
   if name.endswith('/version.json'):
    row['version_declarations']=d
   else:row['version_declarations']={k:d[k] for k in ('Name','Version','Commit','Time','Type') if k in d}
  except (ValueError,UnicodeError) as exc:row['error_type']=type(exc).__name__
 out['metadata'].append(row)
out['runtime_files']=[receipt(p) for p in ['/unitree/module/master_service/master_service','/unitree/module/master_service/master_service.json','/unitree/ota/pipe/ota_pipe_service']]
out['official_processes']=[]
for entry in Path('/proc').iterdir():
 if not entry.name.isdigit():continue
 try:
  args=[p.decode(errors='replace') for p in (entry/'cmdline').read_bytes().split(b'\0') if p]
  if args and (args[0] in ('/unitree/module/master_service/master_service','/unitree/ota/pipe/ota_pipe_service','/unitree/module/video_hub_pc4/videohub_pc4') or args[0].endswith('/MegaFlowRuntimeDaemon')):
   ticks=int((entry/'stat').read_text().rsplit(')',1)[1].split()[19]);out['official_processes'].append(dict(pid=int(entry.name),start_ticks=ticks,argv=args))
 except (OSError,ValueError):pass
out['version_log_candidates']=[];out['errors']=[];seen=0
for base in ['/unitree/robot/pkg','/unitree/var/data','/unitree/ota','/unitree/log','/unitree/module/master_service']:
 root=Path(base)
 if not root.exists():continue
 for current,dirs,files in os.walk(root,followlinks=False,onerror=lambda e:out['errors'].append(dict(path=e.filename,error_type=type(e).__name__))):
  dirs[:]=[n for n in dirs if not Path(current,n).is_symlink() and len(Path(current,n).relative_to(root).parts)<=5]
  for name in files:
   seen+=1
   if seen>1600:break
   p=Path(current,name)
   if p.is_symlink():continue
   if re.search(r'version|firmware|sport|loco|motion|mainboard|\.log$',name,re.I):
    try:
     size=p.stat().st_size
     if size>1048576:out['version_log_candidates'].append(dict(path=str(p),bytes=size,skipped='above 1 MiB'));continue
     raw=p.read_bytes()
     if b'\x00' in raw[:1024]:continue
     lines=raw.decode('utf-8',errors='replace').splitlines()
     hits=[line[:360] for line in lines if re.search(r'(firmware|loco|sport|motion|version|ver)[ :=_-]+v?\d+\.\d+',line,re.I)]
     out['version_log_candidates'].append(dict(path=str(p),bytes=size,sha256=hashlib.sha256(raw).hexdigest(),version_lines=hits[:30]))
    except OSError as exc:out['errors'].append(dict(path=str(p),error_type=type(exc).__name__))
  if seen>1600:break
 if seen>1600:break
out['scan']=dict(max_depth=5,max_files=1600,max_text_bytes=1048576,files_seen=seen)
out['completed_at_utc']=datetime.now(timezone.utc).isoformat()
print(json.dumps(out,ensure_ascii=False,indent=2))
