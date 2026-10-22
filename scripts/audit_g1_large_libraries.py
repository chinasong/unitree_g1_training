"""Read two named library files with system binutils; never load or run them."""
import datetime,hashlib,json,re,subprocess
from pathlib import Path

TARGETS=['/usr/local/lib/libcryptopp.a','/usr/local/lib/librealsense2.so.2.50.0']
PATTERN=re.compile(r'unitree::security::|Mixer|UT_MIXER|ParseConfigContent|decrypt_and_parse|master_service|\bFMX\b',re.I)
MARKERS=['unitree::security::','Mixer','UT_MIXER','ParseConfigContent','decrypt_and_parse','master_service','FMX']
ALGORITHMS=['Blowfish','AES','SHA256','CBC_Mode','ECB_Mode']
def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda:stream.read(1048576),b''):h.update(chunk)
    return h.hexdigest()
def symbols(path, dynamic=False):
    args=['/usr/bin/nm','--demangle','--print-file-name']
    if dynamic:args.append('--dynamic')
    args.append(str(path))
    result=subprocess.run(args,capture_output=True,timeout=45,text=True,errors='replace')
    lines=result.stdout.splitlines()
    hits=[s for s in lines if PATTERN.search(s)]
    return {'command':args,'returncode':result.returncode,'output_bytes':len(result.stdout.encode()),'line_count':len(lines),'has_symbol_output':bool(result.stdout.strip()),'matches':hits[:100],'matches_truncated':len(hits)>100,'generic_algorithm_line_counts':{name:sum(name in s for s in lines) for name in ALGORITHMS},'realsense_api_line_count':sum('rs2_' in s for s in lines),'stderr_sha256':hashlib.sha256(result.stderr.encode()).hexdigest(),'stderr_nonempty':bool(result.stderr)}
out=dict(schema_version=1,started_at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),read_only=True,scope='Only two exact large files; full symbol tables, dynamic exports, printable strings and fixed token bytes; no dlopen, archive extraction or vendor execution.',max_file_bytes=209715200,command_writes=0,rpc_calls=0,vendor_program_executions=0,runtime_memory_reads=0,boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),files=[],recheck=[],processes=[])
for value in TARGETS:
    p=Path(value);assert p.is_file() and not p.is_symlink() and p.stat().st_size<=out['max_file_bytes']
    before=p.stat();digest=sha(p)
    header=p.open('rb').read(8)
    kind='archive' if header==b'!<arch>\n' else 'ELF'
    assert kind=='archive' or header[:4]==b'\x7fELF'
    record=dict(path=str(p),bytes=before.st_size,sha256=digest,kind=kind,symbols=symbols(p))
    if kind=='archive':
        r=subprocess.run(['/usr/bin/ar','t',str(p)],capture_output=True,timeout=20,text=True,errors='replace')
        members=r.stdout.splitlines();assert r.returncode==0
        record['archive_members']=len(members)
        record['archive_member_name_matches']=[name for name in members if PATTERN.search(name)]
    else:
        record['dynamic_symbols']=symbols(p,True)
        r=subprocess.run(['/usr/bin/readelf','-d','-n','-S','-W',str(p)],capture_output=True,timeout=20,text=True,errors='replace');assert r.returncode==0
        record['needed']=re.findall(r'\(NEEDED\).*?\[(.*?)\]',r.stdout)
        record['sonames']=re.findall(r'\(SONAME\).*?\[(.*?)\]',r.stdout)
        record['build_ids']=re.findall(r'Build ID: (\w+)',r.stdout)
        record['has_symtab']='.symtab' in r.stdout
        record['has_debug_info']='.debug_info' in r.stdout
    result=subprocess.run(['/usr/bin/strings','-a','-n','4',str(p)],capture_output=True,timeout=45)
    assert result.returncode==0
    # Only token counts leave the host; arbitrary strings are never exported.
    record['printable_strings']={'returncode':result.returncode,'total_lines':len(result.stdout.splitlines()),'marker_line_counts':{m:sum(m.lower().encode() in line.lower() for line in result.stdout.splitlines()) for m in MARKERS}}
    with p.open('rb') as stream:raw=stream.read()
    record['exact_byte_markers']={m:raw.count(m.encode()) for m in MARKERS}
    record['standalone_fmx_string_count']=len(re.findall(rb'(?<![\x20-\x7e])FMX(?:\x00|\x01)',raw))
    del raw
    record['sha256_after']=sha(p);after=p.stat()
    record['unchanged_during_read']=digest==record['sha256_after'] and (before.st_size,before.st_mtime_ns)==(after.st_size,after.st_mtime_ns)
    assert record['unchanged_during_read']
    out['files'].append(record)
for value in ['/unitree/module/master_service/master_service','/unitree/module/master_service/master_service.json','/unitree/robot/pkg/module/master_service_pc4/module.json']:
    p=Path(value);out['recheck'].append(dict(path=str(p),bytes=p.stat().st_size,sha256=sha(p)))
for pid in [1296,1298,2557,2676]:
    p=Path('/proc')/str(pid)
    out['processes'].append(dict(pid=pid,argv=p.joinpath('cmdline').read_bytes().decode().strip('\0').split('\0'),start_ticks=int(p.joinpath('stat').read_text().rsplit(')',1)[1].split()[19])))
out['completed_at_utc']=datetime.datetime.now(datetime.timezone.utc).isoformat()
print(json.dumps(out))
