"""Bounded Orin official text/log search. No vendor program, RPC or file writes.

Retain token counts and narrow numeric version candidates, not full logs/configs.
An occurrence of a version string does not establish installed motion firmware.
"""
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import stat

ROOTS = ('/unitree/etc', '/unitree/module', '/unitree/robot/pkg', '/unitree/ota',
         '/unitree/var/log', '/unitree/var/data', '/etc/systemd/system')
TOKENS = ('humanoid', 'ai_sport', 'locostate', '7301', 'basic_softlimit', 'basic_clearoip')
MAX_FILES, MAX_TOTAL, MAX_FILE, MAX_LOG = 4000, 32*1024*1024, 2*1024*1024, 512*1024


def inspect():
    out = dict(current=False, checked_at_utc=datetime.now(timezone.utc).isoformat(), roots=ROOTS,
        boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
        limits=dict(files=MAX_FILES, total_bytes=MAX_TOTAL, text_file_bytes=MAX_FILE,
                    log_tail_bytes=MAX_LOG, max_depth=8),
        files_seen=0, bytes_read=0, text_files_read=0, candidates=[], skipped=[], errors=[],
        rpc_calls=0, command_writes=0, lease_requests=0, vendor_program_executions=0,
        device_motion_firmware=None, exact_7301_predicate=None)
    for root in ROOTS:
        if not Path(root).exists():
            out['skipped'].append(dict(path=root, reason='missing root'))
            continue
        for directory, dirs, files in os.walk(root, followlinks=False,
                onerror=lambda e:out['errors'].append(dict(path=e.filename, error=type(e).__name__))):
            dirs[:] = sorted(d for d in dirs if not (Path(directory)/d).is_symlink())
            if len(Path(directory).relative_to(root).parts) >= 8:
                if dirs:
                    out['skipped'].append(dict(path=directory, reason='depth bound'))
                dirs[:] = []
            for name in sorted(files):
                out['files_seen'] += 1
                if out['files_seen'] > MAX_FILES or out['bytes_read'] >= MAX_TOTAL:
                    out['skipped'].append(dict(path=root, reason='global bound'))
                    break
                p = Path(directory)/name
                try:
                    st = p.lstat()
                    if not stat.S_ISREG(st.st_mode):
                        continue
                    is_log = '/var/log/' in str(p)
                    if not (is_log or p.suffix.lower() in ('.json', '.yaml', '.yml', '.xml', '.ini',
                            '.conf', '.cfg', '.sh', '.py', '.hpp', '.h', '.cpp', '.service', '.txt')
                            or re.search(r'humanoid|ai_sport|version', name, re.I)):
                        continue
                    limit = MAX_LOG if is_log else MAX_FILE
                    if st.st_size > limit and not is_log:
                        out['skipped'].append(dict(path=str(p), bytes=st.st_size, reason='file bound'))
                        continue
                    amount = min(st.st_size, limit)
                    if out['bytes_read']+amount > MAX_TOTAL:
                        out['skipped'].append(dict(path=str(p), reason='total byte bound'))
                        continue
                    with p.open('rb') as f:
                        offset = max(0, st.st_size-limit) if is_log else 0
                        f.seek(offset)
                        raw = f.read(limit)
                    out['bytes_read'] += len(raw)
                    if b'\0' in raw[:1024]:
                        continue
                    text = raw.decode('utf-8', errors='replace')
                    out['text_files_read'] += 1
                    counts = {t:len(re.findall(re.escape(t), text, re.I)) for t in TOKENS
                              if re.search(re.escape(t), text, re.I)}
                    if not counts and not re.search(r'humanoid|ai_sport', str(p), re.I):
                        continue
                    versions = []
                    for line_num, line in enumerate(text.splitlines(), 1):
                        if len(line) > 4096 or not re.search(r'humanoid|ai_sport', line, re.I):
                            continue
                        numeric = re.findall(r'(?<![\w.])v?(\d+(?:\.\d+){2,4})(?![\w.])', line)
                        if numeric:
                            versions.append(dict(line_in_read_window=line_num,
                                versions=sorted(set(numeric)), installation_identity_proven=False))
                    out['candidates'].append(dict(path=str(p), bytes=st.st_size, read_bytes=len(raw),
                        offset=offset, read_window_sha256=hashlib.sha256(raw).hexdigest(),
                        token_counts=counts, numeric_version_candidates=versions[:30],
                        unchanged_during_read=st.st_size == p.stat().st_size and st.st_mtime_ns == p.stat().st_mtime_ns))
                except OSError as exc:
                    out['errors'].append(dict(path=str(p), error=type(exc).__name__))
            if out['files_seen'] > MAX_FILES or out['bytes_read'] >= MAX_TOTAL:
                break
        if out['files_seen'] > MAX_FILES or out['bytes_read'] >= MAX_TOTAL:
            break
    out['completed_at_utc'] = datetime.now(timezone.utc).isoformat()
    out['limitation'] = ('Bounded Orin text/config and log-tail search, not inspection of .161 files. '
        'Opaque files, earlier log bytes, remote files and unlisted roots are not decoded or excluded. '
        'Numeric candidates and token occurrences are not installed firmware or server predicate evidence.')
    return out


if __name__ == '__main__':
    print(json.dumps(inspect(), ensure_ascii=False, indent=2))
