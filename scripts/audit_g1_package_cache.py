#!/usr/bin/env python3
"""Run via python stdin on Orin. Read bounded package metadata; never install/extract/run."""
import datetime
import hashlib
import io
import json
import os
from pathlib import Path
import re
import stat
import subprocess
import tarfile
import zipfile

ROOTS = ['/unitree/ota', '/unitree/robot/pkg', '/unitree/var/data',
         '/opt/ota_package', '/var/cache/apt/archives']
MAX_FILES = 3000
MAX_BYTES = 128 * 1024 * 1024
MAX_TEXT = 1024 * 1024
TOKENS = {'mixer_gen': rb'(?:Mixer::Gen|_ZN.*Mixer.*Gen|class\s+Mixer)',
          'fmx_format': rb'(?:FMX|fmx_format)', 'master_service': rb'master_service',
          'motor_calibration': rb'(?:motor.*calibrat|encoder.*offset|zero_offset|joint.*offset)'}


def now():
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def matches(raw):
    # No source strings/configuration values leave the device.
    return {key: len(re.findall(pattern, raw)) for key, pattern in TOKENS.items()
            if re.search(pattern, raw)}


def identity(path):
    p = Path(path)
    before = p.stat()
    h = hashlib.sha256()
    with p.open('rb') as f:
        for part in iter(lambda: f.read(1024 * 1024), b''):
            h.update(part)
    after = p.stat()
    return dict(path=str(p), bytes=after.st_size, sha256=h.hexdigest(),
                unchanged_during_read=(before.st_size, before.st_mtime_ns) ==
                (after.st_size, after.st_mtime_ns))


def archive_members(raw):
    """Only names and bounded file bytes, in memory. No extractall or symlink traversal."""
    if zipfile.is_zipfile(io.BytesIO(raw)):
        with zipfile.ZipFile(io.BytesIO(raw)) as z:
            infos = z.infolist()
            rows, total = [], 0
            for i in infos[:3000]:
                total += i.file_size
                if total > MAX_BYTES:
                    return rows, True
                rows.append(dict(name=i.filename, bytes=i.file_size, tokens=matches(z.read(i))
                                 if i.file_size <= MAX_TEXT and not i.is_dir() else {}))
            return rows, len(infos) > 3000
    with tarfile.open(fileobj=io.BytesIO(raw), mode='r:*') as tar:
        rows = []
        total = 0
        for i, entry in enumerate(tar):
            if i >= 3000:
                return rows, True
            total += entry.size
            if total > MAX_BYTES:
                return rows, True
            data = tar.extractfile(entry).read(MAX_TEXT + 1) if entry.isfile() and entry.size <= MAX_TEXT else b''
            rows.append(dict(name=entry.name, bytes=entry.size, tokens=matches(data)))
        return rows, False


def run():
    out = dict(current=False, started_at_utc=now(), read_only=True, roots=ROOTS,
               max_files=MAX_FILES, max_depth=16, max_file_bytes=MAX_BYTES,
               files=[], packages=[], binaries=[], manifests=[], text_candidates=[], errors=[], missing_roots=[],
               vendor_program_executions=0, command_writes=0, rpc_calls=0,
               configuration_calls=0, calibration_calls=0, motor_configuration_values_obtained=0,
               runtime_memory_reads=0, boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip())
    out['identity_before'] = [identity('/unitree/module/master_service/' + n)
                              for n in ('master_service', 'master_service.json')]
    count = 0
    for root in ROOTS:
        if not Path(root).exists():
            out['missing_roots'].append(root)
            continue
        for directory, dirs, files in os.walk(root, followlinks=False,
                                             onerror=lambda e: out['errors'].append(dict(path=e.filename, error=type(e).__name__))):
            dirs[:] = sorted(d for d in dirs if not (Path(directory)/d).is_symlink())
            if len(Path(directory).relative_to(root).parts) >= 16:
                if dirs:
                    out['errors'].append(dict(path=directory, error='depth bound'))
                dirs[:] = []
            for name in sorted(files):
                count += 1
                if count > MAX_FILES:
                    out['errors'].append(dict(path=root, error='file count bound'))
                    break
                p = Path(directory)/name
                try:
                    st = p.lstat()
                    if not stat.S_ISREG(st.st_mode):
                        continue
                    row = dict(path=str(p), bytes=st.st_size)
                    out['files'].append(row)
                    with p.open('rb') as f:
                        head = f.read(32)
                    if head.startswith(b'\x7fELF') and st.st_size <= MAX_BYTES:
                        binary = identity(p)
                        result = subprocess.run(['nm', '-C', '--defined-only', str(p)], capture_output=True, timeout=30, check=False)
                        lines = result.stdout.decode(errors='replace').splitlines()
                        binary['mixer_fmx_symbols'] = [line for line in lines if re.search(r'Mixer::|FMX|fmx', line)][:80]
                        binary['implementation_match_confirmed'] = False
                        out['binaries'].append(binary)
                    if p.name == 'module.json' and st.st_size <= MAX_TEXT:
                        manifest = json.loads(p.read_text())
                        out['manifests'].append(dict(**identity(p), identity={k:manifest[k] for k in ('name','version','commit') if k in manifest}))
                    is_package = p.suffix.lower() in ('.upk', '.deb', '.zip', '.gz', '.xz', '.tgz', '.tar')
                    if is_package and st.st_size <= MAX_BYTES:
                        package = dict(**identity(p), magic_hex=head[:8].hex(), format='unknown',
                                       payload_decoded=False, members=[], truncated=False)
                        if head.startswith(b'UTPK'):
                            package['format'] = 'UTPK opaque'
                        elif p.suffix == '.deb':
                            package['format'] = 'Debian archive'
                            result = subprocess.run(['dpkg-deb', '--field', str(p), 'Package', 'Version', 'Architecture'],
                                                    capture_output=True, timeout=15, check=False)
                            package['metadata'] = result.stdout.decode(errors='replace')[:2000]
                            result = subprocess.run(['dpkg-deb', '--contents', str(p)], capture_output=True, timeout=15, check=False)
                            package['members'] = result.stdout.decode(errors='replace').splitlines()[:3000]
                            package['truncated'] = len(result.stdout) > 1024*1024 or len(result.stdout.splitlines()) > 3000
                        else:
                            try:
                                members, truncated = archive_members(p.read_bytes())
                                package.update(format='standard archive', members=members, truncated=truncated, payload_decoded=True)
                            except (tarfile.TarError, zipfile.BadZipFile, OSError):
                                package['format'] = 'unrecognized archive'
                        out['packages'].append(package)
                    elif st.st_size <= MAX_TEXT and p.suffix in ('.h', '.hpp', '.cpp', '.c', '.md', '.txt', '.json', '.yaml', '.xml', '.sh'):
                        raw = p.read_bytes()
                        hits = matches(raw)
                        if hits:
                            out['text_candidates'].append(dict(**identity(p), tokens=hits,
                                interpretation='token occurrence only; no implementation or motor value confirmed'))
                except (OSError, subprocess.TimeoutExpired) as e:
                    out['errors'].append(dict(path=str(p), error=type(e).__name__))
            if count > MAX_FILES:
                break
        if count > MAX_FILES:
            break
    # OS package database query is read-only and does not invoke maintainer scripts.
    r = subprocess.run(['dpkg-query', '-W', '-f=${Package}\t${Version}\t${Architecture}\n'], capture_output=True, timeout=15, check=False)
    out['installed_relevant_packages'] = [line for line in r.stdout.decode(errors='replace').splitlines()
                                          if re.search(r'unitree|megaflow|master.service|nvidia-l4t-(?:core|bootloader|kernel)\t', line, re.I)]
    out['identity_after'] = [identity('/unitree/module/master_service/' + n)
                             for n in ('master_service', 'master_service.json')]
    out['completed_at_utc'] = now()
    return out


if __name__ == '__main__':
    print(json.dumps(run(), ensure_ascii=False))
