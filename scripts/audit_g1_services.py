#!/usr/bin/env python3
"""Bounded, unprivileged Orin service inspection. Never executes vendor scripts/RPC.

Run on the robot and redirect sanitized JSON on the Mac. No environment dumps,
pipe reads, sudo, service lifecycle operations, binary decoding or DB writes.
"""
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess


ROOTS = ('/unitree/etc', '/unitree/module', '/unitree/var', '/unitree/robot',
         '/home/unitree/megaflow_sdk/ws/bin', '/home/unitree/megaflow_sdk/ws/debug',
         '/home/unitree/megaflow_sdk/ws/data', '/home/unitree/megaflow_sdk/ws/flow')
UNITS = ('master_service.service', 'megaflowruntime.service')
FILES = ('/etc/init.d/master_service', '/run/systemd/generator.late/master_service.service',
         '/etc/systemd/system/megaflowruntime.service',
         '/home/unitree/megaflow_sdk/ws/bin/run.sh',
         '/home/unitree/megaflow_sdk/ws/bin/config.bin')
TERMS = re.compile(r'motor|joint|calibrat|demarcat|softlimit|taumax|zero|offset|config|标定|零位', re.I)
SECRET = re.compile(r'password|passwd|secret|token|credential|private.?key|access.?key|cookie|authorization', re.I)
SUFFIXES = ('.json', '.xml', '.yaml', '.yml', '.conf', '.ini', '.sh', '.service', '.log')
WINDOW = 65536


def sanitize(text):
    # Do not transfer credential-bearing lines, even from ostensibly public logs.
    lines = []
    for line in text.splitlines():
        if SECRET.search(line):
            lines.append('[credential-bearing line omitted]')
        else:
            line = re.sub(r'(https?://)[^\s/@]+:[^\s/@]+@', r'\1[redacted]@', line)
            line = re.sub(r'(https?://[^\s?]+)\?\S+', r'\1?[redacted]', line)
            lines.append(line[:400])
    return '\n'.join(lines)


def proc_identity():
    rows = []
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        try:
            argv = (proc/'cmdline').read_bytes().decode(errors='replace').rstrip('\0').split('\0')
            if not argv or not (argv[0].startswith('/unitree/') or
                               argv[0].endswith('/MegaFlowRuntimeDaemon') or
                               argv == ['/bin/bash', '/home/unitree/megaflow_sdk/ws//bin/run.sh']):
                continue
            # Values are restricted to path/numeric/flag/loopback tokens; never env.
            safe_argv = [a if (i == 0 or a.startswith(('/', '-')) or re.fullmatch(r'[\d.:]*', a))
                         and not SECRET.search(a) else '[redacted]' for i, a in enumerate(argv)]
            stat = (proc/'stat').read_text().split(') ', 1)[1].split()
            row = dict(pid=int(proc.name), ppid=int(stat[1]), start_ticks=int(stat[19]),
                       argv=safe_argv, cgroup=(proc/'cgroup').read_text().splitlines(), links={})
            for key in ('exe', 'cwd', 'fd/1', 'fd/2'):
                try:
                    row['links'][key] = os.readlink(proc/key)
                except OSError as error:
                    row['links'][key] = type(error).__name__
            rows.append(row)
        except (OSError, ValueError):
            continue
    return sorted(rows, key=lambda r: r['pid'])


def inspect():
    result = dict(current=False, checked_at_utc=datetime.now(timezone.utc).isoformat(),
        boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
        clock_authority='Mac receipt time is recorded separately; robot journal clocks can differ',
        command_writes=0, rpc_calls=0, configuration_calls=0, calibration_calls=0,
        motor_configuration_values_obtained=0, processes=proc_identity(), units=[], files=[],
        inventory=[], logs=[], errors=[], roots=list(ROOTS),
        bounds=dict(max_files=1800, max_depth=5, max_file_bytes=WINDOW, max_logs=16,
                    max_matches_per_log=24, max_log_search_bytes=16000000),
        limitations=['Root process exe/cwd/fd links may be unreadable; no privilege escalation',
                    'Inventory and log tails are bounded; absence of matches is not proof of absence',
                    'Installed files and log messages do not prove active motor-controller use',
                    'Opaque binaries/config.bin/databases are not decoded; no parameter protocol guessed'])
    for unit in UNITS:
        run = subprocess.run(['systemctl', 'show', unit, '--no-pager',
            '--property=Id,LoadState,ActiveState,SubState,FragmentPath,DropInPaths,ExecStart,WorkingDirectory,StandardOutput,StandardError,User,MainPID'],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True, timeout=10)
        result['units'].append(dict(unit=unit, code=run.returncode, fields=sanitize(run.stdout), error=sanitize(run.stderr)))
    selected = set(FILES)
    logs = []
    for root in ROOTS:
        if not Path(root).exists():
            result['errors'].append(dict(path=root, error='not found'))
            continue
        def walk_error(error):
            result['errors'].append(dict(path=error.filename, error=type(error).__name__))
        for directory, dirs, files in os.walk(root, onerror=walk_error, followlinks=False):
            dirs[:] = sorted(d for d in dirs if not Path(directory, d).is_symlink())
            if len(Path(directory).relative_to(root).parts) >= 5:
                dirs[:] = []
            for name in sorted(files):
                path = Path(directory, name)
                if path.is_symlink():
                    continue
                if len(result['inventory']) >= 1800:
                    break
                try:
                    size = path.stat().st_size
                    result['inventory'].append(dict(path=str(path), bytes=size))
                    if name.endswith('.log') or '/log/' in str(path):
                        if len(logs) < 16:
                            logs.append(path)
                    elif (name.endswith(SUFFIXES) or '/etc/master_service/' in str(path)) and size <= WINDOW and not SECRET.search(name) and name != 'cos_config.json':
                        selected.add(str(path))
                except OSError as error:
                    result['errors'].append(dict(path=str(path), error=type(error).__name__))
    for name in sorted(selected):
        path = Path(name)
        try:
            with path.open('rb') as stream:
                content = stream.read(WINDOW+1)
            if len(content) > WINDOW or b'\0' in content:
                result['files'].append(dict(path=name, bytes=path.stat().st_size,
                    sha256=hashlib.sha256(content).hexdigest() if len(content)<=WINDOW else None,
                    format='opaque', loaded_by_active_motor_controller_verified=False,
                    meaning='非明文或超出读取上限；未解码，不能核定电机配置'))
                continue
            text = content.decode('utf-8')
            # Startup files plus relevant textual configuration, with credential lines omitted.
            if name in FILES or '/etc/master_service/' in name or TERMS.search(text) or path.name in ('master_service.json', 'default_logconf.json', 'cyclonedds.xml'):
                result['files'].append(dict(path=name, bytes=len(content), sha256=hashlib.sha256(content).hexdigest(),
                    format='text', loaded_by_active_motor_controller_verified=False, text=sanitize(text)))
        except (OSError, UnicodeError) as error:
            result['errors'].append(dict(path=name, error=type(error).__name__))
    # Current MegaFlow logs are outside its subdirectories; fixed filename selection only.
    logs += sorted(Path('/home/unitree/megaflow_sdk/ws').glob('libhv.20261021.log'))
    for path in logs[:16]:
        try:
            with path.open('rb') as stream:
                stream.seek(0, 2)
                size = stream.tell()
                stream.seek(max(0, size-WINDOW))
                content = stream.read(WINDOW)
            lines = content.decode(errors='replace').splitlines()
            if size > WINDOW:
                lines = lines[1:]  # omit partial first line
            matches = [sanitize(line) for line in lines if TERMS.search(line)]
            result['logs'].append(dict(path=str(path), bytes=size, tail_bytes=len(content),
                tail_sha256=hashlib.sha256(content).hexdigest(), matching_lines=len(matches),
                matches=matches[-24:], truncated=size>WINDOW))
            if str(path) == '/unitree/var/log/master_service/master_service.LOG':
                # Fixed regular log only. Bounded full search catches startup lines outside the tail.
                with path.open('rb') as stream:
                    searched = stream.read(16000000)
                matches = [sanitize(line) for line in searched.decode(errors='replace').splitlines() if TERMS.search(line)]
                result['logs'].append(dict(path=str(path), search_bytes=len(searched),
                    search_sha256=hashlib.sha256(searched).hexdigest(), matching_lines=len(matches),
                    matches=matches[-24:], truncated=size>len(searched),
                    meaning='文件历史日志；日期及 PID 均不足以证明属于当前启动'))
        except OSError as error:
            result['errors'].append(dict(path=str(path), error=type(error).__name__))
    for unit in UNITS:
        run = subprocess.run(['journalctl', '--no-pager', '-u', unit, '-b', '-n', '150', '-o', 'short-iso'],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True, timeout=10)
        lines = run.stdout.splitlines()
        matches = [sanitize(line) for line in lines if TERMS.search(line)]
        result['logs'].append(dict(unit=unit, code=run.returncode, lines_returned=len(lines),
            matches=matches[-24:], error=sanitize(run.stderr), bounded=True))
    result['processes_after'] = proc_identity()
    result['completed_at_utc'] = datetime.now(timezone.utc).isoformat()
    return result


if __name__ == '__main__':
    print(json.dumps(inspect(), ensure_ascii=False, indent=2))
