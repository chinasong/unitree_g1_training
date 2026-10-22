"""Orin-side deploy steps, sent as source by scripts/deploy_orin_console.py (system Python 3.8 compatible).

Only touches the project's own directory and its own console process. Never deletes state/, never kills
a process it cannot prove is this project's console, never uses SIGKILL.
"""
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import socket
import tarfile
import time

ROOT = Path('/home/unitree/workspace/g1-body-console')
RECEIPT = 'state/damping-test.json'


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def valid_id(release_id):
    if not re.fullmatch(r'[a-f0-9]{12}', release_id or ''):
        raise RuntimeError('Invalid release identifier')
    return release_id


def listening(port):
    with socket.socket() as sock:
        sock.settimeout(1)
        return sock.connect_ex(('127.0.0.1', port)) == 0


def own_console(root, pid):
    """Return process facts only if pid is this project's console started from a release directory."""
    proc = Path('/proc') / str(pid)
    try:
        argv = (proc / 'cmdline').read_bytes().split(b'\0')
        cwd = str((proc / 'cwd').resolve())
        start_ticks = (proc / 'stat').read_text().rsplit(')', 1)[1].split()[19]
        uid = proc.stat().st_uid
    except (OSError, IndexError):
        return None
    releases = str((root / 'releases').resolve())
    if uid != os.getuid() or b'g1_console' not in argv or b'-m' not in argv:
        return None
    if os.path.commonpath((cwd, releases)) != releases:
        return None
    return dict(pid=pid, cwd=cwd, release=Path(cwd).name, start_ticks=start_ticks,
                argv=[a.decode(errors='replace') for a in argv if a])


def inspect(root=ROOT, port=8765, target=None):
    root = Path(root)
    pid = None
    try:
        pid = int((root / 'console.pid').read_text().strip())
    except (OSError, ValueError):
        pass
    receipt = root / RECEIPT
    return dict(active_release=(root / 'active-release').read_text().strip(),
                target_present=bool(target) and (root / 'releases' / valid_id(target)).is_dir(),
                receipt_sha256=sha256(receipt) if receipt.is_file() else None,
                listening=listening(port), process=own_console(root, pid) if pid else None)


def verify_tree(release, release_id):
    manifest = json.loads((release / 'release-manifest.json').read_text())
    digest = hashlib.sha256(json.dumps(manifest, sort_keys=True, indent=2).encode()).hexdigest()[:12]
    if digest != release_id:
        raise RuntimeError('Manifest fingerprint mismatch')
    base = str(release.resolve())
    for name, expected in manifest.items():
        target = (release / name).resolve()
        if os.path.commonpath((str(target), base)) != base:
            raise RuntimeError('Manifest path escapes release')
        if sha256(target) != expected:
            raise RuntimeError('Release file fingerprint mismatch: ' + name)
    return len(manifest)


def install(release_id, root=ROOT):
    """Extract incoming-<id>.tar.gz into releases/<id> after verifying every file; idempotent."""
    root, release_id = Path(root), valid_id(release_id)
    release = root / 'releases' / release_id
    archive = root / ('incoming-%s.tar.gz' % release_id)
    if release.is_dir():
        files = verify_tree(release, release_id)
        if archive.exists():
            archive.unlink()
        return dict(installed=False, files=files)
    staging = root / 'releases' / (release_id + '.tmp')
    if staging.exists():
        raise RuntimeError('Stale staging directory; inspect it before retrying')
    staging.mkdir()
    with tarfile.open(str(archive)) as tar:
        for member in tar.getmembers():
            target = (staging / member.name).resolve()
            if not member.isfile() or os.path.commonpath((str(target), str(staging.resolve()))) != str(staging.resolve()):
                raise RuntimeError('Archive member not allowed: ' + member.name)
        tar.extractall(str(staging))
    files = verify_tree(staging, release_id)
    staging.rename(release)
    archive.unlink()
    return dict(installed=True, files=files)


def activate(release_id, root=ROOT):
    root, release_id = Path(root), valid_id(release_id)
    if not (root / 'releases' / release_id / 'g1_console/__main__.py').is_file():
        raise RuntimeError('Release not installed')
    temporary = root / 'active-release.tmp'
    temporary.write_text(release_id + '\n')
    temporary.replace(root / 'active-release')
    return dict(active_release=release_id)


def stop(pid, start_ticks, root=ROOT, port=8765, wait_s=15):
    """SIGTERM only the exact process inspected earlier (pid + start time), then wait for the port to close."""
    facts = own_console(Path(root), pid)
    if not facts or facts['start_ticks'] != start_ticks:
        raise RuntimeError('Process is no longer the inspected project console; nothing stopped')
    os.kill(pid, signal.SIGTERM)
    deadline = time.monotonic() + wait_s
    while time.monotonic() < deadline:
        if own_console(Path(root), pid) is None and not listening(port):
            return dict(stopped=True)
        time.sleep(.2)
    raise RuntimeError('Console did not exit within %ss after SIGTERM; not escalating' % wait_s)
