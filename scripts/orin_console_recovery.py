"""Restore only this project's console after a reboot; run over pinned SSH.

Compatible with the Orin system Python 3.8. No systemd, network configuration,
SDK commands, process termination or privilege escalation is used here.
"""
import fcntl
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import socket
import subprocess
import time


ROOT = Path('/home/unitree/workspace/g1-body-console')


def verified_release(root):
    release_id = (root / 'active-release').read_text().strip()
    if not re.fullmatch(r'[a-f0-9]{12}', release_id):
        raise RuntimeError('Invalid project release identifier')
    release = root / 'releases' / release_id
    manifest = json.loads((release / 'release-manifest.json').read_text())
    digest = hashlib.sha256(json.dumps(manifest, sort_keys=True, indent=2).encode()).hexdigest()[:12]
    if digest != release_id:
        raise RuntimeError('Project manifest fingerprint mismatch')
    for name, expected in manifest.items():
        target = (release / name).resolve()
        if os.path.commonpath((str(target), str(release.resolve()))) != str(release.resolve()):
            raise RuntimeError('Project manifest path escapes release')
        if hashlib.sha256(target.read_bytes()).hexdigest() != expected:
            raise RuntimeError('Project release file fingerprint mismatch')
    if not (release / 'g1_console/__main__.py').is_file():
        raise RuntimeError('Console module missing')
    return release_id, release


def listening(port):
    with socket.socket() as sock:
        sock.settimeout(1)
        return sock.connect_ex(('127.0.0.1', port)) == 0


def running_project(root):
    """A live own process with a cold HTTP port must not be duplicated."""
    releases = str((root / 'releases').resolve())
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        try:
            argv = (proc / 'cmdline').read_bytes().split(b'\0')
            if b'-m' in argv and b'g1_console' in argv and proc.stat().st_uid == os.getuid():
                if os.path.commonpath((str((proc / 'cwd').resolve()), releases)) == releases:
                    return True
        except (OSError, ValueError):
            pass
    return False


def restore(root=ROOT, manual=False, port=8765, verify_only=False, accessory_test=False, damping_test=False):
    if damping_test and (not manual or verify_only or accessory_test):
        raise ValueError('Damping test requires separate manual scope')
    if accessory_test and (not manual or verify_only):
        raise ValueError('Accessory test requires separate manual scope')
    if verify_only and not manual:
        raise ValueError('Verification requires manual mode')
    root = root.resolve()
    # The lock is in the existing project, not a system service or login item.
    with (root / 'recovery.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if listening(port):
            return dict(started=False, reason='port_already_listening')
        if running_project(root):
            return dict(started=False, reason='own_process_already_running')
        release_id, release = verified_release(root)
        python = root / '.venv/bin/python'
        if not python.is_file():
            raise RuntimeError('Existing project Python missing; no installation attempted')
        env = os.environ.copy()
        env.update(CYCLONEDDS_HOME=str(root / 'runtime/cyclonedds'),
                   LD_LIBRARY_PATH=str(root / 'runtime/cyclonedds/lib'),
                   G1_CAMERA_BACKEND='unitree', G1_READ_ONLY='0' if manual else '1',
                   G1_MANUAL_REMOTE='1' if manual else '0', G1_HOST='127.0.0.1',
                   G1_VERIFY_ONLY='1' if verify_only else '0',
                   G1_ACCESSORY_TEST='1' if accessory_test else '0',
                   G1_DAMPING_TEST='1' if damping_test else '0',
                   G1_PORT=str(port), G1_DATA_DIR=str(root / 'state'))
        with (root / 'console.log').open('a') as log:
            child = subprocess.Popen([str(python), '-m', 'g1_console',
                                      '--manual-remote' if manual else '--read-only'] + (['--verify-only'] if verify_only else []) + (['--accessory-test'] if accessory_test else []) + (['--damping-test'] if damping_test else []),
                                     cwd=str(release), env=env, stdin=subprocess.DEVNULL,
                                     stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        (root / 'console.pid').write_text(str(child.pid))
        for _ in range(100):
            if child.poll() is not None:
                raise RuntimeError('Project console exited; see console.log')
            if listening(port):
                return dict(started=True, release=release_id, pid=child.pid)
            time.sleep(.1)
        raise RuntimeError('Project console not ready; process left untouched')


def audit():
    result = dict(model=Path('/proc/device-tree/model').read_text().strip('\0'),
                  architecture=platform.machine(),
                  boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
                  captured_at=time.time(), command_status={})
    # Read-only metadata; no Wi-Fi secrets, credentials or connection modifications.
    for name, argv in (
        ('addresses', ['ip', '-j', '-4', 'addr']),
        ('wifi_devices', ['nmcli', '-t', '-f', 'DEVICE,TYPE,STATE', 'device', 'status']),
        ('wifi_autoconnect', ['nmcli', '-t', '-f', 'UUID,TYPE,AUTOCONNECT', 'connection', 'show']),
        ('ssh_enabled', ['systemctl', 'is-enabled', 'ssh']),
        ('ssh_active', ['systemctl', 'is-active', 'ssh']),
        ('ssh_listeners', ['ss', '-H', '-ltn', '( sport = :22 )']),
        ('rfkill', ['rfkill', '--json']),
        ('network_services', ['systemctl', 'show', '--property=Id,ActiveState,SubState,UnitFileState',
                              'NetworkManager.service', 'wpa_supplicant.service', 'ssh.service']),
    ):
        try:
            reading = subprocess.run(argv, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                     timeout=2, text=True)
            result[name] = reading.stdout[:16000].strip() or None
            result['command_status'][name] = dict(status='completed', returncode=reading.returncode)
        except (OSError, subprocess.TimeoutExpired) as exc:
            result[name] = None
            result['command_status'][name] = dict(
                status='timeout' if isinstance(exc, subprocess.TimeoutExpired) else 'unavailable', returncode=None)
    result['wifi_driver'] = {}
    for device in Path('/sys/class/net').glob('wl*'):
        driver = device / 'device/driver'
        result['wifi_driver'][device.name] = driver.resolve().name if driver.exists() else None
    # Keep event categories, never raw journal lines containing SSIDs or credentials.
    result['boot_log_events'] = boot_log_events()
    return result


def boot_log_events():
    argv = ['journalctl', '--boot', '--no-pager', '--quiet', '--lines=500',
            '--unit=NetworkManager', '--unit=wpa_supplicant', '--unit=ssh']
    try:
        reading = subprocess.run(argv, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                 timeout=2, text=True)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return dict(status='timeout' if isinstance(exc, subprocess.TimeoutExpired) else 'unavailable',
                    returncode=None, events=None)
    if reading.returncode != 0 or not reading.stdout.strip():
        return dict(status='unavailable', returncode=reading.returncode, events=None)
    patterns = {
        'wifi_authentication_failure': r'WRONG_KEY|4-Way Handshake failed|SSID-TEMP-DISABLED.*AUTH_FAILED',
        'wifi_network_not_found': r'ssid-not-found|no suitable network found',
        'wifi_rfkill_event': r'rfkill|RF-KILL',
        'wifi_dhcp_failure': (r'device\s*\(wl[^)]*\).*ip-config-unavailable|'
                              r'dhcp[46]?\s*\(wl[^)]*\).*(?:timed out|failed|state changed .* -> (?:timeout|expire))'),
        'wifi_activation_failed': r'Activation: failed|supplicant-timeout',
        'network_activation_succeeded': r'Activation: successful',
    }
    events = {name: sum(bool(re.search(pattern, line, re.I)) for line in reading.stdout.splitlines())
              for name, pattern in patterns.items()}
    return dict(status='completed', returncode=0, sampled_lines=len(reading.stdout.splitlines()),
                limit=500, events=events,
                scope='readable current-boot service journal tail; zero matches do not prove absence')


def main(manual=False, port=8765, verify_only=False, accessory_test=False, damping_test=False):
    if platform.machine() != 'aarch64' or 'Orin' not in Path('/proc/device-tree/model').read_text():
        raise RuntimeError('Pinned host is not the expected Orin platform')
    result = dict(audit=audit())
    try:
        result.update(restore(manual=manual, port=port, verify_only=verify_only, accessory_test=accessory_test, damping_test=damping_test))
    except (OSError, RuntimeError) as exc:
        result.update(started=False, restore_error=str(exc)[:300])
    print(json.dumps(result))
