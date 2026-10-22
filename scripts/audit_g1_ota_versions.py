"""One pinned Orin OTA list query; never updates modules, services, or motion.

This binary takes -T in microseconds (its TCP WaitRecv divides by 1000 before
poll). The former -T 3 yielded a zero-millisecond wait, not a three-second wait.
The configured endpoint reports PC4 modules, not the motion-host firmware.
"""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
import time

BINARY = Path('/unitree/sbin/ota_pipe_cli')
BINARY_SHA256 = 'bbaff3c92449f46d4e5a2c1cef6f990eaadaf6e89139967cb5f8f5f59a275c8d'
HOSTS = Path('/unitree/etc/ota_pipe/ota_pipe_hosts.json')
EXPECTED_HOSTS = {'default': 'tcp://192.168.123.164:1026'}
TIMEOUT_US = 3_000_000


def query(binary=BINARY, hosts=HOSTS, run=subprocess.run):
    assert hashlib.sha256(binary.read_bytes()).hexdigest() == BINARY_SHA256, 'unexpected OTA binary'
    assert json.loads(hosts.read_text()) == EXPECTED_HOSTS, 'unexpected OTA host configuration'
    args = [str(binary), '-a', 'default', '-c', 'listmoduleversion', '-T', str(TIMEOUT_US)]
    out = dict(current=False, checked_at_utc=datetime.now(timezone.utc).isoformat(), argv=args,
        binary_sha256=BINARY_SHA256, timeout_us=TIMEOUT_US, timeout_unit='microseconds',
        action='documented list command only', motion_writes=0, ota_mutations=0,
        version_result_authority='PC4 OTA module metadata only; motion-host firmware remains unverified')
    started = time.monotonic()
    try:
        result = run(args, cwd='/tmp', capture_output=True, text=True, timeout=12)
        out.update(returncode=result.returncode, output=result.stdout[-6000:], stderr=result.stderr[-3000:])
    except subprocess.TimeoutExpired as exc:
        def tail(value, size):
            return (value.decode('utf-8', 'replace') if isinstance(value, bytes) else value or '')[-size:]
        out.update(timed_out=True, output=tail(exc.stdout, 6000), stderr=tail(exc.stderr, 3000))
    out['elapsed_s'] = round(time.monotonic()-started, 3)
    return out


if __name__ == '__main__':
    print(json.dumps(query(), ensure_ascii=False, indent=2))
