#!/usr/bin/env python3
"""Repeatable deploy / scope switch of the project console on the verified Orin.

Order: verify local package → fixed-fingerprint SSH (password typed by the operator) → preflight on the
running service (no controller, not armed, navigation idle) → upload + per-file SHA verify → stop only the
inspected project process → activate → restore in the requested scope → verify scope, release and the
single-use damping receipt. A failed restore rolls back to the previous release and scope.
Never sends a robot command; the restored service starts with no controller and no motion permit.
"""
import argparse
import getpass
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts.connect_orin_console import (RemoteAPI, connect_verified, orin_identity, prepare_readers, restore_console,
                                          verify_configuration)

REMOTE_ROOT = '/home/unitree/workspace/g1-body-console'
SCOPES = {
    'readonly': dict(manual=False),
    'verify': dict(manual=True, verify_only=True),
    'accessory': dict(manual=True, accessory_test=True),
    'damping': dict(manual=True, damping_test=True),
    'manual': dict(manual=True),
}


def scope_flags(scope):
    flags = dict(verify_only=False, accessory_test=False, damping_test=False)
    flags.update({k: v for k, v in SCOPES[scope].items() if k != 'manual'})
    return SCOPES[scope]['manual'], flags


def scope_of(state):
    """Name the running service's scope from its own snapshot; None if it matches no known scope."""
    if state.get('read_only') is True:
        return 'readonly'
    remote = state.get('manual_remote') or {}
    if remote.get('enabled') is not True:
        return None
    active = [name for name, flag in (('verify', 'verify_only'), ('accessory', 'accessory_test'),
                                      ('damping', 'damping_test')) if remote.get(flag) is True]
    return active[0] if len(active) == 1 else 'manual' if not active else None


def local_package(root=ROOT):
    deploy = root / '.g1-deploy'
    release_id = (deploy / 'release-id').read_text().strip()
    manifest = json.loads((deploy / 'release-manifest.json').read_text())
    digest = hashlib.sha256(json.dumps(manifest, sort_keys=True, indent=2).encode()).hexdigest()[:12]
    if digest != release_id:
        raise RuntimeError('本地 release-id 与清单不符；先重新运行 scripts/package_orin_console.py')
    stale = [name for name, expected in manifest.items()
             if not (root / name).is_file() or hashlib.sha256((root / name).read_bytes()).hexdigest() != expected]
    if stale:
        raise RuntimeError(f'本地源码已偏离打包清单（{len(stale)} 个文件，如 {stale[0]}）；先重新打包')
    return release_id, deploy / 'console-release.tar.gz', len(manifest)


def connector_running():
    result = subprocess.run(['pgrep', '-f', 'scripts/connect_orin_console.py'], capture_output=True, text=True)
    return [line for line in result.stdout.split() if line]


def preflight(state):
    """The running service must be idle: nobody holds control, nothing armed, navigation idle."""
    problems = []
    if (state.get('control') or {}).get('owner') is not None:
        problems.append('有窗口持有控制权')
    if (state.get('manual_remote') or {}).get('armed') is not False:
        problems.append('人工遥控处于授权状态')
    if (state.get('navigation') or {}).get('status') not in ('idle', None):
        problems.append('导航任务未空闲')
    return problems


class RemoteOps:
    """Thin wrapper around one verified SSH transport; every remote step returns JSON."""

    def __init__(self, transport, port):
        self.transport, self.port = transport, port
        self.source = (ROOT / 'scripts/orin_console_deploy_remote.py').read_text()

    def _call(self, expression, timeout=120):
        source = self.source + f'\nprint(json.dumps({expression}))\n'
        with self.transport.open_session(timeout=5) as channel:
            channel.settimeout(timeout)
            channel.exec_command('/usr/bin/python3 -')
            channel.sendall(source.encode())
            channel.shutdown_write()
            output = channel.makefile('rb').read(200000)
            errors = channel.makefile_stderr('rb').read(20000)
            if channel.recv_exit_status() != 0:
                detail = errors.decode(errors='replace').strip().splitlines() or ['无错误输出']
                raise RuntimeError('Orin 步骤失败：' + detail[-1][:300])
        return json.loads(output)

    def inspect(self, target):
        return self._call(f'inspect(port={self.port!r}, target={target!r})')

    def upload(self, archive, release_id):
        sftp = self.transport.open_sftp_client()
        try:
            sftp.put(str(archive), f'{REMOTE_ROOT}/incoming-{release_id}.tar.gz')
        finally:
            sftp.close()

    def install(self, release_id):
        return self._call(f'install({release_id!r})', timeout=300)

    def stop(self, pid, start_ticks):
        return self._call(f'stop({pid!r}, {start_ticks!r}, port={self.port!r})')

    def activate(self, release_id):
        return self._call(f'activate({release_id!r})')

    def restore(self, scope):
        manual, flags = scope_flags(scope)
        return restore_console(self.transport, manual, self.port, **flags)

    def connect_readers(self, scope, initialize):
        manual, flags = scope_flags(scope)
        api = RemoteAPI(self.transport, self.port)
        for retry in range(10):
            try:
                return prepare_readers(api, manual, initialize=initialize, **flags)
            except (OSError, EOFError, RuntimeError):
                if retry == 9:
                    raise
                time.sleep(1)

    def state(self):
        api = RemoteAPI(self.transport, self.port)
        api.request('/api/session')
        return api.request('/api/state')


def deploy(ops, release_id, archive, scope, log=print):
    """Run the deploy against `ops`; returns the record. Raises before any change if preflight fails."""
    record = dict(target_release=release_id, target_scope=scope, steps=[])
    step = lambda name, **data: (record['steps'].append(dict(step=name, at=time.time(), **data)), log(f'· {name}'))
    before = ops.inspect(release_id)
    record['before'] = before
    old_state = ops.state() if before['listening'] else None
    old_scope = scope_of(old_state) if old_state else None
    record.update(old_scope=old_scope, old_run_id=old_state and old_state.get('run_id'))
    if old_state:
        if problems := preflight(old_state):
            raise RuntimeError('服务不空闲，未做任何改动：' + '、'.join(problems))
        if old_scope is None:
            raise RuntimeError('无法识别现有服务范围，未做任何改动')
    process = before['process']
    if before['listening'] and not process:
        raise RuntimeError('8765 由非本项目可核验进程占用，未做任何改动')
    if process and process['release'] == release_id and old_scope == scope:
        step('already_current')
        record['result'] = 'unchanged'
        return record

    if not before['target_present']:
        ops.upload(archive, release_id)
        step('uploaded')
    step('installed', **ops.install(release_id))
    if process:
        step('stopped', **ops.stop(process['pid'], process['start_ticks']))
    step('activated', **ops.activate(release_id))
    restored = ops.restore(scope)
    step('restore', started=restored.get('started'), error=restored.get('restore_error'))
    if not restored.get('started'):
        if process:
            ops.activate(before['active_release'])
            back = ops.restore(old_scope)
            step('rolled_back', release=before['active_release'], scope=old_scope, started=back.get('started'))
            if back.get('started'):
                ops.connect_readers(old_scope, initialize=True)
        raise RuntimeError('新发布未能启动：' + str(restored.get('restore_error'))[:200])

    state = ops.connect_readers(scope, initialize=True)
    manual, flags = scope_flags(scope)
    verify_configuration(state, manual, **flags)
    after = ops.inspect(release_id)
    record['after'] = after
    checks = dict(
        release_running=bool(after['process']) and after['process']['release'] == release_id,
        active_release=after['active_release'] == release_id,
        scope=scope_of(state) == scope,
        idle=not preflight(state),
        receipt_kept=before['receipt_sha256'] is None or after['receipt_sha256'] == before['receipt_sha256'],
    )
    record.update(checks=checks, new_run_id=state.get('run_id'))
    if not all(checks.values()):
        raise RuntimeError('部署后核验未通过：' + '、'.join(k for k, v in checks.items() if not v))
    record['result'] = 'deployed'
    return record


def main():
    parser = argparse.ArgumentParser(description='部署本地打包的控制台并按指定范围在 Orin 上重启；不发送任何机器人指令')
    parser.add_argument('--scope', required=True, choices=sorted(SCOPES),
                        help='readonly 只读；verify 接口验证；accessory 声光测试；damping 单次阻尼；manual 完整人工遥控（行走/手势）')
    parser.add_argument('--host', help='默认取本地 .g1-console/orin.json')
    parser.add_argument('--user', default='unitree')
    parser.add_argument('--interface', default='en0')
    parser.add_argument('--remote-port', type=int, default=8765)
    args = parser.parse_args()

    release_id, archive, files = local_package()
    print(f'本地发布 {release_id}（{files} 个文件），目标范围 {args.scope}')
    if pids := connector_running():
        raise SystemExit(f'Mac 连接器仍在运行（PID {", ".join(pids)}），它可能在重启窗口内按旧范围恢复服务。'
                         '请先关闭连接窗口，部署后再用对应范围的启动器重连。')
    # Ask before connecting: an open transport waiting on a slow prompt is dropped by sshd's login grace
    # time ("No existing session"). The password is still sent only after the host fingerprint matches.
    identity = orin_identity()  # Fail on a missing local identity before asking for the password.
    args.host = args.host or identity['host']
    password = getpass.getpass('Orin SSH 密码（连接后先核验主机指纹再发送）：')
    transport = connect_verified(args.host, args.user, lambda: password, args.interface)
    record = dict(started_at=time.time(), host=args.host)
    try:
        record.update(deploy(RemoteOps(transport, args.remote_port), release_id, archive, args.scope))
    except Exception as exc:
        record.update(result='failed', error=f'{type(exc).__name__}: {exc}')
        raise SystemExit(f'部署中止：{record["error"]}')
    finally:
        transport.close()
        out = ROOT / '.g1-deploy/research' / time.strftime('deploy-%Y%m%d-%H%M%S.json')
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(record, ensure_ascii=False, indent=2))
        print(f'部署记录：{out}')
    print(json.dumps({k: record.get(k) for k in ('result', 'target_release', 'old_scope', 'target_scope',
                                                 'old_run_id', 'new_run_id', 'checks')}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
