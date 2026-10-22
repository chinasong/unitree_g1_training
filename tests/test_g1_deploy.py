"""Deploy / scope switch: ordering, refusals and rollback with a recording fake; Orin-side steps on a temp tree."""
import hashlib
import json
import tarfile

import pytest

from scripts import orin_console_deploy_remote as remote
from scripts.deploy_orin_console import SCOPES, deploy, preflight, scope_flags, scope_of

NEW, OLD = 'aaaaaaaaaaaa', 'bbbbbbbbbbbb'
RECEIPT = 'r' * 64


def state(scope, owner=None, armed=False, nav='idle', run_id='run-old'):
    flags = dict(verify_only=False, accessory_test=False, damping_test=False)
    if scope == 'readonly':
        return dict(read_only=True, run_id=run_id, mode='robot', control=dict(owner=owner),
                    manual_remote=dict(enabled=False, armed=armed, **flags), navigation=dict(status=nav))
    flags.update({k: v for k, v in scope_flags(scope)[1].items()})
    return dict(read_only=False, run_id=run_id, mode='robot', control=dict(owner=owner),
                manual_remote=dict(enabled=True, armed=armed, **flags,
                                   allowed_write_operations=['damp'] if flags['damping_test'] else
                                   ['led', 'tts'] if flags['accessory_test'] else []),
                navigation=dict(status=nav))


class FakeOps:
    def __init__(self, running_scope='damping', release=OLD, restore_ok=True, receipt_after=RECEIPT, **state_changes):
        self.calls, self.running_scope, self.release = [], running_scope, release
        self.restore_ok, self.receipt_after, self.active = restore_ok, receipt_after, release
        self.current = {**state(running_scope), **state_changes} if running_scope else None

    def inspect(self, target):
        self.calls.append(('inspect',))
        process = dict(pid=42, start_ticks='9', release=self.release) if self.current else None
        return dict(active_release=self.active, target_present=False, listening=bool(self.current),
                    receipt_sha256=RECEIPT if len(self.calls) == 1 else self.receipt_after, process=process)

    def state(self):
        self.calls.append(('state',))
        return self.current

    def upload(self, archive, release_id):
        self.calls.append(('upload', release_id))

    def install(self, release_id):
        self.calls.append(('install', release_id))
        return dict(installed=True, files=3)

    def stop(self, pid, start_ticks):
        self.calls.append(('stop', pid, start_ticks))
        self.current = None
        return dict(stopped=True)

    def activate(self, release_id):
        self.calls.append(('activate', release_id))
        self.active = release_id
        return dict(active_release=release_id)

    def restore(self, scope):
        self.calls.append(('restore', scope))
        ok = self.restore_ok or scope != self.target
        if ok:
            self.running_scope, self.release = scope, self.active
        return dict(started=ok, restore_error=None if ok else 'boom')

    def connect_readers(self, scope, initialize):
        self.calls.append(('connect', scope))
        self.current = state(scope, run_id='run-new')
        return self.current


def run_deploy(ops, scope):
    ops.target = scope
    return deploy(ops, NEW, 'archive.tgz', scope, log=lambda *_: None)


def names(ops):
    return [c[0] for c in ops.calls if c[0] not in ('inspect', 'state')]


def test_scope_table_matches_running_service_names():
    assert set(SCOPES) == {'readonly', 'verify', 'accessory', 'damping', 'manual'}
    for scope in SCOPES:
        assert scope_of(state(scope)) == scope
    assert scope_of(dict(read_only=False, manual_remote=dict(enabled=True, verify_only=True, damping_test=True))) is None


def test_switch_damping_to_manual_in_safe_order():
    ops = FakeOps()
    record = run_deploy(ops, 'manual')
    assert names(ops) == ['upload', 'install', 'stop', 'activate', 'restore', 'connect']
    assert ('stop', 42, '9') in ops.calls
    assert record['result'] == 'deployed' and record['old_scope'] == 'damping' and all(record['checks'].values())


@pytest.mark.parametrize('change', [dict(control=dict(owner='someone')), dict(navigation=dict(status='running'))])
def test_busy_service_is_never_touched(change):
    ops = FakeOps(**change)
    with pytest.raises(RuntimeError, match='不空闲'):
        run_deploy(ops, 'manual')
    assert names(ops) == []


def test_armed_service_is_never_touched():
    ops = FakeOps()
    ops.current['manual_remote']['armed'] = True
    with pytest.raises(RuntimeError):
        run_deploy(ops, 'manual')
    assert names(ops) == []


def test_same_release_and_scope_is_a_no_op():
    ops = FakeOps(running_scope='manual', release=NEW)
    assert run_deploy(ops, 'manual')['result'] == 'unchanged'
    assert names(ops) == []


def test_failed_restore_rolls_back_release_and_scope():
    ops = FakeOps(restore_ok=False)
    with pytest.raises(RuntimeError, match='未能启动'):
        run_deploy(ops, 'manual')
    assert names(ops) == ['upload', 'install', 'stop', 'activate', 'restore', 'activate', 'restore', 'connect']
    assert ops.calls[-3:] == [('activate', OLD), ('restore', 'damping'), ('connect', 'damping')]


def test_lost_damping_receipt_fails_verification():
    ops = FakeOps(receipt_after=None)
    with pytest.raises(RuntimeError, match='receipt_kept'):
        run_deploy(ops, 'manual')


def test_preflight_lists_every_blocker():
    assert preflight(state('manual')) == []
    assert len(preflight(state('manual', owner='x', armed=True, nav='running'))) == 3


def release_tree(tmp_path, files):
    root = tmp_path / 'g1-body-console'
    (root / 'releases').mkdir(parents=True)
    source = tmp_path / 'src'
    for name, body in files.items():
        (source / name).parent.mkdir(parents=True, exist_ok=True)
        (source / name).write_text(body)
    manifest = {name: hashlib.sha256(body.encode()).hexdigest() for name, body in files.items()}
    encoded = json.dumps(manifest, sort_keys=True, indent=2).encode()
    release_id = hashlib.sha256(encoded).hexdigest()[:12]
    (source / 'release-manifest.json').write_bytes(encoded)
    return root, source, release_id


def pack(source, archive, names):
    with tarfile.open(archive, 'w:gz') as tar:
        for name in names:
            tar.add(source / name, arcname=name, recursive=False)


def test_remote_install_verifies_every_file_then_activates(tmp_path):
    files = {'g1_console/__main__.py': 'main', 'g1_console/app.py': 'app'}
    root, source, release_id = release_tree(tmp_path, files)
    pack(source, root / f'incoming-{release_id}.tar.gz', [*files, 'release-manifest.json'])
    assert remote.install(release_id, root) == dict(installed=True, files=2)
    assert not (root / f'incoming-{release_id}.tar.gz').exists()
    assert remote.install(release_id, root) == dict(installed=False, files=2)
    (root / 'active-release').write_text(OLD + '\n')
    assert remote.activate(release_id, root) == dict(active_release=release_id)
    assert (root / 'active-release').read_text().strip() == release_id


def test_remote_install_rejects_tampered_archive(tmp_path):
    files = {'g1_console/__main__.py': 'main'}
    root, source, release_id = release_tree(tmp_path, files)
    (source / 'g1_console/__main__.py').write_text('tampered')
    pack(source, root / f'incoming-{release_id}.tar.gz', [*files, 'release-manifest.json'])
    with pytest.raises(RuntimeError, match='fingerprint'):
        remote.install(release_id, root)
    assert not (root / 'releases' / release_id).exists()


def test_remote_steps_reject_bad_ids_and_uninstalled_release(tmp_path):
    for bad in ('../etc', 'ABC', ''):
        with pytest.raises(RuntimeError):
            remote.install(bad, tmp_path)
    with pytest.raises(RuntimeError, match='not installed'):
        remote.activate(NEW, tmp_path)


def test_remote_stop_refuses_foreign_process(tmp_path):
    with pytest.raises(RuntimeError, match='no longer'):
        remote.stop(1, '0', root=tmp_path)
