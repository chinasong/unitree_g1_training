"""Real SSH/HTTP boundaries and isolated project-process reboot recovery.

All listeners are loopback test fixtures. No robot or motor endpoints are used.
"""
import ast
from contextlib import contextmanager
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import http.client
import ipaddress
import json
import re
from pathlib import Path
import socket
import sys
import threading
from types import SimpleNamespace
import subprocess

import paramiko
import pytest

from scripts import connect_orin_console as connector
from scripts import orin_console_recovery as recovery

FIXTURE_FINGERPRINT = 'SHA256:' + 'A' * 43


@pytest.fixture(autouse=True)
def local_identity(tmp_path, monkeypatch):
    """Tests never read this Mac's real Orin identity."""
    path = tmp_path / 'orin.json'
    path.write_text(json.dumps(dict(host='192.0.2.28', fingerprint=FIXTURE_FINGERPRINT)))
    monkeypatch.setattr(connector, 'ORIN', path)
    return path


@pytest.mark.parametrize('content, message', [(None, '缺少本地 Orin 配置'), ('{', '不是有效 JSON'),
                                              ('{"host": "192.0.2.28", "fingerprint": "SHA256:short"}', '主机指纹'),
                                              ('{"host": "orin", "fingerprint": "SHA256:' + 'A' * 43 + '"}', 'IP 地址')])
def test_orin_identity_is_local_and_refuses_to_guess(tmp_path, content, message):
    path = tmp_path / 'missing.json'
    if content is not None:
        path.write_text(content)
    with pytest.raises(RuntimeError, match=message):
        connector.orin_identity(path)


def test_committed_sources_carry_no_robot_identity():
    for name in ('connect_orin_console.py', 'deploy_orin_console.py'):
        text = (Path(connector.__file__).parent / name).read_text()
        assert not re.search(r'SHA256:[A-Za-z0-9+/]{43}', text) and not re.search(r'192\.168\.\d+\.\d+', text)



def cold_state(**changes):
    return dict(mode='disconnected', read_only=True, manual_remote={'enabled': False, 'armed': False},
                control={'owner': None}, navigation={'status': 'idle'}, connected=False, **changes)


@contextmanager
def serving(server):
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        yield server.server_address[1]
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)


class FixtureTransport:
    def __init__(self, backend):
        self.backend, self.channels, self.closed = backend, [], False

    def open_channel(self, kind, destination, origin, timeout=4):
        assert kind == 'direct-tcpip' and destination == ('127.0.0.1', self.backend)
        channel = socket.create_connection(destination, timeout=timeout)
        self.channels.append(channel)
        return channel

    def is_active(self):
        return not self.closed

    def close(self):
        self.closed = True
        for channel in self.channels:
            channel.close()


def request(port, path, method='GET', body=None, **headers):
    connection = http.client.HTTPConnection('127.0.0.1', port, timeout=3)
    try:
        connection.request(method, path, body, headers)
        response = connection.getresponse()
        return response.status, response.read().decode()
    finally:
        connection.close()


def test_real_ssh_key_exchange_rejects_other_hosts_before_password(monkeypatch):
    key = paramiko.RSAKey.generate(2048)
    auth = []

    class SSHServer(paramiko.ServerInterface):
        def check_auth_password(self, username, password):
            auth.append((username, password))
            return paramiko.AUTH_SUCCESSFUL

    with socket.socket() as listener:
        listener.bind(('127.0.0.1', 0))
        listener.listen(2)
        server_transports = []

        def accept():
            for _ in range(2):
                sock, _ = listener.accept()
                transport = paramiko.Transport(sock)
                server_transports.append(transport)
                transport.add_server_key(key)
                try:
                    transport.start_server(server=SSHServer())
                except (EOFError, paramiko.SSHException):
                    # The rejected client may close before server-side KEX returns.
                    transport.close()

        thread = threading.Thread(target=accept, daemon=True)
        thread.start()
        monkeypatch.setattr(connector, 'bound_socket', lambda *a, **kw: socket.create_connection(listener.getsockname()))
        prompts = []

        def password():
            prompts.append(True)
            return 'test-only-password'

        with pytest.raises(connector.IdentityMismatch):
            connector.connect_verified('other-device', 'fixture', password, 'fixture')
        assert prompts == [] and auth == []
        transport = connector.connect_verified('verified-device', 'fixture', password, 'fixture', expected=connector.fingerprint(key))
        assert prompts == [True] and auth == [('fixture', 'test-only-password')]
        transport.close()
        thread.join(timeout=3)
        for remote in server_transports:
            remote.close()


def test_gateway_offline_requests_are_rejected_and_never_replayed_on_recovery():
    received = []

    class Backend(BaseHTTPRequestHandler):
        def do_GET(self):
            received.append((self.command, self.path))
            body = b'{"run_id":"new-boot"}'
            self.send_response(200)
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *args):
            pass

    link = connector.Link()
    with serving(ThreadingHTTPServer(('127.0.0.1', 0), Backend)) as backend:
        transport = FixtureTransport(backend)
        with serving(connector.Server(('127.0.0.1', 0), connector.handler_for(link, backend))) as local:
            status, body = request(local, '/')
            assert status == 503 and '身体反馈未知' in body
            status, body = request(local, '/api/command', 'POST', '{"capability":"velocity"}')
            assert status == 503 and json.loads(body)['error']['code'] == 'ORIN_OFFLINE'
            assert received == []
            assert request(local, '/__connection', Host='unrelated.example')[0] == 403
            assert request(local, '/__connection', Origin='https://unrelated.example')[0] == 403
            link.publish(transport, '192.0.2.77', {'connected': True})
            assert json.loads(request(local, '/api/state')[1])['run_id'] == 'new-boot'
            assert received == [('GET', '/api/state')]
            link.offline('robot restarted')
            assert transport.closed and request(local, '/api/state')[0] == 503
            assert json.loads(request(local, '/__connection')[1])['ready'] is False
            assert received == [('GET', '/api/state')]


def test_remote_http_session_and_connect_are_read_only_initialization():
    received = []

    class Backend(BaseHTTPRequestHandler):
        def do_GET(self):
            received.append((self.command, self.path))
            self.send_response(200)
            if self.path == '/api/session':
                self.send_header('Set-Cookie', 'fixture=new-boot; HttpOnly; SameSite=Strict')
                value = {'ok': True}
            else:
                assert self.headers.get('Cookie') == 'fixture=new-boot'
                value = cold_state()
            self.end_headers()
            self.wfile.write(json.dumps(value).encode())

        def do_POST(self):
            received.append((self.command, self.path))
            assert self.path == '/api/connect'
            assert self.headers['X-G1-Intent'] == 'operator'
            assert json.loads(self.rfile.read(int(self.headers['Content-Length']))) == {'mode': 'robot', 'interface': 'eth0'}
            self.send_response(200)
            self.end_headers()
            self.wfile.write(json.dumps({**cold_state(), 'mode': 'robot'}).encode())

        def log_message(self, *args):
            pass

    with serving(ThreadingHTTPServer(('127.0.0.1', 0), Backend)) as port:
        transport = FixtureTransport(port)
        state = connector.prepare_readers(connector.RemoteAPI(transport, port))
        assert state['mode'] == 'robot' and state['manual_remote']['armed'] is False
        assert received == [('GET', '/api/session'), ('GET', '/api/state'), ('POST', '/api/connect')]
        transport.close()


@pytest.mark.parametrize('change', [
    {'control': {'owner': 'operator'}}, {'manual_remote': {'armed': True}},
    {'navigation': {'status': 'running'}}, {'navigation': {}}, {'manual_remote': {}},
    {'control': {}}, {'control': None}, {'manual_remote': None}, {'navigation': None},
    {'read_only': False},
])
def test_recovery_never_overrides_active_or_unknown_authority(change):
    class API:
        calls = []
        def request(self, path, data=None):
            self.calls.append((path, data))
            return {**cold_state(), **change}
    api = API()
    api.calls = []
    with pytest.raises(RuntimeError):
        connector.prepare_readers(api)
    assert api.calls == [('/api/session', None), ('/api/state', None)]


@pytest.mark.parametrize('state', [None, [], 'not-project-state'])
def test_malformed_state_does_not_escape_the_recovery_loop(state):
    with pytest.raises(RuntimeError):
        connector.verify_configuration(state)
    with pytest.raises(RuntimeError):
        connector.verify_configuration(state, manual_remote=True)


def test_existing_disconnected_service_is_preserved_until_operator_connects():
    class API:
        def request(self, path, data=None):
            assert path in ('/api/session', '/api/state') and data is None
            return cold_state()
    assert connector.prepare_readers(API(), initialize=False)['mode'] == 'disconnected'


def test_recovery_survives_reboot_absent_device_and_dhcp_address_change(monkeypatch, tmp_path):
    published, connections, restored, saved = [], [], [], []
    old, new = FixtureTransport(1), FixtureTransport(2)
    candidates = iter([['192.0.2.28'], [], ['192.0.2.77']])

    class Link(connector.Link):
        def publish(self, transport, host, state):
            super().publish(transport, host, state)
            published.append(self.snapshot())

    class Stop:
        def is_set(self):
            return len(published) >= 2
        def wait(self, timeout=None):
            return self.is_set()

    class API:
        def __init__(self, transport, port):
            self.transport, self.samples = transport, 0
        def request(self, path, data=None):
            self.samples += 1
            if self.samples > 2 and self.transport is old:
                raise OSError('old boot socket closed')
            assert path in ('/api/session', '/api/state') and data is None
            return {**cold_state(), 'mode': 'robot'}

    def connect(host, *args):
        connections.append(host)
        return old if host.endswith('.28') else new

    def restore(transport, manual, port):
        restored.append(transport)
        return {'started': True, 'audit': {'boot_id': 'new' if transport is new else 'old'}}

    monkeypatch.setattr(connector, 'CACHE', tmp_path / 'absent.json')
    monkeypatch.setattr(connector, 'interface_subnet', lambda _: ('192.0.2.41', ipaddress.ip_network('192.0.2.0/24')))
    monkeypatch.setattr(connector, 'discover', lambda *a: next(candidates))
    monkeypatch.setattr(connector, 'connect_verified', connect)
    monkeypatch.setattr(connector, 'restore_console', restore)
    monkeypatch.setattr(connector, 'RemoteAPI', API)
    monkeypatch.setattr(connector, 'save_connection', lambda *a: saved.append(a))
    link = Link()
    connector.recover(link, Stop(), SimpleNamespace(host=None, interface='fixture', user='unitree', remote_port=8765,
                                                   manual_remote=False, retry_seconds=2), lambda: 'unused')
    assert connections == ['192.0.2.28', '192.0.2.77']
    assert [s['host'] for s in published] == connections
    assert all(s['ready'] and s['verified'] for s in published)
    assert saved[-1] == ('192.0.2.77', {'boot_id': 'new'})
    assert old.closed and restored == [old, new]


def fixture_release(root):
    source = '''import http.server,json,os,pathlib
pathlib.Path(os.environ['G1_DATA_DIR'],'fixture-flags.json').write_text(json.dumps({k:os.environ[k] for k in ('G1_READ_ONLY','G1_MANUAL_REMOTE','G1_VERIFY_ONLY','G1_ACCESSORY_TEST','G1_DAMPING_TEST','G1_HOST')}))
http.server.HTTPServer(('127.0.0.1',int(os.environ['G1_PORT'])),http.server.BaseHTTPRequestHandler).serve_forever()
'''
    manifest = {'g1_console/__main__.py': hashlib.sha256(source.encode()).hexdigest()}
    release_id = hashlib.sha256(json.dumps(manifest, sort_keys=True, indent=2).encode()).hexdigest()[:12]
    release = root / 'releases' / release_id
    (release / 'g1_console').mkdir(parents=True)
    (release / 'g1_console/__main__.py').write_text(source)
    (release / 'release-manifest.json').write_text(json.dumps(manifest))
    (root / 'active-release').write_text(release_id)
    (root / 'state').mkdir()
    (root / '.venv/bin').mkdir(parents=True)
    (root / '.venv/bin/python').symlink_to(sys.executable)
    return release


@pytest.mark.parametrize('manual,verify_only,accessory_test,damping_test', [(False, False, False, False), (True, True, False, False), (True, False, True, False), (True, False, False, True)])
def test_restore_starts_verified_project_once_with_no_motion_authorization(monkeypatch, tmp_path, manual, verify_only, accessory_test, damping_test):
    fixture_release(tmp_path)
    monkeypatch.setattr(recovery, 'running_project', lambda _: False)
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 0))
        port = probe.getsockname()[1]
    children = []
    popen = recovery.subprocess.Popen

    def start(*args, **kwargs):
        children.append(popen(*args, **kwargs))
        return children[-1]

    monkeypatch.setattr(recovery.subprocess, 'Popen', start)
    try:
        result = recovery.restore(tmp_path, port=port, manual=manual, verify_only=verify_only, accessory_test=accessory_test, damping_test=damping_test)
        assert result['started'] and len(children) == 1
        assert json.loads((tmp_path / 'state/fixture-flags.json').read_text()) == {
            'G1_READ_ONLY': '0' if manual else '1', 'G1_MANUAL_REMOTE': '1' if manual else '0',
            'G1_VERIFY_ONLY': '1' if verify_only else '0', 'G1_ACCESSORY_TEST': '1' if accessory_test else '0', 'G1_DAMPING_TEST': '1' if damping_test else '0', 'G1_HOST': '127.0.0.1'}
        assert recovery.restore(tmp_path, port=port) == {'started': False, 'reason': 'port_already_listening'}
        assert len(children) == 1
    finally:
        for child in children:
            child.terminate()
            child.wait(timeout=5)


def test_release_tampering_or_cold_existing_process_never_launches_duplicate(monkeypatch, tmp_path):
    release = fixture_release(tmp_path)
    monkeypatch.setattr(recovery, 'listening', lambda _: False)
    monkeypatch.setattr(recovery, 'running_project', lambda _: True)
    monkeypatch.setattr(recovery.subprocess, 'Popen', lambda *a, **k: pytest.fail('must not start'))
    assert recovery.restore(tmp_path)['reason'] == 'own_process_already_running'
    monkeypatch.setattr(recovery, 'running_project', lambda _: False)
    (release / 'g1_console/__main__.py').write_text('tampered')
    with pytest.raises(RuntimeError, match='fingerprint mismatch'):
        recovery.restore(tmp_path)
    ast.parse(Path(recovery.__file__).read_text(), feature_version=(3, 8))


@pytest.mark.parametrize('address', ['10.0.0.5 netmask 0xff000000', '8.8.8.8 netmask 0xffffff00', '127.0.0.1 netmask 0xff000000'])
def test_discovery_never_expands_into_public_or_large_networks(monkeypatch, address):
    monkeypatch.setattr(connector.subprocess, 'check_output', lambda *a, **k: 'inet ' + address)
    with pytest.raises(RuntimeError):
        connector.interface_subnet('fixture')


def test_boot_log_diagnosis_retains_failure_evidence_without_network_secrets(monkeypatch):
    journal = ('wpa_supplicant: CTRL-EVENT-SSID-TEMP-DISABLED ssid="private-network" reason=WRONG_KEY\n'
               'NetworkManager: dhcp4 (wlan0): request timed out psk=do-not-save\n'
               'NetworkManager: device (wlan0): Activation: failed for connection private-network\n')
    monkeypatch.setattr(recovery.subprocess, 'run', lambda *a, **k: SimpleNamespace(returncode=0, stdout=journal))
    result = recovery.boot_log_events()
    assert result['events']['wifi_authentication_failure'] == 1
    assert result['events']['wifi_dhcp_failure'] == 1
    assert result['events']['wifi_activation_failed'] == 1
    assert result['events']['network_activation_succeeded'] == 0
    assert 'private-network' not in json.dumps(result) and 'do-not-save' not in json.dumps(result)
    assert 'do not prove absence' in result['scope']


def test_normal_dhcp_timeout_budget_and_ethernet_failure_are_not_wifi_failures(monkeypatch):
    journal = ('NetworkManager: dhcp4 (wlan0): activation: beginning transaction (timeout in 45 seconds)\n'
               'NetworkManager: dhcp6 (wlan0): activation: beginning transaction (timeout in 45 seconds)\n'
               'NetworkManager: dhcp4 (wlan0): state changed unknown -> bound\n'
               'NetworkManager: dhcp4 (eth0): request timed out\n'
               'NetworkManager: device (eth0): state change: ip-config -> failed (reason ip-config-unavailable)\n')
    monkeypatch.setattr(recovery.subprocess, 'run', lambda *a, **k: SimpleNamespace(returncode=0, stdout=journal))
    assert recovery.boot_log_events()['events']['wifi_dhcp_failure'] == 0
    journal += 'NetworkManager: dhcp4 (wlan0): state changed unknown -> timeout\n'
    assert recovery.boot_log_events()['events']['wifi_dhcp_failure'] == 1


@pytest.mark.parametrize('response', [
    SimpleNamespace(returncode=1, stdout=''), SimpleNamespace(returncode=0, stdout=''),
    FileNotFoundError(), subprocess.TimeoutExpired('journalctl', 2),
])
def test_unreadable_boot_journal_keeps_cause_unknown(monkeypatch, response):
    def run(*args, **kwargs):
        if isinstance(response, Exception):
            raise response
        return response
    monkeypatch.setattr(recovery.subprocess, 'run', run)
    result = recovery.boot_log_events()
    assert result['events'] is None and result['status'] != 'completed'


def test_audit_distinguishes_stopped_ssh_from_missing_network_tools(monkeypatch):
    read_text = Path.read_text
    def read(path, *args, **kwargs):
        if str(path) == '/proc/device-tree/model':
            return 'NVIDIA Orin NX\0'
        if str(path) == '/proc/sys/kernel/random/boot_id':
            return 'fixture-boot-id'
        return read_text(path, *args, **kwargs)
    commands = []
    def run(argv, **kwargs):
        commands.append(argv)
        if argv[0] in ('rfkill', 'nmcli'):
            raise FileNotFoundError()
        if argv[:2] == ['systemctl', 'is-active']:
            return SimpleNamespace(returncode=3, stdout='inactive\n')
        return SimpleNamespace(returncode=0, stdout='')
    monkeypatch.setattr(Path, 'read_text', read)
    monkeypatch.setattr(recovery.subprocess, 'run', run)
    result = recovery.audit()
    assert result['ssh_active'] == 'inactive'
    assert result['command_status']['ssh_active']['returncode'] == 3
    assert result['rfkill'] is None and result['command_status']['rfkill']['status'] == 'unavailable'
    assert result['wifi_autoconnect'] is None and result['boot_log_events']['events'] is None
    assert all(argv[0] in ('ip', 'nmcli', 'systemctl', 'ss', 'rfkill', 'journalctl') for argv in commands)
    assert all('--show-secrets' not in argv for argv in commands)


def test_console_reader_failure_does_not_discard_boot_network_audit(monkeypatch, tmp_path):
    saved, attempted = [], []
    transport = FixtureTransport(1)
    class Stop:
        def is_set(self):
            return bool(attempted)
        def wait(self, timeout=None):
            return self.is_set()
    class API:
        def __init__(self, *args):
            pass
        def request(self, *args):
            attempted.append(True)
            raise RuntimeError('project reader not ready')
    monkeypatch.setattr(connector, 'CACHE', tmp_path / 'absent.json')
    monkeypatch.setattr(connector, 'interface_subnet', lambda _: ('192.0.2.41', ipaddress.ip_network('192.0.2.0/24')))
    monkeypatch.setattr(connector, 'discover', lambda *a: ['192.0.2.77'])
    monkeypatch.setattr(connector, 'connect_verified', lambda *a: transport)
    monkeypatch.setattr(connector, 'restore_console', lambda *a: {'started': False, 'audit': {'boot_id': 'fixture-boot'}})
    monkeypatch.setattr(connector, 'save_connection', lambda *a: saved.append(a))
    monkeypatch.setattr(connector, 'RemoteAPI', API)
    link = connector.Link()
    connector.recover(link, Stop(), SimpleNamespace(host=None, interface='fixture', user='unitree',
                      remote_port=8765, manual_remote=False, retry_seconds=2), lambda: 'unused')
    assert saved == [('192.0.2.77', {'boot_id': 'fixture-boot'})]
    assert attempted and not link.snapshot()['ready'] and transport.closed


def test_project_start_failure_still_returns_boot_diagnostics(monkeypatch, capsys):
    monkeypatch.setattr(recovery.platform, 'machine', lambda: 'aarch64')
    monkeypatch.setattr(Path, 'read_text', lambda *a, **k: 'NVIDIA Orin NX')
    monkeypatch.setattr(recovery, 'audit', lambda: {'boot_id': 'fixture-boot', 'ssh_active': 'active'})
    def fail(**kwargs):
        raise RuntimeError('Project manifest fingerprint mismatch')
    monkeypatch.setattr(recovery, 'restore', fail)
    recovery.main()
    result = json.loads(capsys.readouterr().out)
    assert result['audit']['boot_id'] == 'fixture-boot'
    assert result['started'] is False and 'fingerprint mismatch' in result['restore_error']
