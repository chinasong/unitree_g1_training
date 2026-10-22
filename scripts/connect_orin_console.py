#!/usr/bin/env python3
"""Persistent, pinned Mac → Orin connection. No routes or system services change."""
import argparse
import base64
from concurrent.futures import ThreadPoolExecutor
import getpass
import hashlib
import http.client
import ipaddress
import json
import os
from pathlib import Path
import platform
import re
import select
import socket
import socketserver
import subprocess
import threading
import time
from urllib.parse import urlsplit
import webbrowser

import paramiko

PROJECT = Path(__file__).resolve().parents[1]
CACHE = PROJECT / '.g1-console/connection.json'
# This robot's address and pinned SSH host key live only on this Mac (.g1-console/ is not committed).
ORIN = PROJECT / '.g1-console/orin.json'


def orin_identity(path=None):
    """{'host', 'fingerprint'} of the verified Orin; refuses to guess when the local file is missing or malformed."""
    path = path or ORIN
    try:
        data = json.loads(path.read_text())
    except FileNotFoundError:
        raise RuntimeError(f'缺少本地 Orin 配置 {path}：写入 {{"host": "<Orin IP>", "fingerprint": "SHA256:<现场核对的主机指纹>"}}') from None
    except ValueError:
        raise RuntimeError(f'本地 Orin 配置 {path} 不是有效 JSON') from None
    fingerprint_value, host = data.get('fingerprint'), data.get('host')
    if not isinstance(fingerprint_value, str) or not re.fullmatch(r'SHA256:[A-Za-z0-9+/]{43}', fingerprint_value):
        raise RuntimeError(f'本地 Orin 配置 {path} 缺少有效的 SHA256 主机指纹')
    try:
        ipaddress.ip_address(host)
    except (TypeError, ValueError):
        raise RuntimeError(f'本地 Orin 配置 {path} 缺少有效的 IP 地址') from None
    return dict(host=host, fingerprint=fingerprint_value)


def verify_service(state, manual_remote=False, verify_only=False, accessory_test=False, damping_test=False):
    verify_configuration(state, manual_remote, verify_only, accessory_test, damping_test)
    if state.get('mode') != 'robot':
        raise RuntimeError('目标不是实机服务；尚未开放转发')


def verify_configuration(state, manual_remote=False, verify_only=False, accessory_test=False, damping_test=False):
    if not isinstance(state, dict):
        raise RuntimeError('项目状态格式不符；不开放转发')
    if damping_test and (not manual_remote or verify_only or accessory_test):
        raise RuntimeError('阻尼测试需要独立人工遥控范围')
    if accessory_test and (not manual_remote or verify_only):
        raise RuntimeError("声光测试需要独立人工遥控范围")
    if manual_remote:
        remote = state.get('manual_remote')
        if state.get('read_only') is not False or not isinstance(remote, dict) or remote.get('enabled') is not True:
            raise RuntimeError('现有服务未启用软件遥控；保留当前服务，不自动切换模式')
        if not (verify_only or accessory_test or damping_test) and any(
                remote.get(flag) is not False for flag in ('verify_only', 'accessory_test', 'damping_test')):
            raise RuntimeError('现有服务是受限测试范围，不是完整人工遥控；不开放转发，不自动切换模式')
        if verify_only and remote.get('verify_only') is not True:
            raise RuntimeError('现有人工遥控服务未锁定接口验证；不开放转发，不自动切换模式')
        if damping_test and (remote.get('damping_test') is not True or remote.get('accessory_test') is not False
                             or remote.get('verify_only') is not False or remote.get('allowed_write_operations') != ['damp']):
            raise RuntimeError('单次阻尼范围不符；不开放转发，不自动切换模式')
        if accessory_test and (remote.get('accessory_test') is not True or remote.get('verify_only') is not False
                               or remote.get('allowed_write_operations') != ['led', 'tts']):
            raise RuntimeError('现有服务声光测试范围不符；不开放转发，不自动切换模式')
    elif state.get('read_only') is not True:
        raise RuntimeError('现有服务未锁定只读；保留当前服务，不自动切换模式')


class IdentityMismatch(paramiko.SSHException):
    pass


def fingerprint(key):
    return 'SHA256:' + base64.b64encode(hashlib.sha256(key.asbytes()).digest()).decode().rstrip('=')


def bound_socket(host, interface, timeout=2):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        if platform.system() == 'Darwin':
            sock.setsockopt(socket.IPPROTO_IP, getattr(socket, 'IP_BOUND_IF', 25), socket.if_nametoindex(interface))
        sock.settimeout(timeout)
        sock.connect((host, 22))
        return sock
    except BaseException:
        sock.close()
        raise


def connect_verified(host, user, password, interface, expected=None):
    """Complete host verification BEFORE asking for or sending a password."""
    expected = expected or orin_identity()['fingerprint']
    transport = paramiko.Transport(bound_socket(host, interface, timeout=3))
    try:
        transport.start_client(timeout=5)
        if not transport.is_active() or fingerprint(transport.get_remote_server_key()) != expected:
            raise IdentityMismatch('Orin 主机指纹不匹配')
        transport.auth_password(user, password(), fallback=False)
        if not transport.is_authenticated():
            raise paramiko.AuthenticationException('SSH authentication failed')
        transport.set_keepalive(5)
        return transport
    except BaseException:
        transport.close()
        raise


def interface_subnet(interface):
    output = subprocess.check_output(['/sbin/ifconfig', interface], text=True, timeout=4)
    match = re.search(r'\binet (\d+\.\d+\.\d+\.\d+) netmask (0x[0-9a-f]+|[\d.]+)', output)
    if not match:
        raise RuntimeError('指定 Wi-Fi 网卡尚无 IPv4 地址，等待网络恢复')
    ip, mask = match.groups()
    if mask.startswith('0x'):
        mask = str(ipaddress.IPv4Address(int(mask, 16)))
    network = ipaddress.IPv4Network(f'{ip}/{mask}', strict=False)
    if not network.is_private or not 24 <= network.prefixlen <= 30:
        raise RuntimeError('仅自动发现当前 Wi-Fi 的私有 /24–/30 网络，最多 254 个地址')
    return ip, network


def discover(network, own_ip, interface, preferred=()):
    def open_ssh(host):
        try:
            bound_socket(host, interface, timeout=.6).close()
            return host
        except OSError:
            return None
    hosts = [str(ip) for ip in network.hosts() if str(ip) != own_ip]
    ordered = list(dict.fromkeys([h for h in preferred if h in hosts] + hosts))
    with ThreadPoolExecutor(max_workers=24) as pool:
        return [host for host in pool.map(open_ssh, ordered) if host]


class RemoteAPI:
    def __init__(self, transport, port):
        self.transport, self.port, self.cookie = transport, port, ''

    def request(self, path, data=None):
        channel = self.transport.open_channel('direct-tcpip', ('127.0.0.1', self.port), ('127.0.0.1', 0), timeout=4)
        channel.settimeout(5)
        connection = http.client.HTTPConnection('127.0.0.1', self.port, timeout=5)
        connection.sock = channel
        try:
            headers = {'Cookie': self.cookie, 'X-G1-Intent': 'operator', 'Content-Type': 'application/json'}
            connection.request('GET' if data is None else 'POST', path,
                               None if data is None else json.dumps(data), headers)
            response = connection.getresponse()
            if path == '/api/session' and response.getheader('Set-Cookie'):
                self.cookie = response.getheader('Set-Cookie').split(';', 1)[0]
            body = response.read(2_000_001)
            if len(body) > 2_000_000 or response.status != 200:
                raise RuntimeError(f'项目接口 {path} 返回 HTTP {response.status}')
            return json.loads(body)
        finally:
            connection.close()


def prepare_readers(api, manual=False, initialize=True, verify_only=False, accessory_test=False, damping_test=False):
    """Initialize an absent reader only after this connector started the service."""
    api.request('/api/session')
    state = api.request('/api/state')
    verify_configuration(state, manual, verify_only, accessory_test, damping_test)
    if state.get('mode') == 'disconnected' and initialize:
        control, remote, navigation = (state.get(key) for key in ('control', 'manual_remote', 'navigation'))
        if (not isinstance(control, dict) or 'owner' not in control or control['owner'] is not None
                or not isinstance(remote, dict) or remote.get('armed') is not False
                or not isinstance(navigation, dict) or navigation.get('status') != 'idle'):
            raise RuntimeError('服务存在控制授权或任务状态；不自动初始化 DDS')
        state = api.request('/api/connect', {'mode': 'robot', 'interface': 'eth0'})
    if state.get('mode') != 'disconnected':
        verify_service(state, manual, verify_only, accessory_test, damping_test)
    return state


def restore_console(transport, manual, port, verify_only=False, accessory_test=False, damping_test=False):
    source = (PROJECT / 'scripts/orin_console_recovery.py').read_text()
    source += f'\nmain(manual={manual!r}, port={port!r}, verify_only={verify_only!r}, accessory_test={accessory_test!r}, damping_test={damping_test!r})\n'
    with transport.open_session(timeout=5) as channel:
        channel.settimeout(45)
        channel.exec_command('/usr/bin/python3 -')
        channel.sendall(source.encode())
        channel.shutdown_write()
        output, errors = bytearray(), bytearray()
        deadline = time.monotonic() + 45
        while not channel.exit_status_ready() or channel.recv_ready() or channel.recv_stderr_ready():
            if time.monotonic() > deadline or len(output) + len(errors) > 100000:
                raise RuntimeError('项目恢复检查超时；不终止任何远端进程')
            if channel.recv_ready():
                output.extend(channel.recv(32768))
            if channel.recv_stderr_ready():
                errors.extend(channel.recv_stderr(32768))
            time.sleep(.02)
        if channel.recv_exit_status() != 0:
            raise RuntimeError('Orin 项目恢复检查失败；请核对已有发布与 console.log')
    result = json.loads(output)
    if not isinstance(result, dict) or type(result.get('started')) is not bool:
        raise RuntimeError('项目恢复回执格式不符；不开放转发')
    return result


class Link:
    def __init__(self, manual=False):
        self.lock = threading.Lock()
        self.transport = None
        self.data = dict(ready=False, phase='等待发现 Orin', detail='身体反馈未知', host=None,
                         verified=False, attempt=0, updated_at=time.time(), manual_remote=manual,
                         last_connected_at=None)

    def update(self, **values):
        with self.lock:
            self.data.update(values, updated_at=time.time())

    def offline(self, detail):
        with self.lock:
            old, self.transport = self.transport, None
            self.data.update(ready=False, verified=False, phase='等待 Orin 恢复连接',
                             detail=detail, updated_at=time.time())
        if old:
            old.close()  # Close old WebSockets too; no requests are buffered or replayed.

    def publish(self, transport, host, state):
        with self.lock:
            self.transport = transport
            self.data.update(ready=True, verified=True, phase='已核验 Orin，控制台已恢复', host=host,
                             detail='身体反馈在线' if state.get('connected') else '等待新的身体反馈，当前状态未知',
                             last_connected_at=time.time(), updated_at=time.time())

    def snapshot(self):
        with self.lock:
            return dict(self.data)

    def current(self):
        with self.lock:
            return self.transport if self.data['ready'] else None


WAIT_HTML = '''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>G1 · 连接恢复</title><style>body{margin:0;background:#11151d;color:#e8edf4;font:16px system-ui;line-height:1.7}main{max-width:760px;margin:12vh auto;padding:32px}small{color:#9ba9bc}h1{font-size:30px}section{padding:24px;border:1px solid #354257;border-radius:16px;background:#192230}strong{color:#91c5ff}p{margin:12px 0}#detail{white-space:pre-wrap}footer{margin-top:24px;color:#a9b5c6}</style>
<main><small>G1 EDU 23 · Mac 连接管理</small><h1 id="phase">等待 Orin 恢复连接</h1><section><strong>未连接实机 · 身体反馈未知</strong><p id="detail">正在核对 Wi-Fi 网络并发现已验证的 Orin。</p><small id="meta"></small></section><footer><p>机器人重启后，本窗口持续重试。发现地址后先核验设备身份，再恢复项目控制台与只读反馈。</p><p>不会恢复控制权、运动授权或上次动作。保持此连接程序运行，Quantumult X 无需改动。</p><p>如果 Orin 没有连入 Wi-Fi 或 SSH 未启动，软件会继续等待；App 视频在线不能证明 Orin 开发计算单元在线。</p></footer></main><script src="/__connection.js"></script></html>'''
WAIT_JS = '''async function refresh(){try{const s=await fetch('/__connection',{cache:'no-store'}).then(r=>r.json());if(s.ready){location.reload();return;}document.getElementById('phase').textContent=s.phase;document.getElementById('detail').textContent=s.detail;document.getElementById('meta').textContent='发现轮次 '+s.attempt+' · '+new Date(s.updated_at*1000).toLocaleTimeString('zh-CN',{hour12:false})+(s.host?' · 最近核验地址 '+s.host:'');}catch{}setTimeout(refresh,2000);}refresh();'''


class Server(socketserver.ThreadingTCPServer):
    daemon_threads = True
    allow_reuse_address = True


def handler_for(link, remote_port):
    class Forward(socketserver.BaseRequestHandler):
        def reply(self, status, content_type, body, head=False):
            data = body.encode()
            headers = (f'HTTP/1.1 {status}\r\nContent-Type: {content_type}\r\nContent-Length: {len(data)}\r\n'
                       "Cache-Control: no-store\r\nConnection: close\r\nX-Content-Type-Options: nosniff\r\n"
                       "Content-Security-Policy: default-src 'self'; style-src 'unsafe-inline'; frame-ancestors 'none'\r\n\r\n")
            self.request.sendall(headers.encode() + (b'' if head else data))

        def handle(self):
            try:
                self.request.settimeout(4)
                initial = b''
                while b'\r\n\r\n' not in initial:
                    chunk = self.request.recv(8192)
                    if not chunk:
                        return
                    initial += chunk
                    if len(initial) > 65536:
                        return
                lines = initial.split(b'\r\n\r\n', 1)[0].decode('iso-8859-1').split('\r\n')
                method, path, _ = lines[0].split(' ', 2)
                headers = dict(line.lower().split(':', 1) for line in lines[1:] if ':' in line)
                allowed = (f'127.0.0.1:{self.server.server_address[1]}', f'localhost:{self.server.server_address[1]}')
                if headers.get('host', '').strip() not in allowed:
                    self.reply('403 Forbidden', 'text/plain; charset=utf-8', '仅接受本机控制台地址')
                    return
                if headers.get('origin') and headers['origin'].strip() not in tuple('http://' + h for h in allowed):
                    self.reply('403 Forbidden', 'text/plain; charset=utf-8', '拒绝跨站请求')
                    return
                resource = urlsplit(path).path
                if resource in ('/__connection', '/__connection.js') and method == 'GET':
                    self.reply('200 OK', 'application/json' if resource == '/__connection' else 'text/javascript',
                               json.dumps(link.snapshot(), ensure_ascii=False) if resource == '/__connection' else WAIT_JS)
                    return
                transport = link.current()
                if transport is None:
                    if method in ('GET', 'HEAD') and not resource.startswith('/api/'):
                        self.reply('503 Service Unavailable', 'text/html; charset=utf-8', WAIT_HTML, method == 'HEAD')
                    else:
                        self.reply('503 Service Unavailable', 'application/json', json.dumps(dict(error=dict(
                            code='ORIN_OFFLINE', message='Orin 连接中断，未转发请求；等待恢复，不重放动作')), ensure_ascii=False))
                    return
                channel = transport.open_channel('direct-tcpip', ('127.0.0.1', remote_port), self.client_address, timeout=4)
                with channel:
                    self.request.settimeout(None)
                    channel.sendall(initial)
                    while transport.is_active():
                        readers, _, _ = select.select([self.request, channel], [], [], 1)
                        for source in readers:
                            data = source.recv(65536)
                            if not data:
                                return
                            (channel if source is self.request else self.request).sendall(data)
            except (OSError, EOFError, ValueError, paramiko.SSHException):
                return
    return Forward


def save_connection(host, audit, path=CACHE):
    # Address and network metadata only; never passwords, cookies or SSH keys.
    path.parent.mkdir(exist_ok=True, parents=True)
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(dict(host=host, fingerprint=orin_identity()['fingerprint'], audit=audit,
                                         verified_at=time.time()), ensure_ascii=False, indent=2))
    temporary.replace(path)


def recover(link, stop, args, password):
    attempt, last_host, identity = 0, args.host, orin_identity()
    try:
        cached = json.loads(CACHE.read_text())
        if cached.get('fingerprint') == identity['fingerprint']:
            last_host = last_host or cached.get('host')
    except (OSError, ValueError):
        pass
    while not stop.is_set():
        attempt += 1
        link.update(attempt=attempt, phase='发现已验证的 Orin', detail='检查当前 Wi-Fi 的 SSH 入口；不会向其他设备发送密码')
        try:
            own_ip, network = interface_subnet(args.interface)
            candidates = discover(network, own_ip, args.interface, (args.host, last_host, identity['host']))
            connected = False
            for host in candidates:
                if stop.is_set():
                    return
                transport = None
                try:
                    transport = connect_verified(host, args.user, password, args.interface)
                    link.update(phase='设备身份已核验，恢复项目控制台', detail='只检查本项目服务，不操作宇树系统服务')
                    verification = {'verify_only': True} if getattr(args, 'verify_only', False) else {}
                    if getattr(args, 'accessory_test', False):
                        verification['accessory_test'] = True
                    if getattr(args, 'damping_test', False):
                        verification['damping_test'] = True
                    restored = restore_console(transport, args.manual_remote, args.remote_port, **verification)
                    # Preserve boot diagnostics even if the console/DDS reader is not ready.
                    save_connection(host, restored.get('audit'))
                    if restored.get('restore_error'):
                        raise RuntimeError(restored['restore_error'])
                    api = RemoteAPI(transport, args.remote_port)
                    for retry in range(10):
                        try:
                            state = prepare_readers(api, args.manual_remote, initialize=restored['started'], **verification)
                            break
                        except (OSError, EOFError, paramiko.ChannelException):
                            if retry == 9 or stop.wait(1):
                                raise
                    last_host = host
                    link.publish(transport, host, state)
                    print(f'Orin 已恢复：{host}；项目服务启动={restored["started"]}；不恢复动作授权', flush=True)
                    connected = True
                    while not stop.wait(2):
                        state = api.request('/api/state')
                        verify_configuration(state, args.manual_remote, **verification)
                        if state.get('mode') not in ('robot', 'disconnected'):
                            raise RuntimeError('目标已变更为非实机环境；停止转发')
                        link.update(detail='身体反馈在线' if state.get('connected') else '项目服务在线，等待新的身体反馈；状态未知')
                    return
                except IdentityMismatch:
                    pass
                except paramiko.AuthenticationException:
                    link.offline('Orin 身份已核验，但 SSH 认证失败。请重新运行连接程序并输入正确密码。')
                    stop.wait()
                    return
                except (OSError, EOFError, ValueError, RuntimeError, http.client.HTTPException, paramiko.SSHException) as exc:
                    link.offline(f'连接未就绪：{str(exc)[:180]}')
                finally:
                    if transport:
                        transport.close()
                if connected:
                    break
            if not connected and not link.snapshot()['detail'].startswith('连接未就绪'):
                link.offline(f'{network} 暂未发现已核验的 Orin SSH。App 显示的地址尚不能证明是 Orin；继续等待。')
        except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as exc:
            link.offline(str(exc)[:180])
        stop.wait(args.retry_seconds)


def main():
    parser = argparse.ArgumentParser(description='Mac → Orin 持续连接；发现地址、核验身份、恢复项目服务和只读反馈')
    parser.add_argument('--host', help='优先检查地址；仍扫描当前 Wi-Fi，且必须通过原有设备指纹核验')
    parser.add_argument('--user', default='unitree')
    parser.add_argument('--interface', default='en0')
    parser.add_argument('--port', type=int, default=18765)
    parser.add_argument('--remote-port', type=int, default=8765)
    parser.add_argument('--retry-seconds', type=float, default=10)
    parser.add_argument('--open', action='store_true')
    parser.add_argument('--manual-remote', action='store_true', help='恢复软件遥控服务；不取得控制权或恢复动作授权')
    parser.add_argument('--verify-only', action='store_true', help='要求最终下发锁定在接口验证阶段；不会降级到可动作模式')
    parser.add_argument('--accessory-test', action='store_true', help='仅开放语音和 RGB 灯，验证并保持最终下发范围')
    parser.add_argument('--damping-test', action='store_true', help='严格保持仅单次官方阻尼测试范围，不自动执行')
    args = parser.parse_args()
    if args.damping_test and (not args.manual_remote or args.verify_only or args.accessory_test):
        parser.error('--damping-test 需要独立 --manual-remote 范围')
    if args.accessory_test and (not args.manual_remote or args.verify_only):
        parser.error('--accessory-test 需要 --manual-remote，且不能同时选择 --verify-only')
    if args.verify_only and not args.manual_remote:
        parser.error('--verify-only 需要 --manual-remote')
    if not all(1024 <= p <= 65535 for p in (args.port, args.remote_port)) or not 2 <= args.retry_seconds <= 120:
        parser.error('端口须为 1024–65535，重试间隔须为 2–120 秒')
    secret = os.environ.pop('G1_SSH_PASSWORD', None)

    def password():
        nonlocal secret
        if secret is None:
            secret = getpass.getpass(f'已核验 Orin，{args.user} SSH 密码（仅当前进程内存，不保存）: ')
        return secret

    link, stop = Link(args.manual_remote), threading.Event()
    try:
        server = Server(('127.0.0.1', args.port), handler_for(link, args.remote_port))
    except OSError as exc:
        parser.exit(1, f'本机端口 {args.port} 已占用或不可用：{exc}。请检查已有连接窗口，勿重复启动。\n')
    worker = threading.Thread(target=recover, args=(link, stop, args, password), daemon=True)
    worker.start()
    url = f'http://127.0.0.1:{args.port}/'
    print(f'G1 持续连接：{url}；保持此窗口运行，机器人重启后自动重试。Quantumult X 无需改动。', flush=True)
    if args.open:
        webbrowser.open(url)
    try:
        server.serve_forever(poll_interval=.3)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        link.offline('Mac 连接程序已关闭')
        server.server_close()
        worker.join(timeout=2)
        secret = None


if __name__ == '__main__':
    main()
