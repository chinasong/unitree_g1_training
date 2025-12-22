#!/usr/bin/env python3
"""
G1 机器人文字转语音服务
支持通过本地网络发送文字让机器人说话
"""

import sys
import time
import argparse
import json
import os
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import parse_qs, urlparse
import threading

sys.path.append('./externals/unitree_sdk2_python')
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient


class G1TTSService:
    """G1机器人TTS服务类"""
    
    def __init__(self, network_interface, g1_ip=None):
        """
        初始化TTS服务
        
        Args:
            network_interface: 连接到G1的网络接口名称，例如 'eth0' 或 'enp3s0'
            g1_ip: G1机器人的IP地址（可选，用于单播模式）
        """
        print(f"🤖 初始化G1机器人连接...")
        
        # 如果提供了G1的IP，配置DDS使用单播模式
        if g1_ip:
            print(f"📡 使用单播模式连接G1: {g1_ip}")
            # 配置DDS单播模式
            cyclonedds_uri = f'''<CycloneDDS>
  <Domain>
    <General>
      <NetworkInterfaceAddress>{network_interface}</NetworkInterfaceAddress>
      <AllowMulticast>false</AllowMulticast>
    </General>
    <Discovery>
      <Peers>
        <Peer address="{g1_ip}"/>
      </Peers>
      <ParticipantIndex>auto</ParticipantIndex>
    </Discovery>
  </Domain>
</CycloneDDS>'''
            os.environ['CYCLONEDDS_URI'] = cyclonedds_uri
        
        ChannelFactoryInitialize(0, network_interface)
        
        self.audio_client = AudioClient()
        self.audio_client.SetTimeout(10.0)
        self.audio_client.Init()
        
        # 获取当前音量
        ret, volume_data = self.audio_client.GetVolume()
        if ret == 0 and volume_data:
            current_volume = volume_data.get('volume', 50)
            print(f"📢 当前音量: {current_volume}")
        else:
            current_volume = 50
            
        self.current_volume = current_volume
        print("✅ G1机器人连接成功！")
    
    def speak(self, text, speaker_id=0):
        """
        让机器人说话
        
        Args:
            text: 要说的文字（支持中英文）
            speaker_id: 说话人ID，默认为0
            
        Returns:
            成功返回0，失败返回错误码
        """
        if not text or not text.strip():
            print("⚠️ 文本为空，跳过")
            return -1
            
        print(f"🗣️ 机器人说话: {text}")
        code = self.audio_client.TtsMaker(text, speaker_id)
        
        if code == 0:
            print("✅ 语音发送成功")
        else:
            print(f"❌ 语音发送失败，错误码: {code}")
            
        return code
    
    def set_volume(self, volume):
        """
        设置音量
        
        Args:
            volume: 音量大小 (0-100)
            
        Returns:
            成功返回0，失败返回错误码
        """
        if not 0 <= volume <= 100:
            print(f"⚠️ 音量必须在0-100之间，当前值: {volume}")
            return -1
            
        print(f"🔊 设置音量: {volume}")
        code = self.audio_client.SetVolume(volume)
        
        if code == 0:
            self.current_volume = volume
            print(f"✅ 音量设置成功: {volume}")
        else:
            print(f"❌ 音量设置失败，错误码: {code}")
            
        return code
    
    def get_volume(self):
        """
        获取当前音量
        
        Returns:
            (错误码, 音量值)
        """
        ret, volume_data = self.audio_client.GetVolume()
        if ret == 0 and volume_data:
            volume = volume_data.get('volume', self.current_volume)
            return ret, volume
        return ret, self.current_volume
    
    def set_led_color(self, r, g, b):
        """
        设置LED灯带颜色
        
        Args:
            r: 红色值 (0-255)
            g: 绿色值 (0-255)
            b: 蓝色值 (0-255)
            
        Returns:
            成功返回0，失败返回错误码
        """
        print(f"💡 设置LED颜色: RGB({r}, {g}, {b})")
        code = self.audio_client.LedControl(r, g, b)
        
        if code == 0:
            print("✅ LED颜色设置成功")
        else:
            print(f"❌ LED颜色设置失败，错误码: {code}")
            
        return code


class TTSHTTPHandler(BaseHTTPRequestHandler):
    """HTTP请求处理器"""
    
    tts_service = None  # 将在服务器启动时设置
    
    def log_message(self, format, *args):
        """重写日志方法，使用自定义格式"""
        print(f"📡 {self.address_string()} - {format % args}")
    
    def _set_headers(self, status=200, content_type='application/json'):
        """设置HTTP响应头"""
        self.send_response(status)
        self.send_header('Content-type', content_type)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()
    
    def _send_json_response(self, data, status=200):
        """发送JSON响应"""
        self._set_headers(status)
        self.wfile.write(json.dumps(data, ensure_ascii=False).encode('utf-8'))
    
    def do_OPTIONS(self):
        """处理OPTIONS请求（CORS预检）"""
        self._set_headers()
    
    def do_GET(self):
        """处理GET请求"""
        parsed_path = urlparse(self.path)
        path = parsed_path.path
        
        if path == '/':
            # 返回API文档
            self._set_headers(content_type='text/html')
            html = """
            <!DOCTYPE html>
            <html>
            <head>
                <title>SONG 的 机器人 TTS 服务</title>
                <meta charset="utf-8">
                <style>
                    body { font-family: Arial, sans-serif; max-width: 800px; margin: 50px auto; padding: 20px; }
                    h1 { color: #333; }
                    .endpoint { background: #f4f4f4; padding: 15px; margin: 10px 0; border-radius: 5px; }
                    .method { color: #0066cc; font-weight: bold; }
                    code { background: #e8e8e8; padding: 2px 5px; border-radius: 3px; }
                    input, textarea { width: 100%; padding: 10px; margin: 5px 0; }
                    button { background: #0066cc; color: white; padding: 10px 20px; border: none; cursor: pointer; }
                    button:hover { background: #0052a3; }
                </style>
            </head>
            <body>
                <h1>🤖 SONG 的 机器人 TTS 服务</h1>
                <p>通过HTTP API控制G1机器人说话</p>
                
                <h2>📝 测试面板</h2>
                <div style="background: #f9f9f9; padding: 20px; border-radius: 5px;">
                    <h3>让机器人说话</h3>
                    <textarea id="textInput" rows="3" placeholder="输入要说的话...">你好，我是宇树G1机器人！</textarea>
                    <button onclick="speak()">🗣️ 说话</button>
                    
                    <h3>设置音量</h3>
                    <input type="range" id="volumeInput" min="0" max="100" value="50">
                    <span id="volumeValue">50</span>
                    <button onclick="setVolume()">🔊 设置音量</button>
                    <button onclick="getVolume()">📊 获取音量</button>
                    
                    <h3>LED灯控制</h3>
                    <input type="color" id="colorInput" value="#ff0000">
                    <button onclick="setLED()">💡 设置LED</button>
                </div>
                
                <h2>📡 API接口</h2>
                
                <div class="endpoint">
                    <span class="method">POST</span> /speak
                    <p>让机器人说话</p>
                    <pre>参数: {"text": "要说的话", "speaker_id": 0}</pre>
                </div>
                
                <div class="endpoint">
                    <span class="method">POST</span> /volume/set
                    <p>设置音量</p>
                    <pre>参数: {"volume": 50}</pre>
                </div>
                
                <div class="endpoint">
                    <span class="method">GET</span> /volume/get
                    <p>获取当前音量</p>
                </div>
                
                <div class="endpoint">
                    <span class="method">POST</span> /led/set
                    <p>设置LED灯颜色</p>
                    <pre>参数: {"r": 255, "g": 0, "b": 0}</pre>
                </div>
                
                <script>
                    document.getElementById('volumeInput').oninput = function() {
                        document.getElementById('volumeValue').textContent = this.value;
                    };
                    
                    function speak() {
                        const text = document.getElementById('textInput').value;
                        fetch('/speak', {
                            method: 'POST',
                            headers: {'Content-Type': 'application/json'},
                            body: JSON.stringify({text: text, speaker_id: 0})
                        }).then(r => r.json()).then(data => alert(data.message));
                    }
                    
                    function setVolume() {
                        const volume = parseInt(document.getElementById('volumeInput').value);
                        fetch('/volume/set', {
                            method: 'POST',
                            headers: {'Content-Type': 'application/json'},
                            body: JSON.stringify({volume: volume})
                        }).then(r => r.json()).then(data => alert(data.message));
                    }
                    
                    function getVolume() {
                        fetch('/volume/get')
                            .then(r => r.json())
                            .then(data => {
                                if(data.success) {
                                    document.getElementById('volumeInput').value = data.volume;
                                    document.getElementById('volumeValue').textContent = data.volume;
                                    alert('当前音量: ' + data.volume);
                                }
                            });
                    }
                    
                    function setLED() {
                        const color = document.getElementById('colorInput').value;
                        const r = parseInt(color.substr(1, 2), 16);
                        const g = parseInt(color.substr(3, 2), 16);
                        const b = parseInt(color.substr(5, 2), 16);
                        fetch('/led/set', {
                            method: 'POST',
                            headers: {'Content-Type': 'application/json'},
                            body: JSON.stringify({r: r, g: g, b: b})
                        }).then(r => r.json()).then(data => alert(data.message));
                    }
                </script>
            </body>
            </html>
            """
            self.wfile.write(html.encode('utf-8'))
            
        elif path == '/volume/get':
            # 获取音量
            ret, volume = self.tts_service.get_volume()
            if ret == 0:
                self._send_json_response({
                    'success': True,
                    'volume': volume,
                    'message': f'当前音量: {volume}'
                })
            else:
                self._send_json_response({
                    'success': False,
                    'message': f'获取音量失败，错误码: {ret}'
                }, 500)
                
        elif path == '/status':
            # 服务状态
            self._send_json_response({
                'success': True,
                'service': 'G1 TTS Service',
                'status': 'running',
                'volume': self.tts_service.current_volume
            })
            
        else:
            self._send_json_response({
                'success': False,
                'message': 'Not Found'
            }, 404)
    
    def do_POST(self):
        """处理POST请求"""
        content_length = int(self.headers['Content-Length'])
        post_data = self.rfile.read(content_length)
        
        try:
            data = json.loads(post_data.decode('utf-8'))
        except json.JSONDecodeError:
            self._send_json_response({
                'success': False,
                'message': 'Invalid JSON'
            }, 400)
            return
        
        parsed_path = urlparse(self.path)
        path = parsed_path.path
        
        if path == '/speak':
            # 让机器人说话
            text = data.get('text', '')
            speaker_id = data.get('speaker_id', 0)
            
            if not text:
                self._send_json_response({
                    'success': False,
                    'message': '文本不能为空'
                }, 400)
                return
            
            code = self.tts_service.speak(text, speaker_id)
            if code == 0:
                self._send_json_response({
                    'success': True,
                    'message': '语音发送成功',
                    'text': text
                })
            else:
                self._send_json_response({
                    'success': False,
                    'message': f'语音发送失败，错误码: {code}'
                }, 500)
                
        elif path == '/volume/set':
            # 设置音量
            volume = data.get('volume')
            if volume is None:
                self._send_json_response({
                    'success': False,
                    'message': '缺少volume参数'
                }, 400)
                return
            
            code = self.tts_service.set_volume(volume)
            if code == 0:
                self._send_json_response({
                    'success': True,
                    'message': f'音量设置成功: {volume}',
                    'volume': volume
                })
            else:
                self._send_json_response({
                    'success': False,
                    'message': f'音量设置失败，错误码: {code}'
                }, 500)
                
        elif path == '/led/set':
            # 设置LED颜色
            r = data.get('r', 0)
            g = data.get('g', 0)
            b = data.get('b', 0)
            
            code = self.tts_service.set_led_color(r, g, b)
            if code == 0:
                self._send_json_response({
                    'success': True,
                    'message': f'LED颜色设置成功: RGB({r}, {g}, {b})',
                    'color': {'r': r, 'g': g, 'b': b}
                })
            else:
                self._send_json_response({
                    'success': False,
                    'message': f'LED颜色设置失败，错误码: {code}'
                }, 500)
                
        else:
            self._send_json_response({
                'success': False,
                'message': 'Not Found'
            }, 404)


def run_http_server(tts_service, host='0.0.0.0', port=8080):
    """
    运行HTTP服务器
    
    Args:
        tts_service: TTS服务实例
        host: 监听地址
        port: 监听端口
    """
    TTSHTTPHandler.tts_service = tts_service
    
    server_address = (host, port)
    httpd = HTTPServer(server_address, TTSHTTPHandler)
    
    print(f"\n🌐 HTTP服务器启动成功！")
    print(f"📍 本地访问: http://localhost:{port}")
    print(f"📍 网络访问: http://{host}:{port}")
    print(f"\n使用说明:")
    print(f"  1. 在浏览器打开上述地址可以看到测试面板")
    print(f"  2. 使用POST请求发送TTS命令:")
    print(f"     curl -X POST http://localhost:{port}/speak \\")
    print(f"          -H 'Content-Type: application/json' \\")
    print(f"          -d '{{\"text\": \"你好，我是G1机器人\"}}'")
    print(f"\n按 Ctrl+C 停止服务\n")
    
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n\n👋 服务器关闭")
        httpd.shutdown()


def interactive_mode(tts_service):
    """
    交互式命令行模式
    
    Args:
        tts_service: TTS服务实例
    """
    print("\n🎤 进入交互模式")
    print("💡 输入文字让机器人说话，输入命令控制机器人")
    print("📝 命令列表:")
    print("   /volume <0-100>  - 设置音量")
    print("   /led <r> <g> <b> - 设置LED颜色 (0-255)")
    print("   /quit 或 /exit   - 退出程序")
    print("   直接输入文字     - 让机器人说话")
    print()
    
    while True:
        try:
            text = input(">>> ").strip()
            
            if not text:
                continue
            
            if text in ['/quit', '/exit', '/q']:
                print("👋 再见！")
                break
            
            elif text.startswith('/volume '):
                try:
                    volume = int(text.split()[1])
                    tts_service.set_volume(volume)
                except (IndexError, ValueError):
                    print("❌ 用法: /volume <0-100>")
            
            elif text.startswith('/led '):
                try:
                    parts = text.split()
                    r, g, b = int(parts[1]), int(parts[2]), int(parts[3])
                    tts_service.set_led_color(r, g, b)
                except (IndexError, ValueError):
                    print("❌ 用法: /led <r> <g> <b>  (0-255)")
            
            elif text.startswith('/'):
                print("❌ 未知命令，输入 /quit 退出")
            
            else:
                # 让机器人说话
                tts_service.speak(text)
                # 等待一下，让语音播放完
                time.sleep(len(text) * 0.2 + 1)
                
        except KeyboardInterrupt:
            print("\n\n👋 再见！")
            break
        except Exception as e:
            print(f"❌ 错误: {e}")


def main():
    parser = argparse.ArgumentParser(
        description='G1机器人TTS服务 - 通过网络让机器人说话',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 启动HTTP服务器（推荐）
  python3 g1_tts_service.py --iface eth0 --server --port 8080
  
  # 交互式命令行模式
  python3 g1_tts_service.py --iface eth0 --interactive
  
  # 直接让机器人说一句话
  python3 g1_tts_service.py --iface eth0 --speak "你好，我是G1机器人"
        """
    )
    
    parser.add_argument('--iface', required=True, 
                       help='网络接口名称（例如: eth0, enp3s0, en0）')
    parser.add_argument('--g1-ip', type=str,
                       help='G1机器人的IP地址（用于手机热点等需要单播模式的场景）')
    parser.add_argument('--server', action='store_true',
                       help='启动HTTP服务器模式')
    parser.add_argument('--interactive', action='store_true',
                       help='启动交互式命令行模式')
    parser.add_argument('--speak', type=str,
                       help='直接让机器人说一句话')
    parser.add_argument('--port', type=int, default=8080,
                       help='HTTP服务器端口（默认: 8080）')
    parser.add_argument('--host', type=str, default='0.0.0.0',
                       help='HTTP服务器监听地址（默认: 0.0.0.0）')
    parser.add_argument('--volume', type=int,
                       help='设置音量 (0-100)')
    
    args = parser.parse_args()
    
    # 初始化TTS服务
    g1_ip = getattr(args, 'g1_ip', None)
    tts_service = G1TTSService(args.iface, g1_ip=g1_ip)
    
    # 设置音量（如果指定）
    if args.volume is not None:
        tts_service.set_volume(args.volume)
    
    # 根据参数选择运行模式
    if args.speak:
        # 直接说话模式
        tts_service.speak(args.speak)
        time.sleep(len(args.speak) * 0.2 + 2)  # 等待语音播放完
        
    elif args.server:
        # HTTP服务器模式
        run_http_server(tts_service, args.host, args.port)
        
    elif args.interactive:
        # 交互式模式
        interactive_mode(tts_service)
        
    else:
        # 默认启动HTTP服务器
        print("💡 未指定模式，默认启动HTTP服务器")
        print("   使用 --interactive 可以进入交互模式")
        print("   使用 --speak 可以直接说一句话")
        run_http_server(tts_service, args.host, args.port)


if __name__ == '__main__':
    main()

