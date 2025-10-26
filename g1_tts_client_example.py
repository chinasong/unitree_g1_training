#!/usr/bin/env python3
"""
G1 TTS 客户端示例
演示如何通过网络发送文字让G1机器人说话
"""

import requests
import json
import time


class G1TTSClient:
    """G1 TTS客户端"""
    
    def __init__(self, server_url="http://localhost:8080"):
        """
        初始化客户端
        
        Args:
            server_url: TTS服务器地址
        """
        self.server_url = server_url.rstrip('/')
        
    def speak(self, text, speaker_id=0):
        """
        让机器人说话
        
        Args:
            text: 要说的文字
            speaker_id: 说话人ID
            
        Returns:
            (success, message)
        """
        url = f"{self.server_url}/speak"
        data = {
            "text": text,
            "speaker_id": speaker_id
        }
        
        try:
            response = requests.post(url, json=data, timeout=10)
            result = response.json()
            return result.get('success', False), result.get('message', '')
        except Exception as e:
            return False, str(e)
    
    def set_volume(self, volume):
        """
        设置音量
        
        Args:
            volume: 音量 (0-100)
            
        Returns:
            (success, message)
        """
        url = f"{self.server_url}/volume/set"
        data = {"volume": volume}
        
        try:
            response = requests.post(url, json=data, timeout=10)
            result = response.json()
            return result.get('success', False), result.get('message', '')
        except Exception as e:
            return False, str(e)
    
    def get_volume(self):
        """
        获取音量
        
        Returns:
            (success, volume)
        """
        url = f"{self.server_url}/volume/get"
        
        try:
            response = requests.get(url, timeout=10)
            result = response.json()
            return result.get('success', False), result.get('volume', 0)
        except Exception as e:
            return False, 0
    
    def set_led(self, r, g, b):
        """
        设置LED灯颜色
        
        Args:
            r, g, b: RGB颜色值 (0-255)
            
        Returns:
            (success, message)
        """
        url = f"{self.server_url}/led/set"
        data = {"r": r, "g": g, "b": b}
        
        try:
            response = requests.post(url, json=data, timeout=10)
            result = response.json()
            return result.get('success', False), result.get('message', '')
        except Exception as e:
            return False, str(e)


def main():
    """示例程序"""
    # 创建客户端（确保TTS服务已启动）
    client = G1TTSClient("http://localhost:8080")
    
    print("🤖 G1 TTS 客户端示例")
    print("=" * 50)
    
    # 示例1: 让机器人说话
    print("\n📢 示例1: 让机器人说话")
    success, msg = client.speak("你好，我是宇树科技G1人形机器人！")
    print(f"结果: {msg}")
    time.sleep(3)
    
    # 示例2: 设置音量
    print("\n🔊 示例2: 设置音量")
    success, msg = client.set_volume(80)
    print(f"结果: {msg}")
    time.sleep(1)
    
    # 示例3: 获取音量
    print("\n📊 示例3: 获取音量")
    success, volume = client.get_volume()
    if success:
        print(f"当前音量: {volume}")
    time.sleep(1)
    
    # 示例4: 说英文
    print("\n🌍 示例4: 说英文")
    success, msg = client.speak("Hello! I am Unitree G1 humanoid robot.")
    print(f"结果: {msg}")
    time.sleep(3)
    
    # 示例5: 设置LED灯颜色
    print("\n💡 示例5: LED灯光效果")
    colors = [
        (255, 0, 0, "红色"),
        (0, 255, 0, "绿色"),
        (0, 0, 255, "蓝色"),
        (255, 255, 0, "黄色"),
        (255, 0, 255, "紫色")
    ]
    
    for r, g, b, name in colors:
        print(f"设置LED为{name}")
        success, msg = client.set_led(r, g, b)
        print(f"结果: {msg}")
        time.sleep(0.8)
    
    # 示例6: 说长句子
    print("\n📖 示例6: 说长句子")
    long_text = """
    欢迎来到宇树机器人实验室！
    我可以帮助你完成各种任务，
    包括语音交互、视觉识别、自主导航等功能。
    让我们一起探索机器人的无限可能！
    """
    success, msg = client.speak(long_text.strip())
    print(f"结果: {msg}")
    time.sleep(8)
    
    print("\n✅ 所有示例运行完毕！")


def test_with_curl_commands():
    """打印curl命令示例"""
    print("\n" + "=" * 60)
    print("📝 使用curl命令测试（在另一个终端运行）")
    print("=" * 60)
    
    print("\n1️⃣ 让机器人说话:")
    print("""curl -X POST http://localhost:8080/speak \\
     -H "Content-Type: application/json" \\
     -d '{"text": "你好，世界！"}'""")
    
    print("\n2️⃣ 设置音量:")
    print("""curl -X POST http://localhost:8080/volume/set \\
     -H "Content-Type: application/json" \\
     -d '{"volume": 70}'""")
    
    print("\n3️⃣ 获取音量:")
    print("curl http://localhost:8080/volume/get")
    
    print("\n4️⃣ 设置LED颜色为红色:")
    print("""curl -X POST http://localhost:8080/led/set \\
     -H "Content-Type: application/json" \\
     -d '{"r": 255, "g": 0, "b": 0}'""")
    
    print("\n5️⃣ 在浏览器打开测试页面:")
    print("http://localhost:8080")
    print()


if __name__ == '__main__':
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == '--curl':
        test_with_curl_commands()
    else:
        print("\n⚠️  确保TTS服务已启动：")
        print("    python3 g1_tts_service.py --iface eth0 --server\n")
        
        try:
            main()
        except requests.exceptions.ConnectionError:
            print("\n❌ 无法连接到TTS服务器！")
            print("   请先启动TTS服务：")
            print("   python3 g1_tts_service.py --iface eth0 --server\n")
        except KeyboardInterrupt:
            print("\n\n👋 程序中断")
        except Exception as e:
            print(f"\n❌ 错误: {e}")

