#!/usr/bin/env python3
"""
G1机器人语音重复功能
使用麦克风录音 -> 语音识别 -> TTS播放
"""

import sys
import subprocess
import os
import time
import json
import requests
from pathlib import Path

# 添加SDK路径（尝试多个可能的位置）
sdk_paths = [
    './externals/unitree_sdk2_python',
    '/home/unitree/unitree_sdk2_python',
    '/home/unitree/workspace/unitree_g1_training/externals/unitree_sdk2_python'
]

SDK_AVAILABLE = False
for sdk_path in sdk_paths:
    if os.path.exists(sdk_path):
        sys.path.insert(0, sdk_path)
        try:
            from unitree_sdk2py.core.channel import ChannelFactoryInitialize
            from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient
            SDK_AVAILABLE = True
            break
        except ImportError:
            continue

if not SDK_AVAILABLE:
    print("⚠️ 警告: unitree_sdk2_python未找到，将使用HTTP API模式")

class G1VoiceRepeat:
    def __init__(self, network_interface='wlan0', use_http_api=False):
        """
        初始化语音重复功能
        
        Args:
            network_interface: 网络接口名称
            use_http_api: 是否使用HTTP API（如果SDK不可用）
        """
        self.network_interface = network_interface
        self.use_http_api = use_http_api or not SDK_AVAILABLE
        self.audio_client = None
        self.tts_service_url = "http://localhost:8080"
        
        if not self.use_http_api:
            print("🤖 初始化G1机器人连接...")
            ChannelFactoryInitialize(0, network_interface)
            self.audio_client = AudioClient()
            self.audio_client.SetTimeout(10.0)
            self.audio_client.Init()
            print("✅ G1机器人连接成功！")
        else:
            print("📡 使用HTTP API模式")
            print(f"   确保TTS服务已启动: python3 g1_tts_service.py --iface {network_interface} --server")
    
    def record_audio(self, duration=5, output_file="/tmp/g1_voice_input.wav"):
        """
        录音
        
        Args:
            duration: 录音时长（秒）
            output_file: 输出文件路径
            
        Returns:
            成功返回文件路径，失败返回None
        """
        print(f"\n🎤 开始录音（{duration}秒）...")
        print("💡 请对着麦克风说话...")
        
        # 使用USB麦克风录音（单声道，48kHz）
        cmd = [
            'arecord', '-D', 'hw:0,0',
            '-d', str(duration),
            '-f', 'S16_LE',
            '-r', '48000',
            '-c', '1',
            '-t', 'wav',
            output_file
        ]
        
        result = subprocess.run(cmd, capture_output=True, text=True)
        
        if result.returncode == 0 and os.path.exists(output_file):
            file_size = os.path.getsize(output_file)
            if file_size > 10000:  # 大于10KB认为有声音
                print(f"✅ 录音成功！文件大小: {file_size} 字节")
                return output_file
            else:
                print("⚠️ 录音文件太小，可能没有声音输入")
                return None
        else:
            print(f"❌ 录音失败: {result.stderr}")
            return None
    
    def speech_to_text(self, audio_file):
        """
        语音识别（将语音转换为文字）
        
        Args:
            audio_file: 音频文件路径
            
        Returns:
            识别出的文字，失败返回None
        """
        print("\n🔍 正在识别语音...")
        
        # 方法1: 尝试使用speech_recognition库
        try:
            import speech_recognition as sr
            
            r = sr.Recognizer()
            
            with sr.AudioFile(audio_file) as source:
                # 调整环境噪音
                r.adjust_for_ambient_noise(source, duration=0.5)
                audio = r.record(source)
            
            # 尝试使用Google Speech Recognition（需要网络）
            try:
                text = r.recognize_google(audio, language='zh-CN')
                print(f"✅ 识别成功: {text}")
                return text
            except sr.UnknownValueError:
                print("❌ 无法识别语音内容")
                return None
            except sr.RequestError as e:
                print(f"⚠️ 语音识别服务错误: {e}")
                print("   尝试使用离线识别...")
                
                # 尝试使用离线识别（需要安装vosk）
                try:
                    import vosk
                    # 这里可以添加vosk离线识别
                    print("   离线识别需要配置vosk模型")
                    return None
                except ImportError:
                    print("   未安装vosk，无法使用离线识别")
                    return None
                    
        except ImportError:
            print("⚠️ speech_recognition库未安装")
            print("   安装方法: pip3 install SpeechRecognition")
            print("   或使用在线API（需要配置）")
            return None
    
    def text_to_speech(self, text):
        """
        文字转语音（让机器人说话）
        
        Args:
            text: 要说的文字
            
        Returns:
            成功返回True，失败返回False
        """
        if not text or not text.strip():
            print("⚠️ 文本为空，跳过")
            return False
        
        print(f"\n🗣️ 机器人说话: {text}")
        
        if self.use_http_api:
            # 使用HTTP API
            try:
                response = requests.post(
                    f"{self.tts_service_url}/speak",
                    json={"text": text, "speaker_id": 0},
                    timeout=10
                )
                if response.status_code == 200:
                    result = response.json()
                    if result.get('success'):
                        print("✅ 语音发送成功")
                        return True
                    else:
                        print(f"❌ 语音发送失败: {result.get('message')}")
                        return False
                else:
                    print(f"❌ HTTP请求失败: {response.status_code}")
                    return False
            except requests.exceptions.ConnectionError:
                print("❌ 无法连接到TTS服务")
                print(f"   请确保TTS服务已启动: python3 g1_tts_service.py --iface {self.network_interface} --server")
                return False
            except Exception as e:
                print(f"❌ 错误: {e}")
                return False
        else:
            # 使用SDK
            code = self.audio_client.TtsMaker(text, 0)
            if code == 0:
                print("✅ 语音发送成功")
                return True
            else:
                print(f"❌ 语音发送失败，错误码: {code}")
                return False
    
    def voice_repeat(self, duration=5):
        """
        完整的语音重复流程
        
        Args:
            duration: 录音时长（秒）
        """
        print("=" * 50)
        print("G1机器人语音重复功能")
        print("=" * 50)
        
        # 1. 录音
        audio_file = self.record_audio(duration)
        if not audio_file:
            print("\n❌ 录音失败，无法继续")
            return
        
        # 2. 语音识别
        text = self.speech_to_text(audio_file)
        if not text:
            print("\n❌ 语音识别失败，无法继续")
            print("   提示: 可以手动输入文字让机器人说话")
            manual_text = input("   请输入文字（直接回车跳过）: ").strip()
            if manual_text:
                self.text_to_speech(manual_text)
            return
        
        # 3. 文字转语音
        self.text_to_speech(text)
        
        print("\n" + "=" * 50)
        print("✅ 完成！")
        print("=" * 50)
    
    def interactive_mode(self):
        """交互式模式"""
        print("=" * 50)
        print("G1机器人语音交互模式")
        print("=" * 50)
        print("命令:")
        print("  直接回车 - 录音并重复")
        print("  输入文字 - 让机器人说话")
        print("  /quit 或 /exit - 退出")
        print("=" * 50)
        
        while True:
            try:
                user_input = input("\n>>> ").strip()
                
                if not user_input:
                    # 录音模式
                    self.voice_repeat(duration=5)
                elif user_input in ['/quit', '/exit', '/q']:
                    print("👋 再见！")
                    break
                else:
                    # 直接文字模式
                    self.text_to_speech(user_input)
                    
            except KeyboardInterrupt:
                print("\n\n👋 再见！")
                break
            except Exception as e:
                print(f"❌ 错误: {e}")


def main():
    import argparse
    
    parser = argparse.ArgumentParser(
        description='G1机器人语音重复功能 - 录音后让机器人重复',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 录音5秒并重复
  python3 g1_voice_repeat.py --iface wlan0
  
  # 交互式模式
  python3 g1_voice_repeat.py --iface wlan0 --interactive
  
  # 使用HTTP API模式（需要先启动TTS服务）
  python3 g1_voice_repeat.py --iface wlan0 --http-api
        """
    )
    
    parser.add_argument('--iface', default='wlan0',
                       help='网络接口名称（默认: wlan0）')
    parser.add_argument('--duration', type=int, default=5,
                       help='录音时长（秒，默认: 5）')
    parser.add_argument('--interactive', action='store_true',
                       help='交互式模式')
    parser.add_argument('--http-api', action='store_true',
                       help='使用HTTP API模式（需要TTS服务运行）')
    
    args = parser.parse_args()
    
    # 创建语音重复实例
    voice_repeat = G1VoiceRepeat(
        network_interface=args.iface,
        use_http_api=args.http_api
    )
    
    if args.interactive:
        voice_repeat.interactive_mode()
    else:
        voice_repeat.voice_repeat(duration=args.duration)


if __name__ == '__main__':
    main()

