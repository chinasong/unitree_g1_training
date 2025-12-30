#!/usr/bin/env python3
"""
G1机器人语音重复功能（简化版）
使用whisper进行语音识别（离线，无需网络）
"""

import sys
import subprocess
import os
import time

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

def record_audio(duration=5, output_file="/tmp/g1_voice_input.wav"):
    """录音"""
    print(f"\n🎤 开始录音（{duration}秒）...")
    print("💡 请对着麦克风说话...")
    
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
        if file_size > 10000:
            print(f"✅ 录音成功！文件大小: {file_size} 字节")
            return output_file
        else:
            print("⚠️ 录音文件太小")
            return None
    else:
        print(f"❌ 录音失败: {result.stderr}")
        return None

def speech_to_text_whisper(audio_file):
    """使用whisper进行语音识别（离线）"""
    print("\n🔍 正在识别语音（使用Whisper）...")
    
    try:
        import whisper
        
        # 加载模型（首次使用会下载）
        print("   加载Whisper模型...")
        model = whisper.load_model("base")  # 可选: tiny, base, small, medium, large
        
        # 识别
        result = model.transcribe(audio_file, language="zh")
        text = result["text"].strip()
        
        if text:
            print(f"✅ 识别成功: {text}")
            return text
        else:
            print("❌ 未识别到文字")
            return None
            
    except ImportError:
        print("❌ whisper未安装")
        print("   安装方法: pip3 install openai-whisper")
        print("   或使用在线识别: pip3 install SpeechRecognition")
        return None
    except Exception as e:
        print(f"❌ 识别失败: {e}")
        return None

def text_to_speech_sdk(text, network_interface='wlan0'):
    """使用SDK让机器人说话"""
    if not text or not text.strip():
        return False
    
    print(f"\n🗣️ 机器人说话: {text}")
    
    if not SDK_AVAILABLE:
        print("❌ SDK不可用，无法让机器人说话")
        return False
    
    try:
        ChannelFactoryInitialize(0, network_interface)
        audio_client = AudioClient()
        audio_client.SetTimeout(10.0)
        audio_client.Init()
        
        code = audio_client.TtsMaker(text, 0)
        if code == 0:
            print("✅ 语音发送成功")
            return True
        else:
            print(f"❌ 语音发送失败，错误码: {code}")
            return False
    except Exception as e:
        print(f"❌ 错误: {e}")
        return False

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='G1机器人语音重复（使用Whisper）')
    parser.add_argument('--iface', default='wlan0', help='网络接口')
    parser.add_argument('--duration', type=int, default=5, help='录音时长（秒）')
    
    args = parser.parse_args()
    
    print("=" * 50)
    print("G1机器人语音重复功能（Whisper版）")
    print("=" * 50)
    
    # 1. 录音
    audio_file = record_audio(args.duration)
    if not audio_file:
        return
    
    # 2. 语音识别
    text = speech_to_text_whisper(audio_file)
    if not text:
        print("\n❌ 语音识别失败")
        return
    
    # 3. 让机器人说话
    text_to_speech_sdk(text, args.iface)
    
    print("\n" + "=" * 50)
    print("✅ 完成！")
    print("=" * 50)

if __name__ == '__main__':
    main()

