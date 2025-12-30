#!/usr/bin/env python3
"""
G1机器人语音重复功能（离线版 - 使用speech_recognition离线引擎）
如果Whisper安装失败，可以使用这个版本
"""

import sys
import subprocess
import os
import time

# 添加SDK路径
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

def speech_to_text_simple(audio_file):
    """简单的语音识别（使用speech_recognition + Google API）"""
    print("\n🔍 正在识别语音...")
    
    try:
        import speech_recognition as sr
        
        r = sr.Recognizer()
        
        # 转换音频格式（speech_recognition需要特定格式）
        converted_file = "/tmp/g1_voice_converted.wav"
        
        # 使用sox或ffmpeg转换格式
        converted = False
        if subprocess.run(['which', 'sox'], capture_output=True).returncode == 0:
            # 转换为16kHz单声道WAV（Google API要求）
            result = subprocess.run([
                'sox', audio_file, '-r', '16000', '-c', '1', converted_file
            ], capture_output=True, text=True)
            if result.returncode == 0 and os.path.exists(converted_file):
                converted = True
                print("  ✅ 使用sox转换音频格式")
        
        if not converted and subprocess.run(['which', 'ffmpeg'], capture_output=True).returncode == 0:
            result = subprocess.run([
                'ffmpeg', '-i', audio_file, '-ar', '16000', '-ac', '1', converted_file, '-y'
            ], capture_output=True, text=True, stderr=subprocess.DEVNULL)
            if result.returncode == 0 and os.path.exists(converted_file):
                converted = True
                print("  ✅ 使用ffmpeg转换音频格式")
        
        if not converted:
            # 如果没有转换工具，尝试直接使用原文件
            converted_file = audio_file
            print("  ⚠️ 未找到音频转换工具，尝试直接使用原文件")
            print("  💡 建议安装: sudo apt-get install flac sox ffmpeg")
        
        with sr.AudioFile(converted_file) as source:
            r.adjust_for_ambient_noise(source, duration=0.5)
            audio = r.record(source)
        
        try:
            # 使用Google Speech Recognition（需要网络）
            text = r.recognize_google(audio, language='zh-CN')
            print(f"✅ 识别成功: {text}")
            return text
        except sr.UnknownValueError:
            print("❌ 无法识别语音内容")
            return None
        except sr.RequestError as e:
            print(f"❌ 语音识别服务错误: {e}")
            print("   需要网络连接才能使用Google识别服务")
            return None
            
    except ImportError:
        print("❌ speech_recognition未安装")
        print("   安装方法: pip3 install SpeechRecognition")
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
        print("❌ SDK不可用")
        print("   可以使用HTTP API模式或启动TTS服务")
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
    
    parser = argparse.ArgumentParser(description='G1机器人语音重复（离线版）')
    parser.add_argument('--iface', default='wlan0', help='网络接口')
    parser.add_argument('--duration', type=int, default=5, help='录音时长（秒）')
    
    args = parser.parse_args()
    
    print("=" * 50)
    print("G1机器人语音重复功能（离线版）")
    print("=" * 50)
    print("注意: 此版本使用Google Speech Recognition，需要网络连接")
    print("")
    
    # 1. 录音
    audio_file = record_audio(args.duration)
    if not audio_file:
        return
    
    # 2. 语音识别
    text = speech_to_text_simple(audio_file)
    if not text:
        print("\n❌ 语音识别失败")
        print("   可以手动输入文字让机器人说话")
        manual_text = input("   请输入文字（直接回车跳过）: ").strip()
        if manual_text:
            text_to_speech_sdk(manual_text, args.iface)
        return
    
    # 3. 让机器人说话
    text_to_speech_sdk(text, args.iface)
    
    print("\n" + "=" * 50)
    print("✅ 完成！")
    print("=" * 50)

if __name__ == '__main__':
    main()

