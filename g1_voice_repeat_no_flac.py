#!/usr/bin/env python3
"""
G1机器人语音重复功能（无需FLAC版本）
使用Python直接处理音频转换
"""

import sys
import subprocess
import os
import time
import wave
import struct
import math

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

def convert_audio_python(input_file, output_file, target_rate=16000, target_channels=1):
    """
    使用Python直接转换音频格式（无需外部工具）
    
    Args:
        input_file: 输入文件（48kHz单声道）
        output_file: 输出文件（16kHz单声道）
        target_rate: 目标采样率
        target_channels: 目标声道数
    """
    try:
        # 读取输入文件
        with wave.open(input_file, 'rb') as wf_in:
            sample_rate = wf_in.getframerate()
            channels = wf_in.getnchannels()
            sample_width = wf_in.getsampwidth()
            frames = wf_in.readframes(wf_in.getnframes())
        
        # 转换为16位整数
        if sample_width == 2:
            samples = struct.unpack(f'<{len(frames)//2}h', frames)
        else:
            # 如果是其他格式，需要转换
            samples = list(struct.unpack(f'<{len(frames)//sample_width}h', 
                                        frames[:len(frames)//sample_width*sample_width]))
        
        # 如果是立体声，转换为单声道（取平均值）
        if channels == 2:
            samples = [int((samples[i] + samples[i+1]) / 2) 
                      for i in range(0, len(samples), 2)]
        
        # 重采样到目标采样率
        if sample_rate != target_rate:
            ratio = target_rate / sample_rate
            new_samples = []
            for i in range(int(len(samples) * ratio)):
                src_index = i / ratio
                idx1 = int(src_index)
                idx2 = min(idx1 + 1, len(samples) - 1)
                # 线性插值
                frac = src_index - idx1
                value = samples[idx1] * (1 - frac) + samples[idx2] * frac
                new_samples.append(int(value))
            samples = new_samples
        
        # 写入输出文件
        with wave.open(output_file, 'wb') as wf_out:
            wf_out.setnchannels(target_channels)
            wf_out.setsampwidth(2)  # 16位
            wf_out.setframerate(target_rate)
            wf_out.writeframes(struct.pack(f'<{len(samples)}h', *samples))
        
        return True
    except Exception as e:
        print(f"  ⚠️ Python音频转换失败: {e}")
        return False

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

def speech_to_text_no_flac(audio_file):
    """语音识别（无需FLAC）"""
    print("\n🔍 正在识别语音...")
    
    try:
        import speech_recognition as sr
        
        r = sr.Recognizer()
        
        # 转换音频格式（使用Python直接转换）
        converted_file = "/tmp/g1_voice_converted.wav"
        print("  使用Python转换音频格式...")
        
        if convert_audio_python(audio_file, converted_file, target_rate=16000, target_channels=1):
            print("  ✅ 音频格式转换成功")
        else:
            # 如果转换失败，尝试直接使用原文件
            converted_file = audio_file
            print("  ⚠️ 转换失败，尝试直接使用原文件")
        
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
        print("   可以使用HTTP API模式:")
        print("   python3 g1_tts_service.py --iface wlan0 --server")
        print("   然后使用HTTP API发送文字")
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
    
    parser = argparse.ArgumentParser(description='G1机器人语音重复（无需FLAC）')
    parser.add_argument('--iface', default='wlan0', help='网络接口')
    parser.add_argument('--duration', type=int, default=5, help='录音时长（秒）')
    
    args = parser.parse_args()
    
    print("=" * 50)
    print("G1机器人语音重复功能（无需FLAC版本）")
    print("=" * 50)
    print("注意: 使用Google Speech Recognition，需要网络连接")
    print("")
    
    # 1. 录音
    audio_file = record_audio(args.duration)
    if not audio_file:
        return
    
    # 2. 语音识别
    text = speech_to_text_no_flac(audio_file)
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

