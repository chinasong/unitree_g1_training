#!/usr/bin/env python3
"""
G1机器人麦克风测试脚本
测试Type-C麦克风录音功能
"""

import sys
import subprocess
import time
import os

def test_microphone():
    """测试麦克风录音"""
    print("=" * 50)
    print("G1机器人麦克风测试")
    print("=" * 50)
    print()
    
    # 检查录音工具
    print("【1. 检查录音工具】")
    tools = {
        'arecord': 'ALSA录音工具',
        'sox': 'SoX音频工具',
        'pyaudio': 'PyAudio库'
    }
    
    available_tools = []
    for tool, desc in tools.items():
        if tool == 'pyaudio':
            try:
                import pyaudio
                available_tools.append((tool, desc, True))
                print(f"  ✅ {tool} ({desc}) - 可用")
            except ImportError:
                print(f"  ❌ {tool} ({desc}) - 未安装")
        else:
            result = subprocess.run(['which', tool], capture_output=True)
            if result.returncode == 0:
                available_tools.append((tool, desc, True))
                print(f"  ✅ {tool} ({desc}) - 可用")
            else:
                print(f"  ❌ {tool} ({desc}) - 未安装")
    print()
    
    # 列出音频输入设备
    print("【2. 列出音频输入设备】")
    print("PulseAudio输入设备:")
    result = subprocess.run(['pactl', 'list', 'short', 'sources'], 
                          capture_output=True, text=True)
    if result.returncode == 0:
        sources = result.stdout.strip().split('\n')
        if sources and sources[0]:
            for source in sources:
                print(f"  {source}")
        else:
            print("  未找到输入设备")
    else:
        print("  无法获取PulseAudio设备列表")
    print()
    
    print("ALSA输入设备:")
    result = subprocess.run(['arecord', '-l'], capture_output=True, text=True)
    if result.returncode == 0:
        print(result.stdout)
    else:
        print("  无法获取ALSA设备列表")
    print()
    
    # 测试录音
    print("【3. 测试录音功能】")
    test_file = "/tmp/g1_mic_test.wav"
    
    # 使用arecord录音
    if any(tool[0] == 'arecord' for tool in available_tools):
        print(f"  使用arecord录音5秒...")
        print("  💡 请对着麦克风说话...")
        
        result = subprocess.run([
            'arecord', '-d', '5', '-f', 'cd', '-t', 'wav', test_file
        ], capture_output=True, text=True)
        
        if result.returncode == 0 and os.path.exists(test_file):
            file_size = os.path.getsize(test_file)
            print(f"  ✅ 录音成功！文件大小: {file_size} 字节")
            
            if file_size > 10000:  # 大于10KB认为有声音
                print("  ✅ 检测到音频数据，麦克风工作正常")
            else:
                print("  ⚠️ 文件较小，可能没有声音输入")
            
            # 播放录音测试
            print()
            print("  播放录音测试...")
            play_result = subprocess.run(['aplay', test_file], 
                                       capture_output=True, text=True)
            if play_result.returncode == 0:
                print("  ✅ 播放成功")
            else:
                print(f"  ❌ 播放失败: {play_result.stderr}")
        else:
            print(f"  ❌ 录音失败: {result.stderr}")
    
    # 使用PyAudio测试（如果可用）
    elif any(tool[0] == 'pyaudio' for tool in available_tools):
        print("  使用PyAudio录音3秒...")
        print("  💡 请对着麦克风说话...")
        
        try:
            import pyaudio
            import wave
            
            CHUNK = 1024
            FORMAT = pyaudio.paInt16
            CHANNELS = 1
            RATE = 44100
            RECORD_SECONDS = 3
            
            p = pyaudio.PyAudio()
            
            # 列出输入设备
            print("  可用输入设备:")
            for i in range(p.get_device_count()):
                info = p.get_device_info_by_index(i)
                if info['maxInputChannels'] > 0:
                    print(f"    设备 {i}: {info['name']} (输入通道: {info['maxInputChannels']})")
            
            # 录音
            stream = p.open(format=FORMAT,
                          channels=CHANNELS,
                          rate=RATE,
                          input=True,
                          frames_per_buffer=CHUNK)
            
            print("  开始录音...")
            frames = []
            for _ in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
                data = stream.read(CHUNK)
                frames.append(data)
            
            print("  录音完成")
            stream.stop_stream()
            stream.close()
            p.terminate()
            
            # 保存文件
            wf = wave.open(test_file, 'wb')
            wf.setnchannels(CHANNELS)
            wf.setsampwidth(p.get_sample_size(FORMAT))
            wf.setframerate(RATE)
            wf.writeframes(b''.join(frames))
            wf.close()
            
            file_size = os.path.getsize(test_file)
            print(f"  ✅ 录音成功！文件大小: {file_size} 字节")
            
        except Exception as e:
            print(f"  ❌ PyAudio录音失败: {e}")
    else:
        print("  ❌ 没有可用的录音工具")
    
    print()
    print("=" * 50)
    print("测试完成")
    print("=" * 50)
    print()
    print("如果麦克风未工作，请检查：")
    print("1. Type-C麦克风是否正确连接")
    print("2. 设备是否被系统识别")
    print("3. 权限设置是否正确")
    print("4. 尝试重新插拔设备")


if __name__ == '__main__':
    test_microphone()

