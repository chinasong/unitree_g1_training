#!/usr/bin/env python3
"""
G1机器人音响测试脚本
测试USB音响播放功能
"""

import sys
import subprocess
import time
import os

def test_speaker():
    """测试音响播放"""
    print("=" * 50)
    print("G1机器人音响测试")
    print("=" * 50)
    print()
    
    # 检查播放工具
    print("【1. 检查播放工具】")
    tools = {
        'aplay': 'ALSA播放工具',
        'paplay': 'PulseAudio播放工具',
        'speaker-test': 'ALSA测试工具',
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
    
    # 列出音频输出设备
    print("【2. 列出音频输出设备】")
    print("PulseAudio输出设备:")
    result = subprocess.run(['pactl', 'list', 'short', 'sinks'], 
                          capture_output=True, text=True)
    if result.returncode == 0:
        sinks = result.stdout.strip().split('\n')
        if sinks and sinks[0]:
            for sink in sinks:
                print(f"  {sink}")
        else:
            print("  未找到输出设备")
    else:
        print("  无法获取PulseAudio设备列表")
    print()
    
    print("ALSA输出设备:")
    result = subprocess.run(['aplay', '-l'], capture_output=True, text=True)
    if result.returncode == 0:
        print(result.stdout)
    else:
        print("  无法获取ALSA设备列表")
    print()
    
    # 检查USB设备
    print("【3. 检查USB音响设备】")
    result = subprocess.run(['lsusb'], capture_output=True, text=True)
    if result.returncode == 0:
        usb_audio = [line for line in result.stdout.split('\n') 
                    if 'audio' in line.lower() or 'sound' in line.lower()]
        if usb_audio:
            print("  找到USB音频设备:")
            for device in usb_audio:
                print(f"    {device}")
        else:
            print("  未找到USB音频设备")
    else:
        print("  无法获取USB设备列表")
    print()
    
    # 测试播放
    print("【4. 测试播放功能】")
    
    # 方法1: 使用speaker-test（如果可用）
    if any(tool[0] == 'speaker-test' for tool in available_tools):
        print("  使用speaker-test测试左右声道...")
        print("  💡 你应该听到测试音...")
        
        result = subprocess.run([
            'speaker-test', '-t', 'sine', '-f', '440', '-l', '1', '-c', '2'
        ], capture_output=True, text=True, timeout=3)
        
        if result.returncode == 0:
            print("  ✅ speaker-test播放成功")
        else:
            print(f"  ⚠️ speaker-test可能未完成: {result.stderr}")
    
    # 方法2: 生成并播放测试音频
    print()
    print("  生成测试音频文件...")
    test_file = "/tmp/g1_speaker_test.wav"
    
    # 尝试使用sox生成测试音
    result = subprocess.run(['which', 'sox'], capture_output=True)
    if result.returncode == 0:
        # 生成1秒440Hz正弦波
        subprocess.run([
            'sox', '-n', '-r', '44100', '-c', '2', test_file,
            'synth', '1', 'sine', '440'
        ], capture_output=True)
    else:
        # 使用Python生成简单的WAV文件
        try:
            import wave
            import struct
            import math
            
            sample_rate = 44100
            duration = 1
            frequency = 440
            
            with wave.open(test_file, 'wb') as wf:
                wf.setnchannels(2)  # 立体声
                wf.setsampwidth(2)  # 16位
                wf.setframerate(sample_rate)
                
                for i in range(int(sample_rate * duration)):
                    value = int(32767 * math.sin(2 * math.pi * frequency * i / sample_rate))
                    wf.writeframes(struct.pack('<hh', value, value))
            
            print("  ✅ 测试音频文件生成成功")
        except Exception as e:
            print(f"  ❌ 无法生成测试音频: {e}")
            return
    
    if os.path.exists(test_file):
        file_size = os.path.getsize(test_file)
        print(f"  文件大小: {file_size} 字节")
        
        # 使用aplay播放
        if any(tool[0] == 'aplay' for tool in available_tools):
            print()
            print("  使用aplay播放测试音频...")
            print("  💡 你应该听到440Hz的测试音...")
            
            result = subprocess.run(['aplay', test_file], 
                                  capture_output=True, text=True)
            if result.returncode == 0:
                print("  ✅ aplay播放成功")
            else:
                print(f"  ❌ aplay播放失败: {result.stderr}")
        
        # 使用paplay播放
        elif any(tool[0] == 'paplay' for tool in available_tools):
            print()
            print("  使用paplay播放测试音频...")
            print("  💡 你应该听到440Hz的测试音...")
            
            result = subprocess.run(['paplay', test_file], 
                                  capture_output=True, text=True)
            if result.returncode == 0:
                print("  ✅ paplay播放成功")
            else:
                print(f"  ❌ paplay播放失败: {result.stderr}")
    
    # 测试音量控制
    print()
    print("【5. 测试音量控制】")
    result = subprocess.run(['pactl', 'list', 'sinks'], 
                          capture_output=True, text=True)
    if result.returncode == 0:
        print("  当前音量设置:")
        # 提取音量信息
        lines = result.stdout.split('\n')
        for i, line in enumerate(lines):
            if 'Volume:' in line:
                print(f"    {line.strip()}")
                # 显示接下来几行的音量详情
                for j in range(1, 3):
                    if i + j < len(lines):
                        print(f"      {lines[i+j].strip()}")
    print()
    
    print("=" * 50)
    print("测试完成")
    print("=" * 50)
    print()
    print("如果音响未工作，请检查：")
    print("1. USB音响是否正确连接")
    print("2. 设备是否被系统识别")
    print("3. 音量是否调高")
    print("4. 默认输出设备是否正确")
    print("5. 尝试重新插拔USB设备")


if __name__ == '__main__':
    test_speaker()

