#!/bin/bash
# 立即测试语音重复功能（使用已安装的speech_recognition）

echo "=========================================="
echo "G1机器人语音重复功能 - 立即测试"
echo "=========================================="
echo ""

sshpass -p 123 ssh -o StrictHostKeyChecking=no -t unitree@192.168.1.28 bash << 'EOF'
cd /home/unitree/workspace/unitree_g1_training

echo "【检查环境】"
echo "----------------------------------------"
echo "SpeechRecognition:"
python3 -c "import speech_recognition; print('✅ 可用')" 2>&1 || echo "❌ 不可用"
echo ""

echo "SDK:"
python3 -c "import sys; sys.path.insert(0, '/home/unitree/unitree_sdk2_python'); from unitree_sdk2py.core.channel import ChannelFactoryInitialize; print('✅ 可用')" 2>&1 || echo "⚠️ SDK不可用，将提示使用HTTP API"
echo ""

echo "【测试录音】"
echo "----------------------------------------"
echo "录音3秒测试..."
test_file="/tmp/g1_quick_voice_test.wav"
arecord -D hw:0,0 -d 3 -f S16_LE -r 48000 -c 1 -t wav "$test_file" 2>&1 | head -2

if [ -f "$test_file" ]; then
    file_size=$(stat -c%s "$test_file" 2>/dev/null || stat -f%z "$test_file" 2>/dev/null)
    echo "录音文件大小: ${file_size} 字节"
    if [ "$file_size" -gt 10000 ]; then
        echo "✅ 录音成功"
        
        echo ""
        echo "【测试语音识别】"
        echo "----------------------------------------"
        echo "使用speech_recognition识别..."
        
        python3 << PYTHON_EOF
import speech_recognition as sr
import subprocess
import os

audio_file = "/tmp/g1_quick_voice_test.wav"
converted_file = "/tmp/g1_voice_converted.wav"

# 转换音频格式（Google API需要16kHz单声道）
if subprocess.run(['which', 'sox'], capture_output=True).returncode == 0:
    subprocess.run(['sox', audio_file, '-r', '16000', '-c', '1', converted_file], 
                   capture_output=True)
elif subprocess.run(['which', 'ffmpeg'], capture_output=True).returncode == 0:
    subprocess.run(['ffmpeg', '-i', audio_file, '-ar', '16000', '-ac', '1', 
                   converted_file, '-y'], capture_output=True, stderr=subprocess.DEVNULL)
else:
    converted_file = audio_file
    print("⚠️ 未找到音频转换工具，直接使用原文件")

try:
    r = sr.Recognizer()
    with sr.AudioFile(converted_file) as source:
        r.adjust_for_ambient_noise(source, duration=0.5)
        audio = r.record(source)
    
    text = r.recognize_google(audio, language='zh-CN')
    print(f"✅ 识别成功: {text}")
    
    # 让机器人说话
    print(f"\n🗣️ 机器人将说: {text}")
    print("正在发送TTS命令...")
    
    import sys
    sys.path.insert(0, '/home/unitree/unitree_sdk2_python')
    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient
        
        ChannelFactoryInitialize(0, 'wlan0')
        audio_client = AudioClient()
        audio_client.SetTimeout(10.0)
        audio_client.Init()
        
        code = audio_client.TtsMaker(text, 0)
        if code == 0:
            print("✅ 机器人说话成功！")
        else:
            print(f"❌ TTS失败，错误码: {code}")
    except Exception as e:
        print(f"⚠️ SDK不可用: {e}")
        print("   可以使用HTTP API模式或手动输入文字")
        
except sr.UnknownValueError:
    print("❌ 无法识别语音内容")
except sr.RequestError as e:
    print(f"❌ 识别服务错误: {e}")
    print("   需要网络连接才能使用Google识别")
except Exception as e:
    print(f"❌ 错误: {e}")
PYTHON_EOF
        
    else
        echo "⚠️ 录音文件较小"
    fi
else
    echo "⚠️ 录音文件未创建（非交互式模式）"
fi

echo ""
echo "=========================================="
echo "测试完成"
echo "=========================================="
EOF

echo ""
echo "=========================================="
echo "本地完成"
echo "=========================================="

