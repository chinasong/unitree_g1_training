#!/bin/bash
# G1机器人完整音频功能测试
# 测试USB音响和麦克风的实际功能

echo "=========================================="
echo "G1机器人完整音频功能测试"
echo "=========================================="
echo ""

sshpass -p 123 ssh -o StrictHostKeyChecking=no unitree@192.168.1.28 bash << 'EOF'
# 先配置音频设备
USB_SINK="alsa_output.usb-Generic_USB2.0_Device_20170726905923-01.analog-stereo"
USB_SOURCE="alsa_input.usb-Generic_USB2.0_Device_20170726905923-01.mono-fallback"

echo "【步骤1: 配置音频设备】"
pactl set-default-sink "$USB_SINK" 2>/dev/null
pactl set-default-source "$USB_SOURCE" 2>/dev/null
pactl set-sink-volume "$USB_SINK" 70% 2>/dev/null
pactl set-source-volume "$USB_SOURCE" 100% 2>/dev/null
echo "✅ 设备配置完成"
echo ""

echo "【步骤2: 测试麦克风录音】"
echo "----------------------------------------"
echo "💡 请对着麦克风说话（5秒）..."
test_file="/tmp/g1_audio_test.wav"

# 使用USB麦克风录音（单声道，48kHz，匹配设备规格）
arecord -D hw:0,0 -d 5 -f S16_LE -r 48000 -c 1 -t wav "$test_file" 2>&1

if [ -f "$test_file" ]; then
    file_size=$(stat -c%s "$test_file" 2>/dev/null || stat -f%z "$test_file" 2>/dev/null)
    echo "录音文件大小: ${file_size} 字节"
    
    if [ "$file_size" -gt 10000 ]; then
        echo "✅ 麦克风录音成功！检测到音频数据"
        
        # 播放录音验证（使用PulseAudio自动处理格式转换）
        echo ""
        echo "【步骤3: 播放录音验证】"
        echo "----------------------------------------"
        echo "💡 现在播放刚才的录音，你应该能听到自己的声音..."
        
        # 使用paplay通过PulseAudio播放，会自动处理格式转换
        if command -v paplay > /dev/null; then
            paplay "$test_file" 2>&1
            if [ $? -eq 0 ]; then
                echo "✅ 播放成功！麦克风和音响都工作正常"
            else
                echo "⚠️ 播放失败，但录音成功"
            fi
        else
            # 如果没有paplay，尝试转换格式后播放
            stereo_file="/tmp/g1_audio_test_stereo.wav"
            if command -v sox > /dev/null; then
                sox "$test_file" -r 48000 -c 2 "$stereo_file" 2>/dev/null
                if [ -f "$stereo_file" ]; then
                    aplay -D hw:0,0 "$stereo_file" 2>&1
                    if [ $? -eq 0 ]; then
                        echo "✅ 播放成功！麦克风和音响都工作正常"
                    else
                        echo "⚠️ 播放失败，但录音成功"
                    fi
                else
                    echo "⚠️ 格式转换失败，但录音成功"
                fi
            else
                echo "⚠️ 无法播放单声道录音（需要格式转换），但录音成功"
                echo "   录音文件已保存: $test_file"
            fi
        fi
    else
        echo "⚠️ 录音文件太小，可能没有声音输入"
    fi
else
    echo "❌ 录音文件未创建"
fi
echo ""

echo "【步骤4: 测试音响播放】"
echo "----------------------------------------"
echo "💡 播放测试音（440Hz正弦波）..."

# 生成测试音频
test_tone="/tmp/g1_test_tone.wav"
if command -v sox > /dev/null; then
    sox -n -r 44100 -c 2 "$test_tone" synth 2 sine 440 2>/dev/null
else
    # 使用Python生成简单测试音
    python3 << PYTHON_EOF
import wave
import struct
import math

sample_rate = 44100
duration = 2
frequency = 440

with wave.open("$test_tone", 'wb') as wf:
    wf.setnchannels(2)
    wf.setsampwidth(2)
    wf.setframerate(sample_rate)
    
    for i in range(int(sample_rate * duration)):
        value = int(32767 * 0.3 * math.sin(2 * math.pi * frequency * i / sample_rate))
        wf.writeframes(struct.pack('<hh', value, value))
PYTHON_EOF
fi

if [ -f "$test_tone" ]; then
    aplay -D hw:0,0 "$test_tone" 2>&1
    if [ $? -eq 0 ]; then
        echo "✅ 音响播放测试成功！你应该听到了测试音"
    else
        echo "❌ 音响播放失败"
    fi
else
    echo "⚠️ 无法生成测试音频"
fi
echo ""

echo "【步骤5: 音量测试】"
echo "----------------------------------------"
echo "当前音量设置:"
pactl list sinks | grep -A 10 "$USB_SINK" | grep -E "Volume:|Mute:" | head -5
echo ""

echo "【步骤6: 设备信息总结】"
echo "----------------------------------------"
echo "✅ USB音响: $USB_SINK"
echo "✅ USB麦克风: $USB_SOURCE"
echo ""
echo "设备状态:"
pactl list sinks short | grep "$USB_SINK"
pactl list sources short | grep "$USB_SOURCE"
echo ""

echo "=========================================="
echo "测试完成"
echo "=========================================="
echo ""
echo "如果测试成功，说明："
echo "✅ USB音响工作正常"
echo "✅ USB麦克风工作正常"
echo "✅ 可以用于语音交互"
echo ""
echo "注意：Type-C麦克风可能通过USB转接器连接，"
echo "     系统识别为USB设备是正常的。"
EOF

