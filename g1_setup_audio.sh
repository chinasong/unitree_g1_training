#!/bin/bash
# G1机器人音频设备配置脚本
# 设置USB音响和麦克风为默认设备

echo "=========================================="
echo "G1机器人音频设备配置"
echo "=========================================="
echo ""

sshpass -p 123 ssh -o StrictHostKeyChecking=no unitree@192.168.1.28 bash << 'EOF'
# USB设备ID
USB_SINK="alsa_output.usb-Generic_USB2.0_Device_20170726905923-01.analog-stereo"
USB_SOURCE="alsa_input.usb-Generic_USB2.0_Device_20170726905923-01.mono-fallback"

echo "【1. 激活USB音频设备】"
echo "----------------------------------------"
# 激活输出设备（音响）
pactl set-sink-mute "$USB_SINK" 0 2>/dev/null
pactl set-sink-volume "$USB_SINK" 50% 2>/dev/null
echo "✅ USB音响已激活"

# 激活输入设备（麦克风）
pactl set-source-mute "$USB_SOURCE" 0 2>/dev/null
pactl set-source-volume "$USB_SOURCE" 100% 2>/dev/null
echo "✅ USB麦克风已激活"
echo ""

echo "【2. 设置USB设备为默认设备】"
echo "----------------------------------------"
# 设置默认输出设备（音响）
pactl set-default-sink "$USB_SINK" 2>/dev/null
if [ $? -eq 0 ]; then
    echo "✅ USB音响已设置为默认输出设备"
else
    echo "⚠️ 设置默认输出设备失败"
fi

# 设置默认输入设备（麦克风）
pactl set-default-source "$USB_SOURCE" 2>/dev/null
if [ $? -eq 0 ]; then
    echo "✅ USB麦克风已设置为默认输入设备"
else
    echo "⚠️ 设置默认输入设备失败"
fi
echo ""

echo "【3. 验证默认设备设置】"
echo "----------------------------------------"
echo "当前默认输出设备:"
pactl info | grep "Default Sink" | sed 's/^/  /'
echo ""

echo "当前默认输入设备:"
pactl info | grep "Default Source" | sed 's/^/  /'
echo ""

echo "【4. 检查设备状态】"
echo "----------------------------------------"
echo "USB音响状态:"
pactl list sinks short | grep "$USB_SINK" | sed 's/^/  /'
echo ""

echo "USB麦克风状态:"
pactl list sources short | grep "$USB_SOURCE" | sed 's/^/  /'
echo ""

echo "【5. 设置音量】"
echo "----------------------------------------"
# 设置音响音量到70%
pactl set-sink-volume "$USB_SINK" 70% 2>/dev/null
echo "✅ USB音响音量设置为70%"

# 设置麦克风音量到100%
pactl set-source-volume "$USB_SOURCE" 100% 2>/dev/null
echo "✅ USB麦克风音量设置为100%"
echo ""

echo "=========================================="
echo "配置完成"
echo "=========================================="
echo ""
echo "现在可以测试："
echo "1. 录音测试: python3 g1_test_microphone.py"
echo "2. 播放测试: python3 g1_test_speaker.py"
echo "3. 或运行完整测试: ./g1_test_audio_full.sh"
EOF

