#!/bin/bash
# G1机器人音频设备测试脚本
# 测试Type-C麦克风和USB音响

echo "=========================================="
echo "G1机器人音频设备测试"
echo "=========================================="
echo ""

sshpass -p 123 ssh -o StrictHostKeyChecking=no unitree@192.168.1.28 bash << 'EOF'
echo "【1. 检查音频设备列表】"
echo "----------------------------------------"
echo "PulseAudio设备:"
pactl list short sources 2>/dev/null | head -10
echo ""
pactl list short sinks 2>/dev/null | head -10
echo ""

echo "ALSA设备:"
arecord -l 2>/dev/null || echo "arecord未安装或无法访问"
echo ""
aplay -l 2>/dev/null || echo "aplay未安装或无法访问"
echo ""

echo "【2. 检查USB设备】"
echo "----------------------------------------"
lsusb | grep -iE "audio|sound|usb" || echo "未找到USB音频设备"
echo ""

echo "【3. 检查Type-C设备】"
echo "----------------------------------------"
lsusb | grep -iE "type-c|typec" || echo "未找到Type-C设备"
echo ""

echo "【4. 检查当前默认音频设备】"
echo "----------------------------------------"
echo "默认输入设备（麦克风）:"
pactl info 2>/dev/null | grep "Default Source" || echo "无法获取"
echo ""

echo "默认输出设备（音响）:"
pactl info 2>/dev/null | grep "Default Sink" || echo "无法获取"
echo ""

echo "【5. 检查音频设备详细信息】"
echo "----------------------------------------"
echo "输入设备详情:"
pactl list sources short 2>/dev/null
echo ""

echo "输出设备详情:"
pactl list sinks short 2>/dev/null
echo ""

echo "【6. 检查音频服务状态】"
echo "----------------------------------------"
systemctl --user status pulseaudio 2>/dev/null | head -5 || ps aux | grep pulseaudio | grep -v grep | head -3
echo ""

echo "【7. 测试麦克风录音（5秒）】"
echo "----------------------------------------"
echo "开始录音测试（5秒）..."
timeout 5 arecord -d 5 -f cd -t wav /tmp/test_mic.wav 2>&1
if [ -f /tmp/test_mic.wav ]; then
    file_size=$(stat -c%s /tmp/test_mic.wav 2>/dev/null || stat -f%z /tmp/test_mic.wav 2>/dev/null)
    echo "录音文件大小: ${file_size} 字节"
    if [ "$file_size" -gt 1000 ]; then
        echo "✅ 麦克风录音测试成功"
    else
        echo "⚠️ 录音文件太小，可能没有声音输入"
    fi
else
    echo "❌ 录音文件未创建，麦克风可能未工作"
fi
echo ""

echo "【8. 测试音响播放】"
echo "----------------------------------------"
echo "生成测试音频..."
# 生成1秒的440Hz正弦波测试音
timeout 1 sox -n -r 44100 -c 2 /tmp/test_tone.wav synth 1 sine 440 2>/dev/null || \
echo -e "\x52\x49\x46\x46\x24\x08\x00\x00\x57\x41\x56\x45\x66\x6d\x74\x20\x10\x00\x00\x00\x01\x00\x02\x00\x44\xac\x00\x00\x10\xb1\x02\x00\x04\x00\x10\x00\x64\x61\x74\x61\x00\x08\x00\x00" > /tmp/test_tone.wav 2>/dev/null

if [ -f /tmp/test_tone.wav ]; then
    echo "播放测试音频..."
    timeout 2 aplay /tmp/test_tone.wav 2>&1
    if [ $? -eq 0 ]; then
        echo "✅ 音响播放测试成功"
    else
        echo "❌ 音响播放失败"
    fi
else
    echo "⚠️ 无法生成测试音频文件"
fi
echo ""

echo "【9. 检查音频设备权限】"
echo "----------------------------------------"
groups | grep -iE "audio|pulse" || echo "用户可能不在audio组"
echo ""

echo "【10. 列出所有音频相关进程】"
echo "----------------------------------------"
ps aux | grep -iE "pulse|alsa|audio" | grep -v grep | head -5
echo ""

echo "=========================================="
echo "测试完成"
echo "=========================================="
echo ""
echo "如果麦克风或音响未检测到，请检查："
echo "1. 设备是否正确连接"
echo "2. USB/Type-C接口是否正常"
echo "3. 设备驱动是否安装"
echo "4. 用户权限是否正确"
EOF

