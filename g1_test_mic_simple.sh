#!/bin/bash
# 简单的麦克风录音测试

echo "=========================================="
echo "G1机器人麦克风录音测试"
echo "=========================================="
echo ""

sshpass -p 123 ssh -o StrictHostKeyChecking=no unitree@192.168.1.28 bash << 'EOF'
USB_SOURCE="alsa_input.usb-Generic_USB2.0_Device_20170726905923-01.mono-fallback"
pactl set-default-source "$USB_SOURCE" 2>/dev/null
pactl set-source-volume "$USB_SOURCE" 100% 2>/dev/null

test_file="/tmp/g1_mic_test.wav"

echo "💡 请对着麦克风说话（5秒）..."
echo "开始录音..."

# 使用正确的格式：单声道，48kHz
arecord -D hw:0,0 -d 5 -f S16_LE -r 48000 -c 1 -t wav "$test_file" 2>&1

if [ -f "$test_file" ]; then
    file_size=$(stat -c%s "$test_file" 2>/dev/null || stat -f%z "$test_file" 2>/dev/null)
    echo ""
    echo "录音文件大小: ${file_size} 字节"
    
    if [ "$file_size" -gt 10000 ]; then
        echo "✅ 录音成功！"
        echo ""
        echo "播放录音验证..."
        aplay -D hw:0,0 "$test_file" 2>&1
        echo ""
        echo "✅ 如果听到刚才的录音，说明麦克风和音响都工作正常！"
    else
        echo "⚠️ 录音文件太小，可能没有声音输入"
    fi
else
    echo "❌ 录音失败"
fi
EOF

