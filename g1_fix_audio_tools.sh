#!/bin/bash
# 安装音频转换工具

echo "=========================================="
echo "安装音频转换工具"
echo "=========================================="
echo ""

sshpass -p 123 ssh -o StrictHostKeyChecking=no unitree@192.168.1.28 bash << 'EOF'
cd /home/unitree/workspace/unitree_g1_training

echo "【步骤1: 安装FLAC工具】"
echo "----------------------------------------"
sudo apt-get update -qq 2>&1 | tail -5
sudo apt-get install -y flac 2>&1 | tail -10
echo ""

echo "验证安装:"
which flac && echo "✅ FLAC已安装" || echo "❌ FLAC安装失败"
echo ""

echo "【步骤2: 安装sox（备选音频工具）】"
echo "----------------------------------------"
sudo apt-get install -y sox 2>&1 | tail -10
echo ""

echo "验证安装:"
which sox && echo "✅ sox已安装" || echo "⚠️ sox未安装"
echo ""

echo "【步骤3: 安装ffmpeg（另一个备选）】"
echo "----------------------------------------"
sudo apt-get install -y ffmpeg 2>&1 | tail -10
echo ""

echo "验证安装:"
which ffmpeg && echo "✅ ffmpeg已安装" || echo "⚠️ ffmpeg未安装"
echo ""

echo "【步骤4: 测试音频转换】"
echo "----------------------------------------"
test_file="/tmp/g1_test_audio.wav"
if [ -f "$test_file" ] || arecord -D hw:0,0 -d 1 -f S16_LE -r 48000 -c 1 -t wav "$test_file" 2>/dev/null; then
    if [ -f "$test_file" ]; then
        echo "测试sox转换:"
        if command -v sox > /dev/null; then
            sox "$test_file" -r 16000 -c 1 /tmp/g1_test_converted.wav 2>&1 | head -2
            if [ -f /tmp/g1_test_converted.wav ]; then
                echo "✅ sox转换成功"
            fi
        fi
        
        echo ""
        echo "测试ffmpeg转换:"
        if command -v ffmpeg > /dev/null; then
            ffmpeg -i "$test_file" -ar 16000 -ac 1 /tmp/g1_test_ffmpeg.wav -y 2>&1 | tail -2
            if [ -f /tmp/g1_test_ffmpeg.wav ]; then
                echo "✅ ffmpeg转换成功"
            fi
        fi
    fi
fi
echo ""

echo "=========================================="
echo "安装完成"
echo "=========================================="
EOF

echo ""
echo "=========================================="
echo "本地完成"
echo "=========================================="

