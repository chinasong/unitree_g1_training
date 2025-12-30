#!/bin/bash
# 安装音频工具（使用密码）

echo "=========================================="
echo "安装音频工具（FLAC, sox, ffmpeg）"
echo "=========================================="
echo ""

sshpass -p 123 ssh -o StrictHostKeyChecking=no -t unitree@192.168.1.28 bash << 'EOF'
echo "123" | sudo -S apt-get update -qq 2>&1 | tail -5
echo ""
echo "安装FLAC..."
echo "123" | sudo -S apt-get install -y flac 2>&1 | tail -10
echo ""

echo "安装sox..."
echo "123" | sudo -S apt-get install -y sox 2>&1 | tail -10
echo ""

echo "安装ffmpeg..."
echo "123" | sudo -S apt-get install -y ffmpeg 2>&1 | tail -10
echo ""

echo "验证安装:"
which flac && echo "✅ FLAC已安装" || echo "❌ FLAC未安装"
which sox && echo "✅ sox已安装" || echo "❌ sox未安装"
which ffmpeg && echo "✅ ffmpeg已安装" || echo "❌ ffmpeg未安装"
EOF

echo ""
echo "=========================================="
echo "安装完成"
echo "=========================================="

