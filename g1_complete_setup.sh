#!/bin/bash
# 完整设置语音重复功能

echo "=========================================="
echo "G1机器人语音重复功能 - 完整设置"
echo "=========================================="
echo ""

G1_HOST="192.168.1.28"
G1_USER="unitree"
G1_PASSWORD="123"
G1_DIR="/home/unitree/workspace/unitree_g1_training"

echo "【步骤1: 安装音频工具】"
echo "----------------------------------------"
sshpass -p "$G1_PASSWORD" ssh -o StrictHostKeyChecking=no "${G1_USER}@${G1_HOST}" bash << 'EOF'
sudo apt-get update -qq
sudo apt-get install -y flac sox ffmpeg 2>&1 | tail -10
echo ""
echo "验证安装:"
which flac && echo "✅ FLAC已安装" || echo "❌ FLAC未安装"
which sox && echo "✅ sox已安装" || echo "⚠️ sox未安装"
which ffmpeg && echo "✅ ffmpeg已安装" || echo "⚠️ ffmpeg未安装"
EOF

echo ""
echo "【步骤2: 上传修复后的脚本】"
echo "----------------------------------------"
if [ -f "g1_voice_repeat_offline.py" ]; then
    sshpass -p "$G1_PASSWORD" scp -o StrictHostKeyChecking=no \
        g1_voice_repeat_offline.py "${G1_USER}@${G1_HOST}:${G1_DIR}/" 2>&1
    echo "✅ 脚本已上传"
else
    echo "⚠️ 脚本文件不存在"
fi
echo ""

echo "【步骤3: 测试环境】"
echo "----------------------------------------"
sshpass -p "$G1_PASSWORD" ssh -o StrictHostKeyChecking=no "${G1_USER}@${G1_HOST}" bash << EOF
cd ${G1_DIR}

echo "检查依赖:"
python3 -c "import speech_recognition; print('✅ SpeechRecognition')" 2>&1
python3 -c "import numpy; print('✅ numpy', numpy.__version__)" 2>&1

echo ""
echo "检查工具:"
which flac && echo "✅ FLAC" || echo "❌ FLAC"
which sox && echo "✅ sox" || echo "❌ sox"
which ffmpeg && echo "✅ ffmpeg" || echo "❌ ffmpeg"

echo ""
echo "检查脚本:"
ls -lh g1_voice_repeat_offline.py 2>&1
EOF

echo ""
echo "=========================================="
echo "设置完成"
echo "=========================================="
echo ""
echo "现在可以测试:"
echo "  ssh unitree@192.168.1.28"
echo "  cd /home/unitree/workspace/unitree_g1_training"
echo "  python3 g1_voice_repeat_offline.py --iface wlan0"
echo ""
echo "或运行测试:"
echo "  ./g1_test_voice_repeat_now.sh"

