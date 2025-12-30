#!/bin/bash
# 上传离线版脚本并测试

echo "=========================================="
echo "上传离线版脚本并测试语音重复功能"
echo "=========================================="
echo ""

G1_HOST="192.168.1.28"
G1_USER="unitree"
G1_PASSWORD="123"
G1_DIR="/home/unitree/workspace/unitree_g1_training"

echo "【步骤1: 上传离线版脚本】"
echo "----------------------------------------"
if [ -f "g1_voice_repeat_offline.py" ]; then
    sshpass -p "$G1_PASSWORD" scp -o StrictHostKeyChecking=no \
        g1_voice_repeat_offline.py "${G1_USER}@${G1_HOST}:${G1_DIR}/" 2>&1
    if [ $? -eq 0 ]; then
        echo "✅ g1_voice_repeat_offline.py 上传成功"
    else
        echo "❌ 上传失败"
    fi
else
    echo "⚠️ g1_voice_repeat_offline.py 不存在"
fi
echo ""

echo "【步骤2: 修复numpy并测试】"
echo "----------------------------------------"
sshpass -p "$G1_PASSWORD" ssh -o StrictHostKeyChecking=no "${G1_USER}@${G1_HOST}" bash << EOF
cd ${G1_DIR}

echo "升级numpy..."
pip3 install --trusted-host pypi.org --trusted-host files.pythonhosted.org --upgrade "numpy>=1.22" 2>&1 | tail -5

echo ""
echo "验证环境:"
python3 -c "import speech_recognition; print('✅ SpeechRecognition可用')" 2>&1
python3 -c "import numpy; print('✅ numpy版本:', numpy.__version__)" 2>&1

echo ""
echo "检查脚本:"
ls -lh g1_voice_repeat*.py 2>&1 | head -5
EOF

echo ""
echo "=========================================="
echo "完成"
echo "=========================================="
echo ""
echo "现在可以在G1上测试:"
echo "  ssh unitree@192.168.1.28"
echo "  cd /home/unitree/workspace/unitree_g1_training"
echo "  python3 g1_voice_repeat_offline.py --iface wlan0"
echo ""
echo "或运行完整测试:"
echo "  ./g1_test_voice_repeat_now.sh"

