#!/bin/bash
# 修复numpy问题并测试语音重复功能

echo "=========================================="
echo "修复numpy并测试语音重复功能"
echo "=========================================="
echo ""

sshpass -p 123 ssh -o StrictHostKeyChecking=no unitree@192.168.1.28 bash << 'EOF'
cd /home/unitree/workspace/unitree_g1_training

echo "【步骤1: 升级numpy（修复Whisper问题）】"
echo "----------------------------------------"
echo "当前numpy版本:"
python3 -c "import numpy; print(numpy.__version__)" 2>&1
echo ""

echo "升级numpy到1.22+..."
pip3 install --trusted-host pypi.org --trusted-host files.pythonhosted.org --upgrade "numpy>=1.22" 2>&1 | tail -10
echo ""

echo "验证numpy版本:"
python3 -c "import numpy; print('numpy版本:', numpy.__version__)" 2>&1
echo ""

echo "【步骤2: 测试Whisper（如果numpy修复成功）】"
echo "----------------------------------------"
python3 -c "import whisper; print('✅ Whisper可用')" 2>&1 || echo "⚠️ Whisper仍有问题，将使用speech_recognition"
echo ""

echo "【步骤3: 上传离线版脚本（使用speech_recognition）】"
echo "----------------------------------------"
# 脚本应该已经存在，检查一下
if [ -f "g1_voice_repeat_offline.py" ]; then
    echo "✅ g1_voice_repeat_offline.py 已存在"
else
    echo "⚠️ g1_voice_repeat_offline.py 不存在，需要上传"
fi
echo ""

echo "【步骤4: 测试speech_recognition】"
echo "----------------------------------------"
python3 -c "import speech_recognition; print('✅ SpeechRecognition可用')" 2>&1
echo ""

echo "【步骤5: 检查SDK路径】"
echo "----------------------------------------"
if [ -d "/home/unitree/unitree_sdk2_python" ]; then
    echo "✅ SDK路径存在: /home/unitree/unitree_sdk2_python"
    python3 -c "import sys; sys.path.insert(0, '/home/unitree/unitree_sdk2_python'); from unitree_sdk2py.core.channel import ChannelFactoryInitialize; print('✅ SDK可用')" 2>&1 || echo "⚠️ SDK导入失败"
else
    echo "⚠️ SDK路径不存在"
fi
echo ""

echo "=========================================="
echo "准备完成"
echo "=========================================="
echo ""
echo "现在可以使用:"
echo "1. 如果Whisper可用: python3 g1_voice_repeat_simple.py --iface wlan0"
echo "2. 使用speech_recognition: python3 g1_voice_repeat_offline.py --iface wlan0"
echo "3. 或使用完整版: python3 g1_voice_repeat.py --iface wlan0"
EOF

echo ""
echo "=========================================="
echo "本地完成"
echo "=========================================="

