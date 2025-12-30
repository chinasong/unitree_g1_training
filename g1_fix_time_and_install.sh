#!/bin/bash
# 修复系统时间并安装Whisper

echo "=========================================="
echo "修复系统时间并安装Whisper"
echo "=========================================="
echo ""

sshpass -p 123 ssh -o StrictHostKeyChecking=no unitree@192.168.1.28 bash << 'EOF'
cd /home/unitree/workspace/unitree_g1_training

echo "【步骤1: 检查系统时间】"
echo "----------------------------------------"
date
echo ""

echo "【步骤2: 尝试同步系统时间】"
echo "----------------------------------------"
# 尝试使用ntpdate同步时间（如果可用）
if command -v ntpdate > /dev/null; then
    echo "使用ntpdate同步时间..."
    sudo ntpdate -s time.nist.gov 2>&1 || echo "ntpdate同步失败"
elif command -v timedatectl > /dev/null; then
    echo "使用timedatectl同步时间..."
    sudo timedatectl set-ntp true 2>&1
    sleep 2
    timedatectl status | head -5
else
    echo "⚠️ 没有找到时间同步工具"
    echo "   可以手动设置时间: sudo date -s '2025-01-11 07:30:00'"
fi
echo ""

echo "【步骤3: 检查当前时间】"
echo "----------------------------------------"
date
echo ""

echo "【步骤4: 安装Whisper（跳过SSL验证）】"
echo "----------------------------------------"
echo "使用--trusted-host跳过SSL验证..."
pip3 install --trusted-host pypi.org --trusted-host files.pythonhosted.org openai-whisper 2>&1 | tail -20
echo ""

echo "【步骤5: 验证安装】"
echo "----------------------------------------"
python3 -c "import whisper; print('✅ Whisper安装成功')" 2>&1 || echo "❌ 安装失败"
echo ""

echo "【步骤6: 如果Whisper安装失败，尝试安装speech_recognition】"
echo "----------------------------------------"
if ! python3 -c "import whisper" 2>/dev/null; then
    echo "安装speech_recognition作为备选方案..."
    pip3 install --trusted-host pypi.org --trusted-host files.pythonhosted.org SpeechRecognition 2>&1 | tail -10
    python3 -c "import speech_recognition; print('✅ SpeechRecognition安装成功')" 2>&1 || echo "❌ 安装失败"
fi
echo ""

echo "=========================================="
echo "完成"
echo "=========================================="
EOF

echo ""
echo "=========================================="
echo "本地完成"
echo "=========================================="

