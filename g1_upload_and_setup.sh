#!/bin/bash
# 上传脚本文件到G1并安装依赖

echo "=========================================="
echo "上传脚本到G1并安装依赖"
echo "=========================================="
echo ""

G1_HOST="192.168.1.28"
G1_USER="unitree"
G1_PASSWORD="123"
G1_DIR="/home/unitree/workspace/unitree_g1_training"

echo "【步骤1: 上传脚本文件到G1】"
echo "----------------------------------------"

# 需要上传的文件
FILES=(
    "g1_voice_repeat.py"
    "g1_voice_repeat_simple.py"
)

for file in "${FILES[@]}"; do
    if [ -f "$file" ]; then
        echo "上传 $file ..."
        sshpass -p "$G1_PASSWORD" scp -o StrictHostKeyChecking=no "$file" "${G1_USER}@${G1_HOST}:${G1_DIR}/" 2>&1
        if [ $? -eq 0 ]; then
            echo "✅ $file 上传成功"
        else
            echo "❌ $file 上传失败"
        fi
    else
        echo "⚠️ $file 不存在，跳过"
    fi
done
echo ""

echo "【步骤2: 在G1上安装依赖】"
echo "----------------------------------------"
sshpass -p "$G1_PASSWORD" ssh -o StrictHostKeyChecking=no "${G1_USER}@${G1_HOST}" bash << EOF
cd ${G1_DIR}

echo "安装Whisper..."
pip3 install openai-whisper 2>&1 | tail -10

echo ""
echo "验证安装:"
python3 -c "import whisper; print('✅ Whisper安装成功')" 2>&1 || echo "❌ 安装失败，可能需要手动安装"

echo ""
echo "检查脚本文件:"
ls -lh g1_voice_repeat*.py 2>&1
EOF

echo ""
echo "=========================================="
echo "完成"
echo "=========================================="
echo ""
echo "现在可以在G1上测试:"
echo "  ssh unitree@192.168.1.28"
echo "  cd /home/unitree/workspace/unitree_g1_training"
echo "  python3 g1_voice_repeat_simple.py --iface wlan0"

