#!/bin/bash
# 上传无需FLAC版本的脚本

echo "=========================================="
echo "上传无需FLAC版本的语音重复脚本"
echo "=========================================="
echo ""

sshpass -p 123 scp -o StrictHostKeyChecking=no \
    g1_voice_repeat_no_flac.py \
    unitree@192.168.1.28:/home/unitree/workspace/unitree_g1_training/ 2>&1

if [ $? -eq 0 ]; then
    echo "✅ 上传成功"
    echo ""
    echo "现在可以在G1上测试:"
    echo "  ssh unitree@192.168.1.28"
    echo "  cd /home/unitree/workspace/unitree_g1_training"
    echo "  python3 g1_voice_repeat_no_flac.py --iface wlan0"
else
    echo "❌ 上传失败"
fi

