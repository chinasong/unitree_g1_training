#!/bin/bash
# 最终测试语音重复功能

echo "=========================================="
echo "G1机器人语音重复功能 - 最终测试"
echo "=========================================="
echo ""

sshpass -p 123 ssh -o StrictHostKeyChecking=no -t unitree@192.168.1.28 bash << 'EOF'
cd /home/unitree/workspace/unitree_g1_training

echo "【环境检查】"
echo "----------------------------------------"
echo "SpeechRecognition:"
python3 -c "import speech_recognition; print('✅ 可用')" 2>&1 || echo "❌ 不可用"
echo ""

echo "音频工具:"
which flac > /dev/null && echo "✅ FLAC" || echo "❌ FLAC未安装"
which sox > /dev/null && echo "✅ sox" || echo "❌ sox未安装"
which ffmpeg > /dev/null && echo "✅ ffmpeg" || echo "❌ ffmpeg未安装"
echo ""

echo "【开始测试】"
echo "----------------------------------------"
echo "💡 请对着麦克风说话（5秒）..."
echo ""

# 运行语音重复功能
python3 g1_voice_repeat_offline.py --iface wlan0 --duration 5

echo ""
echo "=========================================="
echo "测试完成"
echo "=========================================="
EOF

echo ""
echo "=========================================="
echo "本地完成"
echo "=========================================="

