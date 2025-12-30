#!/bin/bash
# G1机器人语音重复功能测试脚本

echo "=========================================="
echo "G1机器人语音重复功能测试"
echo "=========================================="
echo ""

sshpass -p 123 ssh -o StrictHostKeyChecking=no unitree@192.168.1.28 bash << 'EOF'
cd /home/unitree/workspace/unitree_g1_training

echo "【1. 检查Python环境】"
echo "----------------------------------------"
python3 --version
echo ""

echo "【2. 检查依赖库】"
echo "----------------------------------------"
echo "检查whisper:"
python3 -c "import whisper; print('✅ whisper已安装')" 2>&1 || echo "❌ whisper未安装"
echo ""

echo "检查speech_recognition:"
python3 -c "import speech_recognition; print('✅ speech_recognition已安装')" 2>&1 || echo "❌ speech_recognition未安装"
echo ""

echo "检查requests:"
python3 -c "import requests; print('✅ requests已安装')" 2>&1 || echo "❌ requests未安装"
echo ""

echo "检查unitree_sdk2_python:"
python3 -c "import sys; sys.path.append('./externals/unitree_sdk2_python'); from unitree_sdk2py.core.channel import ChannelFactoryInitialize; print('✅ SDK可用')" 2>&1 || echo "❌ SDK不可用或路径不正确"
echo ""

echo "【3. 检查音频设备】"
echo "----------------------------------------"
echo "默认输入设备:"
pactl info | grep "Default Source" | sed 's/^/  /'
echo ""

echo "默认输出设备:"
pactl info | grep "Default Sink" | sed 's/^/  /'
echo ""

echo "【4. 检查录音工具】"
echo "----------------------------------------"
which arecord && echo "✅ arecord可用" || echo "❌ arecord不可用"
echo ""

echo "【5. 测试脚本语法】"
echo "----------------------------------------"
echo "检查g1_voice_repeat.py:"
python3 -m py_compile g1_voice_repeat.py 2>&1 && echo "✅ 语法正确" || echo "❌ 语法错误"
echo ""

echo "检查g1_voice_repeat_simple.py:"
python3 -m py_compile g1_voice_repeat_simple.py 2>&1 && echo "✅ 语法正确" || echo "❌ 语法错误"
echo ""

echo "【6. 检查TTS服务】"
echo "----------------------------------------"
if pgrep -f "g1_tts_service.py" > /dev/null; then
    echo "✅ TTS服务正在运行"
else
    echo "⚠️ TTS服务未运行"
    echo "   可以使用: python3 g1_tts_service.py --iface wlan0 --server"
fi
echo ""

echo "=========================================="
echo "测试完成"
echo "=========================================="
echo ""
echo "如果依赖未安装，可以运行:"
echo "  ./g1_voice_repeat_install.sh"
echo "  或手动安装: pip3 install openai-whisper"
EOF

echo ""
echo "=========================================="
echo "本地测试完成"
echo "=========================================="
echo ""
echo "请将以上输出提供给AI助手进行分析"

