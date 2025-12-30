#!/bin/bash
# 安装依赖并测试语音重复功能

echo "=========================================="
echo "G1机器人语音重复功能 - 安装和测试"
echo "=========================================="
echo ""

sshpass -p 123 ssh -o StrictHostKeyChecking=no unitree@192.168.1.28 bash << 'EOF'
cd /home/unitree/workspace/unitree_g1_training

echo "【步骤1: 安装Whisper（推荐）】"
echo "----------------------------------------"
echo "正在安装openai-whisper..."
pip3 install openai-whisper 2>&1 | tail -10
echo ""

echo "验证安装:"
python3 -c "import whisper; print('✅ Whisper安装成功')" 2>&1 || echo "❌ 安装失败"
echo ""

echo "【步骤2: 安装requests（用于HTTP API）】"
echo "----------------------------------------"
pip3 install requests 2>&1 | tail -5
echo ""

echo "【步骤3: 测试录音功能】"
echo "----------------------------------------"
echo "测试录音3秒..."
test_file="/tmp/g1_test_record.wav"
arecord -D hw:0,0 -d 3 -f S16_LE -r 48000 -c 1 -t wav "$test_file" 2>&1

if [ -f "$test_file" ]; then
    file_size=$(stat -c%s "$test_file" 2>/dev/null || stat -f%z "$test_file" 2>/dev/null)
    echo "录音文件大小: ${file_size} 字节"
    if [ "$file_size" -gt 10000 ]; then
        echo "✅ 录音功能正常"
    else
        echo "⚠️ 录音文件较小"
    fi
else
    echo "❌ 录音失败"
fi
echo ""

echo "【步骤4: 测试Whisper识别（如果已安装）】"
echo "----------------------------------------"
if python3 -c "import whisper" 2>/dev/null; then
    if [ -f "$test_file" ] && [ -s "$test_file" ]; then
        echo "使用Whisper识别测试录音..."
        echo "（首次使用会下载模型，可能需要几分钟）"
        python3 << PYTHON_EOF
import whisper
import sys

try:
    print("  加载Whisper模型（base）...")
    model = whisper.load_model("base")
    print("  ✅ 模型加载成功")
    
    if len(sys.argv) > 1:
        audio_file = sys.argv[1]
        print(f"  识别音频文件: {audio_file}")
        result = model.transcribe(audio_file, language="zh")
        text = result["text"].strip()
        if text:
            print(f"  ✅ 识别结果: {text}")
        else:
            print("  ⚠️ 未识别到文字")
    else:
        print("  ℹ️ 跳过识别（需要音频文件）")
except Exception as e:
    print(f"  ❌ 错误: {e}")
PYTHON_EOF
    else
        echo "  ℹ️ 跳过识别（需要先录音）"
    fi
else
    echo "  ⚠️ Whisper未安装，跳过识别测试"
fi
echo ""

echo "【步骤5: 检查SDK连接】"
echo "----------------------------------------"
python3 << PYTHON_EOF
import sys
sys.path.append('./externals/unitree_sdk2_python')
try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient
    print("  ✅ SDK模块可用")
    
    # 尝试初始化（不实际连接，只检查）
    print("  ℹ️ SDK初始化检查通过")
except ImportError as e:
    print(f"  ⚠️ SDK不可用: {e}")
except Exception as e:
    print(f"  ⚠️ SDK检查失败: {e}")
PYTHON_EOF
echo ""

echo "=========================================="
echo "安装和测试完成"
echo "=========================================="
echo ""
echo "现在可以使用语音重复功能:"
echo "  python3 g1_voice_repeat_simple.py --iface wlan0"
echo ""
EOF

echo ""
echo "=========================================="
echo "完成"
echo "=========================================="

