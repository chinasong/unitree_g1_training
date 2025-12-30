#!/bin/bash
# 完整设置语音重复功能

echo "=========================================="
echo "G1机器人语音重复功能 - 完整设置"
echo "=========================================="
echo ""

sshpass -p 123 ssh -o StrictHostKeyChecking=no unitree@192.168.1.28 bash << 'EOF'
cd /home/unitree/workspace/unitree_g1_training

echo "【步骤1: 检查脚本文件】"
echo "----------------------------------------"
if [ -f "g1_voice_repeat.py" ]; then
    echo "✅ g1_voice_repeat.py 存在"
else
    echo "❌ g1_voice_repeat.py 不存在"
    echo "   需要从本地复制文件到G1"
fi

if [ -f "g1_voice_repeat_simple.py" ]; then
    echo "✅ g1_voice_repeat_simple.py 存在"
else
    echo "❌ g1_voice_repeat_simple.py 不存在"
    echo "   需要从本地复制文件到G1"
fi
echo ""

echo "【步骤2: 安装Whisper（推荐）】"
echo "----------------------------------------"
echo "正在安装openai-whisper..."
echo "（这可能需要几分钟，首次使用会下载模型）"
pip3 install openai-whisper 2>&1 | tail -20
echo ""

echo "验证安装:"
python3 -c "import whisper; print('✅ Whisper安装成功')" 2>&1
echo ""

echo "【步骤3: 检查SDK路径】"
echo "----------------------------------------"
if [ -d "/home/unitree/unitree_sdk2_python" ]; then
    echo "✅ 找到SDK: /home/unitree/unitree_sdk2_python"
    echo "   更新脚本中的SDK路径..."
    
    # 检查脚本是否存在
    if [ -f "g1_voice_repeat_simple.py" ]; then
        # 更新SDK路径
        sed -i 's|sys.path.append.*unitree_sdk2_python|sys.path.append("/home/unitree/unitree_sdk2_python")|g' g1_voice_repeat_simple.py
        echo "✅ SDK路径已更新"
    fi
else
    echo "⚠️ SDK路径不存在，将使用HTTP API模式"
fi
echo ""

echo "【步骤4: 测试录音功能】"
echo "----------------------------------------"
echo "快速录音测试（2秒）..."
test_file="/tmp/g1_quick_test.wav"
timeout 3 arecord -D hw:0,0 -d 2 -f S16_LE -r 48000 -c 1 -t wav "$test_file" 2>&1 | head -3

if [ -f "$test_file" ]; then
    file_size=$(stat -c%s "$test_file" 2>/dev/null || stat -f%z "$test_file" 2>/dev/null)
    echo "录音文件大小: ${file_size} 字节"
    if [ "$file_size" -gt 5000 ]; then
        echo "✅ 录音功能正常"
        rm -f "$test_file"
    else
        echo "⚠️ 录音文件较小"
    fi
else
    echo "⚠️ 录音文件未创建（这是正常的，因为是非交互式）"
fi
echo ""

echo "【步骤5: 创建快速测试脚本】"
echo "----------------------------------------"
cat > /tmp/test_voice_repeat_quick.sh << 'TEST_SCRIPT'
#!/bin/bash
cd /home/unitree/workspace/unitree_g1_training

# 检查依赖
echo "检查依赖..."
python3 -c "import whisper" 2>&1 && echo "✅ Whisper可用" || echo "❌ Whisper不可用"

# 测试录音
echo "测试录音（3秒）..."
arecord -D hw:0,0 -d 3 -f S16_LE -r 48000 -c 1 -t wav /tmp/test_rec.wav 2>&1 | head -2

if [ -f /tmp/test_rec.wav ]; then
    echo "✅ 录音文件创建成功"
    # 测试识别
    if python3 -c "import whisper" 2>/dev/null; then
        echo "测试Whisper识别..."
        python3 << PYTHON_EOF
import whisper
print("加载模型...")
model = whisper.load_model("base")
result = model.transcribe("/tmp/test_rec.wav", language="zh")
text = result["text"].strip()
if text:
    print(f"识别结果: {text}")
else:
    print("未识别到文字")
PYTHON_EOF
    fi
fi
TEST_SCRIPT

chmod +x /tmp/test_voice_repeat_quick.sh
echo "✅ 快速测试脚本已创建: /tmp/test_voice_repeat_quick.sh"
echo ""

echo "=========================================="
echo "设置完成"
echo "=========================================="
echo ""
echo "下一步："
echo "1. 确保脚本文件已上传到G1"
echo "2. 运行测试: /tmp/test_voice_repeat_quick.sh"
echo "3. 或直接使用: python3 g1_voice_repeat_simple.py --iface wlan0"
EOF

echo ""
echo "=========================================="
echo "本地设置完成"
echo "=========================================="

