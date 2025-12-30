#!/bin/bash
# 安装语音识别依赖

echo "=========================================="
echo "安装G1语音识别依赖"
echo "=========================================="
echo ""

echo "【选项1: 使用Whisper（推荐，离线识别）】"
echo "优点: 离线工作，无需网络，准确度高"
echo "安装: pip3 install openai-whisper"
echo ""

echo "【选项2: 使用SpeechRecognition（在线识别）】"
echo "优点: 轻量级，使用Google API"
echo "安装: pip3 install SpeechRecognition"
echo ""

echo "【选项3: 使用Vosk（离线，轻量）】"
echo "优点: 完全离线，速度快"
echo "安装: pip3 install vosk"
echo "     需要下载中文模型"
echo ""

read -p "请选择安装方式 (1/2/3，或按回车跳过): " choice

case $choice in
    1)
        echo "安装Whisper..."
        pip3 install openai-whisper
        echo "✅ Whisper安装完成"
        echo ""
        echo "首次使用时会自动下载模型（约150MB-3GB，取决于模型大小）"
        echo "推荐使用 'base' 模型（平衡速度和准确度）"
        ;;
    2)
        echo "安装SpeechRecognition..."
        pip3 install SpeechRecognition
        echo "✅ SpeechRecognition安装完成"
        echo ""
        echo "注意: 需要网络连接才能使用Google识别服务"
        ;;
    3)
        echo "安装Vosk..."
        pip3 install vosk
        echo "✅ Vosk安装完成"
        echo ""
        echo "需要下载中文模型:"
        echo "  wget https://alphacephei.com/vosk/models/vosk-model-small-cn-0.22.zip"
        echo "  unzip vosk-model-small-cn-0.22.zip"
        ;;
    *)
        echo "跳过安装"
        ;;
esac

echo ""
echo "=========================================="
echo "安装完成"
echo "=========================================="

