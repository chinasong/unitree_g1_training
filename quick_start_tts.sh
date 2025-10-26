#!/bin/bash
# G1 TTS 服务快速启动脚本

echo "🤖 G1 机器人 TTS 服务快速启动"
echo "================================"
echo ""

# 检查网络接口
echo "📡 检测可用网络接口..."
echo ""
ifconfig -a | grep -E "^[a-z]" | cut -d: -f1
echo ""

# 提示用户输入网络接口
read -p "请输入连接G1的网络接口名称 (例如: eth0): " IFACE

if [ -z "$IFACE" ]; then
    echo "❌ 网络接口不能为空！"
    exit 1
fi

echo ""
echo "选择运行模式:"
echo "  1) HTTP服务器模式 (推荐)"
echo "  2) 交互式命令行模式"
echo "  3) 测试模式 (让机器人说一句话)"
echo ""

read -p "请选择 (1/2/3): " MODE

case $MODE in
    1)
        echo ""
        echo "🌐 启动HTTP服务器..."
        echo "   访问地址: http://localhost:8080"
        echo "   按 Ctrl+C 停止服务"
        echo ""
        python3 g1_tts_service.py --iface $IFACE --server
        ;;
    2)
        echo ""
        echo "💻 启动交互式模式..."
        echo "   输入文字让机器人说话"
        echo "   输入 /quit 退出"
        echo ""
        python3 g1_tts_service.py --iface $IFACE --interactive
        ;;
    3)
        echo ""
        read -p "输入要说的话: " TEXT
        if [ -z "$TEXT" ]; then
            TEXT="你好，我是宇树G1机器人！"
        fi
        echo ""
        echo "🗣️ 让机器人说话..."
        python3 g1_tts_service.py --iface $IFACE --speak "$TEXT"
        ;;
    *)
        echo "❌ 无效选择！"
        exit 1
        ;;
esac

