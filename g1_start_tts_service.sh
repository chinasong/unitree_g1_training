#!/bin/bash
# 启动G1 TTS服务脚本

echo "=========================================="
echo "G1机器人TTS服务启动脚本"
echo "=========================================="
echo ""

# 检测网络接口
echo "检测网络接口..."
INTERFACE=$(sshpass -p 123 ssh -o StrictHostKeyChecking=no unitree@192.168.1.28 "ip route | grep default | awk '{print \$5}' | head -1")
echo "检测到的网络接口: $INTERFACE"
echo ""

# 询问使用模式
echo "请选择运行模式:"
echo "1) HTTP服务器模式（推荐，可通过浏览器和API访问）"
echo "2) 交互式命令行模式"
echo "3) 直接说一句话"
read -p "请选择 (1-3): " mode

case $mode in
    1)
        echo ""
        echo "启动HTTP服务器模式..."
        echo "服务将在 http://192.168.1.28:8080 启动"
        echo "按 Ctrl+C 停止服务"
        echo ""
        sshpass -p 123 ssh -o StrictHostKeyChecking=no unitree@192.168.1.28 bash << EOF
cd /home/unitree/workspace/unitree_g1_training
export PYTHONPATH=/home/unitree/unitree_sdk2_python:\$PYTHONPATH
python3 g1_tts_service.py --iface $INTERFACE --server --port 8080
EOF
        ;;
    2)
        echo ""
        echo "启动交互式命令行模式..."
        echo "输入文字让机器人说话，输入 /quit 退出"
        echo ""
        sshpass -p 123 ssh -o StrictHostKeyChecking=no -t unitree@192.168.1.28 bash << EOF
cd /home/unitree/workspace/unitree_g1_training
export PYTHONPATH=/home/unitree/unitree_sdk2_python:\$PYTHONPATH
python3 g1_tts_service.py --iface $INTERFACE --interactive
EOF
        ;;
    3)
        read -p "请输入要说的话: " text
        echo ""
        echo "让机器人说话: $text"
        sshpass -p 123 ssh -o StrictHostKeyChecking=no unitree@192.168.1.28 bash << EOF
cd /home/unitree/workspace/unitree_g1_training
export PYTHONPATH=/home/unitree/unitree_sdk2_python:\$PYTHONPATH
python3 g1_tts_service.py --iface $INTERFACE --speak "$text"
EOF
        ;;
    *)
        echo "无效选择"
        exit 1
        ;;
esac

