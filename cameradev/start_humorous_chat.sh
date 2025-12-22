#!/bin/bash
# G1机器人智能幽默聊天系统快速启动脚本

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  G1机器人智能幽默聊天系统${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# 检查参数
if [ $# -eq 0 ]; then
    echo -e "${YELLOW}用法: $0 <网络接口名称>${NC}"
    echo ""
    echo "示例:"
    echo "  $0 eth0"
    echo "  $0 enp3s0"
    echo ""
    echo "查找网络接口:"
    echo "  ifconfig"
    echo "  或"
    echo "  ip addr"
    exit 1
fi

NET_IFACE=$1

# 检查Python
if ! command -v python3 &> /dev/null; then
    echo -e "${RED}❌ 未找到python3，请先安装Python 3${NC}"
    exit 1
fi

# 检查ROS
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}⚠️  未检测到ROS环境，尝试source ROS setup.bash${NC}"
    if [ -f "/opt/ros/noetic/setup.bash" ]; then
        source /opt/ros/noetic/setup.bash
    elif [ -f "/opt/ros/melodic/setup.bash" ]; then
        source /opt/ros/melodic/setup.bash
    else
        echo -e "${RED}❌ 请先配置ROS环境${NC}"
        exit 1
    fi
fi

# 检查必要的Python包
echo -e "${YELLOW}📦 检查依赖...${NC}"
python3 -c "import cv2, rospy, ultralytics" 2>/dev/null
if [ $? -ne 0 ]; then
    echo -e "${RED}❌ 缺少必要的Python包，请运行:${NC}"
    echo "  pip3 install opencv-python rospy ultralytics"
    exit 1
fi

# 检查可选依赖
python3 -c "import speech_recognition" 2>/dev/null
if [ $? -ne 0 ]; then
    echo -e "${YELLOW}⚠️  未安装speech_recognition，语音识别功能将不可用${NC}"
fi

python3 -c "import openai" 2>/dev/null
if [ $? -ne 0 ]; then
    echo -e "${YELLOW}⚠️  未安装openai，将使用简单的回复生成器${NC}"
fi

# 检查环境变量
if [ -z "$DEEPSEEK_API_KEY" ]; then
    echo -e "${YELLOW}⚠️  未设置DEEPSEEK_API_KEY环境变量${NC}"
    echo -e "${YELLOW}   将使用简单规则生成回复${NC}"
    echo -e "${YELLOW}   要使用AI生成回复，请设置: export DEEPSEEK_API_KEY=\"your-key\"${NC}"
    echo -e "${YELLOW}   详细说明请查看: UBUNTU_ENV_SETUP.md${NC}"
else
    echo -e "${GREEN}✅ DEEPSEEK_API_KEY已设置${NC}"
fi

# 检查YOLO模型
if [ ! -f "yolo11n.pt" ]; then
    echo -e "${YELLOW}⚠️  未找到yolo11n.pt，YOLO将在首次运行时自动下载${NC}"
fi

# 检查标签映射文件
if [ ! -f "en_to_zh.js" ]; then
    echo -e "${RED}❌ 未找到en_to_zh.js文件${NC}"
    exit 1
fi

# 切换到脚本目录
cd "$(dirname "$0")"

echo ""
echo -e "${GREEN}✅ 所有检查通过！${NC}"
echo ""
echo -e "${GREEN}🚀 启动G1智能幽默聊天系统...${NC}"
echo ""

# 运行主程序
python3 g1_humorous_chat.py "$NET_IFACE"

