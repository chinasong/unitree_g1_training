#!/bin/bash
# G1机器人远程命令执行脚本
# 使用方法: ./g1_remote_exec.sh "命令"
# 例如: ./g1_remote_exec.sh "ls -la"

G1_HOST="192.168.1.28"
G1_USER="unitree"
G1_PASSWORD="123"

if [ -z "$1" ]; then
    echo "使用方法: $0 \"命令\""
    echo "例如: $0 \"ls -la\""
    echo "      $0 \"ps aux\""
    echo "      $0 \"df -h\""
    exit 1
fi

COMMAND="$1"

# 检查是否安装了sshpass
if command -v sshpass &> /dev/null; then
    sshpass -p "${G1_PASSWORD}" ssh -T -o StrictHostKeyChecking=no -o ConnectTimeout=10 -o RequestTTY=no ${G1_USER}@${G1_HOST} "${COMMAND}"
else
    echo "错误: 未安装sshpass，无法自动输入密码" >&2
    echo "请安装sshpass:" >&2
    echo "  macOS: brew install hudochenkov/sshpass/sshpass" >&2
    echo "  Ubuntu: sudo apt-get install sshpass" >&2
    echo "" >&2
    echo "或者手动执行SSH命令:" >&2
    echo "  ssh ${G1_USER}@${G1_HOST} \"${COMMAND}\"" >&2
    echo "  密码: ${G1_PASSWORD}" >&2
    exit 1
fi

