#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
if [[ ! -x .venv/bin/python ]]; then
  if command -v uv >/dev/null 2>&1; then
    uv venv --python 3.11 .venv
  elif [[ -x "$HOME/.local/bin/uv" ]]; then
    "$HOME/.local/bin/uv" venv --python 3.11 .venv
  else
    echo "请先安装 uv 或建立 Python 3.11 项目虚拟环境。按回车关闭。"
    read -r
    exit 1
  fi
fi
if ! .venv/bin/python -c 'import paramiko; assert paramiko.__version__ == "5.1.0"' >/dev/null 2>&1; then
  if command -v uv >/dev/null 2>&1; then
    uv pip install --python .venv/bin/python -r requirements-mac-connection.txt
  elif [[ -x "$HOME/.local/bin/uv" ]]; then
    "$HOME/.local/bin/uv" pip install --python .venv/bin/python -r requirements-mac-connection.txt
  else
    .venv/bin/python -m pip install -r requirements-mac-connection.txt
  fi
fi
echo "连接 Orin 完整人工遥控范围：机器人须已由实体遥控器站立并进入行走模式（FSM 500）；软件仅开放按住行走、手臂手势与停止。"
echo "起身、阻尼、模式切换在软件中仍锁定；实体遥控器全程在手，L2+B 为急停兜底。只连接完整人工遥控服务，遇到测试范围服务会拒绝转发。"
echo "自动发现 Orin、核验身份并恢复项目服务；保持此窗口运行，机器人重启后持续重连。"
echo "重连不恢复控制权或运动授权；不会修改 Quantumult X、系统网络或机器人系统服务。"
if ! .venv/bin/python scripts/connect_orin_console.py --manual-remote --open "$@"; then
  echo "连接程序无法启动，请查看上方错误；已有连接窗口时直接使用现有界面。按回车关闭。"
  read -r
  exit 1
fi
