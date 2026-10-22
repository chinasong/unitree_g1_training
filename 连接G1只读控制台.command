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
echo "连接 Orin 只读控制台；SSH 密码交互输入，不保存。"
echo "自动发现 Orin、核验身份并恢复项目服务；保持此窗口运行，机器人重启后持续重连。"
echo "不会修改 Quantumult X、系统网络或机器人系统服务；不会执行动作。"
if ! .venv/bin/python scripts/connect_orin_console.py --open "$@"; then
  echo "连接程序无法启动，请查看上方错误；已有连接窗口时直接使用现有界面。按回车关闭。"
  read -r
  exit 1
fi
