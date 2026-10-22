#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
if [[ ! -x .venv/bin/python ]]; then
  echo "请先运行 启动G1控制台.command 建立项目环境。按回车关闭。"
  read -r
  exit 1
fi
echo "Mac Codex 只读自主观察：请保持 Orin 转发连接，并在网页环境观察中点击开始观察。"
echo "使用已登录的 Codex 服务分析图像和身体状态；不执行动作。Ctrl+C 结束观察进程。"
echo "默认使用本账号 CLI 已验证的 gpt-5.6-sol，不修改桌面模型配置；可用 --model 覆盖。"
if ! .venv/bin/python scripts/observe_g1.py --model gpt-5.6-sol "$@"; then
  echo "观察进程已退出；请检查终端信息。按回车关闭。"
  read -r
  exit 1
fi
