#!/usr/bin/env bash
cd "$(dirname "$0")"
./scripts/start_g1_console.sh "$@"
task_result=$?
if [[ $task_result != 0 ]]; then
  echo "启动失败（$task_result），请查看上方错误。按回车关闭。"
  read -r
fi
