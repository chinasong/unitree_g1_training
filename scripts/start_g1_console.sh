#!/usr/bin/env bash
set -euo pipefail
TASK_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$TASK_ROOT"

# Finder launches without interactive shell/NVM setup.
if ! command -v node >/dev/null 2>&1; then
  for task_node in "$HOME"/.nvm/versions/node/*/bin; do
    if [[ -x "$task_node/node" ]]; then export PATH="$task_node:$PATH"; fi
  done
fi
if [[ "${G1_SKIP_WEB_BUILD:-0}" != "1" ]] && ! command -v npm >/dev/null 2>&1; then
  echo "需要 Node.js 22.12+ / npm，安装后重新打开。" >&2; exit 1
fi
if [[ ! -x .venv/bin/python ]]; then
  if command -v uv >/dev/null 2>&1; then
    uv venv --python 3.11 .venv
  elif [[ -x "$HOME/.local/bin/uv" ]]; then
    "$HOME/.local/bin/uv" venv --python 3.11 .venv
  else
    python3 -m venv .venv
  fi
fi

.venv/bin/python -c 'import sys; assert sys.version_info >= (3,11), "控制台需要 Python 3.11+；请用 uv 创建项目虚拟环境"'
task_fingerprint="$(.venv/bin/python -c 'import hashlib; print(hashlib.sha256(open("requirements-console.lock.txt","rb").read()).hexdigest())')"
if [[ ! -f .venv/.g1-console-deps ]] || [[ "$(cat .venv/.g1-console-deps)" != "$task_fingerprint" ]]; then
  if command -v uv >/dev/null 2>&1; then
    uv pip install --python .venv/bin/python -r requirements-console.lock.txt
  elif [[ -x "$HOME/.local/bin/uv" ]]; then
    "$HOME/.local/bin/uv" pip install --python .venv/bin/python -r requirements-console.lock.txt
  else
    .venv/bin/python -m pip install -r requirements-console.lock.txt
  fi
  echo "$task_fingerprint" > .venv/.g1-console-deps
fi
.venv/bin/python scripts/sync_unitree_deps.py --check || {
  echo "正在恢复锁定的官方依赖；已有修改的目录不会被覆盖。"
  .venv/bin/python scripts/sync_unitree_deps.py
}
if [[ "${G1_SKIP_WEB_BUILD:-0}" == "1" ]]; then
  if [[ ! -f console/web/dist/index.html ]]; then
    echo "缺少预构建界面 console/web/dist；请先在开发机运行 npm run build 并复制 dist。" >&2; exit 1
  fi
else
  if [[ ! -d console/web/node_modules ]]; then
    (cd console/web && npm ci)
  fi
  (cd console/web && npm run build)
fi
if [[ -d .venv/opt/cyclonedds ]]; then
  export CYCLONEDDS_HOME="$TASK_ROOT/.venv/opt/cyclonedds"
fi
echo "G1 身体控制台：http://127.0.0.1:${G1_PORT:-8765}"
if [[ "${G1_NO_BROWSER:-0}" == "1" ]]; then
  exec .venv/bin/python -m g1_console "$@"
else
  exec .venv/bin/python -m g1_console --open "$@"
fi
