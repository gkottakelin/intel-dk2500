#!/usr/bin/env bash
set -euo pipefail

APP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="${JETARM_VENV:-${APP_DIR}/.venv}"

if ! command -v python3 >/dev/null 2>&1; then
  echo "未找到 python3。请先执行："
  echo "  sudo apt install python3 python3-venv python3-tk fonts-noto-cjk"
  exit 1
fi

if ! python3 -c "import tkinter" >/dev/null 2>&1; then
  echo "缺少 Tkinter。请先执行："
  echo "  sudo apt install python3-venv python3-tk fonts-noto-cjk"
  exit 1
fi

python3 -m venv "${VENV_DIR}"
"${VENV_DIR}/bin/python" -m pip install --upgrade pip
"${VENV_DIR}/bin/python" -m pip install -r "${APP_DIR}/requirements.txt"

echo "环境创建完成：${VENV_DIR}"
echo "预览界面：bash run.sh --dry-run"
