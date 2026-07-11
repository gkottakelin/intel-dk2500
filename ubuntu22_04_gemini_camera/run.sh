#!/usr/bin/env bash
set -euo pipefail

APP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="${GEMINI_VENV:-${APP_DIR}/.venv}"
PYTHON_BIN="${VENV_DIR}/bin/python"
SDK_LIB_DIR="${APP_DIR}/sdk/x64"

if [[ ! -x "${PYTHON_BIN}" ]]; then
  echo "未创建 Ubuntu 虚拟环境，请先运行：bash setup.sh" >&2
  exit 1
fi

export LD_LIBRARY_PATH="${SDK_LIB_DIR}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
cd "${APP_DIR}"
exec "${PYTHON_BIN}" gemini_camera.py --color-only "$@"
