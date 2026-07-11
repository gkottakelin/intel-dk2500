#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ -n "${JETARM_CONTROL_CENTER_PYTHON:-}" ]]; then
  PYTHON_BIN="${JETARM_CONTROL_CENTER_PYTHON}"
elif [[ -x "${PROJECT_ROOT}/.venv-ai/bin/python" ]]; then
  PYTHON_BIN="${PROJECT_ROOT}/.venv-ai/bin/python"
else
  PYTHON_BIN="$(command -v python3 || true)"
fi

if [[ -z "${PYTHON_BIN}" ]]; then
  echo "未找到Python 3。请先安装 python3 和 python3-tk。" >&2
  exit 1
fi

cd "${PROJECT_ROOT}"
exec "${PYTHON_BIN}" -m src.jetarm_control_center "$@"
