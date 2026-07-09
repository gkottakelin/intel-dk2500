#!/usr/bin/env bash
set -euo pipefail

APP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="${JETARM_VENV:-${APP_DIR}/.venv}"
PYTHON_BIN="${VENV_DIR}/bin/python"

if [[ ! -x "${PYTHON_BIN}" ]]; then
  echo "Ubuntu virtual environment was not found. Run: bash setup.sh" >&2
  exit 1
fi

cd "${APP_DIR}"
exec "${PYTHON_BIN}" camera_vector_terminal.py "$@"
