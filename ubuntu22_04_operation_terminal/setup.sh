#!/usr/bin/env bash
set -euo pipefail

APP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="${JETARM_VENV:-${APP_DIR}/.venv}"

if ! command -v python3 >/dev/null 2>&1; then
  echo "python3 was not found. Install the Ubuntu packages first:"
  echo "  sudo apt install python3 python3-venv python3-tk python3-numpy python3-serial fonts-noto-cjk"
  exit 1
fi

if ! python3 -c "import tkinter, numpy, serial" >/dev/null 2>&1; then
  echo "One or more Ubuntu Python packages are missing. Install them with:"
  echo "  sudo apt update"
  echo "  sudo apt install python3-venv python3-tk python3-numpy python3-serial fonts-noto-cjk"
  exit 1
fi

python3 -m venv --system-site-packages "${VENV_DIR}"
"${VENV_DIR}/bin/python" -c "import tkinter, numpy, serial"

echo "JetArm environment is ready: ${VENV_DIR}"
echo "Preview the UI with: bash run.sh --dry-run"
