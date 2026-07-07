#!/usr/bin/env bash
set -euo pipefail

APP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="${GEMINI_VENV:-${APP_DIR}/.venv}"

if ! command -v python3 >/dev/null 2>&1; then
  echo "未找到 python3。请先安装 Ubuntu 依赖：" >&2
  echo "  sudo apt update" >&2
  echo "  sudo apt install -y python3 python3-venv python3-tk python3-numpy python3-opencv libusb-1.0-0 usbutils fonts-noto-cjk" >&2
  exit 1
fi

case "$(uname -m)" in
  x86_64|amd64) ;;
  *)
    echo "当前内置 OrbbecSDK 只支持 Intel/AMD x86_64，检测到：$(uname -m)" >&2
    exit 1
    ;;
esac

if ! python3 -c "import tkinter, numpy, cv2" >/dev/null 2>&1; then
  echo "缺少 Ubuntu Python 依赖。请执行：" >&2
  echo "  sudo apt update" >&2
  echo "  sudo apt install -y python3 python3-venv python3-tk python3-numpy python3-opencv libusb-1.0-0 usbutils fonts-noto-cjk" >&2
  exit 1
fi

python3 -m venv --system-site-packages "${VENV_DIR}"
"${VENV_DIR}/bin/python" -c "import tkinter, numpy, cv2"

if [[ ! -f /etc/udev/rules.d/99-obsensor-libusb.rules ]]; then
  echo "注意：尚未安装 Orbbec USB 权限规则。请执行："
  echo "  sudo bash ${APP_DIR}/install_udev_rules.sh"
fi

echo "Gemini Ubuntu 环境已就绪：${VENV_DIR}"
echo "下一步：bash run.sh --diagnose"
