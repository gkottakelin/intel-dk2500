#!/usr/bin/env bash
set -euo pipefail

APP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_RULES="${APP_DIR}/sdk/99-obsensor-libusb.rules"
TARGET_RULES="/etc/udev/rules.d/99-obsensor-libusb.rules"

if [[ "${EUID}" -ne 0 ]]; then
  echo "请使用 sudo 运行：sudo bash install_udev_rules.sh" >&2
  exit 1
fi

install -m 0644 "${SOURCE_RULES}" "${TARGET_RULES}"
udevadm control --reload-rules
udevadm trigger

echo "已安装 Orbbec udev 规则：${TARGET_RULES}"
echo "请拔插 Gemini 相机；如果设备仍无权限，请注销并重新登录。"
