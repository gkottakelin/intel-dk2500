"""List connected Orbbec devices on Windows."""

from __future__ import annotations

from gemini_common import import_orbbec_sdk


def main() -> None:
    print("开始枚举 Orbbec 设备...", flush=True)
    sdk = import_orbbec_sdk()
    print("pyorbbecsdk 导入成功", flush=True)
    context = sdk.Context()
    print("Context 创建成功", flush=True)
    devices = context.query_devices()
    count = devices.get_count()

    print(f"发现 Orbbec 设备数量：{count}", flush=True)
    if count == 0:
        print("未发现设备。Gemini Pro Plus 是 USB 深度相机，不通过 COM 口访问。", flush=True)
        print("请先运行 windows_usb_check.py，确认 Windows 设备管理器层面能看到相机。", flush=True)
        print("同时关闭 Orbbec Viewer，避免 SDK 枚举时设备被占用。", flush=True)
        return

    for index in range(count):
        device = devices.get_device_by_index(index)
        info = device.get_device_info()
        print(f"\n[{index}]")
        print(f"  name: {info.get_name()}")
        print(f"  serial: {info.get_serial_number()}")
        print(f"  pid: {info.get_pid()}")
        print(f"  vid: {info.get_vid()}")
        try:
            print(f"  firmware: {info.get_firmware_version()}")
        except Exception:
            pass
        try:
            print(f"  connection: {info.get_connection_type()}")
        except Exception:
            pass


if __name__ == "__main__":
    main()
