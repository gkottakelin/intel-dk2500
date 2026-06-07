"""Check whether Windows can see Gemini/Orbbec USB devices.

This script does not require pyorbbecsdk. It only queries Windows PnP devices,
so it is useful when Orbbec SDK reports zero devices.
"""

from __future__ import annotations

import json
import subprocess


KEYWORDS = (
    "orbbec",
    "gemini",
    "astra",
    "depth",
    "camera",
    "usb video",
    "uvc",
    "vid_2bc5",
    "sv1301",
)


def _run_powershell_json(command: str) -> list[dict]:
    result = subprocess.run(
        ["powershell", "-NoProfile", "-ExecutionPolicy", "Bypass", "-Command", command],
        capture_output=True,
        text=True,
        timeout=20,
    )
    if result.returncode != 0:
        print(result.stderr.strip() or result.stdout.strip() or "PowerShell 查询失败")
        return []
    text = result.stdout.strip()
    if not text:
        return []
    data = json.loads(text)
    if isinstance(data, dict):
        return [data]
    return data


def main() -> None:
    print("Windows USB/PnP 相机设备检查", flush=True)
    print("说明：Gemini Pro Plus 是 USB 设备，不是 COM 口设备。", flush=True)
    print("")

    devices = _run_powershell_json(
        "Get-PnpDevice | "
        "Select-Object Status,Class,FriendlyName,InstanceId | "
        "ConvertTo-Json -Depth 3"
    )
    matches: list[dict] = []
    for device in devices:
        haystack = " ".join(str(device.get(key, "")) for key in ("Class", "FriendlyName", "InstanceId")).lower()
        if any(keyword in haystack for keyword in KEYWORDS):
            matches.append(device)

    if not matches:
        print("未在 Windows PnP 设备中找到明显的 Orbbec/Gemini/Camera 设备。")
        print("请检查 USB 线、接口、供电、设备管理器，以及 Orbbec Viewer 是否能识别相机。")
    else:
        print(f"找到可能相关设备：{len(matches)}")
        for index, device in enumerate(matches):
            print(f"\n[{index}]")
            print(f"  Status: {device.get('Status')}")
            print(f"  Class: {device.get('Class')}")
            print(f"  FriendlyName: {device.get('FriendlyName')}")
            print(f"  InstanceId: {device.get('InstanceId')}")

    print("\nUSB 控制器概览：")
    controllers = _run_powershell_json(
        "Get-PnpDevice -Class USB | "
        "Select-Object Status,Class,FriendlyName,InstanceId | "
        "ConvertTo-Json -Depth 3"
    )
    for device in controllers[:40]:
        name = str(device.get("FriendlyName", ""))
        status = device.get("Status")
        if "USB" in name.upper() or "XHCI" in name.upper() or "HUB" in name.upper():
            print(f"  [{status}] {name}")


if __name__ == "__main__":
    main()
