"""Run bundled OpenNI sample programs for Gemini Pro Plus on Windows."""

from __future__ import annotations

import argparse
import os
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
OPENNI_ROOT = ROOT / "gemini深度相机windows资料" / "Windows" / "Windows" / "OpenNI_v2.3.0.85_20220615_1b09bbfd_windows_x64_x86_release"
BIN_DIR = OPENNI_ROOT / "samples" / "bin"

SAMPLES = {
    "depth": "DepthReaderPoll.exe",
    "depth-event": "DepthReaderEvent.exe",
    "color": "ColorReaderPoll.exe",
    "color-event": "ColorReaderEvent.exe",
    "color-uvc": "ColorReaderUVC.exe",
    "infrared": "InfraredReaderPoll.exe",
    "infrared-event": "InfraredReaderEvent.exe",
    "viewer": "SimpleViewer.exe",
    "pointcloud": "GeneratePointCloud.exe",
    "multi-depth": "MultiDepthViewer.exe",
    "extended-api": "ExtendedAPI.exe",
}


def main() -> None:
    parser = argparse.ArgumentParser(description="Run bundled OpenNI sample programs")
    parser.add_argument("sample", choices=sorted(SAMPLES), help="sample program to run")
    parser.add_argument("args", nargs=argparse.REMAINDER, help="extra args passed to the sample")
    parsed = parser.parse_args()

    exe = BIN_DIR / SAMPLES[parsed.sample]
    if not exe.exists():
        raise FileNotFoundError(f"未找到示例程序：{exe}")

    env = os.environ.copy()
    env["PATH"] = str(BIN_DIR) + os.pathsep + env.get("PATH", "")

    print(f"OpenNI SDK: {OPENNI_ROOT}")
    print(f"运行示例: {exe.name}")
    print("提示：请先关闭 Orbbec Viewer；按任意键或 ESC 可退出部分示例。")
    if parsed.sample in {"color", "color-event"}:
        print("注意：Gemini Pro Plus 的彩色头在 Windows 下通常是 UVC 设备；如果本示例启动失败，请使用 color-uvc。")
    if parsed.sample == "color-uvc":
        print("说明：color-uvc 通过 UVC 读取彩色 MJPEG/YUV，更适合当前 Gemini Pro Plus。")
    print("")

    subprocess.run([str(exe), *parsed.args], cwd=str(BIN_DIR), env=env, check=False)


if __name__ == "__main__":
    main()
