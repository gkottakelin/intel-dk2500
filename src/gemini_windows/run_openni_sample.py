"""Run bundled OpenNI sample programs for Gemini Pro Plus on Windows."""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
OPENNI_ROOT = (
    ROOT
    / "gemini深度相机windows资料"
    / "Windows"
    / "Windows"
    / "OpenNI_v2.3.0.85_20220615_1b09bbfd_windows_x64_x86_release"
)
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

PYTHON_SAMPLES = {
    "depth-viewer": ["depth_viewer.py"],
    "pointcloud-viewer": ["pointcloud_viewer.py", "--live"],
    "pointcloud-watch": ["pointcloud_viewer.py", "--watch"],
}


def openni_env() -> dict[str, str]:
    env = os.environ.copy()
    env["PATH"] = str(BIN_DIR) + os.pathsep + env.get("PATH", "")
    return env


def run_python_sample(sample: str, extra_args: list[str]) -> None:
    script_args = PYTHON_SAMPLES[sample]
    script = Path(__file__).resolve().parent / script_args[0]
    command = [sys.executable, str(script), *script_args[1:], *extra_args]

    print("运行 Python 点云可视化工具")
    print(" ".join(command))
    print("")
    subprocess.run(command, cwd=str(ROOT), check=False)


def run_openni_sample(sample: str, extra_args: list[str]) -> None:
    exe = BIN_DIR / SAMPLES[sample]
    if not exe.exists():
        raise FileNotFoundError(f"未找到示例程序：{exe}")

    print(f"OpenNI SDK: {OPENNI_ROOT}")
    print(f"运行示例: {exe.name}")
    print("提示：请先关闭 Orbbec Viewer；按任意键或 ESC 可退出部分示例。")

    if sample in {"color", "color-event"}:
        print("注意：Gemini Pro Plus 的彩色头在 Windows 下通常是 UVC 设备；如本示例启动失败，请使用 color-uvc。")
    if sample == "color-uvc":
        print("说明：color-uvc 通过 UVC 读取彩色 MJPEG/YUV，更适合当前 Gemini Pro Plus。")
    if sample == "viewer":
        print("说明：viewer 是 RGB-D 二维对齐叠加显示，不是三维点云显示。")
    if sample == "pointcloud":
        print("说明：GeneratePointCloud.exe 官方示例只生成 50 帧，到 50 帧会自动停止。")
        print("如需 Viewer 风格的连续点云可视化，请运行：")
        print("python project/src/gemini_windows/run_openni_sample.py pointcloud-viewer")
    print("")

    subprocess.run([str(exe), *extra_args], cwd=str(BIN_DIR), env=openni_env(), check=False)


def main() -> None:
    parser = argparse.ArgumentParser(description="Run bundled OpenNI sample programs")
    choices = sorted([*SAMPLES, *PYTHON_SAMPLES])
    parser.add_argument("sample", choices=choices, help="sample program to run")
    parser.add_argument("args", nargs=argparse.REMAINDER, help="extra args passed to the sample")
    parsed = parser.parse_args()

    if parsed.sample in PYTHON_SAMPLES:
        run_python_sample(parsed.sample, parsed.args)
    else:
        run_openni_sample(parsed.sample, parsed.args)


if __name__ == "__main__":
    main()
