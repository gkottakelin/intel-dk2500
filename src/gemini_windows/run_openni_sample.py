"""Run bundled Gemini/OpenNI sample programs and project viewer tools."""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path

try:
    from .openni2_ctypes import BIN_DIR, OPENNI_ROOT
except ImportError:
    from openni2_ctypes import BIN_DIR, OPENNI_ROOT  # type: ignore


ROOT = Path(__file__).resolve().parents[3]

SAMPLES = {
    "depth": "DepthReaderPoll.exe",
    "depth-event": "DepthReaderEvent.exe",
    "color": "ColorReaderPoll.exe",
    "color-event": "ColorReaderEvent.exe",
    "color-uvc": "ColorReaderUVC.exe",
    "infrared": "InfraredReaderPoll.exe",
    "infrared-event": "InfraredReaderEvent.exe",
    "simple-viewer": "SimpleViewer.exe",
    "pointcloud": "GeneratePointCloud.exe",
    "multi-depth": "MultiDepthViewer.exe",
    "extended-api": "ExtendedAPI.exe",
}

PYTHON_SAMPLES = {
    "viewer": ["openni_rgbd_viewer.py"],
    "depth-viewer": ["openni_rgbd_viewer.py"],
    "pointcloud-viewer": ["pointcloud_viewer.py", "--live"],
    "pointcloud-watch": ["pointcloud_viewer.py", "--watch"],
}

DEFAULT_ARGS = {
    # Official SimpleViewer.exe [0:Non UVC/1:UVC] [colorMirror: 0/1].
    "simple-viewer": ["1", "1"],
}


def openni_env() -> dict[str, str]:
    env = os.environ.copy()
    env["PATH"] = str(BIN_DIR) + os.pathsep + env.get("PATH", "")
    return env


def run_python_sample(sample: str, extra_args: list[str]) -> None:
    script_args = PYTHON_SAMPLES[sample]
    script = Path(__file__).resolve().parent / script_args[0]
    command = [sys.executable, str(script), *script_args[1:], *extra_args]

    print("Running Python Gemini viewer tool:")
    print(" ".join(command))
    print("")
    subprocess.run(command, cwd=str(ROOT), check=False)


def sample_args(sample: str, extra_args: list[str]) -> list[str]:
    if extra_args:
        return extra_args
    return DEFAULT_ARGS.get(sample, [])


def run_openni_sample(sample: str, extra_args: list[str]) -> None:
    exe = BIN_DIR / SAMPLES[sample]
    if not exe.exists():
        raise FileNotFoundError(f"Sample executable not found: {exe}")

    actual_args = sample_args(sample, extra_args)

    print(f"OpenNI SDK: {OPENNI_ROOT}")
    print(f"Running sample: {exe.name}")
    if actual_args:
        print(f"Sample args: {' '.join(actual_args)}")
    print("Close Orbbec Viewer before running SDK/OpenNI samples.")

    if sample in {"color", "color-event"}:
        print("Gemini Pro Plus color usually appears as a Windows UVC camera; prefer color-uvc or viewer.")
    if sample == "color-uvc":
        print("ColorReaderUVC uses the tutorial UVC MJPEG/YUV color path.")
    if sample == "simple-viewer":
        print("simple-viewer is the original official RGB-D overlay demo.")
        print("For separate depth / infrared / color windows, run: viewer")
    if sample == "pointcloud":
        print("GeneratePointCloud.exe stops after 50 frames by design.")
        print("For continuous point cloud viewing, run: pointcloud-viewer")
    print("")

    subprocess.run([str(exe), *actual_args], cwd=str(BIN_DIR), env=openni_env(), check=False)


def main() -> None:
    parser = argparse.ArgumentParser(description="Run bundled OpenNI samples or project Gemini viewer tools")
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
