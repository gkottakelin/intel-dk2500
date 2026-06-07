"""Diagnose the Windows Python environment for Gemini Pro Plus development."""

from __future__ import annotations

import importlib.util
import os
import platform
import subprocess
import sys


def log(message: str) -> None:
    print(message, flush=True)


def check_module(module_name: str) -> None:
    spec = importlib.util.find_spec(module_name)
    if spec is None:
        log(f"[MISS] {module_name}: 未安装或当前 Python 找不到")
    else:
        log(f"[ OK ] {module_name}: {spec.origin}")


def main() -> None:
    log("Gemini Windows 环境诊断开始")
    log(f"Python: {sys.version}")
    log(f"Executable: {sys.executable}")
    log(f"Platform: {platform.platform()}")
    log(f"CWD: {os.getcwd()}")
    log("")

    for name in ("numpy", "cv2", "pyorbbecsdk"):
        check_module(name)

    log("")
    log("开始实际导入 numpy...")
    import numpy  # noqa: F401

    log("numpy 导入成功")
    log("开始实际导入 cv2...")
    import cv2  # noqa: F401

    log("cv2 导入成功")
    log("开始在子进程中实际导入 pyorbbecsdk...")
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            "import pyorbbecsdk; print('pyorbbecsdk import ok', flush=True)",
        ],
        capture_output=True,
        text=True,
        timeout=15,
    )
    log(f"pyorbbecsdk 子进程返回码：{result.returncode}")
    log(f"pyorbbecsdk 子进程 stdout：{result.stdout.strip() or '<empty>'}")
    log(f"pyorbbecsdk 子进程 stderr：{result.stderr.strip() or '<empty>'}")
    if result.returncode != 0 or "pyorbbecsdk import ok" not in result.stdout:
        log("结论：pyorbbecsdk 原生扩展导入失败，当前问题不在相机业务代码。")
        return

    log("pyorbbecsdk 导入成功")
    log("环境诊断完成：依赖导入正常")


if __name__ == "__main__":
    main()
