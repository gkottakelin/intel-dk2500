"""Manual pixel closed-loop V2 profile.

The interaction workflow, pixel-scale model, dynamic tolerance bands, staged
descent, final grasp, and Home behavior are intentionally reused from the V1
manual pixel test.  Only the grasp-point default and Cartesian motion runtime
change in V2.
"""

from __future__ import annotations

import argparse
import math


DEFAULT_MANUAL_GRASP_X = 320.0
DEFAULT_MANUAL_GRASP_Y = 147.0
CAMERA_VECTOR_VERSION = "v2"


def validate_grasp_point_pixel(
    x_value: object,
    y_value: object,
    *,
    image_width: int,
    image_height: int,
) -> tuple[float, float]:
    """Validate a grasp-point pixel entered in the V2 startup dialog."""

    try:
        x = float(x_value)
        y = float(y_value)
    except (TypeError, ValueError) as exc:
        raise ValueError("抓取点X、Y必须是数字") from exc
    if not math.isfinite(x) or not math.isfinite(y):
        raise ValueError("抓取点X、Y必须是有限数字")
    if image_width <= 0 or image_height <= 0:
        raise ValueError("图像宽度和高度必须大于0")
    if not 0.0 <= x < float(image_width):
        raise ValueError(f"抓取点X必须在0到{image_width - 1}之间")
    if not 0.0 <= y < float(image_height):
        raise ValueError(f"抓取点Y必须在0到{image_height - 1}之间")
    return x, y


def prompt_grasp_point_pixel(
    *,
    initial_x: float,
    initial_y: float,
    image_width: int,
    image_height: int,
) -> tuple[float, float] | None:
    """Show the V2 grasp-point dialog before any arm connection is created."""

    try:
        import tkinter as tk
        from tkinter import ttk
    except ImportError as exc:  # pragma: no cover - host package dependency.
        raise RuntimeError("缺少Tkinter，无法打开抓取点像素配置窗口") from exc

    try:
        root = tk.Tk()
    except Exception as exc:  # pragma: no cover - depends on desktop session.
        raise RuntimeError(f"无法打开抓取点像素配置窗口：{exc}") from exc

    root.title("基于摄像头的机械臂操控 · 抓取点像素配置")
    root.resizable(False, False)
    result: dict[str, tuple[float, float] | None] = {"point": None}
    x_text = tk.StringVar(value=f"{initial_x:g}")
    y_text = tk.StringVar(value=f"{initial_y:g}")
    error_text = tk.StringVar(value="")

    body = ttk.Frame(root, padding=20)
    body.grid(row=0, column=0, sticky="nsew")
    body.columnconfigure(1, weight=1)
    ttk.Label(
        body,
        text="启动前设置固定抓取点像素",
        font=("Noto Sans CJK SC", 13, "bold"),
    ).grid(row=0, column=0, columnspan=2, sticky="w", pady=(0, 8))
    ttk.Label(
        body,
        text=(
            f"图像尺寸：{image_width}×{image_height}；左上角为(0,0)，"
            "X向右、Y向下。仅本次人工测试生效。"
        ),
        wraplength=430,
    ).grid(row=1, column=0, columnspan=2, sticky="w", pady=(0, 14))
    ttk.Label(body, text="抓取点像素 X").grid(row=2, column=0, sticky="w", padx=(0, 12))
    x_entry = ttk.Entry(body, textvariable=x_text, width=22)
    x_entry.grid(row=2, column=1, sticky="ew", pady=4)
    ttk.Label(body, text="抓取点像素 Y").grid(row=3, column=0, sticky="w", padx=(0, 12))
    y_entry = ttk.Entry(body, textvariable=y_text, width=22)
    y_entry.grid(row=3, column=1, sticky="ew", pady=4)
    ttk.Label(body, textvariable=error_text, foreground="#c62828").grid(
        row=4, column=0, columnspan=2, sticky="w", pady=(8, 0)
    )

    def cancel() -> None:
        result["point"] = None
        root.destroy()

    def confirm() -> None:
        try:
            point = validate_grasp_point_pixel(
                x_text.get(),
                y_text.get(),
                image_width=image_width,
                image_height=image_height,
            )
        except ValueError as exc:
            error_text.set(str(exc))
            return
        result["point"] = point
        root.destroy()

    buttons = ttk.Frame(body)
    buttons.grid(row=5, column=0, columnspan=2, sticky="e", pady=(16, 0))
    ttk.Button(buttons, text="取消", command=cancel).pack(side="left", padx=(0, 8))
    ttk.Button(buttons, text="确认并启动", command=confirm).pack(side="left")
    root.protocol("WM_DELETE_WINDOW", cancel)
    root.bind("<Escape>", lambda _event: cancel())
    root.bind("<Return>", lambda _event: confirm())
    x_entry.focus_set()
    x_entry.selection_range(0, tk.END)
    root.mainloop()
    return result["point"]


async def run_manual_pixel_test_v2(args: argparse.Namespace) -> int:
    """Run the shared manual workflow with the V2 camera-vector runtime."""

    if args.manual_image_width <= 0 or args.manual_image_height <= 0:
        raise ValueError("manual image width/height必须大于0")
    initial_x = (
        float(args.manual_grasp_x)
        if args.manual_grasp_x is not None
        else DEFAULT_MANUAL_GRASP_X
    )
    initial_y = (
        float(args.manual_grasp_y)
        if args.manual_grasp_y is not None
        else DEFAULT_MANUAL_GRASP_Y
    )
    point = prompt_grasp_point_pixel(
        initial_x=initial_x,
        initial_y=initial_y,
        image_width=args.manual_image_width,
        image_height=args.manual_image_height,
    )
    if point is None:
        print("已取消基于摄像头的机械臂操控；未连接机械臂。")
        return 0

    configured_args = argparse.Namespace(**vars(args))
    configured_args.manual_grasp_x = point[0]
    configured_args.manual_grasp_y = point[1]

    # Import lazily so cli.py can expose this profile as a mutually exclusive
    # mode without introducing a module-import cycle.
    from .cli import _run_manual_pixel_test

    return await _run_manual_pixel_test(
        configured_args,
        default_grasp_x=DEFAULT_MANUAL_GRASP_X,
        default_grasp_y=DEFAULT_MANUAL_GRASP_Y,
        camera_vector_version=CAMERA_VECTOR_VERSION,
        display_name="基于摄像头的机械臂操控",
    )
