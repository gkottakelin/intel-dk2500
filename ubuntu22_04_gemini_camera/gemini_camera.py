"""Standalone Gemini RGB-D viewer for Ubuntu 22.04.

This is the Linux port of ``src/gemini_windows``.  It uses the legacy Linux
OrbbecSDK shipped with this project because Gemini (USB PID 0614/0511) is an
OpenNI-protocol device and is not enumerated by the newer pyorbbecsdk2 path.
"""

from __future__ import annotations

import argparse
import json
import os
import platform
import subprocess
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from time import strftime
from typing import Any, Callable, Optional

import numpy as np

try:
    from .orbbec_native import (
        APP_ROOT,
        DEFAULT_SDK_CONFIG,
        DEFAULT_SDK_LIBRARY,
        OB_FORMAT_BGR,
        OB_FORMAT_BGRA,
        OB_FORMAT_I420,
        OB_FORMAT_MJPG,
        OB_FORMAT_NV12,
        OB_FORMAT_NV21,
        OB_FORMAT_RGB,
        OB_FORMAT_UYVY,
        OB_FORMAT_YUY2,
        OB_FORMAT_YUYV,
        CameraDeviceInfo,
        Intrinsics,
        NativeFrame,
        OrbbecSession,
        depth_frame_to_mm,
        enumerate_devices,
    )
except ImportError:  # Standalone execution from this directory.
    from orbbec_native import (
        APP_ROOT,
        DEFAULT_SDK_CONFIG,
        DEFAULT_SDK_LIBRARY,
        OB_FORMAT_BGR,
        OB_FORMAT_BGRA,
        OB_FORMAT_I420,
        OB_FORMAT_MJPG,
        OB_FORMAT_NV12,
        OB_FORMAT_NV21,
        OB_FORMAT_RGB,
        OB_FORMAT_UYVY,
        OB_FORMAT_YUY2,
        OB_FORMAT_YUYV,
        CameraDeviceInfo,
        Intrinsics,
        NativeFrame,
        OrbbecSession,
        depth_frame_to_mm,
        enumerate_devices,
    )

try:
    import tkinter as tk
    from tkinter import messagebox, ttk
except ImportError:  # pragma: no cover - depends on Ubuntu system packages
    tk = None
    ttk = None
    messagebox = None


DEFAULT_CONFIG_PATH = APP_ROOT / "config" / "camera.json"
WINDOW_NAME = "Gemini Ubuntu RGB-D"
ESC_KEY = 27


@dataclass(frozen=True)
class CameraSettings:
    serial_number: str
    frame_timeout_ms: int
    min_depth_mm: float
    max_depth_mm: float
    click_window: int
    mirror_color: bool
    mirror_depth: bool
    save_dir: Path
    sdk_library: Path
    sdk_config: Path

    @classmethod
    def from_file(cls, path: str | Path = DEFAULT_CONFIG_PATH) -> "CameraSettings":
        config_path = Path(path).resolve()
        with config_path.open("r", encoding="utf-8") as file:
            data = json.load(file)

        def app_path(value: str) -> Path:
            candidate = Path(value)
            return candidate.resolve() if candidate.is_absolute() else (APP_ROOT / candidate).resolve()

        device = data["device"]
        stream = data["stream"]
        depth = data["depth"]
        display = data["display"]
        capture = data["capture"]
        sdk = data.get("sdk", {})
        settings = cls(
            serial_number=str(device.get("serial_number", "")).strip(),
            frame_timeout_ms=int(stream["frame_timeout_ms"]),
            min_depth_mm=float(depth["min_mm"]),
            max_depth_mm=float(depth["max_mm"]),
            click_window=int(depth["click_window"]),
            mirror_color=bool(display["mirror_color"]),
            mirror_depth=bool(display["mirror_depth"]),
            save_dir=app_path(str(capture["save_dir"])),
            sdk_library=app_path(str(sdk.get("library", DEFAULT_SDK_LIBRARY))),
            sdk_config=app_path(str(sdk.get("config", DEFAULT_SDK_CONFIG))),
        )
        settings.validate()
        return settings

    def validate(self) -> None:
        if self.frame_timeout_ms < 1:
            raise ValueError("stream.frame_timeout_ms 必须大于 0")
        if self.min_depth_mm < 0 or self.max_depth_mm <= self.min_depth_mm:
            raise ValueError("depth.max_mm 必须大于 depth.min_mm")
        if self.click_window < 1 or self.click_window % 2 == 0:
            raise ValueError("depth.click_window 必须是正奇数")


class ClickState:
    def __init__(self) -> None:
        self.u: Optional[int] = None
        self.v: Optional[int] = None


def select_camera_device(devices: list[CameraDeviceInfo], explicit_key: Optional[str]) -> CameraDeviceInfo:
    if not devices:
        raise RuntimeError("未发现 Orbbec 相机。请检查 USB 连接和 udev 权限")
    if explicit_key:
        match = next((item for item in devices if explicit_key in {item.serial_number, item.uid}), None)
        if match is None:
            available = "\n  ".join(item.label for item in devices)
            raise RuntimeError(f"找不到相机 {explicit_key}。当前设备:\n  {available}")
        return match
    if len(devices) > 1:
        raise RuntimeError("发现多个相机，请用 --serial 指定:\n  " + "\n  ".join(item.label for item in devices))
    return devices[0]


def choose_camera_dialog(
    root: Any,
    discover: Callable[[], list[CameraDeviceInfo]],
    initial_key: Optional[str] = None,
) -> Optional[CameraDeviceInfo]:
    """Show the camera equivalent of the arm terminal's serial-port dialog."""

    if tk is None or ttk is None:
        raise RuntimeError("缺少 Tkinter，请安装 python3-tk，或用 --serial 跳过可视化选择")

    result: dict[str, Optional[CameraDeviceInfo]] = {"device": None}
    devices_by_label: dict[str, CameraDeviceInfo] = {}
    dialog = tk.Toplevel(root)
    dialog.title("Gemini 相机设置")
    dialog.resizable(False, False)
    if root.winfo_viewable():
        dialog.transient(root)

    body = ttk.Frame(dialog, padding=18)
    body.grid(row=0, column=0, sticky="nsew")
    body.columnconfigure(0, weight=1)
    ttk.Label(body, text="选择 Gemini 深度相机", font=("Noto Sans CJK SC", 12, "bold")).grid(
        row=0, column=0, columnspan=2, sticky="w", pady=(0, 10)
    )
    ttk.Label(body, text="USB 设备 / 序列号").grid(row=1, column=0, columnspan=2, sticky="w")

    selected_label = tk.StringVar(value="")
    device_box = ttk.Combobox(body, textvariable=selected_label, width=66, state="readonly")
    device_box.grid(row=2, column=0, sticky="ew", pady=(6, 8), padx=(0, 8))
    status_value = tk.StringVar(value="")
    ttk.Label(body, textvariable=status_value, foreground="#526172").grid(
        row=3, column=0, columnspan=2, sticky="w", pady=(0, 12)
    )

    def refresh_devices() -> None:
        nonlocal devices_by_label
        try:
            devices = discover()
        except Exception as exc:
            devices = []
            status_value.set(f"枚举失败: {exc}")
        else:
            status_value.set(f"发现 {len(devices)} 台 Orbbec 相机" if devices else "未发现相机，请连接后刷新")
        devices_by_label = {item.label: item for item in devices}
        labels = list(devices_by_label)
        device_box.configure(values=labels)
        preferred = next(
            (item.label for item in devices if initial_key and initial_key in {item.serial_number, item.uid}),
            labels[0] if labels else "",
        )
        selected_label.set(preferred)

    def accept() -> None:
        selected = devices_by_label.get(selected_label.get())
        if selected is None:
            if messagebox is not None:
                messagebox.showwarning("Gemini 相机设置", "请选择相机", parent=dialog)
            return
        result["device"] = selected
        dialog.destroy()

    def cancel() -> None:
        result["device"] = None
        dialog.destroy()

    ttk.Button(body, text="刷新", command=refresh_devices).grid(row=2, column=1, sticky="ew", pady=(6, 8))
    buttons = ttk.Frame(body)
    buttons.grid(row=4, column=0, columnspan=2, sticky="e")
    ttk.Button(buttons, text="取消", command=cancel).pack(side="left", padx=(0, 8))
    ttk.Button(buttons, text="连接", command=accept).pack(side="left")
    dialog.protocol("WM_DELETE_WINDOW", cancel)
    dialog.bind("<Return>", lambda _event: accept())
    dialog.bind("<Escape>", lambda _event: cancel())

    refresh_devices()
    device_box.focus_set()
    dialog.update_idletasks()
    x = root.winfo_screenwidth() // 2 - dialog.winfo_reqwidth() // 2
    y = root.winfo_screenheight() // 2 - dialog.winfo_reqheight() // 2
    dialog.geometry(f"+{max(0, x)}+{max(0, y)}")
    dialog.lift()
    dialog.attributes("-topmost", True)
    dialog.after(500, lambda: dialog.attributes("-topmost", False))
    dialog.wait_visibility()
    dialog.grab_set()
    root.wait_window(dialog)
    return result["device"]


def color_frame_to_bgr(frame: NativeFrame) -> Optional[np.ndarray]:
    import cv2

    data = np.frombuffer(frame.data, dtype=np.uint8)
    pixels = frame.width * frame.height
    if frame.frame_format == OB_FORMAT_MJPG:
        return cv2.imdecode(data, cv2.IMREAD_COLOR)
    if frame.frame_format == OB_FORMAT_RGB and data.size >= pixels * 3:
        rgb = data[: pixels * 3].reshape((frame.height, frame.width, 3))
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    if frame.frame_format == OB_FORMAT_BGR and data.size >= pixels * 3:
        return data[: pixels * 3].reshape((frame.height, frame.width, 3)).copy()
    if frame.frame_format == OB_FORMAT_BGRA and data.size >= pixels * 4:
        bgra = data[: pixels * 4].reshape((frame.height, frame.width, 4))
        return cv2.cvtColor(bgra, cv2.COLOR_BGRA2BGR)
    if frame.frame_format in {OB_FORMAT_YUYV, OB_FORMAT_YUY2} and data.size >= pixels * 2:
        yuyv = data[: pixels * 2].reshape((frame.height, frame.width, 2))
        return cv2.cvtColor(yuyv, cv2.COLOR_YUV2BGR_YUY2)
    if frame.frame_format == OB_FORMAT_UYVY and data.size >= pixels * 2:
        uyvy = data[: pixels * 2].reshape((frame.height, frame.width, 2))
        return cv2.cvtColor(uyvy, cv2.COLOR_YUV2BGR_UYVY)
    if frame.frame_format == OB_FORMAT_I420 and data.size >= pixels * 3 // 2:
        i420 = data[: pixels * 3 // 2].reshape((frame.height * 3 // 2, frame.width))
        return cv2.cvtColor(i420, cv2.COLOR_YUV2BGR_I420)
    if frame.frame_format == OB_FORMAT_NV12 and data.size >= pixels * 3 // 2:
        nv12 = data[: pixels * 3 // 2].reshape((frame.height * 3 // 2, frame.width))
        return cv2.cvtColor(nv12, cv2.COLOR_YUV2BGR_NV12)
    if frame.frame_format == OB_FORMAT_NV21 and data.size >= pixels * 3 // 2:
        nv21 = data[: pixels * 3 // 2].reshape((frame.height * 3 // 2, frame.width))
        return cv2.cvtColor(nv21, cv2.COLOR_YUV2BGR_NV21)
    return None


def render_depth(depth_mm: np.ndarray, min_mm: float, max_mm: float) -> np.ndarray:
    import cv2

    valid = (depth_mm >= min_mm) & (depth_mm <= max_mm)
    clipped = np.clip(depth_mm, min_mm, max_mm)
    normalized = ((clipped - min_mm) * 255.0 / max(max_mm - min_mm, 1.0)).astype(np.uint8)
    colored = cv2.applyColorMap(normalized, cv2.COLORMAP_JET)
    colored[~valid] = (0, 0, 0)
    return colored


def median_depth_at(depth_mm: np.ndarray, u: int, v: int, window: int) -> Optional[float]:
    if window < 1 or window % 2 == 0:
        raise ValueError("window 必须是正奇数")
    height, width = depth_mm.shape[:2]
    if not (0 <= u < width and 0 <= v < height):
        return None
    radius = window // 2
    roi = depth_mm[max(0, v - radius) : min(height, v + radius + 1), max(0, u - radius) : min(width, u + radius + 1)]
    valid = roi[(roi >= 20.0) & (roi <= 10000.0)]
    return float(np.median(valid)) if valid.size else None


def map_display_to_depth(u: int, v: int, display_size: tuple[int, int], depth_size: tuple[int, int]) -> tuple[int, int]:
    display_width, display_height = display_size
    depth_width, depth_height = depth_size
    depth_u = min(depth_width - 1, max(0, int(round(u * depth_width / max(display_width, 1)))))
    depth_v = min(depth_height - 1, max(0, int(round(v * depth_height / max(display_height, 1)))))
    return depth_u, depth_v


def pixel_to_camera_point_mm(u: int, v: int, depth_mm: float, intrinsics: Intrinsics) -> tuple[float, float, float]:
    x = (u - intrinsics.cx) * depth_mm / intrinsics.fx
    y = (v - intrinsics.cy) * depth_mm / intrinsics.fy
    return x, y, depth_mm


def draw_click_info(
    color: np.ndarray,
    depth_mm: np.ndarray,
    click: ClickState,
    intrinsics: Optional[Intrinsics],
    window: int,
) -> None:
    import cv2

    if click.u is None or click.v is None:
        return
    if not (0 <= click.u < color.shape[1] and 0 <= click.v < color.shape[0]):
        return
    depth_u, depth_v = map_display_to_depth(
        click.u,
        click.v,
        (color.shape[1], color.shape[0]),
        (depth_mm.shape[1], depth_mm.shape[0]),
    )
    value = median_depth_at(depth_mm, depth_u, depth_v, window)
    cv2.circle(color, (click.u, click.v), 6, (0, 255, 255), 2)
    if value is None:
        text = f"({click.u},{click.v}) depth: invalid"
    else:
        text = f"({click.u},{click.v}) depth: {value:.1f} mm"
        if intrinsics is not None and intrinsics.fx > 0 and intrinsics.fy > 0:
            x, y, z = pixel_to_camera_point_mm(depth_u, depth_v, value, intrinsics)
            text += f" | camera: x={x:.1f}, y={y:.1f}, z={z:.1f} mm"
    cv2.putText(color, text, (20, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)


def save_snapshot(color: np.ndarray, depth_vis: np.ndarray, depth_mm: np.ndarray, output_dir: Path) -> None:
    import cv2

    output_dir.mkdir(parents=True, exist_ok=True)
    stamp = strftime("%Y%m%d_%H%M%S")
    cv2.imwrite(str(output_dir / f"{stamp}_color.png"), color)
    cv2.imwrite(str(output_dir / f"{stamp}_depth_preview.png"), depth_vis)
    np.save(str(output_dir / f"{stamp}_depth_mm.npy"), depth_mm)
    print(f"已保存样本到: {output_dir}")


def run_viewer(device: CameraDeviceInfo, settings: CameraSettings) -> None:
    import cv2

    click = ClickState()

    def on_mouse(event: int, x: int, y: int, _flags: int, _userdata: Any) -> None:
        if event == cv2.EVENT_LBUTTONDOWN:
            # The window contains color on the left and depth on the right.
            if last_color_width[0] and x < last_color_width[0]:
                click.u, click.v = x, y

    last_color_width = [0]
    with OrbbecSession(
        device.selection_key,
        library_path=settings.sdk_library,
        config_path=settings.sdk_config,
    ) as session:
        intrinsics = session.intrinsics()
        print(f"已连接: {device.label}")
        print("相机内参:")
        print(json.dumps({name: asdict(value) for name, value in intrinsics.items()}, ensure_ascii=False, indent=2))

        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(WINDOW_NAME, on_mouse)
        print("运行中：左键点击彩色图读取深度；按 s 保存；按 q 或 ESC 退出。")

        last_color: Optional[np.ndarray] = None
        last_depth_vis: Optional[np.ndarray] = None
        last_depth_mm: Optional[np.ndarray] = None
        unsupported_format_reported: set[int] = set()

        try:
            while True:
                frames = session.wait_for_frames(settings.frame_timeout_ms)
                if frames is None or frames.color is None or frames.depth is None:
                    continue
                color = color_frame_to_bgr(frames.color)
                if color is None:
                    if frames.color.frame_format not in unsupported_format_reported:
                        print(f"不支持的彩色格式: {frames.color.frame_format}", file=sys.stderr)
                        unsupported_format_reported.add(frames.color.frame_format)
                    continue
                depth_mm = depth_frame_to_mm(frames.depth)
                if settings.mirror_color:
                    color = cv2.flip(color, 1)
                if settings.mirror_depth:
                    depth_mm = cv2.flip(depth_mm, 1)
                depth_vis = render_depth(depth_mm, settings.min_depth_mm, settings.max_depth_mm)

                display_color = color.copy()
                draw_click_info(display_color, depth_mm, click, intrinsics.get("depth"), settings.click_window)
                if depth_vis.shape[:2] != display_color.shape[:2]:
                    depth_display = cv2.resize(
                        depth_vis,
                        (display_color.shape[1], display_color.shape[0]),
                        interpolation=cv2.INTER_NEAREST,
                    )
                else:
                    depth_display = depth_vis
                last_color_width[0] = display_color.shape[1]
                cv2.imshow(WINDOW_NAME, np.hstack((display_color, depth_display)))

                last_color, last_depth_vis, last_depth_mm = color, depth_vis, depth_mm
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), ord("Q"), ESC_KEY):
                    break
                if key in (ord("s"), ord("S")) and last_color is not None and last_depth_vis is not None and last_depth_mm is not None:
                    save_snapshot(last_color, last_depth_vis, last_depth_mm, settings.save_dir)
        finally:
            cv2.destroyAllWindows()


def list_devices(settings: CameraSettings) -> list[CameraDeviceInfo]:
    devices = enumerate_devices(settings.sdk_library, settings.sdk_config)
    if not devices:
        print("未发现 Orbbec/Gemini 相机")
        return []
    for item in devices:
        print(f"[{item.index}] {item.label} | UID: {item.uid}")
    return devices


def diagnose(settings: CameraSettings) -> int:
    print("Gemini Ubuntu 环境诊断")
    print(f"Python: {sys.version.splitlines()[0]}")
    print(f"Platform: {platform.platform()}")
    print(f"Architecture: {platform.machine()}")
    print(f"SDK library: {settings.sdk_library} ({'OK' if settings.sdk_library.is_file() else 'MISSING'})")
    print(f"SDK config: {settings.sdk_config} ({'OK' if settings.sdk_config.is_file() else 'MISSING'})")
    rules = Path("/etc/udev/rules.d/99-obsensor-libusb.rules")
    print(f"udev rules: {rules} ({'OK' if rules.is_file() else 'MISSING'})")
    try:
        import grp

        group_names = {grp.getgrgid(group_id).gr_name for group_id in os.getgroups()}
        print(f"video group: {'yes' if 'video' in group_names else 'no（udev 规则使用 GROUP=video）'}")
    except (ImportError, KeyError):
        print("video group: 无法检测，请执行 groups 确认")
    try:
        import cv2

        print(f"OpenCV: {cv2.__version__}")
    except ImportError as exc:
        print(f"OpenCV: MISSING ({exc})")
        return 1
    print(f"NumPy: {np.__version__}")
    if shutil_which("lsusb"):
        result = subprocess.run(["lsusb"], capture_output=True, text=True, timeout=5, check=False)
        matches = [line for line in result.stdout.splitlines() if "2bc5:" in line.lower()]
        print("lsusb Orbbec:")
        print("\n".join(f"  {line}" for line in matches) if matches else "  未发现 VID 2bc5")
    try:
        devices = list_devices(settings)
    except Exception as exc:
        print(f"SDK 枚举失败: {exc}", file=sys.stderr)
        return 1
    return 0 if devices else 1


def shutil_which(command: str) -> Optional[str]:
    # Kept local to avoid importing another module during normal viewer startup.
    from shutil import which

    return which(command)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Gemini Pro Plus RGB-D viewer for Ubuntu 22.04")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG_PATH), help="camera JSON config")
    parser.add_argument("--serial", default=None, help="camera serial number or UID")
    parser.add_argument("--first-device", action="store_true", help="use the only/first device without opening the chooser")
    parser.add_argument("--list-devices", action="store_true", help="list Orbbec devices and exit")
    parser.add_argument("--read-intrinsics", action="store_true", help="print camera intrinsics and exit")
    parser.add_argument("--diagnose", action="store_true", help="check Ubuntu dependencies, USB access and SDK enumeration")
    return parser


def _select_for_cli(args: argparse.Namespace, settings: CameraSettings) -> Optional[CameraDeviceInfo]:
    requested = args.serial or settings.serial_number or None
    if requested or args.first_device:
        devices = enumerate_devices(settings.sdk_library, settings.sdk_config)
        if args.first_device and not requested:
            return devices[0] if devices else select_camera_device(devices, None)
        return select_camera_device(devices, requested)

    if tk is None:
        raise RuntimeError("缺少 Tkinter；请安装 python3-tk，或使用 --serial/--first-device")
    root = tk.Tk()
    root.withdraw()
    try:
        return choose_camera_dialog(
            root,
            lambda: enumerate_devices(settings.sdk_library, settings.sdk_config),
            settings.serial_number or None,
        )
    finally:
        root.destroy()


def main(argv: Optional[list[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        settings = CameraSettings.from_file(args.config)
        if args.diagnose:
            return diagnose(settings)
        if args.list_devices:
            return 0 if list_devices(settings) else 1

        device = _select_for_cli(args, settings)
        if device is None:
            return 0
        if args.read_intrinsics:
            with OrbbecSession(
                device.selection_key,
                library_path=settings.sdk_library,
                config_path=settings.sdk_config,
            ) as session:
                print(json.dumps({name: asdict(value) for name, value in session.intrinsics().items()}, ensure_ascii=False, indent=2))
            return 0
        run_viewer(device, settings)
        return 0
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
