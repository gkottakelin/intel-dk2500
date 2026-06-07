"""Depth-only viewer for Gemini Pro Plus on Windows.

Default device settings:
    - laser enabled
    - LDP / close-range protection disabled

Run:
    python project/src/gemini_windows/depth_viewer.py
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
from time import strftime
from typing import Any

import cv2
import numpy as np

from gemini_common import depth_frame_to_mm, import_orbbec_sdk, median_depth_at


ESC_KEY = 27
WINDOW_NAME = "Gemini Depth Viewer"
DEFAULT_MIN_MM = 200.0
DEFAULT_MAX_MM = 4000.0


@dataclass
class DeviceSettingResult:
    name: str
    requested: bool
    applied: bool
    message: str


@dataclass
class ClickState:
    u: int | None = None
    v: int | None = None
    depth_mm: float | None = None


def _on_mouse(event: int, x: int, y: int, _flags: int, click: ClickState) -> None:
    if event == cv2.EVENT_LBUTTONDOWN:
        click.u = x
        click.v = y
        click.depth_mm = None


def set_bool_property_if_supported(device: Any, prop: Any, value: bool, label: str) -> DeviceSettingResult:
    try:
        supported = bool(device.is_property_supported(prop))
    except Exception as exc:
        return DeviceSettingResult(label, value, False, f"check support failed: {exc}")

    if not supported:
        return DeviceSettingResult(label, value, False, "property not supported")

    try:
        device.set_bool_property(prop, value)
        actual = bool(device.get_bool_property(prop))
    except Exception as exc:
        return DeviceSettingResult(label, value, False, f"set failed: {exc}")

    if actual != value:
        return DeviceSettingResult(label, value, False, f"actual state is {actual}")
    return DeviceSettingResult(label, value, True, f"actual state is {actual}")


def configure_depth_device(device: Any, *, enable_laser: bool, disable_ldp: bool) -> list[DeviceSettingResult]:
    sdk = import_orbbec_sdk()
    results: list[DeviceSettingResult] = []

    results.append(
        set_bool_property_if_supported(
            device,
            sdk.OBPropertyID.OB_PROP_LASER_BOOL,
            enable_laser,
            "laser",
        )
    )
    results.append(
        set_bool_property_if_supported(
            device,
            sdk.OBPropertyID.OB_PROP_LDP_BOOL,
            not disable_ldp,
            "ldp_close_range_protection",
        )
    )
    return results


def print_setting_results(results: list[DeviceSettingResult]) -> None:
    for result in results:
        status = "OK" if result.applied else "WARN"
        print(f"[{status}] {result.name}: requested={result.requested} | {result.message}")


def render_depth_view(depth_mm: np.ndarray, min_mm: float, max_mm: float) -> np.ndarray:
    clipped = np.where((depth_mm >= min_mm) & (depth_mm <= max_mm), depth_mm, 0.0)
    valid = clipped > 0

    normalized = np.zeros_like(clipped, dtype=np.uint8)
    if np.any(valid):
        scaled = (clipped[valid] - min_mm) * 255.0 / max(max_mm - min_mm, 1.0)
        # Invert so nearer areas are warmer/brighter, closer to common depth viewer behavior.
        normalized[valid] = 255 - np.clip(scaled, 0, 255).astype(np.uint8)

    color = cv2.applyColorMap(normalized, cv2.COLORMAP_JET)
    color[~valid] = (0, 0, 0)
    return color


def draw_overlay(
    image: np.ndarray,
    depth_mm: np.ndarray,
    click: ClickState,
    min_mm: float,
    max_mm: float,
    laser_state: bool,
    ldp_state: bool,
    frame_count: int,
) -> None:
    h, w = depth_mm.shape[:2]
    valid = depth_mm[(depth_mm >= min_mm) & (depth_mm <= max_mm)]
    valid_ratio = valid.size / max(depth_mm.size, 1)
    center_depth = median_depth_at(depth_mm, w // 2, h // 2)

    lines = [
        f"frame: {frame_count}  range: {min_mm:.0f}-{max_mm:.0f} mm  valid: {valid_ratio:.1%}",
        f"laser: {'ON' if laser_state else 'OFF'}  LDP/near protection: {'ON' if ldp_state else 'OFF'}",
        f"center depth: {center_depth:.1f} mm" if center_depth is not None else "center depth: invalid",
        "L: laser toggle | P: LDP toggle | +/- range | S: save | Q/Esc: quit",
    ]

    if click.u is not None and click.v is not None and 0 <= click.u < w and 0 <= click.v < h:
        click.depth_mm = median_depth_at(depth_mm, click.u, click.v)
        cv2.drawMarker(image, (click.u, click.v), (255, 255, 255), cv2.MARKER_CROSS, 18, 2)
        if click.depth_mm is None:
            lines.append(f"click ({click.u},{click.v}): invalid")
        else:
            lines.append(f"click ({click.u},{click.v}): {click.depth_mm:.1f} mm")

    y = 26
    for line in lines:
        cv2.putText(image, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(image, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (245, 245, 245), 1, cv2.LINE_AA)
        y += 25

    # Viewer-style depth color bar.
    bar_h = min(260, image.shape[0] - 30)
    x0 = image.shape[1] - 38
    y0 = image.shape[0] - bar_h - 20
    for i in range(bar_h):
        value = np.uint8(255 - int(i * 255 / max(bar_h - 1, 1)))
        color = cv2.applyColorMap(np.array([[value]], dtype=np.uint8), cv2.COLORMAP_JET)[0, 0]
        cv2.line(image, (x0, y0 + i), (x0 + 18, y0 + i), tuple(int(v) for v in color.tolist()), 1)
    cv2.putText(image, f"{min_mm:.0f}", (x0 - 10, y0 + bar_h + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (245, 245, 245), 1)
    cv2.putText(image, f"{max_mm:.0f}", (x0 - 10, y0 - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (245, 245, 245), 1)


def save_depth_sample(output_dir: Path, depth_mm: np.ndarray, depth_vis: np.ndarray) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    stamp = strftime("%Y%m%d_%H%M%S")
    cv2.imwrite(str(output_dir / f"{stamp}_depth_view.png"), depth_vis)
    np.save(str(output_dir / f"{stamp}_depth_mm.npy"), depth_mm)
    print(f"saved depth sample: {output_dir}")


def choose_depth_config(pipeline: Any, width: int, height: int, fps: int) -> Any:
    sdk = import_orbbec_sdk()
    config = sdk.Config()
    profiles = pipeline.get_stream_profile_list(sdk.OBSensorType.DEPTH_SENSOR)

    profile = None
    if width > 0 and height > 0 and fps > 0:
        for fmt in (sdk.OBFormat.Y16, sdk.OBFormat.UNKNOWN):
            try:
                profile = profiles.get_video_stream_profile(width, height, fmt, fps)
                break
            except Exception:
                profile = None

    if profile is None:
        profile = profiles.get_default_video_stream_profile()
    config.enable_stream(profile)
    return config


def main() -> None:
    parser = argparse.ArgumentParser(description="Gemini Pro Plus depth visualization viewer")
    parser.add_argument("--min-mm", type=float, default=DEFAULT_MIN_MM, help="minimum rendered depth")
    parser.add_argument("--max-mm", type=float, default=DEFAULT_MAX_MM, help="maximum rendered depth")
    parser.add_argument("--width", type=int, default=0, help="requested depth width; 0 uses SDK default")
    parser.add_argument("--height", type=int, default=0, help="requested depth height; 0 uses SDK default")
    parser.add_argument("--fps", type=int, default=0, help="requested depth fps; 0 uses SDK default")
    parser.add_argument("--save-dir", default="project/data/rgbd_samples", help="sample output directory")
    parser.add_argument("--laser-off", action="store_true", help="do not enable laser at startup")
    parser.add_argument("--ldp-on", action="store_true", help="keep LDP / close-range protection enabled")
    args = parser.parse_args()

    sdk = import_orbbec_sdk()
    pipeline = sdk.Pipeline()
    config = choose_depth_config(pipeline, args.width, args.height, args.fps)
    device = pipeline.get_device()

    laser_state = not args.laser_off
    ldp_state = bool(args.ldp_on)
    print_setting_results(
        configure_depth_device(
            device,
            enable_laser=laser_state,
            disable_ldp=not ldp_state,
        )
    )

    try:
        pipeline.start(config)
    except Exception as exc:
        print(f"start depth stream failed: {exc}")
        print("请确认 Gemini Pro Plus 已连接，且 Orbbec Viewer / OpenNI 示例没有占用相机。")
        return

    click = ClickState()
    output_dir = Path(args.save_dir)
    min_mm = args.min_mm
    max_mm = args.max_mm
    frame_count = 0

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(WINDOW_NAME, _on_mouse, click)
    print("Depth viewer running. Default: laser ON, LDP/near protection OFF.")

    try:
        while True:
            frames = pipeline.wait_for_frames(1000)
            if frames is None:
                continue
            depth_frame = frames.get_depth_frame()
            if depth_frame is None:
                continue

            frame_count += 1
            depth_mm = depth_frame_to_mm(depth_frame)
            depth_vis = render_depth_view(depth_mm, min_mm, max_mm)
            draw_overlay(depth_vis, depth_mm, click, min_mm, max_mm, laser_state, ldp_state, frame_count)
            cv2.imshow(WINDOW_NAME, depth_vis)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q"), ESC_KEY):
                break
            if key in (ord("s"), ord("S")):
                save_depth_sample(output_dir, depth_mm, depth_vis)
            elif key in (ord("l"), ord("L")):
                laser_state = not laser_state
                print_setting_results(
                    [
                        set_bool_property_if_supported(
                            device,
                            sdk.OBPropertyID.OB_PROP_LASER_BOOL,
                            laser_state,
                            "laser",
                        )
                    ]
                )
            elif key in (ord("p"), ord("P")):
                ldp_state = not ldp_state
                print_setting_results(
                    [
                        set_bool_property_if_supported(
                            device,
                            sdk.OBPropertyID.OB_PROP_LDP_BOOL,
                            ldp_state,
                            "ldp_close_range_protection",
                        )
                    ]
                )
            elif key in (ord("+"), ord("=")):
                max_mm = min(max_mm + 500.0, 10000.0)
            elif key in (ord("-"), ord("_")):
                max_mm = max(max_mm - 500.0, min_mm + 100.0)
    finally:
        cv2.destroyAllWindows()
        try:
            pipeline.stop()
        except Exception:
            pass


if __name__ == "__main__":
    main()

