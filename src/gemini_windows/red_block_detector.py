"""Detect a red block in UVC color and return matched OpenNI depth.

This script intentionally reuses the working Gemini data path:
- depth: OpenNI SENSOR_DEPTH, 640x400@30 by default
- color: Windows UVC MJPG, 640x480@30 by default
- laser: enabled by default
- LDP / close-range protection: disabled by default
"""

from __future__ import annotations

import argparse
import ctypes
import json
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from time import strftime

import numpy as np

try:
    from .openni2_ctypes import (
        OBEXTENSION_ID_LASER_EN,
        ONI_PIXEL_FORMAT_DEPTH_100_UM,
        ONI_PIXEL_FORMAT_DEPTH_1_MM,
        ONI_SENSOR_DEPTH,
        OpenNI2,
    )
    from .openni_rgbd_viewer import depth_to_mm, open_color_camera, render_depth
except ImportError:
    from openni2_ctypes import (  # type: ignore
        OBEXTENSION_ID_LASER_EN,
        ONI_PIXEL_FORMAT_DEPTH_100_UM,
        ONI_PIXEL_FORMAT_DEPTH_1_MM,
        ONI_SENSOR_DEPTH,
        OpenNI2,
    )
    from openni_rgbd_viewer import depth_to_mm, open_color_camera, render_depth  # type: ignore


OBEXTENSION_ID_CAM_PARAMS = 14
CAMERA_PARAM_BASE_WIDTH = 640.0
CAMERA_PARAM_BASE_HEIGHT = 480.0


class OBCameraParams(ctypes.Structure):
    _fields_ = [
        ("l_intr_p", ctypes.c_float * 4),
        ("r_intr_p", ctypes.c_float * 4),
        ("r2l_r", ctypes.c_float * 9),
        ("r2l_t", ctypes.c_float * 3),
        ("l_k", ctypes.c_float * 5),
        ("r_k", ctypes.c_float * 5),
    ]


@dataclass(frozen=True)
class DepthIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float


@dataclass(frozen=True)
class RedBlockDetection:
    color_center: tuple[int, int]
    depth_pixel: tuple[int, int]
    depth_mm: float
    camera_xyz_mm: tuple[float, float, float] | None
    color_bbox: tuple[int, int, int, int]
    area_px: float
    matched_depth_pixels: int
    match_mode: str


def read_depth_intrinsics(device: OpenNI2, depth_width: int, depth_height: int) -> DepthIntrinsics | None:
    params = OBCameraParams()
    size = ctypes.c_int(ctypes.sizeof(params))
    rc = device.lib.oniDeviceGetProperty(
        device.device,
        OBEXTENSION_ID_CAM_PARAMS,
        ctypes.byref(params),
        ctypes.byref(size),
    )
    if rc != 0:
        return None

    fx, fy, cx, cy = [float(params.l_intr_p[i]) for i in range(4)]
    if fx <= 0 or fy <= 0:
        return None

    # Same scaling rule used by the tutorial GeneratePointCloud.cpp sample.
    sx = float(depth_width) / CAMERA_PARAM_BASE_WIDTH
    sy = float(depth_height) / CAMERA_PARAM_BASE_HEIGHT
    return DepthIntrinsics(fx=fx * sx, fy=fy * sy, cx=cx * sx, cy=cy * sy)


def pixel_to_camera_xyz(depth_u: int, depth_v: int, depth_mm_value: float, intr: DepthIntrinsics | None):
    if intr is None or depth_mm_value <= 0:
        return None
    x = (float(depth_u) - intr.cx) * depth_mm_value / intr.fx
    y = (float(depth_v) - intr.cy) * depth_mm_value / intr.fy
    return (float(x), float(y), float(depth_mm_value))


def detect_red_mask(color_bgr: np.ndarray, *, min_area: float):
    import cv2

    hsv = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, np.array([0, 80, 50]), np.array([10, 255, 255]))
    mask2 = cv2.inRange(hsv, np.array([170, 80, 50]), np.array([180, 255, 255]))
    mask = cv2.bitwise_or(mask1, mask2)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = [contour for contour in contours if cv2.contourArea(contour) >= min_area]
    if not contours:
        return None, mask

    contour = max(contours, key=cv2.contourArea)
    selected = np.zeros(mask.shape, dtype=np.uint8)
    cv2.drawContours(selected, [contour], -1, 255, thickness=-1)

    moments = cv2.moments(contour)
    if moments["m00"] == 0:
        return None, mask
    center = (int(moments["m10"] / moments["m00"]), int(moments["m01"] / moments["m00"]))
    bbox = tuple(int(v) for v in cv2.boundingRect(contour))
    return {
        "mask": selected,
        "center": center,
        "bbox": bbox,
        "area": float(cv2.contourArea(contour)),
    }, mask


def _valid_depth_values(depth_mm_image: np.ndarray, mask: np.ndarray, min_depth_mm: float, max_depth_mm: float):
    valid = (mask > 0) & (depth_mm_image >= min_depth_mm) & (depth_mm_image <= max_depth_mm)
    ys, xs = np.where(valid)
    if xs.size == 0:
        return None
    values = depth_mm_image[ys, xs]
    return xs, ys, values


def match_depth_by_scaled_mask(
    red_mask: np.ndarray,
    color_center: tuple[int, int],
    depth_mm_image: np.ndarray,
    *,
    min_depth_mm: float,
    max_depth_mm: float,
    x_offset_px: int = 0,
    y_offset_px: int = 0,
):
    import cv2

    depth_h, depth_w = depth_mm_image.shape[:2]
    mask = cv2.resize(red_mask, (depth_w, depth_h), interpolation=cv2.INTER_NEAREST)
    if x_offset_px or y_offset_px:
        shifted = np.zeros_like(mask)
        src_x0 = max(0, -x_offset_px)
        src_y0 = max(0, -y_offset_px)
        src_x1 = min(depth_w, depth_w - x_offset_px)
        src_y1 = min(depth_h, depth_h - y_offset_px)
        dst_x0 = max(0, x_offset_px)
        dst_y0 = max(0, y_offset_px)
        dst_x1 = dst_x0 + max(0, src_x1 - src_x0)
        dst_y1 = dst_y0 + max(0, src_y1 - src_y0)
        if src_x1 > src_x0 and src_y1 > src_y0:
            shifted[dst_y0:dst_y1, dst_x0:dst_x1] = mask[src_y0:src_y1, src_x0:src_x1]
        mask = shifted

    valid = _valid_depth_values(depth_mm_image, mask, min_depth_mm, max_depth_mm)
    if valid is None:
        u = int(round(color_center[0] * depth_w / red_mask.shape[1])) + x_offset_px
        v = int(round(color_center[1] * depth_h / red_mask.shape[0])) + y_offset_px
        u = max(0, min(depth_w - 1, u))
        v = max(0, min(depth_h - 1, v))
        radius = 8
        local_mask = np.zeros((depth_h, depth_w), dtype=np.uint8)
        local_mask[max(0, v - radius) : min(depth_h, v + radius + 1), max(0, u - radius) : min(depth_w, u + radius + 1)] = 255
        valid = _valid_depth_values(depth_mm_image, local_mask, min_depth_mm, max_depth_mm)
        if valid is None:
            return None

    xs, ys, values = valid
    median_depth = float(np.median(values))
    close = np.abs(values - median_depth) <= max(30.0, median_depth * 0.04)
    if np.any(close):
        xs_used = xs[close]
        ys_used = ys[close]
        values_used = values[close]
    else:
        xs_used = xs
        ys_used = ys
        values_used = values

    return {
        "depth_u": int(round(float(np.median(xs_used)))),
        "depth_v": int(round(float(np.median(ys_used)))),
        "depth_mm": float(np.median(values_used)),
        "count": int(values_used.size),
        "mask": mask,
    }


def detect_red_block(
    color_bgr: np.ndarray,
    depth_mm_image: np.ndarray,
    *,
    intrinsics: DepthIntrinsics | None = None,
    min_area: float = 600.0,
    min_depth_mm: float = 100.0,
    max_depth_mm: float = 4000.0,
    x_offset_px: int = 0,
    y_offset_px: int = 0,
) -> tuple[RedBlockDetection | None, np.ndarray]:
    red, full_mask = detect_red_mask(color_bgr, min_area=min_area)
    if red is None:
        return None, full_mask

    match = match_depth_by_scaled_mask(
        red["mask"],
        red["center"],
        depth_mm_image,
        min_depth_mm=min_depth_mm,
        max_depth_mm=max_depth_mm,
        x_offset_px=x_offset_px,
        y_offset_px=y_offset_px,
    )
    if match is None:
        return None, full_mask

    depth_u = int(match["depth_u"])
    depth_v = int(match["depth_v"])
    depth_value = float(match["depth_mm"])
    xyz = pixel_to_camera_xyz(depth_u, depth_v, depth_value, intrinsics)

    detection = RedBlockDetection(
        color_center=tuple(red["center"]),
        depth_pixel=(depth_u, depth_v),
        depth_mm=depth_value,
        camera_xyz_mm=xyz,
        color_bbox=tuple(red["bbox"]),
        area_px=float(red["area"]),
        matched_depth_pixels=int(match["count"]),
        match_mode="scaled-red-mask",
    )
    return detection, full_mask


def draw_detection(color_bgr: np.ndarray, depth_vis: np.ndarray, detection: RedBlockDetection | None):
    import cv2

    if detection is None:
        cv2.putText(color_bgr, "red block: none", (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        return

    x, y, w, h = detection.color_bbox
    cu, cv = detection.color_center
    du, dv = detection.depth_pixel
    cv2.rectangle(color_bgr, (x, y), (x + w, y + h), (0, 255, 255), 2)
    cv2.circle(color_bgr, (cu, cv), 5, (0, 255, 255), -1)
    cv2.circle(depth_vis, (du, dv), 5, (255, 255, 255), -1)
    label = f"red ({cu},{cv}) depth={detection.depth_mm:.0f}mm"
    cv2.putText(color_bgr, label, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
    cv2.putText(depth_vis, f"({du},{dv}) {detection.depth_mm:.0f}mm", (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)


def choose_color_camera_index(args):
    if args.color_index is not None:
        cap = open_color_camera(args.color_index, args.color_width, args.color_height, args.fps, True)
        return args.color_index, cap

    for index in range(args.max_color_index + 1):
        cap = open_color_camera(index, args.color_width, args.color_height, args.fps, True)
        if cap is None:
            continue
        ok, frame = cap.read()
        if ok and frame is not None:
            print(f"[OK] color camera index {index}: {frame.shape[1]}x{frame.shape[0]}")
            return index, cap
        cap.release()
    return None, None


def save_debug_frame(save_dir: Path, color, depth_vis, mask, detection: RedBlockDetection | None) -> None:
    import cv2

    save_dir.mkdir(parents=True, exist_ok=True)
    stamp = strftime("%Y%m%d_%H%M%S")
    cv2.imwrite(str(save_dir / f"{stamp}_red_color.png"), color)
    cv2.imwrite(str(save_dir / f"{stamp}_red_depth.png"), depth_vis)
    cv2.imwrite(str(save_dir / f"{stamp}_red_mask.png"), mask)
    if detection is not None:
        (save_dir / f"{stamp}_red_detection.json").write_text(
            json.dumps(asdict(detection), ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
    print(f"Saved debug frame to {save_dir}")


def run_detector(args) -> None:
    import cv2

    with OpenNI2() as device:
        info = device.device_info()
        print(
            "OpenNI device: "
            f"{info['vendor']} {info['name']} vid=0x{int(info['vid']):04x} pid=0x{int(info['pid']):04x}"
        )
        device.configure_laser_and_ldp(laser_on=True, close_range_protection_off=True)

        depth_stream = device.create_stream(
            ONI_SENSOR_DEPTH,
            width=args.depth_width,
            height=args.depth_height,
            fps=args.fps,
            pixel_formats=[ONI_PIXEL_FORMAT_DEPTH_1_MM, ONI_PIXEL_FORMAT_DEPTH_100_UM],
            mirror=args.mirror_depth,
        )
        depth_stream.start()

        intrinsics = read_depth_intrinsics(device, args.depth_width, args.depth_height)
        if intrinsics is None:
            print("[--] depth intrinsics unavailable; returning pixel + depth only")
        else:
            print(f"[OK] depth intrinsics: fx={intrinsics.fx:.2f}, fy={intrinsics.fy:.2f}, cx={intrinsics.cx:.2f}, cy={intrinsics.cy:.2f}")

        color_index, color_cap = choose_color_camera_index(args)
        if color_cap is None:
            raise RuntimeError("No usable UVC color camera found. Pass --color-index if needed.")
        print(f"Color UVC stream: index={color_index}, requested {args.color_width}x{args.color_height}@{args.fps}, MJPG")

        cv2.namedWindow("Red Block Color", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Red Block Depth", cv2.WINDOW_NORMAL)
        if args.show_mask:
            cv2.namedWindow("Red Block Mask", cv2.WINDOW_NORMAL)

        last_depth_mm = None
        last_detection = None
        last_print = 0.0
        save_dir = Path(args.save_dir)

        print("Running. JSON detections print to console. Press s to save, q or ESC to quit.")
        try:
            while True:
                changed = device.wait_for_any_stream([depth_stream], timeout_ms=30)
                if changed is not None:
                    depth_frame = depth_stream.read_frame()
                    last_depth_mm = depth_to_mm(depth_frame.data, int(depth_frame.video_mode.pixelFormat))

                ok, color = color_cap.read()
                if not ok or color is None or last_depth_mm is None:
                    key = cv2.waitKey(1) & 0xFF
                    if key in (27, ord("q"), ord("Q")):
                        break
                    continue

                if args.mirror_color:
                    color = cv2.flip(color, 1)

                detection, mask = detect_red_block(
                    color,
                    last_depth_mm,
                    intrinsics=intrinsics,
                    min_area=args.min_area,
                    min_depth_mm=args.min_depth_mm,
                    max_depth_mm=args.max_depth_mm,
                    x_offset_px=args.depth_x_offset,
                    y_offset_px=args.depth_y_offset,
                )
                last_detection = detection

                depth_vis = render_depth(last_depth_mm, args.min_depth_mm, args.max_depth_mm)
                color_vis = color.copy()
                draw_detection(color_vis, depth_vis, detection)

                now = time.perf_counter()
                if detection is not None and now - last_print >= args.print_interval:
                    print(json.dumps(asdict(detection), ensure_ascii=False))
                    last_print = now

                cv2.imshow("Red Block Color", color_vis)
                cv2.imshow("Red Block Depth", depth_vis)
                if args.show_mask:
                    cv2.imshow("Red Block Mask", mask)

                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q"), ord("Q")):
                    break
                if key in (ord("s"), ord("S")):
                    save_debug_frame(save_dir, color_vis, depth_vis, mask, last_detection)
        finally:
            color_cap.release()
            cv2.destroyAllWindows()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Detect a red block and return color/depth/camera coordinates")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--depth-width", type=int, default=640)
    parser.add_argument("--depth-height", type=int, default=400)
    parser.add_argument("--color-width", type=int, default=640)
    parser.add_argument("--color-height", type=int, default=480)
    parser.add_argument("--color-index", type=int, default=None)
    parser.add_argument("--max-color-index", type=int, default=6)
    parser.add_argument("--min-area", type=float, default=600.0)
    parser.add_argument("--min-depth-mm", type=float, default=100.0)
    parser.add_argument("--max-depth-mm", type=float, default=4000.0)
    parser.add_argument("--depth-x-offset", type=int, default=0, help="Optional x offset after scaling red mask into depth space.")
    parser.add_argument("--depth-y-offset", type=int, default=0, help="Optional y offset after scaling red mask into depth space.")
    parser.add_argument("--print-interval", type=float, default=0.2)
    parser.add_argument("--save-dir", default="project/data/rgbd_samples")
    parser.add_argument("--show-mask", action="store_true")
    parser.add_argument("--mirror-color", action="store_true")
    parser.add_argument("--mirror-depth", action="store_true")
    return parser.parse_args()


def main() -> None:
    run_detector(parse_args())


if __name__ == "__main__":
    main()
