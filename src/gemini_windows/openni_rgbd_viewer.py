"""Gemini Pro Plus OpenNI/UVC viewer based on the Windows tutorial samples.

Depth and IR are read through OpenNI2, following DepthReaderPoll and
InfraredReaderPoll. Color is read as a 640x480 MJPEG UVC stream, following the
ColorReaderUVC stream profile from the tutorial package.
"""

from __future__ import annotations

import argparse
import time
from dataclasses import dataclass
from pathlib import Path
from time import strftime

import numpy as np

try:
    from .openni2_ctypes import (
        BIN_DIR,
        ONI_PIXEL_FORMAT_DEPTH_100_UM,
        ONI_PIXEL_FORMAT_DEPTH_1_MM,
        ONI_PIXEL_FORMAT_GRAY16,
        ONI_SENSOR_DEPTH,
        ONI_SENSOR_IR,
        OpenNI2,
        OpenNIError,
        OpenNIStream,
    )
except ImportError:
    from openni2_ctypes import (  # type: ignore
        BIN_DIR,
        ONI_PIXEL_FORMAT_DEPTH_100_UM,
        ONI_PIXEL_FORMAT_DEPTH_1_MM,
        ONI_PIXEL_FORMAT_GRAY16,
        ONI_SENSOR_DEPTH,
        ONI_SENSOR_IR,
        OpenNI2,
        OpenNIError,
        OpenNIStream,
    )


DEPTH_WINDOW = "Gemini Depth - OpenNI"
IR_WINDOW = "Gemini Infrared - OpenNI"
COLOR_WINDOW = "Gemini Color - UVC"
COMBINED_WINDOW = "Gemini OpenNI/UVC Viewer"


@dataclass
class FpsMeter:
    last_time: float = 0.0
    value: float = 0.0

    def tick(self) -> None:
        now = time.perf_counter()
        if self.last_time:
            dt = now - self.last_time
            if dt > 0:
                instant = 1.0 / dt
                self.value = instant if self.value == 0 else self.value * 0.85 + instant * 0.15
        self.last_time = now


def depth_to_mm(raw: np.ndarray, pixel_format: int) -> np.ndarray:
    if pixel_format == ONI_PIXEL_FORMAT_DEPTH_100_UM:
        return raw.astype(np.float32) * 0.1
    return raw.astype(np.float32)


def render_depth(depth_mm: np.ndarray, min_mm: float, max_mm: float):
    import cv2

    valid = (depth_mm > 0) & (depth_mm >= min_mm) & (depth_mm <= max_mm)
    clipped = np.clip(depth_mm, min_mm, max_mm)
    normalized = ((clipped - min_mm) * 255.0 / max(max_mm - min_mm, 1.0)).astype(np.uint8)
    colored = cv2.applyColorMap(normalized, cv2.COLORMAP_JET)
    colored[~valid] = (0, 0, 0)
    return colored


def render_ir(ir_raw: np.ndarray):
    import cv2

    valid = ir_raw[ir_raw > 0]
    if valid.size:
        low = float(np.percentile(valid, 1.0))
        high = float(np.percentile(valid, 99.7))
        if high <= low:
            high = low + 1.0
        gray = np.clip((ir_raw.astype(np.float32) - low) * 255.0 / (high - low), 0, 255).astype(np.uint8)
    else:
        gray = np.zeros(ir_raw.shape, dtype=np.uint8)
    return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)


def draw_label(image, title: str, fps: FpsMeter | None = None, extra: str = ""):
    import cv2

    label = title
    if fps is not None and fps.value:
        label += f"  {fps.value:4.1f} fps"
    if extra:
        label += f"  {extra}"
    cv2.rectangle(image, (0, 0), (min(image.shape[1], 430), 30), (0, 0, 0), -1)
    cv2.putText(image, label, (10, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (245, 245, 245), 1, cv2.LINE_AA)


def fit_tile(image, size: tuple[int, int]):
    import cv2

    target_w, target_h = size
    canvas = np.zeros((target_h, target_w, 3), dtype=np.uint8)
    if image is None:
        return canvas
    h, w = image.shape[:2]
    scale = min(target_w / max(w, 1), target_h / max(h, 1))
    new_w = max(1, int(w * scale))
    new_h = max(1, int(h * scale))
    resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_AREA)
    x0 = (target_w - new_w) // 2
    y0 = (target_h - new_h) // 2
    canvas[y0 : y0 + new_h, x0 : x0 + new_w] = resized
    return canvas


def make_combined(depth_vis, ir_vis, color_vis):
    tile_size = (640, 480)
    top = np.hstack((fit_tile(depth_vis, tile_size), fit_tile(ir_vis, tile_size)))
    bottom = np.hstack((fit_tile(color_vis, tile_size), fit_tile(None, tile_size)))
    return np.vstack((top, bottom))


def open_color_camera(index: int, width: int, height: int, fps: int, auto_exposure: bool):
    import cv2

    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    if not cap.isOpened():
        cap.release()
        return None
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    if auto_exposure:
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)
        cap.set(cv2.CAP_PROP_AUTO_WB, 1.0)
    for _ in range(5):
        cap.read()
    return cap


def choose_color_camera(args):
    if args.no_color:
        return None, None
    if args.color_index is not None:
        cap = open_color_camera(
            args.color_index,
            args.color_width,
            args.color_height,
            args.fps,
            not args.no_color_auto_exposure,
        )
        return args.color_index, cap

    print(f"Probing UVC color cameras 0..{args.max_color_index}")
    for index in range(args.max_color_index + 1):
        cap = open_color_camera(index, args.color_width, args.color_height, args.fps, not args.no_color_auto_exposure)
        if cap is None:
            print(f"[--] color camera index {index}: unavailable")
            continue
        ok, frame = cap.read()
        if ok and frame is not None:
            print(f"[OK] color camera index {index}: {frame.shape[1]}x{frame.shape[0]}")
            return index, cap
        print(f"[--] color camera index {index}: no frame")
        cap.release()
    return None, None


def print_modes(device: OpenNI2) -> None:
    names = {ONI_SENSOR_DEPTH: "depth", ONI_SENSOR_IR: "ir"}
    for sensor_type, name in names.items():
        print(f"{name} modes:")
        for mode in device.supported_modes(sensor_type):
            print(f"  {mode.resolutionX}x{mode.resolutionY} fps={mode.fps} format={mode.pixelFormat}")


def create_streams(device: OpenNI2, args) -> tuple[OpenNIStream, OpenNIStream | None]:
    depth = device.create_stream(
        ONI_SENSOR_DEPTH,
        width=args.depth_width,
        height=args.depth_height,
        fps=args.fps,
        pixel_formats=[ONI_PIXEL_FORMAT_DEPTH_1_MM, ONI_PIXEL_FORMAT_DEPTH_100_UM],
        mirror=args.mirror_depth,
    )
    depth.start()
    print(
        "Depth stream requested "
        f"{args.depth_width}x{args.depth_height}@{args.fps}; "
        "OpenNI selected the closest supported mode."
    )

    ir = None
    if not args.no_ir:
        try:
            ir = device.create_stream(
                ONI_SENSOR_IR,
                width=args.ir_width,
                height=args.ir_height,
                fps=args.fps,
                pixel_formats=[ONI_PIXEL_FORMAT_GRAY16],
                mirror=args.mirror_ir,
            )
            ir.start()
            print(
                "IR stream requested "
                f"{args.ir_width}x{args.ir_height}@{args.fps}; "
                "OpenNI selected the closest supported mode."
            )
        except OpenNIError as exc:
            print(f"[--] IR stream disabled: {exc}")
            ir = None
    return depth, ir


def save_snapshot(save_dir: Path, color, depth_vis, depth_mm, ir_vis, ir_raw) -> None:
    import cv2

    save_dir.mkdir(parents=True, exist_ok=True)
    stamp = strftime("%Y%m%d_%H%M%S")
    if color is not None:
        cv2.imwrite(str(save_dir / f"{stamp}_color_uvc.png"), color)
    if depth_vis is not None:
        cv2.imwrite(str(save_dir / f"{stamp}_depth_preview.png"), depth_vis)
    if depth_mm is not None:
        np.save(str(save_dir / f"{stamp}_depth_mm.npy"), depth_mm)
    if ir_vis is not None:
        cv2.imwrite(str(save_dir / f"{stamp}_ir_preview.png"), ir_vis)
    if ir_raw is not None:
        np.save(str(save_dir / f"{stamp}_ir_raw.npy"), ir_raw)
    print(f"Saved snapshot to {save_dir}")


def run_viewer(args) -> None:
    with OpenNI2() as device:
        info = device.device_info()
        print(
            "OpenNI device: "
            f"{info['vendor']} {info['name']} vid=0x{int(info['vid']):04x} pid=0x{int(info['pid']):04x}"
        )
        if args.list_modes:
            print_modes(device)
            return

        import cv2

        device.configure_laser_and_ldp(laser_on=not args.laser_off, ldp_on=args.ldp_on)
        depth_stream, ir_stream = create_streams(device, args)
        openni_streams = [depth_stream] + ([ir_stream] if ir_stream is not None else [])

        color_index, color_cap = choose_color_camera(args)
        if color_cap is not None:
            print(
                "Color UVC stream: "
                f"index={color_index}, requested {args.color_width}x{args.color_height}@{args.fps}, MJPG, "
                f"auto_exposure={'on' if not args.no_color_auto_exposure else 'off'}"
            )
        elif not args.no_color:
            print("[--] Color UVC stream disabled: no usable camera index found")

        if args.single_window:
            cv2.namedWindow(COMBINED_WINDOW, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(COMBINED_WINDOW, 1280, 960)
        else:
            cv2.namedWindow(DEPTH_WINDOW, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(DEPTH_WINDOW, args.depth_width, args.depth_height)
            cv2.moveWindow(DEPTH_WINDOW, 40, 40)
            if ir_stream is not None:
                cv2.namedWindow(IR_WINDOW, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(IR_WINDOW, args.ir_width, args.ir_height)
                cv2.moveWindow(IR_WINDOW, 720, 40)
            if color_cap is not None:
                cv2.namedWindow(COLOR_WINDOW, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(COLOR_WINDOW, args.color_width, args.color_height)
                cv2.moveWindow(COLOR_WINDOW, 40, 500)

        depth_fps = FpsMeter()
        ir_fps = FpsMeter()
        color_fps = FpsMeter()
        last_depth_mm = None
        last_depth_vis = None
        last_ir_raw = None
        last_ir_vis = None
        last_color = None

        print("Running. Press s to save frames, q or ESC to quit.")
        try:
            while True:
                changed = device.wait_for_any_stream(openni_streams, timeout_ms=1)
                if changed is not None:
                    stream = openni_streams[changed]
                    frame = stream.read_frame()
                    if stream.sensor_type == ONI_SENSOR_DEPTH:
                        depth_fps.tick()
                        last_depth_mm = depth_to_mm(frame.data, int(frame.video_mode.pixelFormat))
                        last_depth_vis = render_depth(last_depth_mm, args.min_depth_mm, args.max_depth_mm)
                        draw_label(last_depth_vis, "Depth", depth_fps, f"{frame.data.shape[1]}x{frame.data.shape[0]}")
                    elif stream.sensor_type == ONI_SENSOR_IR:
                        ir_fps.tick()
                        last_ir_raw = frame.data
                        last_ir_vis = render_ir(last_ir_raw)
                        draw_label(last_ir_vis, "Infrared", ir_fps, f"{frame.data.shape[1]}x{frame.data.shape[0]}")

                if color_cap is not None:
                    ok, color = color_cap.read()
                    if ok and color is not None:
                        color_fps.tick()
                        if args.mirror_color:
                            color = cv2.flip(color, 1)
                        last_color = color
                        draw_label(last_color, "Color UVC", color_fps, f"{last_color.shape[1]}x{last_color.shape[0]}")

                if args.single_window:
                    cv2.imshow(COMBINED_WINDOW, make_combined(last_depth_vis, last_ir_vis, last_color))
                else:
                    if last_depth_vis is not None:
                        cv2.imshow(DEPTH_WINDOW, last_depth_vis)
                    if ir_stream is not None and last_ir_vis is not None:
                        cv2.imshow(IR_WINDOW, last_ir_vis)
                    if color_cap is not None and last_color is not None:
                        cv2.imshow(COLOR_WINDOW, last_color)

                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q"), ord("Q")):
                    break
                if key in (ord("s"), ord("S")):
                    save_snapshot(Path(args.save_dir), last_color, last_depth_vis, last_depth_mm, last_ir_vis, last_ir_raw)
        finally:
            if color_cap is not None:
                color_cap.release()
            cv2.destroyAllWindows()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Gemini Pro Plus OpenNI depth/IR + UVC color viewer")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--depth-width", type=int, default=640)
    parser.add_argument("--depth-height", type=int, default=400)
    parser.add_argument("--ir-width", type=int, default=640)
    parser.add_argument("--ir-height", type=int, default=400)
    parser.add_argument("--color-width", type=int, default=640)
    parser.add_argument("--color-height", type=int, default=480)
    parser.add_argument("--color-index", type=int, default=None)
    parser.add_argument("--max-color-index", type=int, default=6)
    parser.add_argument("--min-depth-mm", type=float, default=300.0)
    parser.add_argument("--max-depth-mm", type=float, default=3000.0)
    parser.add_argument("--save-dir", default="project/data/rgbd_samples")
    parser.add_argument("--single-window", action="store_true")
    parser.add_argument("--list-modes", action="store_true")
    parser.add_argument("--no-color", action="store_true")
    parser.add_argument("--no-ir", action="store_true")
    parser.add_argument("--laser-off", action="store_true", help="Do not enable the laser emitter at startup.")
    parser.add_argument("--ldp-on", action="store_true", help="Keep LDP / close-range protection enabled.")
    parser.add_argument("--no-color-auto-exposure", action="store_true")
    parser.add_argument("--mirror-color", action="store_true")
    parser.add_argument("--mirror-depth", action="store_true")
    parser.add_argument("--mirror-ir", action="store_true")
    parser.add_argument("--print-openni-bin", action="store_true")
    args = parser.parse_args()
    if args.print_openni_bin:
        print(BIN_DIR)
        raise SystemExit(0)
    return args


def main() -> None:
    args = parse_args()
    run_viewer(args)


if __name__ == "__main__":
    main()
