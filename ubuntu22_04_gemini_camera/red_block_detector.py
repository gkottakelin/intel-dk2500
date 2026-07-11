"""Standalone red block detector for Ubuntu 22.04 — Orbbec Gemini camera.

Connects to the Gemini colour stream through the bundled OrbbecSDK (same
pipeline as ``gemini_camera.py`` and ``run.sh``), converts frames to BGR,
then applies HSV-based red-block detection.

Requirements
------------
* ``opencv-python`` and ``numpy`` (already in this package's requirements.txt)
* The Orbbec SDK ``.so`` must be on ``LD_LIBRARY_PATH`` — use ``run.sh`` or::

      export LD_LIBRARY_PATH=sdk/x64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}

Usage as a script
-----------------
.. code:: bash

    # Auto-detect camera, real-time preview
    bash run.sh red_block_detector.py

    # Specify serial number, headless mode
    bash run.sh red_block_detector.py --serial CP2C2420001X --no-preview

    # Adjust thresholds for darker / less saturated reds
    bash run.sh red_block_detector.py --sat-min 100 --val-min 60

Usage as a library
------------------
.. code:: python

    from red_block_detector import RedBlockDetector, open_gemini_camera

    session = open_gemini_camera()
    detector = RedBlockDetector()
    frame = session.wait_for_color_frame(1000)
    bgr = color_frame_to_bgr(frame)
    result = detector.detect(bgr)
    if result:
        print(f"Red block at {result.center}")
    session.close()
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np

try:
    from .orbbec_native import (
        APP_ROOT,
        DEFAULT_SDK_CONFIG,
        DEFAULT_SDK_LIBRARY,
        CameraDeviceInfo,
        NativeFrame,
        OrbbecSession,
        enumerate_devices,
        validate_linux_x64,
    )
    from .gemini_camera import color_frame_to_bgr
except ImportError:
    from orbbec_native import (  # type: ignore[no-redef]
        APP_ROOT,
        DEFAULT_SDK_CONFIG,
        DEFAULT_SDK_LIBRARY,
        CameraDeviceInfo,
        NativeFrame,
        OrbbecSession,
        enumerate_devices,
        validate_linux_x64,
    )
    from gemini_camera import color_frame_to_bgr  # type: ignore[no-redef]


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class RedBlockResult:
    """Result of a successful red-block detection.

    Attributes:
        center: Pixel ``(x, y)`` of the centroid (top-left origin, x→right, y↓).
        bbox: Axis-aligned bounding box ``(x, y, width, height)``.
        area: Contour area in square pixels.
    """

    center: tuple[int, int]
    bbox: tuple[int, int, int, int]
    area: float


# ---------------------------------------------------------------------------
# Detector
# ---------------------------------------------------------------------------


class RedBlockDetector:
    """HSV-based red block detector with configurable thresholds.

    Parameters
    ----------
    hue_low1 / hue_high1:
        First red hue range (0-180).  Red wraps around the HSV hue circle
        so two ranges are needed.
    hue_low2 / hue_high2:
        Second red hue range.
    sat_min:
        Minimum saturation (0-255).  Higher = purer red required.
    val_min:
        Minimum value / brightness (0-255).  Higher = brighter red required.
    min_area:
        Minimum contour area in px².  Smaller regions are treated as noise.
    kernel_size:
        Side length of the square morphological kernel (must be odd).
    """

    def __init__(
        self,
        *,
        hue_low1: int = 0,
        hue_high1: int = 8,
        hue_low2: int = 172,
        hue_high2: int = 180,
        sat_min: int = 150,
        val_min: int = 100,
        min_area: float = 150.0,
        kernel_size: int = 7,
    ) -> None:
        self._lower1 = np.array([hue_low1, sat_min, val_min], dtype=np.uint8)
        self._upper1 = np.array([hue_high1, 255, 255], dtype=np.uint8)
        self._lower2 = np.array([hue_low2, sat_min, val_min], dtype=np.uint8)
        self._upper2 = np.array([hue_high2, 255, 255], dtype=np.uint8)
        self.min_area = float(min_area)
        ks = int(kernel_size)
        if ks < 1 or ks % 2 == 0:
            raise ValueError("kernel_size 必须是正奇数")
        self._kernel = np.ones((ks, ks), np.uint8)

    # -- Detection ------------------------------------------------------------

    def detect(self, bgr_frame: np.ndarray) -> RedBlockResult | None:
        """Detect the largest red region in *bgr_frame*.

        Returns ``None`` when no qualifying red block is found.
        """
        import cv2

        hsv = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2HSV)

        mask1 = cv2.inRange(hsv, self._lower1, self._upper1)
        mask2 = cv2.inRange(hsv, self._lower2, self._upper2)
        mask = cv2.bitwise_or(mask1, mask2)

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._kernel)
        mask = cv2.dilate(mask, self._kernel, iterations=1)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        best: RedBlockResult | None = None
        best_area = 0.0

        for cnt in contours:
            area = float(cv2.contourArea(cnt))
            if area <= self.min_area:
                continue
            if area <= best_area:
                continue

            moments = cv2.moments(cnt)
            if moments["m00"] == 0:
                continue
            cx = int(moments["m10"] / moments["m00"])
            cy = int(moments["m01"] / moments["m00"])
            bx, by, bw, bh = (int(v) for v in cv2.boundingRect(cnt))

            best_area = area
            best = RedBlockResult(
                center=(cx, cy),
                bbox=(bx, by, bw, bh),
                area=area,
            )

        return best

    def detect_all(self, bgr_frame: np.ndarray) -> list[RedBlockResult]:
        """Return all red blocks above *min_area*, sorted largest-first."""
        import cv2

        hsv = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, self._lower1, self._upper1)
        mask2 = cv2.inRange(hsv, self._lower2, self._upper2)
        mask = cv2.bitwise_or(mask1, mask2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._kernel)
        mask = cv2.dilate(mask, self._kernel, iterations=1)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        results: list[RedBlockResult] = []
        for cnt in contours:
            area = float(cv2.contourArea(cnt))
            if area <= self.min_area:
                continue
            moments = cv2.moments(cnt)
            if moments["m00"] == 0:
                continue
            cx = int(moments["m10"] / moments["m00"])
            cy = int(moments["m01"] / moments["m00"])
            bx, by, bw, bh = (int(v) for v in cv2.boundingRect(cnt))
            results.append(
                RedBlockResult(center=(cx, cy), bbox=(bx, by, bw, bh), area=area)
            )

        results.sort(key=lambda r: r.area, reverse=True)
        return results

    # -- Visualisation --------------------------------------------------------

    @staticmethod
    def draw(
        bgr_frame: np.ndarray,
        result: RedBlockResult | None,
        *,
        box_color: tuple[int, int, int] = (0, 255, 0),
        center_color: tuple[int, int, int] = (255, 0, 0),
        thickness: int = 2,
        label: bool = True,
    ) -> np.ndarray:
        """Draw bounding box and centre dot for *result* onto *bgr_frame* (in-place)."""
        import cv2

        if result is None:
            return bgr_frame

        x, y, w, h = result.bbox
        cx, cy = result.center
        cv2.rectangle(bgr_frame, (x, y), (x + w, y + h), box_color, thickness)
        cv2.circle(bgr_frame, (cx, cy), 4, center_color, -1)
        if label:
            text = f"Red (X:{cx}, Y:{cy})"
            cv2.putText(
                bgr_frame, text, (x, max(y - 10, 16)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, thickness,
            )
        return bgr_frame


# ---------------------------------------------------------------------------
# Gemini camera helpers (Orbbec SDK)
# ---------------------------------------------------------------------------


def discover_camera(
    serial: str | None = None,
    *,
    library_path: str = str(DEFAULT_SDK_LIBRARY),
    config_path: str = str(DEFAULT_SDK_CONFIG),
) -> CameraDeviceInfo:
    """Enumerate Orbbec devices and pick the matching camera.

    If *serial* is ``None`` and exactly one device is connected it is
    returned automatically.  Raises ``RuntimeError`` with a clear
    message when zero or multiple devices are found.
    """
    validate_linux_x64()
    devices = enumerate_devices(library_path=library_path, config_path=config_path)
    if not devices:
        raise RuntimeError("未发现 Orbbec 相机。请检查 USB 连接和 udev 权限。")
    if serial:
        match = next(
            (d for d in devices if serial in {d.serial_number, d.uid}), None
        )
        if match is None:
            available = "\n  ".join(d.label for d in devices)
            raise RuntimeError(
                f"找不到相机 serial={serial}。当前设备:\n  {available}"
            )
        return match
    if len(devices) > 1:
        available = "\n  ".join(d.label for d in devices)
        raise RuntimeError(
            f"发现多个相机，请用 --serial 指定:\n  {available}"
        )
    return devices[0]


def open_gemini_camera(
    serial: str | None = None,
    *,
    library_path: str = str(DEFAULT_SDK_LIBRARY),
    config_path: str = str(DEFAULT_SDK_CONFIG),
) -> OrbbecSession:
    """Discover and open an Orbbec Gemini camera session.

    Returns an active ``OrbbecSession`` that must be closed by the caller.
    """
    device = discover_camera(
        serial, library_path=library_path, config_path=config_path
    )
    session = OrbbecSession(
        device.selection_key, library_path=library_path, config_path=config_path
    )
    session.__enter__()
    return session


# ---------------------------------------------------------------------------
# Real-time preview
# ---------------------------------------------------------------------------


def run_preview(
    serial: str | None = None,
    *,
    detector: RedBlockDetector | None = None,
    frame_timeout_ms: int = 1000,
    mirror: bool = False,
    fps_print_interval: float = 2.0,
) -> None:
    """Open the Gemini camera and run the real-time red-block preview loop.

    Press ``q`` or ``Esc`` to quit.
    """
    if detector is None:
        detector = RedBlockDetector()

    import cv2

    session = open_gemini_camera(serial)
    device = session.device_info
    print(
        f"Gemini 相机已连接: {device.name if device else serial or 'auto'}  |  "
        "按 'q' 或 Esc 退出。"
    )

    window = "Red Block Detection - Gemini"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)

    last_fps_time = time.monotonic()
    frame_count = 0

    try:
        while True:
            native_frame = session.wait_for_color_frame(frame_timeout_ms)
            if native_frame is None:
                continue

            bgr = color_frame_to_bgr(native_frame)
            if bgr is None:
                continue
            if mirror:
                bgr = cv2.flip(bgr, 1)

            result = detector.detect(bgr)
            detector.draw(bgr, result)
            cv2.imshow(window, bgr)

            frame_count += 1
            now = time.monotonic()
            elapsed = now - last_fps_time
            if elapsed >= fps_print_interval:
                fps = frame_count / elapsed
                print(f"FPS: {fps:.1f}  |  ", end="")
                if result is not None:
                    print(
                        f"红色物块 中心({result.center[0]}, {result.center[1]})  "
                        f"面积={result.area:.0f}px²"
                    )
                else:
                    print("未检测到红色物块")
                frame_count = 0
                last_fps_time = now

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                break
    finally:
        session.close()
        cv2.destroyAllWindows()


def headless_detect(
    serial: str | None = None,
    *,
    detector: RedBlockDetector | None = None,
    frame_timeout_ms: int = 1000,
    mirror: bool = False,
) -> None:
    """Print detection results without a GUI window (suitable for SSH)."""
    if detector is None:
        detector = RedBlockDetector()

    import cv2

    session = open_gemini_camera(serial)
    device = session.device_info
    print(
        f"无预览模式  |  Gemini: {device.name if device else serial or 'auto'}  |  "
        "按 Ctrl+C 退出。"
    )
    try:
        while True:
            native_frame = session.wait_for_color_frame(frame_timeout_ms)
            if native_frame is None:
                time.sleep(0.01)
                continue

            bgr = color_frame_to_bgr(native_frame)
            if bgr is None:
                time.sleep(0.01)
                continue
            if mirror:
                bgr = cv2.flip(bgr, 1)

            result = detector.detect(bgr)
            if result is not None:
                print(
                    f"红色物块: 中心({result.center[0]}, {result.center[1]})  "
                    f"边界框{result.bbox}  面积={result.area:.0f}px²"
                )
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n已退出。")
    finally:
        session.close()


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Gemini 相机红色物块实时检测 (Ubuntu 22.04 + Orbbec SDK)"
    )
    parser.add_argument(
        "--serial",
        default=None,
        help="Gemini 相机序列号或 UID（单设备可省略）",
    )
    parser.add_argument(
        "--hue-low1", type=int, default=0,
    )
    parser.add_argument(
        "--hue-high1", type=int, default=8,
    )
    parser.add_argument(
        "--hue-low2", type=int, default=172,
    )
    parser.add_argument(
        "--hue-high2", type=int, default=180,
    )
    parser.add_argument(
        "--sat-min", type=int, default=150,
        help="最小饱和度 (0-255)，值越高要求红色越纯",
    )
    parser.add_argument(
        "--val-min", type=int, default=100,
        help="最小明度 (0-255)，值越高要求红色越亮",
    )
    parser.add_argument(
        "--min-area", type=float, default=150.0,
        help="最小轮廓面积 (px²)，小于此值的视为噪点",
    )
    parser.add_argument(
        "--kernel-size", type=int, default=7,
        help="形态学卷积核边长 (奇数)",
    )
    parser.add_argument(
        "--frame-timeout", type=int, default=1000,
        help="等待帧的超时毫秒数",
    )
    parser.add_argument(
        "--no-preview", action="store_true",
        help="不显示预览窗口，只打印检测坐标",
    )
    parser.add_argument(
        "--mirror", action="store_true",
        help="水平翻转画面",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    detector = RedBlockDetector(
        hue_low1=args.hue_low1,
        hue_high1=args.hue_high1,
        hue_low2=args.hue_low2,
        hue_high2=args.hue_high2,
        sat_min=args.sat_min,
        val_min=args.val_min,
        min_area=args.min_area,
        kernel_size=args.kernel_size,
    )

    try:
        if args.no_preview:
            headless_detect(
                args.serial,
                detector=detector,
                frame_timeout_ms=args.frame_timeout,
                mirror=args.mirror,
            )
        else:
            run_preview(
                args.serial,
                detector=detector,
                frame_timeout_ms=args.frame_timeout,
                mirror=args.mirror,
            )
    except RuntimeError as exc:
        print(f"错误: {exc}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
