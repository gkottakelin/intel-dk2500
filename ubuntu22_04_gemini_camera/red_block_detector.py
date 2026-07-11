"""Standalone red block detector for Ubuntu 22.04.

Works with any UVC camera (including the Gemini colour stream) and provides
both a reusable API and a real-time preview.  Independent of the Orbbec SDK —
only ``opencv-python`` and ``numpy`` are required.

Usage as a script::

    python red_block_detector.py
    python red_block_detector.py --camera 1 --no-preview

Usage as a library::

    from red_block_detector import RedBlockDetector

    detector = RedBlockDetector()
    detector.open_camera(0)
    frame = detector.read_frame()
    result = detector.detect(frame)
    if result:
        print(f"Red block at {result.center}")
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np


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
    hue_low1:
        Lower bound of the first red hue range (0-180).
    hue_high1:
        Upper bound of the first red hue range.
    hue_low2:
        Lower bound of the second red hue range.
    hue_high2:
        Upper bound of the second red hue range.
    sat_min:
        Minimum saturation (0-255).  Higher = purer red required.
    val_min:
        Minimum value / brightness (0-255).  Higher = brighter red required.
    min_area:
        Minimum contour area in px².  Smaller regions are treated as noise.
    kernel_size:
        Side length of the square morphological kernel.
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
        self._cap: cv2.VideoCapture | None = None

    # -- Camera helpers -------------------------------------------------------

    def open_camera(self, index: int = 0) -> None:
        """Open a UVC camera by index via OpenCV VideoCapture."""
        cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
        if not cap.isOpened():
            raise RuntimeError(f"无法打开摄像头 index={index}")
        self._cap = cap

    def close_camera(self) -> None:
        """Release the camera resource."""
        if self._cap is not None:
            self._cap.release()
            self._cap = None

    @property
    def cap(self) -> cv2.VideoCapture:
        if self._cap is None:
            raise RuntimeError("摄像头未打开，请先调用 open_camera()")
        return self._cap

    def read_frame(self, *, flip: bool = True) -> np.ndarray | None:
        """Read one BGR frame from the camera.

        Returns ``None`` when no frame could be read.
        """
        ret, frame = self.cap.read()
        if not ret or frame is None:
            return None
        if flip:
            frame = cv2.flip(frame, 1)
        return frame

    # -- Detection ------------------------------------------------------------

    def detect(self, bgr_frame: np.ndarray) -> RedBlockResult | None:
        """Detect the largest red region in *bgr_frame*.

        Returns ``None`` when no qualifying red block is found.
        """
        hsv = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2HSV)

        mask1 = cv2.inRange(hsv, self._lower1, self._upper1)
        mask2 = cv2.inRange(hsv, self._lower2, self._upper2)
        mask = cv2.bitwise_or(mask1, mask2)

        # Morphological noise removal
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
# Real-time preview
# ---------------------------------------------------------------------------


def run_preview(
    camera_index: int = 0,
    *,
    detector: RedBlockDetector | None = None,
    fps_print_interval: float = 2.0,
) -> None:
    """Open a camera and run the real-time red-block preview loop.

    Press ``q`` or ``Esc`` to quit.
    """
    if detector is None:
        detector = RedBlockDetector()

    detector.open_camera(camera_index)
    print(f"摄像头 index={camera_index} 已打开。按 'q' 或 Esc 退出。")

    window = "Red Block Detection"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)

    last_fps_time = time.monotonic()
    frame_count = 0

    try:
        while True:
            frame = detector.read_frame()
            if frame is None:
                print("无法读取画面帧。")
                break

            result = detector.detect(frame)
            detector.draw(frame, result)

            cv2.imshow(window, frame)

            # FPS counter
            frame_count += 1
            now = time.monotonic()
            elapsed = now - last_fps_time
            if elapsed >= fps_print_interval:
                fps = frame_count / elapsed
                print(f"FPS: {fps:.1f}  |  ", end="")
                if result is not None:
                    print(f"红色物块 中心({result.center[0]}, {result.center[1]})  面积={result.area:.0f}px²")
                else:
                    print("未检测到红色物块")
                frame_count = 0
                last_fps_time = now

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q"), 27):  # 27 = Esc
                break
    finally:
        detector.close_camera()
        cv2.destroyAllWindows()


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="基于OpenCV的红色物块实时检测 (Ubuntu 22.04)"
    )
    parser.add_argument(
        "--camera", type=int, default=0,
        help="OpenCV摄像头索引 (默认: 0)",
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
        "--no-preview", action="store_true",
        help="不显示预览窗口，只打印检测坐标",
    )
    parser.add_argument(
        "--no-flip", action="store_true",
        help="不水平翻转画面",
    )
    return parser


def headless_detect(detector: RedBlockDetector, flip: bool = True) -> None:
    """Print detection results to stdout without opening a GUI window."""
    print("无预览模式运行中，按 Ctrl+C 退出。")
    try:
        while True:
            frame = detector.read_frame(flip=flip)
            if frame is None:
                time.sleep(0.01)
                continue
            result = detector.detect(frame)
            if result is not None:
                print(
                    f"红色物块: 中心({result.center[0]}, {result.center[1]})  "
                    f"边界框{result.bbox}  面积={result.area:.0f}px²"
                )
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n已退出。")


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
        detector.open_camera(args.camera)
    except RuntimeError as exc:
        print(f"错误: {exc}", file=sys.stderr)
        return 1

    try:
        if args.no_preview:
            headless_detect(detector, flip=not args.no_flip)
        else:
            run_preview(args.camera, detector=detector)
    finally:
        detector.close_camera()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
