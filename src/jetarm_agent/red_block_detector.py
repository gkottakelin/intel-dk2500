"""Pure-vision red block detection for the JetArm Agent.

Provides stateless functions that accept BGR numpy arrays and return
``RedBlockResult | None``.  No camera or SDK dependency — callers are
responsible for capturing and decoding frames.

The HSV parameters mirror the Ubuntu ``RedBlockDetector`` class in
``ubuntu22_04_gemini_camera/red_block_detector.py``.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable

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
# Core detection from a BGR array
# ---------------------------------------------------------------------------


def detect_red_block_from_bgr(
    bgr_frame: np.ndarray,
    *,
    hue_low1: int = 0,
    hue_high1: int = 8,
    hue_low2: int = 172,
    hue_high2: int = 180,
    sat_min: int = 150,
    val_min: int = 100,
    min_area: float = 150.0,
    kernel_size: int = 3,
) -> RedBlockResult | None:
    """Detect the largest red region in *bgr_frame*.

    Returns ``None`` when no qualifying red block is found.

    Parameters
    ----------
    bgr_frame:
        BGR image as a numpy array (H×W×3, uint8).
    min_area:
        Minimum contour area in px².  Smaller regions are treated as noise.
    """
    import cv2

    hsv = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2HSV)

    lower1 = np.array([hue_low1, sat_min, val_min], dtype=np.uint8)
    upper1 = np.array([hue_high1, 255, 255], dtype=np.uint8)
    lower2 = np.array([hue_low2, sat_min, val_min], dtype=np.uint8)
    upper2 = np.array([hue_high2, 255, 255], dtype=np.uint8)

    mask1 = cv2.inRange(hsv, lower1, upper1)
    mask2 = cv2.inRange(hsv, lower2, upper2)
    mask = cv2.bitwise_or(mask1, mask2)

    if kernel_size < 1 or kernel_size % 2 == 0:
        raise ValueError("kernel_size 必须是正奇数")
    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.dilate(mask, kernel, iterations=1)

    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    best: RedBlockResult | None = None
    best_area = 0.0

    for cnt in contours:
        area = float(cv2.contourArea(cnt))
        if area <= min_area:
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


def detect_red_block_mask(
    bgr_frame: np.ndarray,
    **kwargs: Any,
) -> tuple[RedBlockResult | None, np.ndarray]:
    """Like :func:`detect_red_block_from_bgr` but also returns the binary mask.

    Returns
    -------
    tuple
        ``(result, mask)`` where *mask* is the uint8 binary image after
        morphological operations (same H×W as *bgr_frame*).
    """
    import cv2

    hue_low1 = int(kwargs.pop("hue_low1", 0))
    hue_high1 = int(kwargs.pop("hue_high1", 8))
    hue_low2 = int(kwargs.pop("hue_low2", 172))
    hue_high2 = int(kwargs.pop("hue_high2", 180))
    sat_min = int(kwargs.pop("sat_min", 150))
    val_min = int(kwargs.pop("val_min", 100))
    kernel_size = int(kwargs.pop("kernel_size", 3))

    min_area = float(kwargs.pop("min_area", 150.0))

    hsv = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2HSV)

    lower1 = np.array([hue_low1, sat_min, val_min], dtype=np.uint8)
    upper1 = np.array([hue_high1, 255, 255], dtype=np.uint8)
    lower2 = np.array([hue_low2, sat_min, val_min], dtype=np.uint8)
    upper2 = np.array([hue_high2, 255, 255], dtype=np.uint8)

    mask1 = cv2.inRange(hsv, lower1, upper1)
    mask2 = cv2.inRange(hsv, lower2, upper2)
    mask = cv2.bitwise_or(mask1, mask2)

    if kernel_size < 1 or kernel_size % 2 == 0:
        raise ValueError("kernel_size 必须是正奇数")
    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.dilate(mask, kernel, iterations=1)

    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    best: RedBlockResult | None = None
    best_area = 0.0

    for cnt in contours:
        area = float(cv2.contourArea(cnt))
        if area <= min_area:
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

    return best, mask


# ---------------------------------------------------------------------------
# Camera integration wrapper
# ---------------------------------------------------------------------------


def detect_red_block_from_camera(
    selection_key: str,
    *,
    capture_rgb: Callable[..., Any] | None = None,
    **detector_kwargs: Any,
) -> RedBlockResult | None:
    """Capture a JPEG frame from *selection_key*, decode to BGR, and detect.

    Parameters
    ----------
    selection_key:
        Camera identifier passed to *capture_rgb*.
    capture_rgb:
        Callable ``(selection_key) -> RGBJpegFrame``.  When ``None``, the
        default ``capture_rgb_jpeg`` from ``.rgb_camera`` is used.
    **detector_kwargs:
        Forwarded to :func:`detect_red_block_from_bgr`.
    """
    import cv2

    if capture_rgb is None:
        from .rgb_camera import capture_rgb_jpeg as _default_capture

        capture_rgb = _default_capture

    frame = capture_rgb(selection_key)
    jpeg_bytes = getattr(frame, "data", None)
    if jpeg_bytes is None:
        raise RuntimeError("capture_rgb 未返回有效的JPEG数据")

    nparr = np.frombuffer(jpeg_bytes, np.uint8)
    bgr = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
    if bgr is None:
        raise RuntimeError("无法将JPEG解码为BGR图像")

    return detect_red_block_from_bgr(bgr, **detector_kwargs)
