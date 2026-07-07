"""Capture one RGB frame from a configured Linux V4L2 camera."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class RGBJpegFrame:
    data: bytes
    width: int
    height: int
    mime_type: str = "image/jpeg"


def capture_rgb_jpeg(
    device: str,
    *,
    width: int = 640,
    height: int = 480,
    warmup_frames: int = 3,
    jpeg_quality: int = 85,
    cv2_module: Any = None,
) -> RGBJpegFrame:
    """Open ``device``, read a recent RGB frame, encode it as JPEG, then close it."""

    camera_path = str(device).strip()
    if not camera_path:
        raise RuntimeError("未配置RGB相机接口，请先运行设备配置程序")
    if width <= 0 or height <= 0:
        raise ValueError("RGB图像宽高必须大于0")
    if warmup_frames < 1:
        raise ValueError("warmup_frames必须大于0")
    if not 1 <= jpeg_quality <= 100:
        raise ValueError("jpeg_quality必须在1到100之间")

    if cv2_module is None:
        try:
            import cv2 as cv2_module
        except ImportError as exc:
            raise RuntimeError(
                "缺少OpenCV，请执行: python -m pip install -r requirements-ai.txt"
            ) from exc

    backend = getattr(cv2_module, "CAP_V4L2", None)
    capture = (
        cv2_module.VideoCapture(camera_path, backend)
        if backend is not None
        else cv2_module.VideoCapture(camera_path)
    )
    try:
        if not capture.isOpened():
            raise RuntimeError(f"无法打开RGB相机: {camera_path}")

        capture.set(cv2_module.CAP_PROP_FRAME_WIDTH, float(width))
        capture.set(cv2_module.CAP_PROP_FRAME_HEIGHT, float(height))
        if hasattr(cv2_module, "CAP_PROP_BUFFERSIZE"):
            capture.set(cv2_module.CAP_PROP_BUFFERSIZE, 1)

        frame = None
        for _ in range(warmup_frames):
            ok, candidate = capture.read()
            if ok and candidate is not None:
                frame = candidate
        if frame is None:
            raise RuntimeError(f"RGB相机未返回有效画面: {camera_path}")

        encode_options = [
            int(cv2_module.IMWRITE_JPEG_QUALITY),
            int(jpeg_quality),
        ]
        encoded_ok, encoded = cv2_module.imencode(".jpg", frame, encode_options)
        if not encoded_ok:
            raise RuntimeError("RGB画面JPEG编码失败")

        frame_height, frame_width = frame.shape[:2]
        return RGBJpegFrame(
            data=encoded.tobytes(),
            width=int(frame_width),
            height=int(frame_height),
        )
    finally:
        capture.release()
