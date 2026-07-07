"""Capture RGB-only JPEG frames from the selected Gemini/Orbbec device."""

from __future__ import annotations

import tempfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

try:
    from ubuntu22_04_gemini_camera.gemini_camera import color_frame_to_bgr
    from ubuntu22_04_gemini_camera.orbbec_native import (
        DEFAULT_SDK_CONFIG,
        DEFAULT_SDK_LIBRARY,
        NativeFrame,
        OrbbecSession,
    )
except ModuleNotFoundError:  # Imported as project.src.jetarm_agent in tests.
    from project.ubuntu22_04_gemini_camera.gemini_camera import color_frame_to_bgr
    from project.ubuntu22_04_gemini_camera.orbbec_native import (
        DEFAULT_SDK_CONFIG,
        DEFAULT_SDK_LIBRARY,
        NativeFrame,
        OrbbecSession,
    )


@dataclass(frozen=True)
class RGBJpegFrame:
    data: bytes
    width: int
    height: int
    mime_type: str = "image/jpeg"


def write_color_only_sdk_config(
    destination: str | Path,
    *,
    source: str | Path = DEFAULT_SDK_CONFIG,
) -> Path:
    """Copy the bundled SDK configuration while disabling the Depth stream."""

    source_path = Path(source).resolve()
    destination_path = Path(destination).resolve()
    tree = ET.parse(source_path)
    stream = tree.getroot().find("./Pipeline/Stream")
    if stream is None:
        raise RuntimeError(f"Orbbec SDK配置缺少Pipeline/Stream: {source_path}")
    depth = stream.find("Depth")
    if depth is not None:
        stream.remove(depth)
    if stream.find("Color") is None:
        raise RuntimeError(f"Orbbec SDK配置缺少Color流: {source_path}")
    tree.write(destination_path, encoding="utf-8", xml_declaration=True)
    return destination_path


def capture_rgb_jpeg(
    selection_key: str,
    *,
    warmup_frames: int = 3,
    frame_timeout_ms: int = 1000,
    jpeg_quality: int = 85,
    sdk_library_path: str | Path = DEFAULT_SDK_LIBRARY,
    sdk_config_path: str | Path = DEFAULT_SDK_CONFIG,
    session_factory: Callable[..., Any] = OrbbecSession,
    frame_converter: Callable[[NativeFrame], Any] = color_frame_to_bgr,
    cv2_module: Any = None,
) -> RGBJpegFrame:
    """Open the selected Orbbec device with Color enabled and Depth disabled."""

    camera_key = str(selection_key).strip()
    if not camera_key:
        raise RuntimeError("未配置Orbbec相机序列号/UID，请先运行设备配置程序")
    if camera_key.startswith("/dev/video"):
        raise RuntimeError("不再支持V4L2节点配置，请重新选择Orbbec USB设备/序列号")
    if warmup_frames < 1:
        raise ValueError("warmup_frames必须大于0")
    if frame_timeout_ms < 1:
        raise ValueError("frame_timeout_ms必须大于0")
    if not 1 <= jpeg_quality <= 100:
        raise ValueError("jpeg_quality必须在1到100之间")

    if cv2_module is None:
        try:
            import cv2 as cv2_module
        except ImportError as exc:
            raise RuntimeError(
                "缺少OpenCV，请执行: python -m pip install -r requirements-ai.txt"
            ) from exc

    with tempfile.TemporaryDirectory(prefix="jetarm-orbbec-rgb-") as directory:
        color_config = write_color_only_sdk_config(
            Path(directory) / "OrbbecSDKColorOnlyConfig.xml",
            source=sdk_config_path,
        )
        with session_factory(
            camera_key,
            library_path=sdk_library_path,
            config_path=color_config,
        ) as session:
            color_frame = None
            for _ in range(warmup_frames):
                candidate = session.wait_for_color_frame(frame_timeout_ms)
                if candidate is not None:
                    color_frame = candidate
            if color_frame is None:
                raise RuntimeError(f"Orbbec RGB流未返回有效画面: {camera_key}")

            frame = frame_converter(color_frame)
            if frame is None:
                raise RuntimeError(
                    f"不支持Orbbec彩色帧格式: {color_frame.frame_format}"
                )
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
