"""Common helpers for Gemini Pro Plus Windows SDK experiments."""

from __future__ import annotations

import subprocess
import sys
from dataclasses import dataclass
from typing import Any, Optional


MIN_VALID_DEPTH_MM = 20.0
MAX_VALID_DEPTH_MM = 10000.0
_ORBBEC_IMPORT_CHECKED = False


@dataclass(frozen=True)
class Intrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float


@dataclass(frozen=True)
class RgbdStreamConfig:
    config: Any
    color_profile: Any
    depth_profile: Any


def import_orbbec_sdk() -> Any:
    """Import pyorbbecsdk with a clear error message."""

    global _ORBBEC_IMPORT_CHECKED
    if not _ORBBEC_IMPORT_CHECKED:
        result = subprocess.run(
            [
                sys.executable,
                "-c",
                "import pyorbbecsdk; print('pyorbbecsdk import ok', flush=True)",
            ],
            capture_output=True,
            text=True,
            timeout=15,
        )
        if result.returncode != 0 or "pyorbbecsdk import ok" not in result.stdout:
            raise RuntimeError(
                "pyorbbecsdk 原生扩展导入失败或导入时直接终止进程。\n"
                f"Python: {sys.executable}\n"
                f"Return code: {result.returncode}\n"
                f"stdout: {result.stdout.strip() or '<empty>'}\n"
                f"stderr: {result.stderr.strip() or '<empty>'}\n"
                "请优先检查 Orbbec SDK wheel、Microsoft Visual C++ 运行库、Windows x64/Python 版本是否匹配。"
            )
        _ORBBEC_IMPORT_CHECKED = True

    try:
        import pyorbbecsdk  # type: ignore
    except ImportError as exc:
        raise RuntimeError(
            "缺少 Orbbec Python SDK。请先安装：pip install pyorbbecsdk2 opencv-python numpy。"
            "如果 Python 3.14 下安装失败，建议新建 Python 3.11/3.12/3.13 虚拟环境。"
        ) from exc
    return pyorbbecsdk


def frame_to_bgr_image(color_frame: Any) -> Optional[np.ndarray]:
    """Convert an Orbbec color frame to an OpenCV BGR image."""

    import cv2
    import numpy as np

    sdk = import_orbbec_sdk()
    width = int(color_frame.get_width())
    height = int(color_frame.get_height())
    frame_format = color_frame.get_format()
    data = np.frombuffer(color_frame.get_data(), dtype=np.uint8)

    if frame_format == sdk.OBFormat.RGB:
        image = data.reshape((height, width, 3))
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    bgr_format = getattr(sdk.OBFormat, "BGR", None)
    if bgr_format is not None and frame_format == bgr_format:
        return data.reshape((height, width, 3)).copy()
    mjpg_format = getattr(sdk.OBFormat, "MJPG", None)
    if mjpg_format is not None and frame_format == mjpg_format:
        image = cv2.imdecode(data, cv2.IMREAD_COLOR)
        return image
    yuyv_format = getattr(sdk.OBFormat, "YUYV", None)
    if yuyv_format is not None and frame_format == yuyv_format:
        image = data.reshape((height, width, 2))
        return cv2.cvtColor(image, cv2.COLOR_YUV2BGR_YUYV)

    return None


def depth_frame_to_mm(depth_frame: Any) -> np.ndarray:
    """Convert an Orbbec depth frame to a float32 depth image in millimeters."""

    import numpy as np

    width = int(depth_frame.get_width())
    height = int(depth_frame.get_height())
    depth_raw = np.frombuffer(depth_frame.get_data(), dtype=np.uint16).reshape((height, width))
    return depth_raw.astype(np.float32) * float(depth_frame.get_depth_scale())


def render_depth(depth_mm: np.ndarray, min_mm: float = 20.0, max_mm: float = 5000.0) -> np.ndarray:
    """Render depth millimeters as a colored image for display only."""

    import cv2
    import numpy as np

    valid = np.where((depth_mm >= min_mm) & (depth_mm <= max_mm), depth_mm, 0)
    normalized = cv2.normalize(valid, None, 0, 255, cv2.NORM_MINMAX)
    return cv2.applyColorMap(normalized.astype(np.uint8), cv2.COLORMAP_JET)


def median_depth_at(depth_mm: np.ndarray, u: int, v: int, window: int = 7) -> Optional[float]:
    """Return the median valid depth around a pixel."""

    import numpy as np

    if window < 1 or window % 2 == 0:
        raise ValueError("window 必须是正奇数，例如 5 或 7")
    h, w = depth_mm.shape[:2]
    if not (0 <= u < w and 0 <= v < h):
        return None

    radius = window // 2
    x0 = max(0, u - radius)
    x1 = min(w, u + radius + 1)
    y0 = max(0, v - radius)
    y1 = min(h, v + radius + 1)
    roi = depth_mm[y0:y1, x0:x1]
    valid = roi[(roi >= MIN_VALID_DEPTH_MM) & (roi <= MAX_VALID_DEPTH_MM)]
    if valid.size == 0:
        return None
    return float(np.median(valid))


def pixel_to_camera_point_mm(u: int, v: int, depth_mm: float, intrinsics: Intrinsics) -> tuple[float, float, float]:
    """Convert pixel + depth to camera coordinates in millimeters."""

    x = (u - intrinsics.cx) * depth_mm / intrinsics.fx
    y = (v - intrinsics.cy) * depth_mm / intrinsics.fy
    return x, y, depth_mm


def _intrinsics_from_sdk(value: Any) -> Intrinsics:
    return Intrinsics(
        width=int(getattr(value, "width")),
        height=int(getattr(value, "height")),
        fx=float(getattr(value, "fx")),
        fy=float(getattr(value, "fy")),
        cx=float(getattr(value, "cx")),
        cy=float(getattr(value, "cy")),
    )


def _is_valid_intrinsics(value: Intrinsics) -> bool:
    return value.width > 0 and value.height > 0 and value.fx > 0 and value.fy > 0


def get_camera_intrinsics(pipeline: Any, stream_config: Optional[RgbdStreamConfig] = None) -> Optional[dict[str, Intrinsics]]:
    """Try to read color/depth intrinsics from stream profiles or active pipeline."""

    result: dict[str, Intrinsics] = {}

    if stream_config is not None:
        for name, profile in (("color", stream_config.color_profile), ("depth", stream_config.depth_profile)):
            try:
                value = _intrinsics_from_sdk(profile.get_intrinsic())
            except Exception:
                continue
            if _is_valid_intrinsics(value):
                result[name] = value
        if result:
            return result

    try:
        profiles = pipeline.get_camera_param()
    except Exception:
        return None

    for name, value in (("color", getattr(profiles, "rgb_intrinsic", None)), ("depth", getattr(profiles, "depth_intrinsic", None))):
        if value is None:
            continue
        intrinsics = _intrinsics_from_sdk(value)
        if _is_valid_intrinsics(intrinsics):
            result[name] = intrinsics
    return result or None


def choose_rgbd_config(pipeline: Any, *, prefer_rgb: bool = True) -> RgbdStreamConfig:
    """Create a Config for simultaneous color and depth streams."""

    sdk = import_orbbec_sdk()
    config = sdk.Config()

    color_profiles = pipeline.get_stream_profile_list(sdk.OBSensorType.COLOR_SENSOR)
    depth_profiles = pipeline.get_stream_profile_list(sdk.OBSensorType.DEPTH_SENSOR)

    color_profile = None
    if prefer_rgb:
        try:
            color_profile = color_profiles.get_video_stream_profile(0, 0, sdk.OBFormat.RGB, 0)
        except Exception:
            color_profile = None
    if color_profile is None:
        color_profile = color_profiles.get_default_video_stream_profile()

    depth_profile = depth_profiles.get_default_video_stream_profile()
    config.enable_stream(color_profile)
    config.enable_stream(depth_profile)
    return RgbdStreamConfig(config=config, color_profile=color_profile, depth_profile=depth_profile)
