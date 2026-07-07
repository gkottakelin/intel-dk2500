"""Minimal ctypes wrapper around the bundled OrbbecSDK 1.5.7 Linux library.

The Gemini camera in this project uses Orbbec's legacy OpenNI protocol.  The
wrapper intentionally exposes only the API needed by the standalone viewer:
device enumeration, default RGB-D streaming, frames and camera intrinsics.
"""

from __future__ import annotations

import ctypes
import platform
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np


APP_ROOT = Path(__file__).resolve().parent
DEFAULT_SDK_CONFIG = APP_ROOT / "sdk" / "OrbbecSDKConfig.xml"
DEFAULT_SDK_LIBRARY = APP_ROOT / "sdk" / "x64" / "libOrbbecSDK.so.1.5.7"

OB_FORMAT_YUYV = 0
OB_FORMAT_YUY2 = 1
OB_FORMAT_UYVY = 2
OB_FORMAT_NV12 = 3
OB_FORMAT_NV21 = 4
OB_FORMAT_MJPG = 5
OB_FORMAT_Y16 = 8
OB_FORMAT_Y8 = 9
OB_FORMAT_Y10 = 10
OB_FORMAT_Y11 = 11
OB_FORMAT_Y12 = 12
OB_FORMAT_I420 = 15
OB_FORMAT_RGB = 22
OB_FORMAT_BGR = 23
OB_FORMAT_Y14 = 24
OB_FORMAT_BGRA = 25


class OrbbecError(RuntimeError):
    """Raised when the native Orbbec SDK reports an error."""


class OBCameraIntrinsic(ctypes.Structure):
    _fields_ = [
        ("fx", ctypes.c_float),
        ("fy", ctypes.c_float),
        ("cx", ctypes.c_float),
        ("cy", ctypes.c_float),
        ("width", ctypes.c_int16),
        ("height", ctypes.c_int16),
    ]


class OBCameraDistortion(ctypes.Structure):
    _fields_ = [(name, ctypes.c_float) for name in ("k1", "k2", "k3", "k4", "k5", "k6", "p1", "p2")]


class OBD2CTransform(ctypes.Structure):
    _fields_ = [("rot", ctypes.c_float * 9), ("trans", ctypes.c_float * 3)]


class OBCameraParam(ctypes.Structure):
    _fields_ = [
        ("depthIntrinsic", OBCameraIntrinsic),
        ("rgbIntrinsic", OBCameraIntrinsic),
        ("depthDistortion", OBCameraDistortion),
        ("rgbDistortion", OBCameraDistortion),
        ("transform", OBD2CTransform),
        ("isMirrored", ctypes.c_bool),
    ]


@dataclass(frozen=True)
class CameraDeviceInfo:
    index: int
    name: str
    serial_number: str
    uid: str
    vid: int
    pid: int

    @property
    def selection_key(self) -> str:
        return self.serial_number or self.uid

    @property
    def label(self) -> str:
        serial = self.serial_number or self.uid or "无序列号"
        return f"{self.name} | SN: {serial} | {self.vid:04x}:{self.pid:04x}"


@dataclass(frozen=True)
class NativeFrame:
    width: int
    height: int
    frame_format: int
    data: bytes
    depth_scale: float = 1.0


@dataclass(frozen=True)
class RgbdFrames:
    color: Optional[NativeFrame]
    depth: Optional[NativeFrame]


@dataclass(frozen=True)
class Intrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float


def _decode(value: Optional[bytes]) -> str:
    return value.decode("utf-8", errors="replace") if value else ""


def validate_linux_x64() -> None:
    if platform.system() != "Linux":
        raise RuntimeError("此相机包只能在 Ubuntu/Linux 上连接真机")
    machine = platform.machine().lower()
    if machine not in {"x86_64", "amd64"}:
        raise RuntimeError(f"当前内置 SDK 仅支持 x86_64，检测到架构: {machine}")


class OrbbecApi:
    """Bound C API with consistent native-error handling."""

    def __init__(self, library_path: str | Path = DEFAULT_SDK_LIBRARY) -> None:
        validate_linux_x64()
        self.library_path = Path(library_path).resolve()
        if not self.library_path.is_file():
            raise FileNotFoundError(f"找不到 Orbbec Linux SDK: {self.library_path}")
        self.lib = ctypes.CDLL(str(self.library_path), mode=ctypes.RTLD_GLOBAL)
        self._bind_api()

    def _bind(self, name: str, restype, *argtypes) -> None:
        function = getattr(self.lib, name)
        function.argtypes = [*argtypes, ctypes.POINTER(ctypes.c_void_p)]
        function.restype = restype

    def _bind_api(self) -> None:
        ptr = ctypes.c_void_p
        u32 = ctypes.c_uint32

        self.lib.ob_error_message.argtypes = [ptr]
        self.lib.ob_error_message.restype = ctypes.c_char_p
        self.lib.ob_error_function.argtypes = [ptr]
        self.lib.ob_error_function.restype = ctypes.c_char_p
        self.lib.ob_delete_error.argtypes = [ptr]
        self.lib.ob_delete_error.restype = None

        self._bind("ob_create_context_with_config", ptr, ctypes.c_char_p)
        self._bind("ob_delete_context", None, ptr)
        self._bind("ob_query_device_list", ptr, ptr)
        self._bind("ob_delete_device_list", None, ptr)
        self._bind("ob_device_list_device_count", u32, ptr)
        self._bind("ob_device_list_get_device_name", ctypes.c_char_p, ptr, u32)
        self._bind("ob_device_list_get_device_serial_number", ctypes.c_char_p, ptr, u32)
        self._bind("ob_device_list_get_device_uid", ctypes.c_char_p, ptr, u32)
        self._bind("ob_device_list_get_device_pid", ctypes.c_int, ptr, u32)
        self._bind("ob_device_list_get_device_vid", ctypes.c_int, ptr, u32)
        self._bind("ob_device_list_get_device", ptr, ptr, u32)
        self._bind("ob_device_list_get_device_by_serial_number", ptr, ptr, ctypes.c_char_p)
        self._bind("ob_device_list_get_device_by_uid", ptr, ptr, ctypes.c_char_p)
        self._bind("ob_delete_device", None, ptr)

        self._bind("ob_create_pipeline_with_device", ptr, ptr)
        self._bind("ob_delete_pipeline", None, ptr)
        self._bind("ob_pipeline_start", None, ptr)
        self._bind("ob_pipeline_stop", None, ptr)
        self._bind("ob_pipeline_wait_for_frameset", ptr, ptr, u32)
        self._bind("ob_pipeline_get_camera_param", OBCameraParam, ptr)

        self._bind("ob_frameset_color_frame", ptr, ptr)
        self._bind("ob_frameset_depth_frame", ptr, ptr)
        self._bind("ob_frame_format", ctypes.c_int, ptr)
        self._bind("ob_frame_data", ptr, ptr)
        self._bind("ob_frame_data_size", u32, ptr)
        self._bind("ob_video_frame_width", u32, ptr)
        self._bind("ob_video_frame_height", u32, ptr)
        self._bind("ob_depth_frame_get_value_scale", ctypes.c_float, ptr)
        self._bind("ob_delete_frame", None, ptr)

    def call(self, name: str, *args):
        error = ctypes.c_void_p()
        result = getattr(self.lib, name)(*args, ctypes.byref(error))
        if error.value:
            try:
                message = _decode(self.lib.ob_error_message(error))
                function = _decode(self.lib.ob_error_function(error))
            finally:
                self.lib.ob_delete_error(error)
            where = f" [{function}]" if function else ""
            raise OrbbecError(f"{message or 'Orbbec SDK 调用失败'}{where}")
        return result

    def safe_delete(self, name: str, value: Optional[int]) -> None:
        if not value:
            return
        try:
            self.call(name, value)
        except Exception:
            pass


def _open_context(api: OrbbecApi, config_path: str | Path) -> int:
    config = Path(config_path).resolve()
    if not config.is_file():
        raise FileNotFoundError(f"找不到 Orbbec SDK 配置: {config}")
    context = api.call("ob_create_context_with_config", str(config).encode("utf-8"))
    if not context:
        raise OrbbecError("Orbbec SDK 未能创建 Context")
    return context


def _read_devices(api: OrbbecApi, device_list: int) -> list[CameraDeviceInfo]:
    count = int(api.call("ob_device_list_device_count", device_list))
    devices: list[CameraDeviceInfo] = []
    for index in range(count):
        idx = ctypes.c_uint32(index)
        devices.append(
            CameraDeviceInfo(
                index=index,
                name=_decode(api.call("ob_device_list_get_device_name", device_list, idx)),
                serial_number=_decode(api.call("ob_device_list_get_device_serial_number", device_list, idx)),
                uid=_decode(api.call("ob_device_list_get_device_uid", device_list, idx)),
                vid=int(api.call("ob_device_list_get_device_vid", device_list, idx)),
                pid=int(api.call("ob_device_list_get_device_pid", device_list, idx)),
            )
        )
    return devices


def enumerate_devices(
    library_path: str | Path = DEFAULT_SDK_LIBRARY,
    config_path: str | Path = DEFAULT_SDK_CONFIG,
) -> list[CameraDeviceInfo]:
    api = OrbbecApi(library_path)
    context = _open_context(api, config_path)
    device_list = 0
    try:
        device_list = api.call("ob_query_device_list", context)
        if not device_list:
            return []
        return _read_devices(api, device_list)
    finally:
        api.safe_delete("ob_delete_device_list", device_list)
        api.safe_delete("ob_delete_context", context)


class OrbbecSession:
    """Own a selected device and a default color/depth pipeline."""

    def __init__(
        self,
        selection_key: str,
        *,
        library_path: str | Path = DEFAULT_SDK_LIBRARY,
        config_path: str | Path = DEFAULT_SDK_CONFIG,
    ) -> None:
        self.api = OrbbecApi(library_path)
        self.config_path = Path(config_path)
        self.selection_key = selection_key
        self.context = 0
        self.device = 0
        self.pipeline = 0
        self.started = False
        self.device_info: Optional[CameraDeviceInfo] = None

    def __enter__(self) -> "OrbbecSession":
        try:
            self.open()
        except Exception:
            self.close()
            raise
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def open(self) -> None:
        self.context = _open_context(self.api, self.config_path)
        device_list = 0
        try:
            device_list = self.api.call("ob_query_device_list", self.context)
            devices = _read_devices(self.api, device_list)
            match = next((item for item in devices if self.selection_key in {item.serial_number, item.uid}), None)
            if match is None:
                available = "\n  ".join(item.label for item in devices) or "无"
                raise RuntimeError(f"找不到相机 {self.selection_key}。当前设备:\n  {available}")
            self.device_info = match
            encoded = self.selection_key.encode("utf-8")
            if match.serial_number == self.selection_key:
                self.device = self.api.call("ob_device_list_get_device_by_serial_number", device_list, encoded)
            else:
                self.device = self.api.call("ob_device_list_get_device_by_uid", device_list, encoded)
        finally:
            self.api.safe_delete("ob_delete_device_list", device_list)

        if not self.device:
            raise OrbbecError("Orbbec SDK 未能打开选中的相机")
        self.pipeline = self.api.call("ob_create_pipeline_with_device", self.device)
        if not self.pipeline:
            raise OrbbecError("Orbbec SDK 未能创建 Pipeline")
        self.api.call("ob_pipeline_start", self.pipeline)
        self.started = True

    def close(self) -> None:
        if self.started and self.pipeline:
            try:
                self.api.call("ob_pipeline_stop", self.pipeline)
            except Exception:
                pass
        self.started = False
        self.api.safe_delete("ob_delete_pipeline", self.pipeline)
        self.pipeline = 0
        self.api.safe_delete("ob_delete_device", self.device)
        self.device = 0
        self.api.safe_delete("ob_delete_context", self.context)
        self.context = 0

    def _copy_frame(self, frame: int, *, is_depth: bool) -> NativeFrame:
        width = int(self.api.call("ob_video_frame_width", frame))
        height = int(self.api.call("ob_video_frame_height", frame))
        frame_format = int(self.api.call("ob_frame_format", frame))
        size = int(self.api.call("ob_frame_data_size", frame))
        data_pointer = self.api.call("ob_frame_data", frame)
        if not data_pointer or size <= 0:
            raise OrbbecError("收到空图像帧")
        data = ctypes.string_at(data_pointer, size)
        scale = float(self.api.call("ob_depth_frame_get_value_scale", frame)) if is_depth else 1.0
        return NativeFrame(width, height, frame_format, data, scale)

    def wait_for_frames(self, timeout_ms: int) -> Optional[RgbdFrames]:
        frameset = self.api.call("ob_pipeline_wait_for_frameset", self.pipeline, ctypes.c_uint32(timeout_ms))
        if not frameset:
            return None
        color_frame = 0
        depth_frame = 0
        try:
            color_frame = self.api.call("ob_frameset_color_frame", frameset)
            depth_frame = self.api.call("ob_frameset_depth_frame", frameset)
            color = self._copy_frame(color_frame, is_depth=False) if color_frame else None
            depth = self._copy_frame(depth_frame, is_depth=True) if depth_frame else None
            return RgbdFrames(color=color, depth=depth)
        finally:
            self.api.safe_delete("ob_delete_frame", color_frame)
            self.api.safe_delete("ob_delete_frame", depth_frame)
            self.api.safe_delete("ob_delete_frame", frameset)

    def wait_for_color_frame(self, timeout_ms: int) -> Optional[NativeFrame]:
        """Wait for one color frame without requesting or copying a depth frame."""

        frameset = self.api.call(
            "ob_pipeline_wait_for_frameset",
            self.pipeline,
            ctypes.c_uint32(timeout_ms),
        )
        if not frameset:
            return None
        color_frame = 0
        try:
            color_frame = self.api.call("ob_frameset_color_frame", frameset)
            return self._copy_frame(color_frame, is_depth=False) if color_frame else None
        finally:
            self.api.safe_delete("ob_delete_frame", color_frame)
            self.api.safe_delete("ob_delete_frame", frameset)

    def intrinsics(self) -> dict[str, Intrinsics]:
        value = self.api.call("ob_pipeline_get_camera_param", self.pipeline)

        def convert(item: OBCameraIntrinsic) -> Intrinsics:
            return Intrinsics(int(item.width), int(item.height), float(item.fx), float(item.fy), float(item.cx), float(item.cy))

        return {"depth": convert(value.depthIntrinsic), "color": convert(value.rgbIntrinsic)}


def depth_frame_to_mm(frame: NativeFrame) -> np.ndarray:
    expected = frame.width * frame.height * 2
    if len(frame.data) < expected:
        raise ValueError(f"深度帧数据不足: {len(frame.data)} < {expected}")
    raw = np.frombuffer(frame.data, dtype=np.uint16, count=frame.width * frame.height)
    return raw.reshape((frame.height, frame.width)).astype(np.float32) * frame.depth_scale
