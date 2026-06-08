"""Minimal OpenNI2 C API wrapper for the bundled Gemini Windows SDK."""

from __future__ import annotations

import ctypes
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np


ROOT = Path(__file__).resolve().parents[3]
OPENNI_FOLDER_NAME = "OpenNI_v2.3.0.85_20220615_1b09bbfd_windows_x64_x86_release"

ONI_API_VERSION = 2003

ONI_STATUS_OK = 0
ONI_STATUS_TIME_OUT = 102

ONI_SENSOR_IR = 1
ONI_SENSOR_COLOR = 2
ONI_SENSOR_DEPTH = 3

ONI_PIXEL_FORMAT_DEPTH_1_MM = 100
ONI_PIXEL_FORMAT_DEPTH_100_UM = 101
ONI_PIXEL_FORMAT_GRAY16 = 203

ONI_STREAM_PROPERTY_VIDEO_MODE = 3
ONI_STREAM_PROPERTY_MIRRORING = 7

OBEXTENSION_ID_LDP_EN = 13
OBEXTENSION_ID_LASER_EN = 15
XN_MODULE_PROPERTY_LDP_ENABLE = 0x1080FFBE


class OpenNIError(RuntimeError):
    """Raised when OpenNI returns a non-OK status."""


class OniVideoMode(ctypes.Structure):
    _fields_ = [
        ("pixelFormat", ctypes.c_int),
        ("resolutionX", ctypes.c_int),
        ("resolutionY", ctypes.c_int),
        ("fps", ctypes.c_int),
    ]


class OniSensorInfo(ctypes.Structure):
    _fields_ = [
        ("sensorType", ctypes.c_int),
        ("numSupportedVideoModes", ctypes.c_int),
        ("pSupportedVideoModes", ctypes.POINTER(OniVideoMode)),
    ]


class OniDeviceInfo(ctypes.Structure):
    _fields_ = [
        ("uri", ctypes.c_char * 256),
        ("vendor", ctypes.c_char * 256),
        ("name", ctypes.c_char * 256),
        ("usbVendorId", ctypes.c_uint16),
        ("usbProductId", ctypes.c_uint16),
    ]


class OniFrame(ctypes.Structure):
    _fields_ = [
        ("dataSize", ctypes.c_int),
        ("data", ctypes.c_void_p),
        ("sensorType", ctypes.c_int),
        ("timestamp", ctypes.c_uint64),
        ("frameIndex", ctypes.c_int),
        ("width", ctypes.c_int),
        ("height", ctypes.c_int),
        ("videoMode", OniVideoMode),
        ("croppingEnabled", ctypes.c_int),
        ("cropOriginX", ctypes.c_int),
        ("cropOriginY", ctypes.c_int),
        ("stride", ctypes.c_int),
    ]


@dataclass(frozen=True)
class FrameData:
    data: np.ndarray
    timestamp: int
    frame_index: int
    video_mode: OniVideoMode


def locate_openni_root() -> Path:
    """Find the tutorial OpenNI folder without hard-coding non-ASCII paths."""

    explicit = os.environ.get("GEMINI_OPENNI_ROOT")
    if explicit:
        path = Path(explicit)
        if path.exists():
            return path

    for child in ROOT.iterdir():
        name = child.name.lower()
        if not child.is_dir() or not name.startswith("gemini") or "windows" not in name:
            continue
        windows_dir = child / "Windows" / "Windows"
        direct = windows_dir / OPENNI_FOLDER_NAME
        if direct.exists():
            return direct
        matches = list(windows_dir.glob("OpenNI_v2.3.0.85_*_windows_x64_x86_release"))
        if matches:
            return matches[0]

    raise FileNotFoundError(
        "Could not locate the bundled OpenNI SDK. Set GEMINI_OPENNI_ROOT to the "
        "OpenNI_v2.3.0.85... folder if the material directory was moved."
    )


OPENNI_ROOT = locate_openni_root()
BIN_DIR = OPENNI_ROOT / "samples" / "bin"
OPENNI_DLL = BIN_DIR / "OpenNI2.dll"


def _decode_c_string(raw: bytes) -> str:
    return raw.split(b"\x00", 1)[0].decode("utf-8", errors="replace")


class OpenNI2:
    def __init__(self, dll_path: Path = OPENNI_DLL) -> None:
        if not dll_path.exists():
            raise FileNotFoundError(f"OpenNI2.dll not found: {dll_path}")
        self.dll_path = dll_path
        self._dll_dirs: list[object] = []
        self.lib = self._load_library(dll_path)
        self._bind_api()
        self.initialized = False
        self.device = ctypes.c_void_p()
        self.streams: list[OpenNIStream] = []

    def _load_library(self, dll_path: Path):
        bin_dir = dll_path.parent
        driver_dir = bin_dir / "OpenNI2" / "Drivers"
        if hasattr(os, "add_dll_directory"):
            self._dll_dirs.append(os.add_dll_directory(str(bin_dir)))
            if driver_dir.exists():
                self._dll_dirs.append(os.add_dll_directory(str(driver_dir)))
        os.environ["PATH"] = str(bin_dir) + os.pathsep + os.environ.get("PATH", "")
        return ctypes.CDLL(str(dll_path))

    def _bind_api(self) -> None:
        lib = self.lib
        lib.oniInitialize.argtypes = [ctypes.c_int]
        lib.oniInitialize.restype = ctypes.c_int
        lib.oniShutdown.argtypes = []
        lib.oniShutdown.restype = None
        lib.oniGetExtendedError.argtypes = []
        lib.oniGetExtendedError.restype = ctypes.c_char_p

        lib.oniDeviceOpen.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_void_p)]
        lib.oniDeviceOpen.restype = ctypes.c_int
        lib.oniDeviceClose.argtypes = [ctypes.c_void_p]
        lib.oniDeviceClose.restype = ctypes.c_int
        lib.oniDeviceGetInfo.argtypes = [ctypes.c_void_p, ctypes.POINTER(OniDeviceInfo)]
        lib.oniDeviceGetInfo.restype = ctypes.c_int
        lib.oniDeviceGetSensorInfo.argtypes = [ctypes.c_void_p, ctypes.c_int]
        lib.oniDeviceGetSensorInfo.restype = ctypes.POINTER(OniSensorInfo)
        lib.oniDeviceCreateStream.argtypes = [
            ctypes.c_void_p,
            ctypes.c_int,
            ctypes.POINTER(ctypes.c_void_p),
        ]
        lib.oniDeviceCreateStream.restype = ctypes.c_int
        lib.oniDeviceSetProperty.argtypes = [
            ctypes.c_void_p,
            ctypes.c_int,
            ctypes.c_void_p,
            ctypes.c_int,
        ]
        lib.oniDeviceSetProperty.restype = ctypes.c_int
        lib.oniDeviceGetProperty.argtypes = [
            ctypes.c_void_p,
            ctypes.c_int,
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_int),
        ]
        lib.oniDeviceGetProperty.restype = ctypes.c_int

        lib.oniStreamStart.argtypes = [ctypes.c_void_p]
        lib.oniStreamStart.restype = ctypes.c_int
        lib.oniStreamStop.argtypes = [ctypes.c_void_p]
        lib.oniStreamStop.restype = None
        lib.oniStreamDestroy.argtypes = [ctypes.c_void_p]
        lib.oniStreamDestroy.restype = None
        lib.oniStreamSetProperty.argtypes = [
            ctypes.c_void_p,
            ctypes.c_int,
            ctypes.c_void_p,
            ctypes.c_int,
        ]
        lib.oniStreamSetProperty.restype = ctypes.c_int
        lib.oniStreamReadFrame.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.POINTER(OniFrame)),
        ]
        lib.oniStreamReadFrame.restype = ctypes.c_int
        lib.oniFrameRelease.argtypes = [ctypes.POINTER(OniFrame)]
        lib.oniFrameRelease.restype = None
        lib.oniWaitForAnyStream.argtypes = [
            ctypes.POINTER(ctypes.c_void_p),
            ctypes.c_int,
            ctypes.POINTER(ctypes.c_int),
            ctypes.c_int,
        ]
        lib.oniWaitForAnyStream.restype = ctypes.c_int

    def __enter__(self) -> "OpenNI2":
        self.initialize()
        self.open_device()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def extended_error(self) -> str:
        message = self.lib.oniGetExtendedError()
        return message.decode("utf-8", errors="replace") if message else ""

    def check(self, rc: int, message: str) -> None:
        if rc != ONI_STATUS_OK:
            detail = self.extended_error()
            suffix = f": {detail}" if detail else ""
            raise OpenNIError(f"{message} failed, status={rc}{suffix}")

    def initialize(self) -> None:
        self.check(self.lib.oniInitialize(ONI_API_VERSION), "OpenNI initialize")
        self.initialized = True

    def open_device(self) -> None:
        self.check(self.lib.oniDeviceOpen(None, ctypes.byref(self.device)), "OpenNI open device")

    def close(self) -> None:
        for stream in list(self.streams):
            stream.destroy()
        self.streams.clear()
        if self.device:
            self.lib.oniDeviceClose(self.device)
            self.device = ctypes.c_void_p()
        if self.initialized:
            self.lib.oniShutdown()
            self.initialized = False

    def device_info(self) -> dict[str, str | int]:
        info = OniDeviceInfo()
        self.check(self.lib.oniDeviceGetInfo(self.device, ctypes.byref(info)), "Get device info")
        return {
            "uri": _decode_c_string(bytes(info.uri)),
            "vendor": _decode_c_string(bytes(info.vendor)),
            "name": _decode_c_string(bytes(info.name)),
            "vid": int(info.usbVendorId),
            "pid": int(info.usbProductId),
        }

    def supported_modes(self, sensor_type: int) -> list[OniVideoMode]:
        sensor = self.lib.oniDeviceGetSensorInfo(self.device, sensor_type)
        if not sensor:
            return []
        info = sensor.contents
        return [info.pSupportedVideoModes[i] for i in range(info.numSupportedVideoModes)]

    def set_device_int(self, property_id: int, value: int, label: str) -> bool:
        data = ctypes.c_int(value)
        rc = self.lib.oniDeviceSetProperty(
            self.device,
            property_id,
            ctypes.byref(data),
            ctypes.sizeof(data),
        )
        if rc == ONI_STATUS_OK:
            print(f"[OK] {label} -> {value}")
            return True
        print(f"[--] {label} not applied, status={rc}: {self.extended_error()}")
        return False

    def get_device_int(self, property_id: int) -> int | None:
        data = ctypes.c_int(0)
        size = ctypes.c_int(ctypes.sizeof(data))
        rc = self.lib.oniDeviceGetProperty(
            self.device,
            property_id,
            ctypes.byref(data),
            ctypes.byref(size),
        )
        return int(data.value) if rc == ONI_STATUS_OK else None

    def configure_laser_and_ldp(self, *, laser_on: bool, ldp_on: bool) -> None:
        self.set_device_int(OBEXTENSION_ID_LASER_EN, int(laser_on), "laser emitter enable")
        if not self.set_device_int(OBEXTENSION_ID_LDP_EN, int(ldp_on), "LDP / close-range protection"):
            self.set_device_int(XN_MODULE_PROPERTY_LDP_ENABLE, int(ldp_on), "legacy LDP / close-range protection")

    def create_stream(
        self,
        sensor_type: int,
        *,
        width: int,
        height: int,
        fps: int,
        pixel_formats: Iterable[int],
        mirror: bool = False,
    ) -> "OpenNIStream":
        if not self.supported_modes(sensor_type):
            raise OpenNIError(f"Sensor type {sensor_type} is not available")

        handle = ctypes.c_void_p()
        self.check(
            self.lib.oniDeviceCreateStream(self.device, sensor_type, ctypes.byref(handle)),
            f"Create stream {sensor_type}",
        )
        stream = OpenNIStream(self, handle, sensor_type)
        self.streams.append(stream)

        mode = self.choose_mode(sensor_type, width=width, height=height, fps=fps, pixel_formats=pixel_formats)
        if mode is not None:
            stream.set_video_mode(mode)
        stream.set_mirroring(mirror)
        return stream

    def choose_mode(
        self,
        sensor_type: int,
        *,
        width: int,
        height: int,
        fps: int,
        pixel_formats: Iterable[int],
    ) -> OniVideoMode | None:
        formats = list(pixel_formats)
        modes = self.supported_modes(sensor_type)
        if not modes:
            return None

        def score(mode: OniVideoMode) -> tuple[int, int, int, int]:
            format_score = 0 if int(mode.pixelFormat) in formats else 1
            size_score = abs(int(mode.resolutionX) - width) + abs(int(mode.resolutionY) - height)
            fps_score = abs(int(mode.fps) - fps)
            return format_score, size_score, fps_score, formats.index(int(mode.pixelFormat)) if int(mode.pixelFormat) in formats else 999

        exact = [
            mode
            for mode in modes
            if int(mode.pixelFormat) in formats
            and int(mode.resolutionX) == width
            and int(mode.resolutionY) == height
            and int(mode.fps) == fps
        ]
        if exact:
            return exact[0]
        return sorted(modes, key=score)[0]

    def wait_for_any_stream(self, streams: list["OpenNIStream"], timeout_ms: int) -> int | None:
        if not streams:
            return None
        handles_type = ctypes.c_void_p * len(streams)
        handles = handles_type(*[stream.handle for stream in streams])
        changed_index = ctypes.c_int(-1)
        rc = self.lib.oniWaitForAnyStream(handles, len(streams), ctypes.byref(changed_index), timeout_ms)
        if rc == ONI_STATUS_TIME_OUT:
            return None
        self.check(rc, "Wait for OpenNI stream")
        return int(changed_index.value)


class OpenNIStream:
    def __init__(self, owner: OpenNI2, handle: ctypes.c_void_p, sensor_type: int) -> None:
        self.owner = owner
        self.handle = handle
        self.sensor_type = sensor_type
        self.started = False
        self.destroyed = False

    def set_video_mode(self, mode: OniVideoMode) -> None:
        self.owner.check(
            self.owner.lib.oniStreamSetProperty(
                self.handle,
                ONI_STREAM_PROPERTY_VIDEO_MODE,
                ctypes.byref(mode),
                ctypes.sizeof(mode),
            ),
            f"Set stream {self.sensor_type} video mode",
        )

    def set_mirroring(self, enabled: bool) -> None:
        data = ctypes.c_int(int(enabled))
        rc = self.owner.lib.oniStreamSetProperty(
            self.handle,
            ONI_STREAM_PROPERTY_MIRRORING,
            ctypes.byref(data),
            ctypes.sizeof(data),
        )
        if rc != ONI_STATUS_OK:
            print(f"[--] stream {self.sensor_type} mirror not applied, status={rc}")

    def start(self) -> None:
        self.owner.check(self.owner.lib.oniStreamStart(self.handle), f"Start stream {self.sensor_type}")
        self.started = True

    def stop(self) -> None:
        if self.started and not self.destroyed:
            self.owner.lib.oniStreamStop(self.handle)
            self.started = False

    def destroy(self) -> None:
        if not self.destroyed:
            self.stop()
            self.owner.lib.oniStreamDestroy(self.handle)
            self.destroyed = True

    def read_frame(self) -> FrameData:
        frame_ptr = ctypes.POINTER(OniFrame)()
        self.owner.check(
            self.owner.lib.oniStreamReadFrame(self.handle, ctypes.byref(frame_ptr)),
            f"Read stream {self.sensor_type}",
        )
        try:
            frame = frame_ptr.contents
            if frame.videoMode.pixelFormat not in (
                ONI_PIXEL_FORMAT_DEPTH_1_MM,
                ONI_PIXEL_FORMAT_DEPTH_100_UM,
                ONI_PIXEL_FORMAT_GRAY16,
            ):
                raise OpenNIError(f"Unexpected pixel format: {frame.videoMode.pixelFormat}")
            row_values = int(frame.stride // ctypes.sizeof(ctypes.c_uint16))
            source = ctypes.cast(frame.data, ctypes.POINTER(ctypes.c_uint16))
            array = np.ctypeslib.as_array(source, shape=(int(frame.height), row_values))
            data = array[:, : int(frame.width)].copy()
            return FrameData(
                data=data,
                timestamp=int(frame.timestamp),
                frame_index=int(frame.frameIndex),
                video_mode=frame.videoMode,
            )
        finally:
            self.owner.lib.oniFrameRelease(frame_ptr)
