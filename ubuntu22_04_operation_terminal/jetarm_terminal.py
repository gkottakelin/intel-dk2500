"""Standalone JetArm operation terminal for Ubuntu 22.04 and Python 3.10."""

from __future__ import annotations

import argparse
import json
import math
import os
import struct
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Optional

import numpy as np

try:
    import tkinter as tk
    from tkinter import messagebox, ttk
except ImportError:  # pragma: no cover - depends on the Ubuntu system package
    tk = None
    ttk = None
    messagebox = None


APP_ROOT = Path(__file__).resolve().parent
DEFAULT_CONFIG_PATH = APP_ROOT / "config" / "terminal.json"
HEADER = b"\x55\x55"
SERVO_MOVE_TIME_WRITE = 1
SERVO_MOVE_STOP = 12
SERVO_POS_READ = 28
SERVO_OR_MOTOR_MODE_WRITE = 29
LINUX_SERIAL_PATTERNS = (
    "ttyUSB*",
    "ttyACM*",
    "ttyAMA*",
    "ttyTHS*",
    "ttyXRUSB*",
    "ttyCH343USB*",
)
KNOWN_USB_SERIAL_VENDOR_IDS = {"0403", "10c4", "1a86", "067b"}
ARM_JOINTS = ("J1", "J2", "J3", "J4")


@dataclass(frozen=True)
class TerminalSettings:
    baudrate: int
    timeout_s: float
    tick_s: float
    vertical_speed_m_s: float
    max_horizontal_speed_m_s: float
    j5_speed: int
    j6_speed: int
    j6_grip_speed: int
    home_run_time_ms: int
    max_joint_step_deg: float
    damping: float
    local_search_step_units: int
    home: dict[str, int]
    joints: dict[str, dict[str, Any]]
    geometry: dict[str, float]

    @classmethod
    def from_file(cls, path: str | Path = DEFAULT_CONFIG_PATH) -> "TerminalSettings":
        with Path(path).open("r", encoding="utf-8") as file:
            data = json.load(file)
        serial = data["serial"]
        control = data["control"]
        return cls(
            baudrate=int(serial["baudrate"]),
            timeout_s=float(serial["timeout_s"]),
            tick_s=float(control["tick_ms"]) / 1000.0,
            vertical_speed_m_s=float(control["vertical_speed_m_s"]),
            max_horizontal_speed_m_s=float(control["max_horizontal_speed_m_s"]),
            j5_speed=int(control["j5_speed"]),
            j6_speed=int(control["j6_speed"]),
            j6_grip_speed=int(control["j6_grip_speed"]),
            home_run_time_ms=int(control["home_run_time_ms"]),
            max_joint_step_deg=float(control["max_joint_step_deg"]),
            damping=float(control["damping"]),
            local_search_step_units=int(control["local_search_step_units"]),
            home={name: int(value) for name, value in data["home"].items()},
            joints={name: dict(value) for name, value in data["joints"].items()},
            geometry={name: float(value) for name, value in data["geometry_m"].items()},
        )

    def servo_id(self, joint_name: str) -> int:
        return int(self.joints[joint_name]["servo_id"])

    def position_limits(self, joint_name: str) -> tuple[int, int]:
        joint = self.joints[joint_name]
        return int(joint["position_min"]), int(joint["position_max"])


@dataclass(frozen=True)
class UsbSerialAdapter:
    vendor_id: str
    product_id: str
    manufacturer: str
    product: str

    @property
    def label(self) -> str:
        name = " ".join(part for part in (self.manufacturer, self.product) if part)
        return f"{name or 'USB serial adapter'} ({self.vendor_id}:{self.product_id})"


def checksum(body: bytes) -> int:
    return (~sum(body)) & 0xFF


def build_packet(servo_id: int, command: int, params: bytes = b"") -> bytes:
    if not 0 <= servo_id <= 254:
        raise ValueError("servo_id must be in 0..254")
    if not 0 <= command <= 255:
        raise ValueError("command must be in 0..255")
    body = bytes((servo_id, len(params) + 3, command)) + params
    return HEADER + body + bytes((checksum(body),))


class BusServoController:
    """Minimal half-duplex bus-servo controller used by the Ubuntu terminal."""

    def __init__(self, device: str, baudrate: int, timeout_s: float) -> None:
        self.device = device
        self.baudrate = baudrate
        self.timeout_s = timeout_s
        self._port: Any = None

    @property
    def port(self) -> Any:
        if self._port is None:
            try:
                import serial  # type: ignore
            except ImportError as exc:
                raise RuntimeError("缺少 pyserial，请先运行 bash setup.sh") from exc
            self._port = serial.Serial(self.device, self.baudrate, timeout=self.timeout_s)
        return self._port

    def close(self) -> None:
        if self._port is not None:
            self._port.close()
            self._port = None

    def write_command(self, servo_id: int, command: int, params: bytes = b"") -> None:
        self.port.write(build_packet(servo_id, command, params))
        self.port.flush()

    def move_servo(self, servo_id: int, target_position: int, run_time_ms: int) -> None:
        if not 0 <= target_position <= 1050:
            raise ValueError("target_position must be in 0..1050")
        if not 0 <= run_time_ms <= 30000:
            raise ValueError("run_time_ms must be in 0..30000")
        params = struct.pack("<HH", target_position, run_time_ms)
        self.write_command(servo_id, SERVO_MOVE_TIME_WRITE, params)

    def stop_servo(self, servo_id: int) -> None:
        """Immediately cancel an in-progress position-mode move."""

        self.write_command(servo_id, SERVO_MOVE_STOP)

    def set_motor_speed(self, servo_id: int, speed: int) -> None:
        if not -1000 <= speed <= 1000:
            raise ValueError("motor speed must be in -1000..1000")
        params = b"\x01\x00" + struct.pack("<h", speed)
        self.write_command(servo_id, SERVO_OR_MOTOR_MODE_WRITE, params)

    def set_servo_mode(self, servo_id: int) -> None:
        """Switch a bus servo from motor mode to position-control mode."""

        params = b"\x00\x00" + struct.pack("<h", 0)
        self.write_command(servo_id, SERVO_OR_MOTOR_MODE_WRITE, params)

    def read_position(self, servo_id: int) -> int:
        reset = getattr(self.port, "reset_input_buffer", None)
        if callable(reset):
            reset()
        self.write_command(servo_id, SERVO_POS_READ)
        frame = self._read_response(servo_id, SERVO_POS_READ)
        return struct.unpack("<h", frame[5:7])[0]

    def _read_response(self, expected_id: int, expected_command: int) -> bytes:
        deadline = time.monotonic() + self.timeout_s
        previous = b""
        while time.monotonic() <= deadline:
            current = self.port.read(1)
            if not current:
                continue
            if previous == b"\x55" and current == b"\x55":
                id_and_length = self._read_exact(2, deadline)
                servo_id, length = id_and_length
                rest = self._read_exact(length - 1, deadline)
                frame = HEADER + id_and_length + rest
                if servo_id != expected_id or frame[4] != expected_command:
                    raise RuntimeError("舵机响应 ID 或命令不匹配")
                if checksum(frame[2:-1]) != frame[-1]:
                    raise RuntimeError("舵机响应校验和错误")
                if len(frame) != length + 3:
                    raise RuntimeError("舵机响应长度错误")
                return frame
            previous = current
        raise TimeoutError(f"等待舵机 {expected_id} 响应超时")

    def _read_exact(self, size: int, deadline: float) -> bytes:
        data = bytearray()
        while len(data) < size:
            if time.monotonic() > deadline:
                raise TimeoutError("串口读取超时")
            chunk = self.port.read(size - len(data))
            if chunk:
                data.extend(chunk)
        return bytes(data)


class DryRunServoController:
    def __init__(self, settings: TerminalSettings, logger: Optional[Callable[[str], None]] = None) -> None:
        self.positions = {settings.servo_id(name): value for name, value in settings.home.items()}
        self.logger = logger or (lambda _message: None)
        self.move_calls: list[tuple[int, int, int]] = []
        self.motor_calls: list[tuple[int, int]] = []
        self.servo_mode_calls: list[int] = []
        self.stop_calls: list[int] = []

    def read_position(self, servo_id: int) -> int:
        return int(self.positions.get(servo_id, 500))

    def move_servo(self, servo_id: int, target_position: int, run_time_ms: int) -> None:
        self.positions[servo_id] = target_position
        self.move_calls.append((servo_id, target_position, run_time_ms))
        self.logger(f"DRY move_servo id={servo_id} target={target_position} time={run_time_ms}ms")

    def set_motor_speed(self, servo_id: int, speed: int) -> None:
        self.motor_calls.append((servo_id, speed))
        self.logger(f"DRY set_motor_speed id={servo_id} speed={speed}")

    def stop_servo(self, servo_id: int) -> None:
        self.stop_calls.append(servo_id)
        self.logger(f"DRY stop_servo id={servo_id}")

    def set_servo_mode(self, servo_id: int) -> None:
        self.servo_mode_calls.append(servo_id)
        self.logger(f"DRY set_servo_mode id={servo_id}")

    def close(self) -> None:
        return


class JetArmKinematics:
    def __init__(self, settings: TerminalSettings) -> None:
        self.settings = settings

    def position_to_model_angle(self, joint_name: str, position: int) -> float:
        joint = self.settings.joints[joint_name]
        low, high = self.settings.position_limits(joint_name)
        ratio = (position - low) / (high - low)
        servo_deg = float(joint["angle_min_deg"]) + ratio * (
            float(joint["angle_max_deg"]) - float(joint["angle_min_deg"])
        )
        return math.radians(float(joint["direction_sign"]) * servo_deg)

    def model_angle_to_position(self, joint_name: str, angle_rad: float) -> int:
        joint = self.settings.joints[joint_name]
        sign = float(joint["direction_sign"])
        servo_deg = math.degrees(angle_rad) / sign
        min_deg = float(joint["angle_min_deg"])
        max_deg = float(joint["angle_max_deg"])
        low, high = self.settings.position_limits(joint_name)
        ratio = (servo_deg - min_deg) / (max_deg - min_deg)
        return max(low, min(high, int(round(low + ratio * (high - low)))))

    def clamp_model_angle(self, joint_name: str, angle_rad: float) -> float:
        joint = self.settings.joints[joint_name]
        sign = float(joint["direction_sign"])
        bounds = (
            math.radians(sign * float(joint["angle_min_deg"])),
            math.radians(sign * float(joint["angle_max_deg"])),
        )
        low, high = min(bounds), max(bounds)
        return max(low, min(high, angle_rad))

    def tcp(self, positions: dict[str, int]) -> np.ndarray:
        q1, q2, q3, q4 = [self.position_to_model_angle(name, positions[name]) for name in ARM_JOINTS]
        g = self.settings.geometry
        pitch2 = q2
        pitch3 = q2 + q3
        pitch4 = q2 + q3 + q4
        radial = (
            g["joint2_to_joint3"] * math.sin(pitch2)
            + g["joint3_to_joint4"] * math.sin(pitch3)
            + (g["joint4_to_joint5"] + g["joint5_to_tcp"]) * math.sin(pitch4)
        )
        z = (
            g["base_to_joint2"]
            + g["joint2_to_joint3"] * math.cos(pitch2)
            + g["joint3_to_joint4"] * math.cos(pitch3)
            + (g["joint4_to_joint5"] + g["joint5_to_tcp"]) * math.cos(pitch4)
        )
        return np.array((radial * math.cos(q1), radial * math.sin(q1), z), dtype=float)

    def jacobian(self, positions: dict[str, int], delta_rad: float = math.radians(0.5)) -> np.ndarray:
        columns = []
        for joint_name in ARM_JOINTS:
            angle = self.position_to_model_angle(joint_name, positions[joint_name])
            plus = dict(positions)
            minus = dict(positions)
            plus[joint_name] = self.model_angle_to_position(joint_name, angle + delta_rad)
            minus[joint_name] = self.model_angle_to_position(joint_name, angle - delta_rad)
            position_delta = self.position_to_model_angle(joint_name, plus[joint_name]) - self.position_to_model_angle(
                joint_name, minus[joint_name]
            )
            if abs(position_delta) < 1e-12:
                columns.append(np.zeros(3))
            else:
                columns.append((self.tcp(plus) - self.tcp(minus)) / position_delta)
        return np.column_stack(columns)


def clamp_unit_circle(x: float, y: float) -> tuple[float, float]:
    length = math.hypot(x, y)
    if length <= 1.0:
        return float(x), float(y)
    return float(x) / length, float(y) / length


class ManualServoRuntime:
    def __init__(
        self,
        controller: Any,
        settings: TerminalSettings,
        *,
        logger: Optional[Callable[[str], None]] = None,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        self.controller = controller
        self.settings = settings
        self.model = JetArmKinematics(settings)
        self.logger = logger or (lambda _message: None)
        self.monotonic = monotonic
        self.positions: dict[str, int] = {}
        self.vertical_direction = 0.0
        self.joystick_x = 0.0
        self.joystick_y = 0.0
        self.j6_grip_locked = False
        self.last_step_at: Optional[float] = None
        self.closed = False

    def initialize(self, *, use_home_positions: bool = False) -> None:
        for joint_name in ARM_JOINTS:
            if use_home_positions:
                value = self.settings.home[joint_name]
            else:
                value = int(self.controller.read_position(self.settings.servo_id(joint_name)))
            low, high = self.settings.position_limits(joint_name)
            if not low <= value <= high:
                raise RuntimeError(f"{joint_name} 当前值 {value} 超出安全范围 {low}..{high}")
            self.positions[joint_name] = value
        self.last_step_at = self.monotonic()
        self.logger("J1-J4 当前位置已载入，终端就绪")

    def set_vertical_direction(self, direction: float) -> None:
        """Set normalized base-Z velocity input.

        The standalone terminal still passes -1/0/1.  A float is accepted so
        the AI adapter can combine vertical and horizontal components for a
        camera-relative Cartesian direction without changing terminal UI
        semantics.
        """

        self.vertical_direction = max(-1.0, min(1.0, float(direction)))

    def set_joystick(self, x: float, y: float) -> None:
        self.joystick_x, self.joystick_y = clamp_unit_circle(x, y)

    def center_joystick(self) -> None:
        self.set_joystick(0.0, 0.0)

    def cartesian_velocity(self) -> np.ndarray:
        return np.array(
            (
                -self.joystick_y * self.settings.max_horizontal_speed_m_s,
                -self.joystick_x * self.settings.max_horizontal_speed_m_s,
                self.vertical_direction * self.settings.vertical_speed_m_s,
            ),
            dtype=float,
        )

    def rotate_j5_counterclockwise(self) -> None:
        self.controller.set_motor_speed(self.settings.servo_id("J5"), -abs(self.settings.j5_speed))

    def rotate_j5_clockwise(self) -> None:
        self.controller.set_motor_speed(self.settings.servo_id("J5"), abs(self.settings.j5_speed))

    def stop_j5(self) -> None:
        self.controller.set_motor_speed(self.settings.servo_id("J5"), 0)

    def set_j6_speed(self, speed: int) -> bool:
        if self.j6_grip_locked:
            self.logger("J6 抓紧锁定中，忽略松/闭误触")
            return False
        self.controller.set_motor_speed(self.settings.servo_id("J6"), speed)
        return True

    def open_j6(self) -> bool:
        return self.set_j6_speed(-abs(self.settings.j6_speed))

    def close_j6(self) -> bool:
        return self.set_j6_speed(abs(self.settings.j6_speed))

    def stop_j6(self) -> bool:
        return self.set_j6_speed(0)

    def toggle_grip_lock(self) -> bool:
        self.j6_grip_locked = not self.j6_grip_locked
        speed = abs(self.settings.j6_grip_speed) if self.j6_grip_locked else 0
        self.controller.set_motor_speed(self.settings.servo_id("J6"), speed)
        return self.j6_grip_locked

    def stop_all(self) -> None:
        self.vertical_direction = 0
        self.center_joystick()
        self.j6_grip_locked = False
        errors: list[str] = []
        stop_servo = getattr(self.controller, "stop_servo", None)
        if callable(stop_servo):
            for joint_name in ARM_JOINTS:
                try:
                    stop_servo(self.settings.servo_id(joint_name))
                except Exception as exc:
                    errors.append(f"{joint_name}: {exc}")
        for joint_name in ("J5", "J6"):
            try:
                self.controller.set_motor_speed(self.settings.servo_id(joint_name), 0)
            except Exception as exc:
                errors.append(f"{joint_name}: {exc}")
        self.logger("全部停止：J1-J4位置运动已取消，J5/J6速度已置零")
        if errors:
            raise RuntimeError("部分关节停止失败: " + "；".join(errors))

    def go_home(self) -> None:
        self.vertical_direction = 0
        self.center_joystick()
        self.controller.set_motor_speed(self.settings.servo_id("J5"), 0)
        for joint_name, target in self.settings.home.items():
            if joint_name == "J6":
                continue
            self.controller.move_servo(
                self.settings.servo_id(joint_name), target, self.settings.home_run_time_ms
            )
            if joint_name in ARM_JOINTS:
                self.positions[joint_name] = target

    def close(self) -> None:
        if self.closed:
            return
        self.closed = True
        try:
            self.stop_all()
        finally:
            self.controller.close()

    def tick(self) -> None:
        if self.closed:
            return
        now = self.monotonic()
        if self.last_step_at is None:
            self.last_step_at = now
            return
        dt = max(0.001, min(0.25, now - self.last_step_at))
        self.last_step_at = now
        self.step_cartesian(dt)

    def step_cartesian(self, dt: float, *, run_time_s: Optional[float] = None) -> bool:
        velocity = self.cartesian_velocity()
        if float(np.linalg.norm(velocity)) < 1e-9:
            return False
        target_positions = self._solve_next_positions(velocity * dt)
        if target_positions == self.positions:
            return False
        execution_time_s = dt if run_time_s is None else run_time_s
        run_time_ms = max(1, int(round(execution_time_s * 1000)))
        for joint_name in ARM_JOINTS:
            target = target_positions[joint_name]
            if target != self.positions[joint_name]:
                self.controller.move_servo(self.settings.servo_id(joint_name), target, run_time_ms)
                self.positions[joint_name] = target
        return True

    def _solve_next_positions(self, target_delta: np.ndarray) -> dict[str, int]:
        jacobian = self.model.jacobian(self.positions)
        damping = (self.settings.damping**2) * np.eye(3)
        try:
            dq = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + damping, target_delta)
        except np.linalg.LinAlgError:
            dq = np.zeros(4)
        max_step = math.radians(self.settings.max_joint_step_deg)
        dq = np.clip(dq, -max_step, max_step)
        seed = dict(self.positions)
        for index, joint_name in enumerate(ARM_JOINTS):
            current = self.model.position_to_model_angle(joint_name, self.positions[joint_name])
            target = self.model.clamp_model_angle(joint_name, current + float(dq[index]))
            seed[joint_name] = self.model.model_angle_to_position(joint_name, target)
        return self._local_refine(seed, target_delta)

    def _local_refine(self, seed: dict[str, int], target_delta: np.ndarray) -> dict[str, int]:
        target_tcp = self.model.tcp(self.positions) + target_delta
        candidates = [dict(self.positions), dict(seed)]
        step = max(1, self.settings.local_search_step_units)
        for joint_name in ARM_JOINTS:
            low, high = self.settings.position_limits(joint_name)
            for direction in (-1, 1):
                candidate = dict(seed)
                candidate[joint_name] = max(low, min(high, candidate[joint_name] + direction * step))
                candidates.append(candidate)
        for direction in (-1, 1):
            candidate = dict(seed)
            for joint_name in ARM_JOINTS[1:]:
                low, high = self.settings.position_limits(joint_name)
                candidate[joint_name] = max(low, min(high, candidate[joint_name] + direction * step))
            candidates.append(candidate)
        return min(candidates, key=lambda item: float(np.linalg.norm(self.model.tcp(item) - target_tcp)))


def _pyserial_list_ports() -> list[Any]:
    try:
        from serial.tools import list_ports  # type: ignore
    except ImportError:
        return []
    return list(list_ports.comports())


def _is_supported_pyserial_port(port: Any) -> bool:
    """Exclude legacy ttyS ports while retaining real USB serial adapters."""

    device = str(getattr(port, "device", "") or "").strip()
    if not device:
        return False
    device_name = Path(device).name
    if any(Path(device_name).match(pattern) for pattern in LINUX_SERIAL_PATTERNS):
        return True

    # PySerial exposes VID/PID and an HWID containing USB for USB adapters.
    # This also supports less common USB serial drivers with nonstandard names.
    if getattr(port, "vid", None) is not None:
        return True
    hwid = str(getattr(port, "hwid", "") or "").upper()
    return "USB" in hwid


def discover_linux_serial_ports(
    device_root: str | Path = "/dev",
    *,
    list_ports_provider: Optional[Callable[[], list[Any]]] = None,
) -> list[str]:
    """Discover serial ports from stable links, device names, and PySerial."""

    root = Path(device_root)
    candidates: list[Path] = []
    for stable_dir_name in ("by-id", "by-path"):
        stable_dir = root / "serial" / stable_dir_name
        if stable_dir.is_dir():
            candidates.extend(
                path
                for path in sorted(stable_dir.iterdir())
                if path.exists() or path.is_symlink()
            )
    for pattern in LINUX_SERIAL_PATTERNS:
        candidates.extend(sorted(root.glob(pattern)))

    provider = list_ports_provider
    if provider is None and root.resolve(strict=False) == Path("/dev"):
        provider = _pyserial_list_ports
    if provider is not None:
        for port in provider():
            if not _is_supported_pyserial_port(port):
                continue
            device = str(getattr(port, "device", "") or "").strip()
            if device:
                candidates.append(Path(device))

    result: list[str] = []
    seen: set[str] = set()
    for path in candidates:
        if not path.exists() and not path.is_symlink():
            continue
        resolved = str(path.resolve(strict=False))
        if resolved not in seen:
            seen.add(resolved)
            result.append(str(path))
    return result


def discover_usb_serial_adapters(
    sys_usb_root: str | Path = "/sys/bus/usb/devices",
) -> list[UsbSerialAdapter]:
    """Find USB-layer serial adapters even when no tty node was created."""

    root = Path(sys_usb_root)
    if not root.is_dir():
        return []
    adapters: list[UsbSerialAdapter] = []

    def read_value(directory: Path, name: str) -> str:
        try:
            return (directory / name).read_text(encoding="utf-8").strip()
        except (FileNotFoundError, OSError, UnicodeError):
            return ""

    for directory in sorted(root.iterdir()):
        vendor_id = read_value(directory, "idVendor").lower()
        product_id = read_value(directory, "idProduct").lower()
        manufacturer = read_value(directory, "manufacturer")
        product = read_value(directory, "product")
        description = f"{manufacturer} {product}".lower()
        looks_serial = any(
            word in description
            for word in ("serial", "uart", "ch340", "ch341", "cp210", "ftdi")
        )
        if vendor_id in KNOWN_USB_SERIAL_VENDOR_IDS or looks_serial:
            adapters.append(
                UsbSerialAdapter(
                    vendor_id=vendor_id or "????",
                    product_id=product_id or "????",
                    manufacturer=manufacturer,
                    product=product,
                )
            )
    return adapters


def serial_discovery_diagnostic(
    *,
    device_root: str | Path = "/dev",
    sys_usb_root: str | Path = "/sys/bus/usb/devices",
    list_ports_provider: Optional[Callable[[], list[Any]]] = None,
) -> str:
    ports = discover_linux_serial_ports(
        device_root, list_ports_provider=list_ports_provider
    )
    if ports:
        return "已发现串口: " + ", ".join(ports)

    adapters = discover_usb_serial_adapters(sys_usb_root)
    if adapters:
        adapter_text = ", ".join(adapter.label for adapter in adapters)
        if any(adapter.vendor_id == "1a86" for adapter in adapters):
            return (
                f"USB层已识别 {adapter_text}，但未创建/dev串口。"
                "请检查ch341驱动；Ubuntu 22.04还可能被brltty抢占。"
                "运行程序的--diagnose-ports查看处理命令。"
            )
        return (
            f"USB层已识别 {adapter_text}，但未创建/dev串口。"
            "请检查对应USB串口驱动和内核日志。"
        )
    return "未发现USB串口设备或/dev串口节点，请检查数据线、供电和USB连接。"


def select_linux_serial_port(
    explicit_port: Optional[str],
    *,
    device_root: str | Path = "/dev",
    access_check: Callable[[str, int], bool] = os.access,
) -> str:
    if explicit_port:
        candidates = [explicit_port]
    else:
        candidates = discover_linux_serial_ports(device_root)
        if not candidates:
            raise RuntimeError("未发现串口，请连接设备或使用 --port /dev/ttyUSB0")
        if len(candidates) > 1:
            raise RuntimeError("发现多个串口，请使用 --port 指定：\n  " + "\n  ".join(candidates))
    selected = candidates[0]
    if not Path(selected).exists():
        raise RuntimeError(f"串口不存在: {selected}")
    if not access_check(selected, os.R_OK | os.W_OK):
        raise PermissionError(
            f"无权读写 {selected}。执行 sudo usermod -aG dialout $USER 后注销并重新登录。"
        )
    return selected


def choose_serial_port_dialog(root: Any, initial_port: Optional[str] = None) -> Optional[str]:
    """Show a modal Linux serial-port chooser and return the validated path."""

    if tk is None or ttk is None:
        raise RuntimeError("缺少 Tkinter，请安装 python3-tk")

    result: dict[str, Optional[str]] = {"port": None}
    dialog = tk.Toplevel(root)
    dialog.title("COM口设置")
    dialog.resizable(False, False)
    dialog.transient(root)

    body = ttk.Frame(dialog, padding=18)
    body.grid(row=0, column=0, sticky="nsew")
    body.columnconfigure(0, weight=1)
    ttk.Label(body, text="选择机械臂串口", font=("Noto Sans CJK SC", 12, "bold")).grid(
        row=0, column=0, columnspan=2, sticky="w", pady=(0, 10)
    )
    ttk.Label(body, text="COM口 / Linux 设备路径").grid(row=1, column=0, columnspan=2, sticky="w")

    port_value = tk.StringVar(value=initial_port or "")
    port_box = ttk.Combobox(body, textvariable=port_value, width=48, state="normal")
    port_box.grid(row=2, column=0, sticky="ew", pady=(6, 8), padx=(0, 8))
    status_value = tk.StringVar(value="")
    ttk.Label(
        body,
        textvariable=status_value,
        foreground="#526172",
        wraplength=620,
        justify="left",
    ).grid(
        row=3, column=0, columnspan=2, sticky="w", pady=(0, 12)
    )

    def refresh_ports() -> None:
        ports = discover_linux_serial_ports()
        port_box.configure(values=ports)
        if ports and not port_value.get().strip():
            port_value.set(ports[0])
        if ports:
            status_value.set(f"发现 {len(ports)} 个串口，也可以手动输入设备路径")
        else:
            status_value.set(serial_discovery_diagnostic())

    def accept() -> None:
        value = port_value.get().strip()
        if not value:
            if messagebox is not None:
                messagebox.showwarning("COM口设置", "请选择或输入串口设备路径", parent=dialog)
            return
        try:
            result["port"] = select_linux_serial_port(value)
        except (RuntimeError, PermissionError) as exc:
            if messagebox is not None:
                messagebox.showerror("COM口设置", str(exc), parent=dialog)
            return
        dialog.destroy()

    def cancel() -> None:
        result["port"] = None
        dialog.destroy()

    ttk.Button(body, text="刷新", command=refresh_ports).grid(row=2, column=1, sticky="ew", pady=(6, 8))
    buttons = ttk.Frame(body)
    buttons.grid(row=4, column=0, columnspan=2, sticky="e")
    ttk.Button(buttons, text="取消", command=cancel).pack(side="left", padx=(0, 8))
    ttk.Button(buttons, text="连接", command=accept).pack(side="left")
    dialog.protocol("WM_DELETE_WINDOW", cancel)
    dialog.bind("<Return>", lambda _event: accept())
    dialog.bind("<Escape>", lambda _event: cancel())

    refresh_ports()
    port_box.focus_set()
    dialog.update_idletasks()
    x = root.winfo_screenwidth() // 2 - dialog.winfo_reqwidth() // 2
    y = root.winfo_screenheight() // 2 - dialog.winfo_reqheight() // 2
    dialog.geometry(f"+{max(0, x)}+{max(0, y)}")
    dialog.lift()
    dialog.attributes("-topmost", True)
    dialog.after(500, lambda: dialog.attributes("-topmost", False))
    dialog.wait_visibility()
    dialog.grab_set()
    root.wait_window(dialog)
    return result["port"]


class OperationTerminalApp:
    FONT = "Noto Sans CJK SC"

    def __init__(
        self,
        root: Any,
        runtime: ManualServoRuntime,
        *,
        dry_run: bool = False,
        serial_port: Optional[str] = None,
    ) -> None:
        if tk is None or ttk is None:
            raise RuntimeError("缺少 Tkinter，请安装 python3-tk")
        self.root = root
        self.runtime = runtime
        self.dry_run = dry_run
        self.serial_port = serial_port
        self.active_releases: list[Callable[[], Any]] = []
        self.joystick_active = False
        self.joystick_center = 130
        self.joystick_radius = 112
        self.knob_radius = 25

        root.title("JetArm Ubuntu 操作终端")
        root.configure(bg="#f4f7fb")
        root.protocol("WM_DELETE_WINDOW", self.on_close)
        root.bind_all("<ButtonRelease-1>", self._global_release, add="+")
        self._configure_styles()
        self._build_layout()
        self._draw_joystick()
        self._schedule_tick()

    def _configure_styles(self) -> None:
        style = ttk.Style()
        style.configure("App.TFrame", background="#f4f7fb")
        style.configure("Panel.TFrame", background="#ffffff", relief="solid", borderwidth=1)
        style.configure("Title.TLabel", background="#f4f7fb", foreground="#142033", font=(self.FONT, 15, "bold"))
        style.configure("Port.TLabel", background="#f4f7fb", foreground="#40516a", font=(self.FONT, 10))
        style.configure("Panel.TLabel", background="#ffffff", foreground="#142033", font=(self.FONT, 11, "bold"))
        style.configure("Action.TButton", font=(self.FONT, 11), padding=(12, 10))
        style.configure("Stop.TButton", font=(self.FONT, 12, "bold"), padding=(14, 11))

    def _build_layout(self) -> None:
        outer = ttk.Frame(self.root, style="App.TFrame", padding=16)
        outer.grid(row=0, column=0, sticky="nsew")
        self.root.rowconfigure(0, weight=1)
        self.root.columnconfigure(0, weight=1)
        outer.columnconfigure(1, weight=1)

        title = "JetArm Ubuntu 操作终端" + ("  [DRY-RUN]" if self.dry_run else "")
        ttk.Label(outer, text=title, style="Title.TLabel").grid(
            row=0, column=0, columnspan=3, sticky="w", pady=(0, 12)
        )
        port_text = "模拟模式" if self.dry_run else (self.serial_port or "未连接")
        ttk.Label(outer, text=f"COM口: {port_text}", style="Port.TLabel").grid(
            row=0, column=3, sticky="e", pady=(0, 12)
        )
        self._vertical_panel(outer).grid(row=1, column=0, sticky="nsew", padx=(0, 10))
        self._joystick_panel(outer).grid(row=1, column=1, sticky="nsew", padx=(0, 10))
        self._j5_panel(outer).grid(row=1, column=2, sticky="nsew", padx=(0, 10))
        self._j6_panel(outer).grid(row=1, column=3, sticky="nsew")
        self._status_panel(outer).grid(row=2, column=0, columnspan=4, sticky="nsew", pady=(10, 0))
        outer.rowconfigure(2, weight=1)

    def _panel(self, parent: Any, title: str) -> Any:
        frame = ttk.Frame(parent, style="Panel.TFrame", padding=12)
        frame.columnconfigure(0, weight=1)
        ttk.Label(frame, text=title, style="Panel.TLabel").grid(row=0, column=0, sticky="w", pady=(0, 10))
        return frame

    def _vertical_panel(self, parent: Any) -> Any:
        frame = self._panel(parent, "上下")
        up = ttk.Button(frame, text="▲  上", style="Action.TButton")
        up.grid(row=1, column=0, sticky="ew", pady=(8, 8), ipady=18)
        self._bind_hold(up, lambda: self.runtime.set_vertical_direction(1), lambda: self.runtime.set_vertical_direction(0))
        down = ttk.Button(frame, text="▼  下", style="Action.TButton")
        down.grid(row=2, column=0, sticky="ew", pady=(0, 18), ipady=18)
        self._bind_hold(down, lambda: self.runtime.set_vertical_direction(-1), lambda: self.runtime.set_vertical_direction(0))
        ttk.Button(frame, text="Home", style="Action.TButton", command=lambda: self._safe_call(self.runtime.go_home)).grid(
            row=3, column=0, sticky="ew", pady=(16, 0), ipady=10
        )
        return frame

    def _joystick_panel(self, parent: Any) -> Any:
        frame = self._panel(parent, "前后左右")
        size = self.joystick_center * 2
        self.joystick = tk.Canvas(frame, width=size, height=size, bg="#ffffff", highlightthickness=0)
        self.joystick.grid(row=1, column=0)
        self.joystick.bind("<ButtonPress-1>", self._move_joystick)
        self.joystick.bind("<B1-Motion>", self._move_joystick)
        self.joystick.bind("<ButtonRelease-1>", lambda _event: self._release_joystick())
        return frame

    def _j5_panel(self, parent: Any) -> Any:
        frame = self._panel(parent, "J5")
        ccw = ttk.Button(frame, text="↶  逆时针", style="Action.TButton")
        ccw.grid(row=1, column=0, sticky="ew", pady=(22, 10), ipady=18)
        self._bind_hold(ccw, self.runtime.rotate_j5_counterclockwise, self.runtime.stop_j5)
        cw = ttk.Button(frame, text="↷  顺时针", style="Action.TButton")
        cw.grid(row=2, column=0, sticky="ew", ipady=18)
        self._bind_hold(cw, self.runtime.rotate_j5_clockwise, self.runtime.stop_j5)
        return frame

    def _j6_panel(self, parent: Any) -> Any:
        frame = self._panel(parent, "J6")
        open_button = ttk.Button(frame, text="松", style="Action.TButton")
        open_button.grid(row=1, column=0, sticky="ew", pady=(10, 8), ipady=15)
        self._bind_hold(open_button, self.runtime.open_j6, self.runtime.stop_j6)
        close_button = ttk.Button(frame, text="闭", style="Action.TButton")
        close_button.grid(row=2, column=0, sticky="ew", pady=(0, 14), ipady=15)
        self._bind_hold(close_button, self.runtime.close_j6, self.runtime.stop_j6)
        self.grip_button = tk.Button(
            frame,
            text="抓紧",
            command=self._toggle_grip,
            bg="#1f9d61",
            fg="#ffffff",
            activebackground="#187d4e",
            relief="flat",
            font=(self.FONT, 12, "bold"),
            padx=12,
            pady=13,
        )
        self.grip_button.grid(row=3, column=0, sticky="ew", pady=(8, 0))
        return frame

    def _status_panel(self, parent: Any) -> Any:
        frame = ttk.Frame(parent, style="Panel.TFrame", padding=12)
        frame.columnconfigure(0, weight=1)
        ttk.Button(frame, text="全部停止", style="Stop.TButton", command=self._stop_all).grid(row=0, column=0, sticky="w")
        self.velocity_label = ttk.Label(frame, text="", style="Panel.TLabel")
        self.velocity_label.grid(row=1, column=0, sticky="w", pady=(10, 6))
        self.log_text = tk.Text(
            frame,
            height=7,
            bg="#101827",
            fg="#dce6f5",
            insertbackground="#ffffff",
            relief="flat",
            font=("DejaVu Sans Mono", 9),
        )
        self.log_text.grid(row=2, column=0, sticky="nsew")
        frame.rowconfigure(2, weight=1)
        return frame

    def append_log(self, message: str) -> None:
        self.log_text.insert("end", f"[{time.strftime('%H:%M:%S')}] {message}\n")
        self.log_text.see("end")

    def _bind_hold(self, button: Any, on_press: Callable[[], Any], on_release: Callable[[], Any]) -> None:
        def press(_event: Any) -> None:
            if on_release not in self.active_releases:
                self.active_releases.append(on_release)
            self._safe_call(on_press)

        button.bind("<ButtonPress-1>", press)
        button.bind("<ButtonRelease-1>", lambda _event: self._release_hold(on_release))

    def _release_hold(self, release: Callable[[], Any]) -> None:
        if release in self.active_releases:
            self.active_releases.remove(release)
            self._safe_call(release)

    def _global_release(self, _event: Any) -> None:
        for release in list(self.active_releases):
            self._release_hold(release)
        if self.joystick_active:
            self._release_joystick()

    def _move_joystick(self, event: Any) -> None:
        self.joystick_active = True
        c = self.joystick_center
        r = self.joystick_radius
        self.runtime.set_joystick((event.x - c) / r, (event.y - c) / r)
        self._draw_joystick()
        self._refresh_velocity()

    def _release_joystick(self) -> None:
        self.joystick_active = False
        self.runtime.center_joystick()
        self._draw_joystick()
        self._refresh_velocity()

    def _draw_joystick(self) -> None:
        c = self.joystick_center
        r = self.joystick_radius
        self.joystick.delete("all")
        self.joystick.create_oval(c - r, c - r, c + r, c + r, fill="#edf2f7", outline="#8f9caf", width=2)
        self.joystick.create_line(c, c - r, c, c + r, fill="#c6d0dd", width=2)
        self.joystick.create_line(c - r, c, c + r, c, fill="#c6d0dd", width=2)
        font = (self.FONT, 13, "bold")
        self.joystick.create_text(c, c - r + 20, text="前", fill="#142033", font=font)
        self.joystick.create_text(c, c + r - 20, text="后", fill="#142033", font=font)
        self.joystick.create_text(c - r + 22, c, text="左", fill="#142033", font=font)
        self.joystick.create_text(c + r - 22, c, text="右", fill="#142033", font=font)
        x = c + self.runtime.joystick_x * r
        y = c + self.runtime.joystick_y * r
        k = self.knob_radius
        self.joystick.create_oval(x - k, y - k, x + k, y + k, fill="#2869df", outline="#153d8b", width=2)

    def _toggle_grip(self) -> None:
        self._safe_call(self.runtime.toggle_grip_lock)
        self._update_grip_color()

    def _update_grip_color(self) -> None:
        if self.runtime.j6_grip_locked:
            self.grip_button.configure(bg="#d32f2f", activebackground="#ad2424")
        else:
            self.grip_button.configure(bg="#1f9d61", activebackground="#187d4e")

    def _stop_all(self) -> None:
        self._safe_call(self.runtime.stop_all)
        self._release_joystick()
        self._update_grip_color()

    def _safe_call(self, action: Callable[[], Any]) -> None:
        try:
            action()
            self._refresh_velocity()
        except Exception as exc:
            self.append_log(f"ERROR: {exc}")
            if messagebox is not None:
                messagebox.showerror("JetArm Ubuntu 操作终端", str(exc))

    def _refresh_velocity(self) -> None:
        velocity = self.runtime.cartesian_velocity() * 100.0
        self.velocity_label.configure(
            text=f"TCP 速度  前 {velocity[0]:+.1f}  左 {velocity[1]:+.1f}  上 {velocity[2]:+.1f} cm/s"
        )

    def _schedule_tick(self) -> None:
        try:
            self.runtime.tick()
            self._refresh_velocity()
        except Exception as exc:
            self.runtime.vertical_direction = 0
            self.runtime.center_joystick()
            self.append_log(f"ERROR: {exc}")
            if messagebox is not None:
                messagebox.showerror("JetArm 控制错误", str(exc))
        self.root.after(max(10, int(self.runtime.settings.tick_s * 1000)), self._schedule_tick)

    def on_close(self) -> None:
        try:
            self.runtime.close()
        finally:
            self.root.destroy()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="JetArm operation terminal for Ubuntu 22.04")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG_PATH), help="terminal JSON config")
    parser.add_argument("--port", default=None, help="serial device, for example /dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=None, help="override configured baudrate")
    parser.add_argument("--timeout", type=float, default=None, help="override serial timeout in seconds")
    parser.add_argument("--dry-run", action="store_true", help="open the UI without a real serial device")
    parser.add_argument("--list-ports", action="store_true", help="list detected USB serial devices and exit")
    parser.add_argument(
        "--diagnose-ports",
        action="store_true",
        help="diagnose USB-visible devices that have no Linux tty node",
    )
    return parser


def main(argv: Optional[list[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    if not sys.platform.startswith("linux"):
        print("WARNING: this standalone version is intended for Ubuntu 22.04", file=sys.stderr)

    if args.list_ports:
        ports = discover_linux_serial_ports()
        if not ports:
            print(serial_discovery_diagnostic())
            return 1
        print("\n".join(ports))
        return 0

    if args.diagnose_ports:
        print(serial_discovery_diagnostic())
        print("\n建议依次执行：")
        print("  lsusb -nn | grep -i -E '1a86|CH340|CH341'")
        print("  lsmod | grep ch341")
        print("  sudo modprobe ch341")
        print("  sudo dmesg | tail -n 80")
        print("  ls -l /dev/ttyUSB* /dev/serial/by-id/* 2>/dev/null")
        print("若dmesg显示brltty抢占，且不使用盲文设备，请检查brltty服务。")
        return 0

    runtime: ManualServoRuntime | None = None
    try:
        settings = TerminalSettings.from_file(args.config)
        if tk is None:
            raise RuntimeError("缺少 Tkinter，请执行 sudo apt install python3-tk")
        root = tk.Tk()
        app_holder: dict[str, OperationTerminalApp] = {}

        def logger(message: str) -> None:
            app = app_holder.get("app")
            if app is None:
                print(message)
            else:
                app.append_log(message)

        selected_port: Optional[str] = None
        if args.dry_run:
            controller: Any = DryRunServoController(settings, logger=logger)
        else:
            if args.port:
                selected_port = select_linux_serial_port(args.port)
            else:
                root.title("JetArm Ubuntu 操作终端")
                root.geometry("720x480")
                selected_port = choose_serial_port_dialog(root)
                if selected_port is None:
                    root.destroy()
                    return 0
                root.geometry("1080x680")
            controller = BusServoController(
                selected_port,
                args.baudrate or settings.baudrate,
                args.timeout if args.timeout is not None else settings.timeout_s,
            )
        runtime = ManualServoRuntime(controller, settings, logger=logger)
        runtime.initialize(use_home_positions=args.dry_run)
        app_holder["app"] = OperationTerminalApp(
            root,
            runtime,
            dry_run=args.dry_run,
            serial_port=selected_port,
        )
        root.mainloop()
        return 0
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    finally:
        if runtime is not None:
            runtime.close()


if __name__ == "__main__":
    raise SystemExit(main())
