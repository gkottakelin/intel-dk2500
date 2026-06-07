"""Direct bus-servo control helpers for the JetArm project.

Protocol source: "02 总线舵机通信协议.pdf".
The bus uses half-duplex UART at 115200 bps by default.
"""

from __future__ import annotations

import argparse
import struct
import time
from dataclasses import dataclass
from typing import Callable, Optional, Protocol


HEADER = b"\x55\x55"
DEFAULT_BAUDRATE = 115200
DEFAULT_TIMEOUT = 0.2

SERVO_MOVE_TIME_WRITE = 1
SERVO_TEMP_READ = 26
SERVO_VIN_READ = 27
SERVO_POS_READ = 28
SERVO_OR_MOTOR_MODE_WRITE = 29

MIN_SERVO_ID = 0
MAX_SERVO_ID = 253
BROADCAST_ID = 254
MIN_POSITION = 0
MAX_POSITION = 1000
MIN_RUN_TIME_MS = 0
MAX_RUN_TIME_MS = 30000
MIN_DUTY_SPEED = -1000
MAX_DUTY_SPEED = 1000
MIN_FIXED_SPEED = -50
MAX_FIXED_SPEED = 50


class SerialLike(Protocol):
    def write(self, data: bytes) -> int:
        ...

    def read(self, size: int = 1) -> bytes:
        ...

    def close(self) -> None:
        ...

    def flush(self) -> None:
        ...


@dataclass(frozen=True)
class ServoStatus:
    """Current status returned by the servo read commands."""

    servo_id: int
    temperature_c: int
    position: int
    voltage_mv: int


class BusServoError(Exception):
    """Base exception for bus-servo operations."""


class BusServoTimeoutError(BusServoError):
    """Raised when a servo response is not received before timeout."""


class BusServoPacketError(BusServoError):
    """Raised when a servo response frame is malformed."""


def _validate_servo_id(servo_id: int, *, allow_broadcast: bool = False) -> None:
    max_id = BROADCAST_ID if allow_broadcast else MAX_SERVO_ID
    if not MIN_SERVO_ID <= servo_id <= max_id:
        extra = "，广播 ID 254 仅用于写入命令" if not allow_broadcast else ""
        raise ValueError(f"舵机 ID 必须在 0..{max_id} 范围内{extra}")


def _validate_u16(value: int, name: str, minimum: int, maximum: int) -> None:
    if not minimum <= value <= maximum:
        raise ValueError(f"{name} 必须在 {minimum}..{maximum} 范围内")


def _validate_i16(value: int, name: str, minimum: int, maximum: int) -> None:
    if not minimum <= value <= maximum:
        raise ValueError(f"{name} 必须在 {minimum}..{maximum} 范围内")


def checksum(body_without_header_and_checksum: bytes) -> int:
    """Return the protocol checksum for ID + Length + Cmd + Params."""

    return (~sum(body_without_header_and_checksum)) & 0xFF


def build_packet(servo_id: int, command: int, params: bytes = b"") -> bytes:
    """Build a complete command frame."""

    _validate_servo_id(servo_id, allow_broadcast=True)
    if not 0 <= command <= 0xFF:
        raise ValueError("命令号必须在 0..255 范围内")
    if len(params) > 252:
        raise ValueError("参数区过长")

    length = len(params) + 3
    body = bytes((servo_id, length, command)) + params
    return HEADER + body + bytes((checksum(body),))


def parse_packet(frame: bytes, *, expected_id: Optional[int] = None, expected_cmd: Optional[int] = None) -> tuple[int, int, bytes]:
    """Parse and validate a complete response frame.

    Returns:
        ``(servo_id, command, params)``.
    """

    if len(frame) < 6:
        raise BusServoPacketError("响应帧长度不足")
    if frame[:2] != HEADER:
        raise BusServoPacketError("响应帧头不是 0x55 0x55")

    servo_id = frame[2]
    length = frame[3]
    command = frame[4]
    expected_total = length + 3
    if len(frame) != expected_total:
        raise BusServoPacketError(f"响应帧长度不匹配：收到 {len(frame)} 字节，应为 {expected_total} 字节")
    if checksum(frame[2:-1]) != frame[-1]:
        raise BusServoPacketError("响应帧校验和错误")
    if expected_id is not None and servo_id != expected_id:
        raise BusServoPacketError(f"响应舵机 ID 不匹配：收到 {servo_id}，期望 {expected_id}")
    if expected_cmd is not None and command != expected_cmd:
        raise BusServoPacketError(f"响应命令不匹配：收到 {command}，期望 {expected_cmd}")

    return servo_id, command, frame[5:-1]


def _u16(value: int) -> bytes:
    return struct.pack("<H", value)


def _i16(value: int) -> bytes:
    return struct.pack("<h", value)


def _read_exact(port: SerialLike, size: int, timeout_s: float) -> bytes:
    deadline = time.monotonic() + timeout_s
    data = bytearray()
    while len(data) < size:
        if time.monotonic() > deadline:
            raise BusServoTimeoutError(f"串口读取超时：需要 {size} 字节，已收到 {len(data)} 字节")
        chunk = port.read(size - len(data))
        if chunk:
            data.extend(chunk)
    return bytes(data)


def _read_response(port: SerialLike, expected_id: int, expected_cmd: int, timeout_s: float) -> bytes:
    deadline = time.monotonic() + timeout_s
    previous = b""
    while time.monotonic() <= deadline:
        current = port.read(1)
        if not current:
            continue
        if previous == b"\x55" and current == b"\x55":
            id_and_length = _read_exact(port, 2, max(0.001, deadline - time.monotonic()))
            length = id_and_length[1]
            if length < 3:
                raise BusServoPacketError(f"响应长度字段非法：{length}")
            rest = _read_exact(port, length - 1, max(0.001, deadline - time.monotonic()))
            frame = HEADER + id_and_length + rest
            parse_packet(frame, expected_id=expected_id, expected_cmd=expected_cmd)
            return frame
        previous = current
    raise BusServoTimeoutError("等待响应帧头超时")


def _open_serial(com_port: str, baudrate: int, timeout_s: float) -> SerialLike:
    try:
        import serial  # type: ignore
    except ImportError as exc:
        raise RuntimeError("缺少 pyserial，请先安装：pip install pyserial") from exc
    return serial.Serial(com_port, baudrate=baudrate, timeout=timeout_s)


class BusServoController:
    """Controller for direct bus-servo UART communication."""

    def __init__(
        self,
        com_port: str,
        *,
        baudrate: int = DEFAULT_BAUDRATE,
        timeout: float = DEFAULT_TIMEOUT,
        serial_factory: Optional[Callable[[str, int, float], SerialLike]] = None,
    ) -> None:
        self.com_port = com_port
        self.baudrate = baudrate
        self.timeout = timeout
        self._serial_factory = serial_factory or _open_serial
        self._port: Optional[SerialLike] = None

    def __enter__(self) -> "BusServoController":
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    @property
    def port(self) -> SerialLike:
        if self._port is None:
            self.open()
        assert self._port is not None
        return self._port

    def open(self) -> None:
        if self._port is None:
            self._port = self._serial_factory(self.com_port, self.baudrate, self.timeout)

    def close(self) -> None:
        if self._port is not None:
            self._port.close()
            self._port = None

    def write_command(self, servo_id: int, command: int, params: bytes = b"", *, allow_broadcast: bool = False) -> None:
        _validate_servo_id(servo_id, allow_broadcast=allow_broadcast)
        frame = build_packet(servo_id, command, params)
        self.port.write(frame)
        self.port.flush()

    def read_command(self, servo_id: int, command: int, expected_param_len: int) -> bytes:
        _validate_servo_id(servo_id)
        self.write_command(servo_id, command)
        response = _read_response(self.port, servo_id, command, self.timeout)
        _, _, params = parse_packet(response, expected_id=servo_id, expected_cmd=command)
        if len(params) != expected_param_len:
            raise BusServoPacketError(f"响应参数长度不匹配：收到 {len(params)}，期望 {expected_param_len}")
        return params

    def read_temperature(self, servo_id: int) -> int:
        """Read current servo temperature in Celsius."""

        return self.read_command(servo_id, SERVO_TEMP_READ, 1)[0]

    def read_position(self, servo_id: int) -> int:
        """Read current servo position. The value is a signed 16-bit integer."""

        params = self.read_command(servo_id, SERVO_POS_READ, 2)
        return struct.unpack("<h", params)[0]

    def read_voltage(self, servo_id: int) -> int:
        """Read current servo input voltage in millivolts."""

        params = self.read_command(servo_id, SERVO_VIN_READ, 2)
        return struct.unpack("<H", params)[0]

    def read_status(self, servo_id: int) -> ServoStatus:
        """Read temperature, position, and voltage from one servo."""

        return ServoStatus(
            servo_id=servo_id,
            temperature_c=self.read_temperature(servo_id),
            position=self.read_position(servo_id),
            voltage_mv=self.read_voltage(servo_id),
        )

    def move_servo(self, servo_id: int, target_position: int, run_time_ms: int) -> None:
        """Set position-control mode target.

        Args:
            servo_id: Servo ID, 0..253. Use 254 only for broadcast writes.
            target_position: 0..1000, corresponding to 0..240 degrees.
            run_time_ms: Move duration, 0..30000 ms.
        """

        _validate_servo_id(servo_id, allow_broadcast=True)
        _validate_u16(target_position, "目标位置", MIN_POSITION, MAX_POSITION)
        _validate_u16(run_time_ms, "运行时间", MIN_RUN_TIME_MS, MAX_RUN_TIME_MS)
        params = _u16(target_position) + _u16(run_time_ms)
        self.write_command(servo_id, SERVO_MOVE_TIME_WRITE, params, allow_broadcast=True)

    def set_servo_mode(self, servo_id: int) -> None:
        """Switch servo to position-control mode."""

        _validate_servo_id(servo_id, allow_broadcast=True)
        params = bytes((0, 0)) + _i16(0)
        self.write_command(servo_id, SERVO_OR_MOTOR_MODE_WRITE, params, allow_broadcast=True)

    def set_motor_speed(self, servo_id: int, speed: int, *, fixed_speed_mode: bool = False) -> None:
        """Switch servo to motor mode and set speed.

        By default this uses duty mode, where speed is -1000..1000.
        If ``fixed_speed_mode=True``, the speed range is -50..50.
        """

        _validate_servo_id(servo_id, allow_broadcast=True)
        if fixed_speed_mode:
            _validate_i16(speed, "固定速度", MIN_FIXED_SPEED, MAX_FIXED_SPEED)
        else:
            _validate_i16(speed, "占空比速度", MIN_DUTY_SPEED, MAX_DUTY_SPEED)
        mode = 1
        turn_mode = 1 if fixed_speed_mode else 0
        params = bytes((mode, turn_mode)) + _i16(speed)
        self.write_command(servo_id, SERVO_OR_MOTOR_MODE_WRITE, params, allow_broadcast=True)


def read_servo_status(com_port: str, servo_id: int, *, baudrate: int = DEFAULT_BAUDRATE) -> ServoStatus:
    """Convenience interface: read temperature, angle/position, and voltage."""

    with BusServoController(com_port, baudrate=baudrate) as controller:
        return controller.read_status(servo_id)


def set_servo_position(
    com_port: str,
    servo_id: int,
    target_position: int,
    run_time_ms: int,
    *,
    baudrate: int = DEFAULT_BAUDRATE,
) -> None:
    """Convenience interface: position-control mode movement."""

    with BusServoController(com_port, baudrate=baudrate) as controller:
        controller.move_servo(servo_id, target_position, run_time_ms)


def set_motor_speed(com_port: str, servo_id: int, speed: int, *, baudrate: int = DEFAULT_BAUDRATE) -> None:
    """Convenience interface: motor mode duty-speed command."""

    with BusServoController(com_port, baudrate=baudrate) as controller:
        controller.set_motor_speed(servo_id, speed)


def _build_cli() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="JetArm 总线舵机调试工具")
    sub = parser.add_subparsers(dest="command", required=True)

    status = sub.add_parser("status", help="读取温度、位置、电压")
    status.add_argument("com_port")
    status.add_argument("servo_id", type=int)

    servo = sub.add_parser("servo", help="servo 模式：设置目标位置和运行时间")
    servo.add_argument("com_port")
    servo.add_argument("servo_id", type=int)
    servo.add_argument("target_position", type=int)
    servo.add_argument("run_time_ms", type=int)

    motor = sub.add_parser("motor", help="motor 模式：设置速度")
    motor.add_argument("com_port")
    motor.add_argument("servo_id", type=int)
    motor.add_argument("speed", type=int)
    motor.add_argument("--fixed-speed-mode", action="store_true", help="使用固定速度模式，速度范围 -50..50")

    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE, help="串口速率，默认 115200")
    return parser


def main() -> None:
    args = _build_cli().parse_args()
    if args.command == "status":
        status = read_servo_status(args.com_port, args.servo_id, baudrate=args.baudrate)
        print(status)
    elif args.command == "servo":
        set_servo_position(args.com_port, args.servo_id, args.target_position, args.run_time_ms, baudrate=args.baudrate)
    elif args.command == "motor":
        with BusServoController(args.com_port, baudrate=args.baudrate) as controller:
            controller.set_motor_speed(args.servo_id, args.speed, fixed_speed_mode=args.fixed_speed_mode)


if __name__ == "__main__":
    main()
