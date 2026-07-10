"""Safe AI tool adapter for the existing Ubuntu JetArm operation terminal."""

from __future__ import annotations

import asyncio
import importlib
import math
import re
import sys
import time
from collections.abc import Awaitable, Callable, Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from .tooling import ToolDefinition, ToolRegistry


DEFAULT_TERMINAL_CONFIG = (
    Path(__file__).resolve().parents[2]
    / "ubuntu22_04_operation_terminal"
    / "config"
    / "terminal.json"
)
ARM_JOINTS = ("J1", "J2", "J3", "J4")
DEFAULT_TCP_SPEED_CM_S = 1.5
MIN_TCP_SPEED_CM_S = 1.0
MAX_TCP_SPEED_CM_S = 5.0
MAX_AGENT_MOVE_COMMAND_CM = 2.0
CAMERA_VECTOR_VERSIONS = ("v1", "v2")
DEFAULT_GRIPPER_RELEASE_POSITION = 370
DEFAULT_GRIPPER_POSITION_RUN_TIME_MS = 500
MIN_PIXEL_ALIGNMENT_SPEED_CM_S = 0.7
MAX_PIXEL_ALIGNMENT_SPEED_CM_S = 1.5
DEFAULT_PIXEL_ALIGNMENT_TOLERANCE_PX = 10.0
DEFAULT_PIXEL_ALIGNMENT_STEP_DURATION_S = 0.4
DEFAULT_PIXEL_ALIGNMENT_SPEED_SATURATION_PX = 120.0
PIXEL_ALIGNMENT_SCALE_NEAR_HEIGHT_CM = 2.0
PIXEL_ALIGNMENT_SCALE_NEAR_PX_PER_CM = 50.0
PIXEL_ALIGNMENT_SCALE_FAR_HEIGHT_CM = 25.0
PIXEL_ALIGNMENT_SCALE_FAR_PX_PER_CM = 18.0
DEFAULT_DESCENT_RECALIBRATION_CM = 2.0
FINAL_ALIGNMENT_THRESHOLD_CM = 2.0
FINAL_GRASP_HEIGHT_CM = 1.0
DESCENT_SPEED_CM_S = 2.0
MANUAL_V2_HORIZONTAL_MAX_Z_CHANGE_CM = 1.0
MANUAL_V2_HORIZONTAL_MAX_ANGLE_ERROR_DEG = 3.0
SleepFunction = Callable[[float], Awaitable[None]]


class ArmControlError(RuntimeError):
    """Raised when a requested arm action violates safety or cannot execute."""


def pixel_alignment_px_per_cm_for_height(height_cm: object) -> float:
    if isinstance(height_cm, bool):
        raise ArmControlError("height_cm must be a number")
    try:
        height = float(height_cm)
    except (TypeError, ValueError) as exc:
        raise ArmControlError("height_cm must be a number") from exc
    if not math.isfinite(height):
        raise ArmControlError("height_cm must be finite")

    slope = (
        PIXEL_ALIGNMENT_SCALE_FAR_PX_PER_CM
        - PIXEL_ALIGNMENT_SCALE_NEAR_PX_PER_CM
    ) / (
        PIXEL_ALIGNMENT_SCALE_FAR_HEIGHT_CM
        - PIXEL_ALIGNMENT_SCALE_NEAR_HEIGHT_CM
    )
    px_per_cm = PIXEL_ALIGNMENT_SCALE_NEAR_PX_PER_CM + (
        height - PIXEL_ALIGNMENT_SCALE_NEAR_HEIGHT_CM
    ) * slope
    if px_per_cm <= 0:
        raise ArmControlError(
            "height_cm is outside the calibrated positive pixel-scale range"
        )
    return px_per_cm


def manual_v2_horizontal_progress_validation(
    delta_x_cm: float,
    delta_y_cm: float,
    delta_z_cm: float,
    camera_line_angle_error_deg: float,
) -> dict[str, Any]:
    """Apply the relaxed manual-V2 horizontal progress acceptance rule."""

    xy_change_cm = math.hypot(float(delta_x_cm), float(delta_y_cm))
    z_change_cm = abs(float(delta_z_cm))
    angle_error_deg = abs(float(camera_line_angle_error_deg))
    z_within_limit = z_change_cm < MANUAL_V2_HORIZONTAL_MAX_Z_CHANGE_CM
    horizontal_dominates = z_change_cm < xy_change_cm
    angle_within_limit = (
        angle_error_deg < MANUAL_V2_HORIZONTAL_MAX_ANGLE_ERROR_DEG
    )
    return {
        "rule": "manual_v2_relaxed_horizontal_progress",
        "accepted": (
            z_within_limit and horizontal_dominates and angle_within_limit
        ),
        "xy_change_cm": round(xy_change_cm, 6),
        "z_change_cm": round(z_change_cm, 6),
        "z_change_limit_cm_exclusive": MANUAL_V2_HORIZONTAL_MAX_Z_CHANGE_CM,
        "z_change_less_than_xy_change": horizontal_dominates,
        "camera_line_angle_error_deg": round(angle_error_deg, 6),
        "camera_line_angle_error_limit_deg_exclusive": (
            MANUAL_V2_HORIZONTAL_MAX_ANGLE_ERROR_DEG
        ),
        "conditions": {
            "z_change_within_limit": z_within_limit,
            "horizontal_change_dominates": horizontal_dominates,
            "camera_line_angle_within_limit": angle_within_limit,
        },
    }


@dataclass(frozen=True)
class ArmControlConfig:
    mode: str = "dry-run"
    serial_port: str | None = None
    terminal_config_path: Path = DEFAULT_TERMINAL_CONFIG
    max_distance_cm: float = MAX_AGENT_MOVE_COMMAND_CM
    allow_extended_distance: bool = False
    fixed_pixel_alignment_distance_cm: float | None = None
    default_speed_cm_s: float = DEFAULT_TCP_SPEED_CM_S
    min_speed_cm_s: float = MIN_TCP_SPEED_CM_S
    max_speed_cm_s: float = MAX_TCP_SPEED_CM_S
    max_motor_duration_s: float = 2.0
    camera_vector_version: str = "v1"

    def validate(self) -> None:
        if self.mode not in {"dry-run", "hardware"}:
            raise ArmControlError(f"机械臂模式必须是dry-run或hardware，收到: {self.mode}")
        if not math.isfinite(self.max_distance_cm) or self.max_distance_cm <= 0:
            raise ArmControlError("max_distance_cm必须是大于0的有限数字")
        if (
            self.max_distance_cm > MAX_AGENT_MOVE_COMMAND_CM
            and not self.allow_extended_distance
        ):
            raise ArmControlError(
                f"max_distance_cm不能超过{MAX_AGENT_MOVE_COMMAND_CM:g}；"
                f"Agent每条移动命令必须严格小于{MAX_AGENT_MOVE_COMMAND_CM:g}cm"
            )
        if self.fixed_pixel_alignment_distance_cm is not None:
            fixed_distance = float(self.fixed_pixel_alignment_distance_cm)
            if not math.isfinite(fixed_distance) or fixed_distance <= 0:
                raise ArmControlError(
                    "fixed_pixel_alignment_distance_cm must be greater than 0"
                )
            if fixed_distance > self.max_distance_cm:
                raise ArmControlError(
                    "fixed_pixel_alignment_distance_cm must be <= max_distance_cm"
                )
        if not 0 < self.min_speed_cm_s <= self.default_speed_cm_s <= self.max_speed_cm_s:
            raise ArmControlError("TCP速度配置必须满足0 < 最小值 <= 默认值 <= 最大值")
        if self.max_motor_duration_s <= 0:
            raise ArmControlError("max_motor_duration_s必须大于0")
        if self.camera_vector_version not in CAMERA_VECTOR_VERSIONS:
            raise ArmControlError(
                "camera_vector_version必须是v1或v2，收到: "
                f"{self.camera_vector_version}"
            )


def _load_terminal_module(camera_vector_version: str = "v1") -> Any:
    import_error: Exception | None = None
    terminal_dir = DEFAULT_TERMINAL_CONFIG.parents[1]
    terminal_dir_text = str(terminal_dir)
    if terminal_dir_text not in sys.path:
        sys.path.insert(0, terminal_dir_text)
    if camera_vector_version not in CAMERA_VECTOR_VERSIONS:
        raise ArmControlError(
            "camera_vector_version必须是v1或v2，收到: "
            f"{camera_vector_version}"
        )
    module_leaf = (
        "camera_vector_terminal_v2"
        if camera_vector_version == "v2"
        else "camera_vector_terminal"
    )
    for module_name in (
        module_leaf,
        f"ubuntu22_04_operation_terminal.{module_leaf}",
        f"project.ubuntu22_04_operation_terminal.{module_leaf}",
    ):
        try:
            return importlib.import_module(module_name)
        except (ImportError, ModuleNotFoundError) as exc:
            import_error = exc
    raise ArmControlError(
        "机械臂工具依赖加载失败，请执行: "
        "python -m pip install -r requirements-ai.txt"
    ) from import_error


def choose_arm_serial_port(initial_port: str | None = None) -> str | None:
    """Open the standalone Ubuntu terminal's modal serial-port chooser."""

    terminal = _load_terminal_module()
    if terminal.tk is None:
        raise ArmControlError("缺少Tkinter，请执行: sudo apt install python3-tk")
    try:
        root = terminal.tk.Tk()
    except Exception as exc:
        raise ArmControlError(f"无法打开串口选择窗口: {exc}") from exc
    root.withdraw()
    try:
        return terminal.choose_serial_port_dialog(root, initial_port)
    finally:
        try:
            root.destroy()
        except Exception:
            # Closing the child dialog can already destroy the Tcl application.
            pass


COMPACT_DIRECTION_NAMES = {
    "前": "forward",
    "后": "backward",
    "左": "left",
    "右": "right",
    "上": "up",
    "下": "down",
}
COMPACT_DIRECTION_LABELS = {value: key for key, value in COMPACT_DIRECTION_NAMES.items()}


def parse_compact_arm_command(command: str) -> tuple[str, float]:
    """Parse compact MCP commands such as ``前5`` or ``上2.5cm``."""

    match = re.fullmatch(
        r"\s*(前|后|左|右|上|下)\s*(\d+(?:\.\d+)?)\s*(?:厘米|cm)?\s*",
        str(command),
        flags=re.IGNORECASE,
    )
    if match is None:
        raise ArmControlError("机械臂移动命令格式错误，应为前5、后2或上1.5cm")
    distance = float(match.group(2))
    if distance <= 0:
        raise ArmControlError("移动距离必须大于0")
    return COMPACT_DIRECTION_NAMES[match.group(1)], distance


def format_compact_arm_command(direction: str, distance_cm: float) -> str:
    try:
        label = COMPACT_DIRECTION_LABELS[direction]
    except KeyError as exc:
        raise ArmControlError(f"不支持的TCP方向: {direction}") from exc
    return f"{label}{distance_cm:g}"


class JetArmToolController:
    """Distance-based wrapper around ``ubuntu22_04_operation_terminal``."""

    def __init__(
        self,
        config: ArmControlConfig,
        *,
        sleep: SleepFunction = asyncio.sleep,
        logger: Callable[[str], None] | None = None,
    ) -> None:
        config.validate()
        self.config = config
        self.sleep = sleep
        self.logger = logger or (lambda _message: None)
        self.closed = False

        terminal = _load_terminal_module(config.camera_vector_version)

        self.terminal = terminal
        self.settings = terminal.TerminalSettings.from_file(
            config.terminal_config_path
        )
        self.serial_port: str | None = None
        if config.mode == "dry-run":
            self.controller = terminal.DryRunServoController(
                self.settings, logger=self.logger
            )
        else:
            try:
                self.serial_port = terminal.select_linux_serial_port(
                    config.serial_port
                )
            except Exception as exc:
                raise ArmControlError(f"无法选择机械臂串口: {exc}") from exc
            self.controller = terminal.BusServoController(
                self.serial_port,
                self.settings.baudrate,
                self.settings.timeout_s,
            )

        camera_config_cls = getattr(terminal, "CameraLineConfig", None)
        if config.camera_vector_version == "v2":
            runtime_cls = getattr(terminal, "CameraVectorV2Runtime", None)
            if runtime_cls is None:
                raise ArmControlError(
                    "camera_vector_terminal_v2缺少CameraVectorV2Runtime"
                )
        else:
            runtime_cls = getattr(
                terminal,
                "CameraRelativeManualServoRuntime",
                terminal.ManualServoRuntime,
            )
        self.camera_config = (
            camera_config_cls.from_file(config.terminal_config_path)
            if camera_config_cls is not None
            else None
        )
        runtime_kwargs: dict[str, Any] = {"logger": self.logger}
        if self.camera_config is not None:
            runtime_kwargs["camera_config"] = self.camera_config
        self.runtime = runtime_cls(self.controller, self.settings, **runtime_kwargs)
        try:
            self.runtime.initialize(use_home_positions=config.mode == "dry-run")
        except Exception:
            self.controller.close()
            raise

    def _validate_positive_number(
        self, value: object, name: str, maximum: float
    ) -> float:
        if isinstance(value, bool):
            raise ArmControlError(f"{name}必须是数字")
        try:
            number = float(value)
        except (TypeError, ValueError) as exc:
            raise ArmControlError(f"{name}必须是数字") from exc
        if not math.isfinite(number) or number <= 0:
            raise ArmControlError(f"{name}必须大于0")
        if number > maximum:
            raise ArmControlError(f"{name}单次不能超过{maximum:g}")
        return number

    def _camera_signed_tilt_rad(self) -> float:
        """Return camera pitch relative to Home, normalized to [-pi, pi]."""

        model = self.runtime.model
        current_pitch = sum(
            model.position_to_model_angle(name, self.runtime.positions[name])
            for name in ("J2", "J3", "J4")
        )
        home_pitch = sum(
            model.position_to_model_angle(name, self.settings.home[name])
            for name in ("J2", "J3", "J4")
        )
        difference = current_pitch - home_pitch
        return math.atan2(math.sin(difference), math.cos(difference))

    def _direction_unit_base(self, direction: str) -> tuple[float, float, float]:
        if self._uses_camera_vector_runtime():
            frame = self.runtime.active_camera_relative_frame()
            try:
                vector = getattr(frame, direction)
            except AttributeError as exc:
                raise ArmControlError(f"涓嶆敮鎸佺殑TCP鏂瑰悜: {direction}") from exc
            return tuple(float(value) for value in vector)

        base_directions = {
            "forward": (1.0, 0.0, 0.0),
            "backward": (-1.0, 0.0, 0.0),
            "left": (0.0, -1.0, 0.0),
            "right": (0.0, 1.0, 0.0),
        }
        if direction in base_directions:
            return base_directions[direction]
        if direction not in {"up", "down"}:
            raise ArmControlError(f"不支持的TCP方向: {direction}")

        # The eye-in-hand camera pitches with J2-J4.  Home is the zero-angle
        # calibration pose, so image-view up is base +Z at Home and rotates in
        # the arm's radial plane as the camera tilts.
        yaw = self.runtime.model.position_to_model_angle(
            "J1", self.runtime.positions["J1"]
        )
        tilt = self._camera_signed_tilt_rad()
        sign = 1.0 if direction == "up" else -1.0
        radial = math.sin(tilt)
        return (
            sign * radial * math.cos(yaw),
            sign * radial * math.sin(yaw),
            sign * math.cos(tilt),
        )

    def _uses_camera_vector_runtime(self) -> bool:
        return callable(getattr(self.runtime, "active_camera_relative_frame", None))

    def _set_cartesian_direction(
        self, direction: str
    ) -> tuple[float, tuple[float, float, float]]:
        self.runtime.set_vertical_direction(0)
        self.runtime.center_joystick()
        if self._uses_camera_vector_runtime():
            unit = self._direction_unit_base(direction)
            if direction == "up":
                self.runtime.set_vertical_direction(1)
                return self.settings.vertical_speed_m_s, unit
            if direction == "down":
                self.runtime.set_vertical_direction(-1)
                return self.settings.vertical_speed_m_s, unit
            if direction == "forward":
                self.runtime.set_joystick(0, -1)
            elif direction == "backward":
                self.runtime.set_joystick(0, 1)
            elif direction == "left":
                self.runtime.set_joystick(-1, 0)
            elif direction == "right":
                self.runtime.set_joystick(1, 0)
            else:
                raise ArmControlError(f"涓嶆敮鎸佺殑TCP鏂瑰悜: {direction}")
            return self.settings.max_horizontal_speed_m_s, unit

        unit = self._direction_unit_base(direction)
        horizontal_length = math.hypot(unit[0], unit[1])
        speed_limits: list[float] = [
            self.settings.max_horizontal_speed_m_s,
            self.settings.vertical_speed_m_s,
        ]
        if horizontal_length > 1e-12:
            speed_limits.append(
                self.settings.max_horizontal_speed_m_s / horizontal_length
            )
        if abs(unit[2]) > 1e-12:
            speed_limits.append(self.settings.vertical_speed_m_s / abs(unit[2]))
        planner_speed_m_s = min(speed_limits)

        self.runtime.set_joystick(
            -unit[1] * planner_speed_m_s / self.settings.max_horizontal_speed_m_s,
            -unit[0] * planner_speed_m_s / self.settings.max_horizontal_speed_m_s,
        )
        self.runtime.set_vertical_direction(
            unit[2] * planner_speed_m_s / self.settings.vertical_speed_m_s
        )
        return planner_speed_m_s, unit

    def _stop_cartesian(self) -> None:
        self.runtime.set_vertical_direction(0)
        self.runtime.center_joystick()

    def _refresh_hardware_positions(self) -> None:
        if self.config.mode != "hardware":
            return
        refreshed: dict[str, int] = {}
        for joint_name in ARM_JOINTS:
            value = int(
                self.controller.read_position(self.settings.servo_id(joint_name))
            )
            low, high = self.settings.position_limits(joint_name)
            if not low <= value <= high:
                raise ArmControlError(
                    f"{joint_name}反馈值{value}超出安全范围{low}..{high}"
                )
            refreshed[joint_name] = value
        self.runtime.positions.update(refreshed)

    def _validate_speed(self, speed_cm_s: object | None) -> float:
        value = self.config.default_speed_cm_s if speed_cm_s is None else speed_cm_s
        if isinstance(value, bool):
            raise ArmControlError("speed_cm_s必须是数字")
        try:
            speed = float(value)
        except (TypeError, ValueError) as exc:
            raise ArmControlError("speed_cm_s必须是数字") from exc
        if not math.isfinite(speed):
            raise ArmControlError("speed_cm_s必须是有限数字")
        if not self.config.min_speed_cm_s <= speed <= self.config.max_speed_cm_s:
            raise ArmControlError(
                f"speed_cm_s必须在{self.config.min_speed_cm_s:g}到"
                f"{self.config.max_speed_cm_s:g}之间"
            )
        return speed

    def _validate_position(self, joint_name: str, value: object) -> int:
        if isinstance(value, bool):
            raise ArmControlError(f"{joint_name} position must be a number")
        try:
            position_float = float(value)
        except (TypeError, ValueError) as exc:
            raise ArmControlError(f"{joint_name} position must be a number") from exc
        if not math.isfinite(position_float):
            raise ArmControlError(f"{joint_name} position must be finite")
        position = int(round(position_float))
        if abs(position - position_float) > 1e-6:
            raise ArmControlError(f"{joint_name} position must be an integer")
        low, high = self.settings.position_limits(joint_name)
        if not low <= position <= high:
            raise ArmControlError(
                f"{joint_name} position must be in {low}..{high}, got {position}"
            )
        return position

    def _validate_run_time_ms(self, value: object | None) -> int:
        raw_value = DEFAULT_GRIPPER_POSITION_RUN_TIME_MS if value is None else value
        if isinstance(raw_value, bool):
            raise ArmControlError("run_time_ms must be a number")
        try:
            run_time_float = float(raw_value)
        except (TypeError, ValueError) as exc:
            raise ArmControlError("run_time_ms must be a number") from exc
        if not math.isfinite(run_time_float) or run_time_float <= 0:
            raise ArmControlError("run_time_ms must be greater than 0")
        run_time_ms = int(round(run_time_float))
        if run_time_ms > 30000:
            raise ArmControlError("run_time_ms must be in 1..30000")
        return run_time_ms

    @staticmethod
    def _validate_pixel_number(value: object, name: str) -> float:
        if isinstance(value, bool):
            raise ArmControlError(f"{name} must be a number")
        try:
            number = float(value)
        except (TypeError, ValueError) as exc:
            raise ArmControlError(f"{name} must be a number") from exc
        if not math.isfinite(number):
            raise ArmControlError(f"{name} must be finite")
        return number

    @staticmethod
    def _pixel_alignment_speed(
        axis_error_px: float, tolerance_px: float, saturation_px: float
    ) -> float:
        if saturation_px <= tolerance_px:
            raise ArmControlError("speed_saturation_px must be greater than tolerance_px")
        over_tolerance = max(0.0, abs(axis_error_px) - tolerance_px)
        ratio = min(1.0, over_tolerance / (saturation_px - tolerance_px))
        return MIN_PIXEL_ALIGNMENT_SPEED_CM_S + ratio * (
            MAX_PIXEL_ALIGNMENT_SPEED_CM_S - MIN_PIXEL_ALIGNMENT_SPEED_CM_S
        )

    @staticmethod
    def _pixel_tolerance_for_height(height_cm: float) -> float:
        if height_cm > 15.0:
            return 40.0
        if height_cm > 10.0:
            return 25.0
        if height_cm > 5.0:
            return 13.0
        return 8.0

    def _current_tcp_cm(self) -> dict[str, float]:
        tcp_cm = self.runtime.model.tcp(self.runtime.positions) * 100.0
        return {
            "forward_x": round(float(tcp_cm[0]), 3),
            "left_y": round(float(tcp_cm[1]), 3),
            "up_z": round(float(tcp_cm[2]), 3),
        }

    @staticmethod
    def _grasp_point_xyz_cm(grasp_point_base_cm: Mapping[str, float]) -> dict[str, float]:
        return {
            "x": round(float(grasp_point_base_cm["left_y"]), 3),
            "y": round(-float(grasp_point_base_cm["forward_x"]), 3),
            "z": round(float(grasp_point_base_cm["up_z"]), 3),
        }

    async def _move_tcp_segment(
        self,
        direction: str,
        distance_cm: float,
        speed_cm_s: float,
        *,
        collect_tcp_samples: bool = False,
    ) -> dict[str, Any]:
        """Execute one Cartesian segment through the selected terminal runtime."""

        if self.config.camera_vector_version != "v2":
            self._refresh_hardware_positions()
        start_tcp = self.runtime.model.tcp(self.runtime.positions).copy()
        horizontal_directions = {"forward", "backward", "left", "right"}
        duration_s = distance_cm / speed_cm_s
        tick_interval = self.settings.tick_s
        steps = 0
        tcp_samples: list[dict[str, Any]] = []
        terminal_input_status: dict[str, Any] | None = None
        v2_terminal_result: dict[str, Any] | None = None

        def append_current_tcp_sample(source: str) -> None:
            if not collect_tcp_samples:
                return
            sample_tcp = self.runtime.model.tcp(self.runtime.positions) * 100.0
            sample_base_cm = {
                "forward_x": round(float(sample_tcp[0]), 3),
                "left_y": round(float(sample_tcp[1]), 3),
                "up_z": round(float(sample_tcp[2]), 3),
            }
            tcp_samples.append(
                {
                    "step": steps,
                    "source": source,
                    "tcp_cm": sample_base_cm,
                    "grasp_point_xyz_cm": self._grasp_point_xyz_cm(
                        sample_base_cm
                    ),
                }
            )

        if self.config.camera_vector_version == "v2":
            execute_terminal_motion = getattr(
                self.terminal, "execute_terminal_motion", None
            )
            if not callable(execute_terminal_motion):
                raise ArmControlError(
                    "camera_vector_terminal_v2缺少execute_terminal_motion"
                )
            camera_reference_before_move = self._camera_pose()
            terminal_result = await execute_terminal_motion(
                self.runtime,
                direction=direction,
                speed_cm_s=speed_cm_s,
                duration_s=duration_s,
                real_time=self.config.mode == "hardware",
                sleep=self.sleep,
                collect_tcp_samples=collect_tcp_samples,
            )
            v2_terminal_result = dict(terminal_result)
            start_tcp = np.asarray(
                v2_terminal_result["start_grasp_point_m"], dtype=float
            )
            terminal_input_status = dict(terminal_result["terminal_input"])
            direction_unit = tuple(
                float(value)
                for value in terminal_input_status["direction_unit_base"]
            )
            steps = int(terminal_result["steps"])
            for step_index, raw_tcp_m in enumerate(
                terminal_result.get("tcp_samples_m", []), start=1
            ):
                sample_tcp = [float(value) * 100.0 for value in raw_tcp_m]
                sample_base_cm = {
                    "forward_x": round(sample_tcp[0], 3),
                    "left_y": round(sample_tcp[1], 3),
                    "up_z": round(sample_tcp[2], 3),
                }
                tcp_samples.append(
                    {
                        "step": step_index,
                        "source": "v2_terminal_input_integrated_fk",
                        "tcp_cm": sample_base_cm,
                        "grasp_point_xyz_cm": self._grasp_point_xyz_cm(
                            sample_base_cm
                        ),
                    }
                )
        else:
            requested_speed_m_s = speed_cm_s / 100.0
            planner_speed_m_s, direction_unit = self._set_cartesian_direction(
                direction
            )
            camera_reference_before_move = self._camera_pose()

            speed_m_s = speed_cm_s / 100.0
            speed_ratio = speed_m_s / planner_speed_m_s
            if direction in horizontal_directions:
                self.runtime.joystick_x *= speed_ratio
                self.runtime.joystick_y *= speed_ratio
            else:
                self.runtime.vertical_direction *= speed_ratio

            try:
                if self.config.mode == "hardware":
                    self.runtime.last_step_at = None
                    self.runtime.tick()
                    deadline = time.monotonic() + duration_s
                    while time.monotonic() < deadline:
                        self.runtime.tick()
                        steps += 1
                        await self.sleep(tick_interval)
                        append_current_tcp_sample("command_integrated_fk")
                else:
                    remaining_s = duration_s
                    while remaining_s > 1e-9:
                        dt = min(tick_interval, remaining_s)
                        execution_dt = (
                            dt * planner_speed_m_s / requested_speed_m_s
                        )
                        if not self.runtime.step_cartesian(
                            dt, run_time_s=execution_dt
                        ):
                            break
                        steps += 1
                        remaining_s -= dt
                        await self.sleep(0)
                        append_current_tcp_sample("command_integrated_fk")
            finally:
                self._stop_cartesian()

        if (
            self.config.mode == "hardware"
            and self.config.camera_vector_version != "v2"
        ):
            await self.sleep(0.05)
            self._refresh_hardware_positions()
        if v2_terminal_result is not None:
            end_tcp = np.asarray(
                v2_terminal_result["actual_grasp_point_m"], dtype=float
            )
        else:
            end_tcp = self.runtime.model.tcp(self.runtime.positions).copy()
        camera_pose_after_move = self._camera_pose()
        if v2_terminal_result is not None:
            camera_angle_before_deg = float(
                v2_terminal_result["target_camera_line_angle_deg"]
            )
            camera_angle_after_deg = float(
                v2_terminal_result["actual_camera_line_angle_deg"]
            )
        else:
            camera_angle_before_deg = float(
                camera_reference_before_move[
                    "line_of_sight_angle_from_vertical_deg"
                ]
            )
            camera_angle_after_deg = float(
                camera_pose_after_move["line_of_sight_angle_from_vertical_deg"]
            )
        line_before = camera_reference_before_move["view_up_unit_base"]
        line_after = camera_pose_after_move["view_up_unit_base"]
        line_dot = (
            float(line_before["forward_x"]) * float(line_after["forward_x"])
            + float(line_before["left_y"]) * float(line_after["left_y"])
            + float(line_before["up_z"]) * float(line_after["up_z"])
        )
        line_dot = max(-1.0, min(1.0, line_dot))
        line_unit_error_deg = math.degrees(math.acos(line_dot))
        delta_cm = (end_tcp - start_tcp) * 100.0
        estimated_distance = sum(
            float(delta_cm[index]) * direction_unit[index] for index in range(3)
        )
        horizontal_motion = direction in horizontal_directions
        horizontal_progress_validation: dict[str, Any] | None = None
        if (
            self.config.camera_vector_version == "v2"
            and horizontal_motion
        ):
            horizontal_progress_validation = manual_v2_horizontal_progress_validation(
                float(delta_cm[0]),
                float(delta_cm[1]),
                float(delta_cm[2]),
                camera_angle_after_deg - camera_angle_before_deg,
            )
            horizontal_progress_validation["original_v2_status"] = (
                v2_terminal_result.get("status")
                if v2_terminal_result is not None
                else None
            )
            horizontal_progress_validation["original_v2_error"] = (
                v2_terminal_result.get("error")
                if v2_terminal_result is not None
                else None
            )
        motion_status = "ok"
        motion_error: str | None = None
        if (
            self.config.camera_vector_version == "v2"
            and v2_terminal_result is not None
            and v2_terminal_result.get("status") != "ok"
        ):
            if (
                horizontal_progress_validation is not None
                and horizontal_progress_validation["accepted"]
            ):
                horizontal_progress_validation["overrode_v2_error"] = True
            else:
                motion_status = "error"
                motion_error = str(
                    v2_terminal_result.get("error")
                    or "V2 absolute grasp-point motion failed"
                )
        elif (
            self.config.camera_vector_version == "v2"
            and distance_cm > 0.0
            and estimated_distance <= 1e-6
        ):
            if (
                horizontal_progress_validation is not None
                and horizontal_progress_validation["accepted"]
            ):
                horizontal_progress_validation["overrode_zero_direction_progress"] = True
            else:
                motion_status = "error"
                motion_error = "V2运动无有效进展，目标可能不可达或已触及关节限位"
        pose_constraint_status: dict[str, Any] | None = None
        get_pose_relaxation = getattr(self.runtime, "pose_relaxation_status", None)
        if callable(get_pose_relaxation):
            pose_constraint_status = dict(get_pose_relaxation())
            pose_constraint_status["motion_progress_ok"] = motion_status == "ok"
        grasp_before_cm = {
            "forward_x": round(float(start_tcp[0] * 100.0), 3),
            "left_y": round(float(start_tcp[1] * 100.0), 3),
            "up_z": round(float(start_tcp[2] * 100.0), 3),
        }
        grasp_after_cm = {
            "forward_x": round(float(end_tcp[0] * 100.0), 3),
            "left_y": round(float(end_tcp[1] * 100.0), 3),
            "up_z": round(float(end_tcp[2] * 100.0), 3),
        }
        height_error_cm = float((end_tcp[2] - start_tcp[2]) * 100.0)
        result = {
            "status": motion_status,
            "mode": self.config.mode,
            "direction": direction,
            "requested_distance_cm": distance_cm,
            "speed_cm_s": speed_cm_s,
            "estimated_distance_cm": round(estimated_distance, 3),
            "motion_loop": (
                str(v2_terminal_result.get("execution_path"))
                if self.config.camera_vector_version == "v2"
                and v2_terminal_result is not None
                else "camera_vector_continuous_command"
            ),
            "terminal_input": terminal_input_status,
            "terminal_hold_duration_s": duration_s,
            "v2_motion_workflow": (
                v2_terminal_result.get("workflow")
                if v2_terminal_result is not None
                else None
            ),
            "v2_motion_plan": (
                v2_terminal_result.get("motion_plan")
                if v2_terminal_result is not None
                else None
            ),
            "v2_returned_camera_line_angle_deg": (
                v2_terminal_result.get("actual_camera_line_angle_deg")
                if v2_terminal_result is not None
                else None
            ),
            "v2_feedback_corrections": (
                v2_terminal_result.get("feedback_corrections")
                if v2_terminal_result is not None
                else None
            ),
            "horizontal_progress_validation": horizontal_progress_validation,
            "feedback_read_policy": (
                "v2_executor_start_and_verified_final_feedback"
                if v2_terminal_result is not None
                else "before_and_after_motion"
                if self.config.mode == "hardware"
                else "dry_run_command_integrated"
            ),
            "grasp_point_before_cm": grasp_before_cm,
            "grasp_point_after_cm": grasp_after_cm,
            "grasp_point_xyz_before_cm": self._grasp_point_xyz_cm(grasp_before_cm),
            "grasp_point_xyz_after_cm": self._grasp_point_xyz_cm(grasp_after_cm),
            "estimated_delta_cm": {
                "forward_x": round(float(delta_cm[0]), 3),
                "left_y": round(float(delta_cm[1]), 3),
                "up_z": round(float(delta_cm[2]), 3),
            },
            "direction_reference": (
                (
                    "camera_vector_v2"
                    if self.config.camera_vector_version == "v2"
                    else "camera_vector"
                )
                if self._uses_camera_vector_runtime()
                else ("camera_view" if direction in {"up", "down"} else "base")
            ),
            "horizontal_motion_frame": (
                (
                    "grasp_to_camera_line_xy_projection"
                    if self.config.camera_vector_version == "v2"
                    else "grasp_point_xyz_xy"
                )
                if horizontal_motion
                else None
            ),
            "horizontal_height_hold": (
                {
                    "target_z_cm": round(float(start_tcp[2] * 100.0), 3),
                    "actual_after_z_cm": round(float(end_tcp[2] * 100.0), 3),
                    "error_cm": round(height_error_cm, 4),
                    "z_participates_in_command": (
                        self.config.camera_vector_version == "v2"
                    ),
                }
                if horizontal_motion
                else None
            ),
            "direction_unit_base": {
                "forward_x": round(direction_unit[0], 6),
                "left_y": round(direction_unit[1], 6),
                "up_z": round(direction_unit[2], 6),
            },
            "camera_pose_before_move": camera_reference_before_move,
            "camera_pose_after_move": camera_pose_after_move,
            "camera_line_angle_hold": {
                "target_deg": round(camera_angle_before_deg, 3),
                "actual_after_deg": round(camera_angle_after_deg, 3),
                "error_deg": round(camera_angle_after_deg - camera_angle_before_deg, 3),
            },
            "camera_pose_constraint": pose_constraint_status,
            "camera_grasp_pose_hold": {
                "line_unit_before": line_before,
                "line_unit_after": line_after,
                "line_unit_error_deg": round(line_unit_error_deg, 3),
                "line_angle_error_deg": round(
                    camera_angle_after_deg - camera_angle_before_deg, 3
                ),
            },
            "steps": steps,
            "joint_positions": dict(self.runtime.positions),
            "tcp_samples_cm": tcp_samples,
        }
        if motion_error is not None:
            result["error"] = motion_error
        return result

    async def move_tcp(
        self,
        direction: str,
        distance_cm: object,
        speed_cm_s: object | None = None,
    ) -> dict[str, Any]:
        """Execute exactly one Agent-issued TCP movement command."""

        distance = self._validate_positive_number(
            distance_cm, "distance_cm", self.config.max_distance_cm
        )
        if distance >= self.config.max_distance_cm:
            raise ArmControlError(
                f"distance_cm单次必须小于{self.config.max_distance_cm:g}；"
                "长距离必须由Agent按最新RGB图像逐步执行，每次重新取图后只决定下一条命令"
            )
        speed = self._validate_speed(speed_cm_s)
        result = await self._move_tcp_segment(direction, distance, speed)
        return {
            **result,
            "command": format_compact_arm_command(direction, distance),
            "command_limit_cm_exclusive": self.config.max_distance_cm,
            "motion_command_count": 1,
        }

    async def execute_compact_command(
        self, command: str, speed_cm_s: object | None = None
    ) -> dict[str, Any]:
        direction, distance_cm = parse_compact_arm_command(command)
        return await self.move_tcp(direction, distance_cm, speed_cm_s)

    async def set_gripper_position(
        self,
        position: object = DEFAULT_GRIPPER_RELEASE_POSITION,
        run_time_ms: object | None = DEFAULT_GRIPPER_POSITION_RUN_TIME_MS,
    ) -> dict[str, Any]:
        target = self._validate_position("J6", position)
        duration_ms = self._validate_run_time_ms(run_time_ms)
        if self.runtime.j6_grip_locked:
            self.runtime.toggle_grip_lock()
        j6_id = self.settings.servo_id("J6")
        set_servo_mode = getattr(self.controller, "set_servo_mode", None)
        if not callable(set_servo_mode):
            raise ArmControlError("当前舵机控制器不支持将J6切换到位置控制模式")
        set_servo_mode(j6_id)
        self.controller.move_servo(j6_id, target, duration_ms)
        if self.config.mode == "hardware":
            await self.sleep(duration_ms / 1000.0)
        return {
            "status": "ok",
            "mode": self.config.mode,
            "joint": "J6",
            "action": "set_position",
            "target_position": target,
            "run_time_ms": duration_ms,
            "grip_locked": False,
            "position_mode_enabled": True,
        }

    async def move_by_pixel_error(
        self,
        block_center_x: object,
        block_center_y: object,
        grasp_point_x: object,
        grasp_point_y: object,
        *,
        tolerance_px: object = DEFAULT_PIXEL_ALIGNMENT_TOLERANCE_PX,
        step_duration_s: object = DEFAULT_PIXEL_ALIGNMENT_STEP_DURATION_S,
        speed_saturation_px: object = DEFAULT_PIXEL_ALIGNMENT_SPEED_SATURATION_PX,
    ) -> dict[str, Any]:
        block_x = self._validate_pixel_number(block_center_x, "block_center_x")
        block_y = self._validate_pixel_number(block_center_y, "block_center_y")
        grasp_x = self._validate_pixel_number(grasp_point_x, "grasp_point_x")
        grasp_y = self._validate_pixel_number(grasp_point_y, "grasp_point_y")
        tolerance = self._validate_pixel_number(tolerance_px, "tolerance_px")
        duration = self._validate_positive_number(
            step_duration_s, "step_duration_s", self.config.max_distance_cm
        )
        saturation = self._validate_pixel_number(
            speed_saturation_px, "speed_saturation_px"
        )
        if tolerance < 0:
            raise ArmControlError("tolerance_px must be greater than or equal to 0")

        self._refresh_hardware_positions()
        current_tcp_cm = self._current_tcp_cm()
        current_xyz_cm = self._grasp_point_xyz_cm(current_tcp_cm)
        height_cm = current_tcp_cm["up_z"]
        pixel_scale_px_per_cm = pixel_alignment_px_per_cm_for_height(height_cm)
        dx = block_x - grasp_x
        dy = block_y - grasp_y
        if abs(dx) <= tolerance and abs(dy) <= tolerance:
            return {
                "status": "ok",
                "mode": self.config.mode,
                "action": "pixel_align",
                "aligned": True,
                "pixel_error": {"dx": round(dx, 3), "dy": round(dy, 3)},
                "tolerance_px": tolerance,
                "grasp_point_before_cm": current_tcp_cm,
                "grasp_point_after_cm": current_tcp_cm,
                "grasp_point_xyz_before_cm": current_xyz_cm,
                "grasp_point_xyz_after_cm": current_xyz_cm,
                "height_cm": height_cm,
                "pixel_to_motion_scale_px_per_cm": round(pixel_scale_px_per_cm, 6),
                "pixel_to_motion_scale_height_cm": height_cm,
                "pixel_to_motion_scale_model": "linear_height_cm",
                "motion_command_count": 0,
            }

        if abs(dx) >= abs(dy):
            axis_error = dx
            direction = "right" if dx > 0 else "left"
            pixel_axis = "x"
        else:
            axis_error = dy
            direction = "backward" if dy > 0 else "forward"
            pixel_axis = "y"

        speed = self._pixel_alignment_speed(axis_error, tolerance, saturation)
        fixed_distance = self.config.fixed_pixel_alignment_distance_cm
        if fixed_distance is None:
            distance = min(
                abs(axis_error) / pixel_scale_px_per_cm,
                self.config.max_distance_cm,
            )
            distance_mode = "pixel_scale"
        else:
            distance = float(fixed_distance)
            distance_mode = "fixed"
        result = await self._move_tcp_segment(direction, distance, speed)
        return {
            **result,
            "action": "pixel_align",
            "aligned": False,
            "pixel_error": {"dx": round(dx, 3), "dy": round(dy, 3)},
            "pixel_axis": pixel_axis,
            "pixel_to_motion_scale_px_per_cm": (
                round(pixel_scale_px_per_cm, 6) if fixed_distance is None else None
            ),
            "pixel_to_motion_scale_height_cm": height_cm,
            "pixel_to_motion_scale_model": "linear_height_cm",
            "tolerance_px": tolerance,
            "speed_saturation_px": saturation,
            "step_duration_s": duration,
            "command_limit_cm": self.config.max_distance_cm,
            "pixel_alignment_distance_mode": distance_mode,
            "fixed_pixel_alignment_distance_cm": fixed_distance,
            "motion_command_count": 1,
        }

    async def control_to_target_pixel(
        self,
        target_x: object,
        target_y: object,
        grasp_point_x: object,
        grasp_point_y: object,
        *,
        descend_when_aligned: object = True,
        descent_step_cm: object = DEFAULT_DESCENT_RECALIBRATION_CM,
        step_duration_s: object = DEFAULT_PIXEL_ALIGNMENT_STEP_DURATION_S,
        speed_saturation_px: object = DEFAULT_PIXEL_ALIGNMENT_SPEED_SATURATION_PX,
        final_alignment_threshold_cm: float = FINAL_ALIGNMENT_THRESHOLD_CM,
        final_grasp_height_cm: float = FINAL_GRASP_HEIGHT_CM,
    ) -> dict[str, Any]:
        """Controller-owned visual servo step from target pixel only.

        The Agent supplies the target pixel.  The controller reads joint
        feedback/FK, chooses tolerance from grasp-point height, and decides
        whether to move in the image plane or descend for the next recalibration.

        When aligned and one more descent step would drop the grasp point to or
        below ``final_alignment_threshold_cm``, the method returns ``aligned_hold``
        so the caller can request a final alignment before descending the last
        segment to ``final_grasp_height_cm``.
        """

        self._refresh_hardware_positions()
        tcp_before = self._current_tcp_cm()
        xyz_before = self._grasp_point_xyz_cm(tcp_before)
        height_cm = tcp_before["up_z"]
        tolerance = self._pixel_tolerance_for_height(height_cm)
        pixel_scale_px_per_cm = pixel_alignment_px_per_cm_for_height(height_cm)
        target_px = self._validate_pixel_number(target_x, "target_x")
        target_py = self._validate_pixel_number(target_y, "target_y")
        grasp_px = self._validate_pixel_number(grasp_point_x, "grasp_point_x")
        grasp_py = self._validate_pixel_number(grasp_point_y, "grasp_point_y")
        dx = target_px - grasp_px
        dy = target_py - grasp_py
        aligned = abs(dx) <= tolerance and abs(dy) <= tolerance

        if not aligned:
            result = await self.move_by_pixel_error(
                target_px,
                target_py,
                grasp_px,
                grasp_py,
                tolerance_px=tolerance,
                step_duration_s=step_duration_s,
                speed_saturation_px=speed_saturation_px,
            )
            return {
                **result,
                "action": "controller_pixel_align",
                "agent_role": "target_pixel_only",
                "controller_decision": "horizontal_align",
                "target_pixel": {"x": target_px, "y": target_py},
                "grasp_point_pixel": {"x": grasp_px, "y": grasp_py},
                "grasp_point_before_cm": result.get(
                    "grasp_point_before_cm", tcp_before
                ),
                "grasp_point_after_cm": result.get("grasp_point_after_cm"),
                "grasp_point_xyz_before_cm": result.get(
                    "grasp_point_xyz_before_cm", xyz_before
                ),
                "grasp_point_xyz_after_cm": result.get("grasp_point_xyz_after_cm"),
                "height_cm": height_cm,
                "height_source": "joint_feedback_fk",
                "dynamic_tolerance_px": tolerance,
                "pixel_to_motion_scale_px_per_cm": result.get(
                    "pixel_to_motion_scale_px_per_cm"
                ),
                "pixel_to_motion_scale_height_cm": result.get(
                    "pixel_to_motion_scale_height_cm"
                ),
                "pixel_to_motion_scale_model": result.get(
                    "pixel_to_motion_scale_model"
                ),
                "requires_new_target_pixel": True,
            }

        should_descend = bool(descend_when_aligned)
        if not should_descend:
            return {
                "status": "ok",
                "mode": self.config.mode,
                "action": "controller_pixel_align",
                "agent_role": "target_pixel_only",
                "controller_decision": "aligned_hold",
                "aligned": True,
                "pixel_error": {"dx": round(dx, 3), "dy": round(dy, 3)},
                "target_pixel": {"x": target_px, "y": target_py},
                "grasp_point_pixel": {"x": grasp_px, "y": grasp_py},
                "grasp_point_before_cm": tcp_before,
                "grasp_point_after_cm": tcp_before,
                "grasp_point_xyz_before_cm": xyz_before,
                "grasp_point_xyz_after_cm": xyz_before,
                "height_cm": height_cm,
                "height_source": "joint_feedback_fk",
                "dynamic_tolerance_px": tolerance,
                "pixel_to_motion_scale_px_per_cm": round(pixel_scale_px_per_cm, 6),
                "pixel_to_motion_scale_height_cm": height_cm,
                "pixel_to_motion_scale_model": "linear_height_cm",
                "motion_command_count": 0,
                "requires_new_target_pixel": False,
            }

        # When one descent step would reach or pass the final-alignment
        # threshold, return aligned_hold so the caller can request a fresh
        # alignment before the last short descent to final_grasp_height_cm.
        if height_cm - float(descent_step_cm) <= final_alignment_threshold_cm:
            remaining_to_final = max(0.0, height_cm - final_grasp_height_cm)
            return {
                "status": "ok",
                "mode": self.config.mode,
                "action": "controller_pixel_align",
                "agent_role": "target_pixel_only",
                "controller_decision": "aligned_hold",
                "aligned": True,
                "pixel_error": {"dx": round(dx, 3), "dy": round(dy, 3)},
                "target_pixel": {"x": target_px, "y": target_py},
                "grasp_point_pixel": {"x": grasp_px, "y": grasp_py},
                "grasp_point_before_cm": tcp_before,
                "grasp_point_after_cm": tcp_before,
                "grasp_point_xyz_before_cm": xyz_before,
                "grasp_point_xyz_after_cm": xyz_before,
                "height_cm": height_cm,
                "height_source": "joint_feedback_fk",
                "dynamic_tolerance_px": tolerance,
                "pixel_to_motion_scale_px_per_cm": round(pixel_scale_px_per_cm, 6),
                "pixel_to_motion_scale_height_cm": height_cm,
                "pixel_to_motion_scale_model": "linear_height_cm",
                "final_alignment_threshold_cm": final_alignment_threshold_cm,
                "final_grasp_height_cm": final_grasp_height_cm,
                "remaining_descent_to_final_cm": round(remaining_to_final, 3),
                "motion_command_count": 0,
                "requires_new_target_pixel": True,
            }

        descent = self._validate_positive_number(
            descent_step_cm, "descent_step_cm", self.config.max_distance_cm
        )
        result = await self._move_tcp_segment(
            "down",
            descent,
            DESCENT_SPEED_CM_S,
            collect_tcp_samples=True,
        )
        if self.config.camera_vector_version == "v2":
            reported_after = result.get("grasp_point_after_cm")
            reported_xyz_after = result.get("grasp_point_xyz_after_cm")
            reported_before = result.get("grasp_point_before_cm")
            reported_xyz_before = result.get("grasp_point_xyz_before_cm")
            tcp_after = (
                dict(reported_after)
                if isinstance(reported_after, Mapping)
                else self._current_tcp_cm()
            )
            xyz_after = (
                dict(reported_xyz_after)
                if isinstance(reported_xyz_after, Mapping)
                else self._grasp_point_xyz_cm(tcp_after)
            )
            move_tcp_before = (
                dict(reported_before)
                if isinstance(reported_before, Mapping)
                else tcp_before
            )
            move_xyz_before = (
                dict(reported_xyz_before)
                if isinstance(reported_xyz_before, Mapping)
                else xyz_before
            )
        else:
            self._refresh_hardware_positions()
            tcp_after = self._current_tcp_cm()
            xyz_after = self._grasp_point_xyz_cm(tcp_after)
            move_tcp_before = tcp_before
            move_xyz_before = xyz_before
        return {
            **result,
            "action": "controller_pixel_align_and_descend",
            "agent_role": "target_pixel_only",
            "controller_decision": "descend_after_alignment",
            "aligned": True,
            "pixel_error": {"dx": round(dx, 3), "dy": round(dy, 3)},
            "target_pixel": {"x": target_px, "y": target_py},
            "grasp_point_pixel": {"x": grasp_px, "y": grasp_py},
            "grasp_point_before_cm": move_tcp_before,
            "grasp_point_after_cm": tcp_after,
            "grasp_point_xyz_before_cm": move_xyz_before,
            "grasp_point_xyz_after_cm": xyz_after,
            "height_before_cm": move_tcp_before["up_z"],
            "height_after_cm": tcp_after["up_z"],
            "height_source": "joint_feedback_fk",
            "dynamic_tolerance_px": tolerance,
            "pixel_to_motion_scale_px_per_cm": round(pixel_scale_px_per_cm, 6),
            "pixel_to_motion_scale_height_cm": height_cm,
            "pixel_to_motion_scale_model": "linear_height_cm",
            "descent_recalibration_interval_cm": descent,
            "final_alignment_threshold_cm": final_alignment_threshold_cm,
            "final_grasp_height_cm": final_grasp_height_cm,
            "requires_new_target_pixel": True,
        }

    async def descend_to_height(
        self, target_height_cm: float, *, speed_cm_s: float = DESCENT_SPEED_CM_S
    ) -> dict[str, Any]:
        """Descend until the grasp-point FK height reaches ``target_height_cm``.

        The method reads joint feedback before moving and strictly clamps the
        commanded descent so that the target height is never overshot.
        """

        self._refresh_hardware_positions()
        current = self._current_tcp_cm()
        current_height = current["up_z"]
        target_tolerance_cm = max(
            0.0,
            float(getattr(self.runtime, "position_target_tolerance_m", 0.0))
            * 100.0,
        )
        pose_relaxation_steps = 0
        pose_relaxation_reasons: list[str] = []
        pose_relaxation_reason_codes: list[str] = []

        def record_pose_relaxation(move_result: Mapping[str, Any]) -> None:
            nonlocal pose_relaxation_steps
            status = move_result.get("camera_pose_constraint")
            if not isinstance(status, Mapping) or not status.get("relaxed"):
                return
            pose_relaxation_steps += int(status.get("relaxed_step_count", 0))
            reason = status.get("reason")
            if isinstance(reason, str) and reason not in pose_relaxation_reasons:
                pose_relaxation_reasons.append(reason)
            reason_code = status.get("reason_code")
            if (
                isinstance(reason_code, str)
                and reason_code not in pose_relaxation_reason_codes
            ):
                pose_relaxation_reason_codes.append(reason_code)

        def pose_relaxation_summary() -> dict[str, Any]:
            relaxed = pose_relaxation_steps > 0
            return {
                "constraint": "camera_grasp_line_inclination",
                "mode": (
                    "relaxed_due_to_downward_limit" if relaxed else "strict"
                ),
                "relaxed": relaxed,
                "reasons": list(pose_relaxation_reasons),
                "reason_codes": list(pose_relaxation_reason_codes),
                "relaxed_step_count": pose_relaxation_steps,
                "allowed_scope": "down_only",
            }

        if current_height <= target_height_cm + target_tolerance_cm:
            return {
                "status": "ok",
                "mode": self.config.mode,
                "action": "descend_to_height",
                "already_at_target": True,
                "height_cm": current_height,
                "target_height_cm": target_height_cm,
                "target_tolerance_cm": target_tolerance_cm,
                "camera_pose_constraint": pose_relaxation_summary(),
            }
        remaining_cm = current_height - target_height_cm
        # Clamp to the per-command limit so the move stays safe.
        effective_cm = min(remaining_cm, self.config.max_distance_cm)
        steps = 0
        heights: list[float] = [current_height]
        while (
            current_height > target_height_cm + target_tolerance_cm
            and steps < 200
        ):
            previous_height = current_height
            remaining_cm = current_height - target_height_cm
            if remaining_cm <= 0:
                break
            effective_cm = min(remaining_cm, self.config.max_distance_cm)
            result = await self._move_tcp_segment(
                "down", effective_cm, speed_cm_s, collect_tcp_samples=True
            )
            record_pose_relaxation(result)
            if self.config.camera_vector_version == "v2":
                reported_after = result.get("grasp_point_after_cm")
                after = (
                    dict(reported_after)
                    if isinstance(reported_after, Mapping)
                    else self._current_tcp_cm()
                )
            else:
                self._refresh_hardware_positions()
                after = self._current_tcp_cm()
            current_height = after["up_z"]
            heights.append(current_height)
            steps += 1
            if result.get("status") != "ok":
                return {
                    **result,
                    "action": "descend_to_height",
                    "target_height_cm": target_height_cm,
                    "target_tolerance_cm": target_tolerance_cm,
                    "height_samples_cm": heights,
                    "height_after_cm": current_height,
                    "steps": steps,
                    "camera_pose_constraint": pose_relaxation_summary(),
                }
            if current_height >= previous_height - 1e-6:
                return {
                    "status": "error",
                    "mode": self.config.mode,
                    "action": "descend_to_height",
                    "error": "下降无有效进展，已停止且不会执行抓取",
                    "target_height_cm": target_height_cm,
                    "target_tolerance_cm": target_tolerance_cm,
                    "height_samples_cm": heights,
                    "height_after_cm": current_height,
                    "steps": steps,
                    "camera_pose_constraint": pose_relaxation_summary(),
                }
        if current_height > target_height_cm + target_tolerance_cm:
            return {
                "status": "error",
                "mode": self.config.mode,
                "action": "descend_to_height",
                "error": "达到最大下降步数后仍未到达目标高度",
                "target_height_cm": target_height_cm,
                "target_tolerance_cm": target_tolerance_cm,
                "height_samples_cm": heights,
                "height_after_cm": current_height,
                "steps": steps,
                "camera_pose_constraint": pose_relaxation_summary(),
            }
        return {
            "status": "ok",
            "mode": self.config.mode,
            "action": "descend_to_height",
            "target_height_cm": target_height_cm,
            "target_tolerance_cm": target_tolerance_cm,
            "height_before_cm": heights[0],
            "height_after_cm": current_height,
            "height_samples_cm": heights,
            "steps": steps,
            "grasp_point_xyz_after_cm": self._grasp_point_xyz_cm(self._current_tcp_cm()),
            "camera_pose_constraint": pose_relaxation_summary(),
        }

    async def rotate_wrist(
        self, direction: str, duration_s: object
    ) -> dict[str, Any]:
        duration = self._validate_positive_number(
            duration_s, "duration_s", self.config.max_motor_duration_s
        )
        if direction not in {"clockwise", "counterclockwise"}:
            raise ArmControlError(f"不支持的J5方向: {direction}")
        try:
            if direction == "clockwise":
                self.runtime.rotate_j5_clockwise()
            else:
                self.runtime.rotate_j5_counterclockwise()
            if self.config.mode == "hardware":
                await self.sleep(duration)
        finally:
            self.runtime.stop_j5()
        return {
            "status": "ok",
            "mode": self.config.mode,
            "joint": "J5",
            "direction": direction,
            "duration_s": duration,
            "stopped": True,
        }

    async def control_gripper(
        self, action: str, duration_s: object = 0.5
    ) -> dict[str, Any]:
        if action == "grip_lock":
            if not self.runtime.j6_grip_locked:
                self.runtime.toggle_grip_lock()
            return {"status": "ok", "action": action, "grip_locked": True}
        if action == "release_lock":
            if self.runtime.j6_grip_locked:
                self.runtime.toggle_grip_lock()
            else:
                self.runtime.stop_j6()
            return {"status": "ok", "action": action, "grip_locked": False}
        if action == "stop":
            if self.runtime.j6_grip_locked:
                self.runtime.toggle_grip_lock()
            else:
                self.runtime.stop_j6()
            return {"status": "ok", "action": action, "grip_locked": False}
        if action not in {"open", "close"}:
            raise ArmControlError(f"不支持的J6动作: {action}")

        duration = self._validate_positive_number(
            duration_s, "duration_s", self.config.max_motor_duration_s
        )
        if self.runtime.j6_grip_locked:
            raise ArmControlError("J6抓紧锁定中，请先执行release_lock")
        try:
            if action == "open":
                self.runtime.open_j6()
            else:
                self.runtime.close_j6()
            if self.config.mode == "hardware":
                await self.sleep(duration)
        finally:
            self.runtime.stop_j6()
        return {
            "status": "ok",
            "mode": self.config.mode,
            "action": action,
            "duration_s": duration,
            "stopped": True,
        }

    async def go_home(self) -> dict[str, Any]:
        self.runtime.go_home()
        if self.config.mode == "hardware":
            await self.sleep(self.settings.home_run_time_ms / 1000.0)
            self._refresh_hardware_positions()
        return {
            "status": "ok",
            "mode": self.config.mode,
            "action": "home",
            "joint_positions": dict(self.runtime.positions),
        }

    async def stop_all(self) -> dict[str, Any]:
        self.runtime.stop_all()
        return {"status": "ok", "mode": self.config.mode, "action": "stop_all"}

    def _camera_pose(self) -> dict[str, Any]:
        tilt_rad = self._camera_signed_tilt_rad()
        up = self._direction_unit_base("up")
        down = self._direction_unit_base("down")
        forward = self._direction_unit_base("forward")
        left = self._direction_unit_base("left")
        vertical_dot = max(-1.0, min(1.0, up[2]))
        return {
            "mount": "eye_in_hand_near_j5_j6",
            "control_frame": (
                "camera_vector" if self._uses_camera_vector_runtime() else "legacy_camera_view"
            ),
            "line_of_sight_angle_from_vertical_deg": round(
                math.degrees(math.acos(vertical_dot)), 3
            ),
            "signed_pitch_from_home_deg": round(math.degrees(tilt_rad), 3),
            "signed_pitch_positive_direction": "toward_arm_radial_forward",
            "view_up_unit_base": {
                "forward_x": round(up[0], 6),
                "left_y": round(up[1], 6),
                "up_z": round(up[2], 6),
            },
            "camera_to_grasp_unit_base": {
                "forward_x": round(down[0], 6),
                "left_y": round(down[1], 6),
                "up_z": round(down[2], 6),
            },
            "plane_forward_unit_base": {
                "forward_x": round(forward[0], 6),
                "left_y": round(forward[1], 6),
                "up_z": round(forward[2], 6),
            },
            "plane_left_unit_base": {
                "forward_x": round(left[0], 6),
                "left_y": round(left[1], 6),
                "up_z": round(left[2], 6),
            },
        }

    def _arm_pose(self) -> dict[str, Any]:
        tcp_cm = self.runtime.model.tcp(self.runtime.positions) * 100.0
        grasp_point = {
            "forward_x": round(float(tcp_cm[0]), 3),
            "left_y": round(float(tcp_cm[1]), 3),
            "up_z": round(float(tcp_cm[2]), 3),
        }
        return {
            "joint_positions": dict(self.runtime.positions),
            "grasp_point_base_cm": grasp_point,
            "grasp_point_xyz_cm": self._grasp_point_xyz_cm(grasp_point),
            "camera": self._camera_pose(),
        }

    def arm_parameters(self) -> dict[str, Any]:
        uses_v2_camera_frame = self.config.camera_vector_version == "v2"
        horizontal_frame = (
            "grasp_to_camera_line_xy_projection"
            if uses_v2_camera_frame
            else "base_horizontal_xy"
        )
        runtime_implementation = (
            "ubuntu22_04_operation_terminal.camera_vector_terminal_v2"
            if uses_v2_camera_frame
            else "ubuntu22_04_operation_terminal.camera_vector_terminal"
        )
        if uses_v2_camera_frame:
            direction_descriptions = {
                "forward": "grasp_to_camera_line_xy_projection",
                "backward": "opposite_of_forward",
                "left": "world_up_cross_forward",
                "right": "opposite_of_left",
            }
        else:
            direction_descriptions = {
                "forward": "grasp_point_xyz_y_decreases",
                "backward": "grasp_point_xyz_y_increases",
                "left": "grasp_point_xyz_x_decreases",
                "right": "grasp_point_xyz_x_increases",
            }
        joints = {
            name: {
                "servo_id": self.settings.servo_id(name),
                "position_min": int(values["position_min"]),
                "position_max": int(values["position_max"]),
                "angle_min_deg": float(values["angle_min_deg"]),
                "angle_max_deg": float(values["angle_max_deg"]),
                "direction_sign": float(values["direction_sign"]),
                "home_position": self.settings.home.get(name),
            }
            for name, values in self.settings.joints.items()
        }
        return {
            "coordinate_frame": {
                "name": "base",
                "x_positive": "arm_forward",
                "y_positive": "arm_left",
                "z_positive": "up",
                "units": "cm",
            },
            "grasp_point_xyz_frame": {
                "name": "manual_grasp_point_xyz",
                "x": "base.left_y; moving left decreases x",
                "y": "-base.forward_x; moving forward decreases y",
                "z": "base.up_z",
                "units": "cm",
            },
            "agent_direction_frames": {
                "forward_backward_left_right": horizontal_frame,
                "up_down": "camera_grasp_line",
                "implementation": runtime_implementation,
                **direction_descriptions,
            },
            "home_joint_positions": dict(self.settings.home),
            "joints": joints,
            "geometry_m": dict(self.settings.geometry),
            "control": {
                "tick_s": self.settings.tick_s,
                "vertical_speed_m_s": self.settings.vertical_speed_m_s,
                "max_horizontal_speed_m_s": self.settings.max_horizontal_speed_m_s,
                "default_agent_tcp_speed_cm_s": self.config.default_speed_cm_s,
                "min_agent_tcp_speed_cm_s": self.config.min_speed_cm_s,
                "max_agent_tcp_speed_cm_s": self.config.max_speed_cm_s,
                "max_agent_move_command_cm_exclusive": self.config.max_distance_cm,
            },
            "camera_orientation_model": {
                "mount": "eye_in_hand_near_j5_j6",
                "control_frame": (
                    "camera_vector_v2" if uses_v2_camera_frame else "camera_vector"
                ),
                "up": "grasp_point_to_camera",
                "down": "camera_to_grasp_point",
                "forward_backward_left_right": horizontal_frame,
                "motion_constraint": "keep_camera_grasp_line_pose_and_lock_horizontal_z",
            },
            "vision_guided_grasp": {
                "pixel_alignment_tolerance_px": DEFAULT_PIXEL_ALIGNMENT_TOLERANCE_PX,
                "pixel_alignment_min_speed_cm_s": MIN_PIXEL_ALIGNMENT_SPEED_CM_S,
                "pixel_alignment_max_speed_cm_s": MAX_PIXEL_ALIGNMENT_SPEED_CM_S,
                "pixel_alignment_default_step_duration_s": DEFAULT_PIXEL_ALIGNMENT_STEP_DURATION_S,
                "pixel_alignment_speed_saturation_px": DEFAULT_PIXEL_ALIGNMENT_SPEED_SATURATION_PX,
                "pixel_to_motion_scale_model": "linear_height_cm",
                "pixel_to_motion_scale_reference_points": [
                    {
                        "height_cm": PIXEL_ALIGNMENT_SCALE_NEAR_HEIGHT_CM,
                        "px_per_cm": PIXEL_ALIGNMENT_SCALE_NEAR_PX_PER_CM,
                    },
                    {
                        "height_cm": PIXEL_ALIGNMENT_SCALE_FAR_HEIGHT_CM,
                        "px_per_cm": PIXEL_ALIGNMENT_SCALE_FAR_PX_PER_CM,
                    },
                ],
                "pixel_to_motion_scale_formula": (
                    "px_per_cm = 50 + (height_cm - 2) * (18 - 50) / (25 - 2)"
                ),
                "pixel_alignment_distance_mode": (
                    "fixed"
                    if self.config.fixed_pixel_alignment_distance_cm is not None
                    else "pixel_scale"
                ),
                "fixed_pixel_alignment_distance_cm": self.config.fixed_pixel_alignment_distance_cm,
                "descent_speed_cm_s": 2.0,
                "pixel_recalculation_descent_interval_cm": DEFAULT_DESCENT_RECALIBRATION_CM,
                "height_tolerance_bands_px": [
                    {"height_cm": ">15", "tolerance_px": 40},
                    {"height_cm": ">10 and <=15", "tolerance_px": 25},
                    {"height_cm": ">5 and <=10", "tolerance_px": 13},
                    {"height_cm": "<=5", "tolerance_px": 8},
                ],
                "final_alignment_threshold_cm": FINAL_ALIGNMENT_THRESHOLD_CM,
                "final_grasp_height_cm": FINAL_GRASP_HEIGHT_CM,
                "j6_release_position_before_success": DEFAULT_GRIPPER_RELEASE_POSITION,
                "pixel_to_motion_mapping": {
                    "positive_dx": "right",
                    "negative_dx": "left",
                    "positive_dy": "backward",
                    "negative_dy": "forward",
                },
            },
        }

    async def pose(self) -> dict[str, Any]:
        self._refresh_hardware_positions()
        return self._arm_pose()

    async def state(self) -> dict[str, Any]:
        self._refresh_hardware_positions()
        arm_pose = self._arm_pose()
        arm_parameters = self.arm_parameters()
        current_height_cm = arm_pose["grasp_point_base_cm"]["up_z"]
        arm_parameters["vision_guided_grasp"][
            "current_pixel_to_motion_scale_px_per_cm"
        ] = round(pixel_alignment_px_per_cm_for_height(current_height_cm), 6)
        return {
            "status": "ok",
            "mode": self.config.mode,
            "serial_port": self.serial_port,
            "joint_positions": arm_pose["joint_positions"],
            "tcp_cm": arm_pose["grasp_point_base_cm"],
            "grasp_point_xyz_cm": arm_pose["grasp_point_xyz_cm"],
            "arm_pose": arm_pose,
            "arm_parameters": arm_parameters,
            "grip_locked": bool(self.runtime.j6_grip_locked),
        }

    def close(self) -> None:
        if self.closed:
            return
        self.closed = True
        self.runtime.close()


def build_arm_tool_registry(controller: JetArmToolController) -> ToolRegistry:
    """Create the fixed allow-list exposed to the model."""

    async def move(arguments: Mapping[str, Any]) -> object:
        return await controller.move_tcp(
            str(arguments.get("direction", "")),
            arguments.get("distance_cm"),
            arguments.get("speed_cm_s"),
        )

    async def wrist(arguments: Mapping[str, Any]) -> object:
        return await controller.rotate_wrist(
            str(arguments.get("direction", "")), arguments.get("duration_s")
        )

    async def gripper(arguments: Mapping[str, Any]) -> object:
        return await controller.control_gripper(
            str(arguments.get("action", "")), arguments.get("duration_s", 0.5)
        )

    async def gripper_position(arguments: Mapping[str, Any]) -> object:
        return await controller.set_gripper_position(
            arguments.get("position", DEFAULT_GRIPPER_RELEASE_POSITION),
            arguments.get("run_time_ms", DEFAULT_GRIPPER_POSITION_RUN_TIME_MS),
        )

    async def pixel_align(arguments: Mapping[str, Any]) -> object:
        return await controller.move_by_pixel_error(
            arguments.get("block_center_x"),
            arguments.get("block_center_y"),
            arguments.get("grasp_point_x"),
            arguments.get("grasp_point_y"),
            tolerance_px=arguments.get(
                "tolerance_px", DEFAULT_PIXEL_ALIGNMENT_TOLERANCE_PX
            ),
            step_duration_s=arguments.get(
                "step_duration_s", DEFAULT_PIXEL_ALIGNMENT_STEP_DURATION_S
            ),
            speed_saturation_px=arguments.get(
                "speed_saturation_px", DEFAULT_PIXEL_ALIGNMENT_SPEED_SATURATION_PX
            ),
        )

    async def target_pixel_control(arguments: Mapping[str, Any]) -> object:
        return await controller.control_to_target_pixel(
            arguments.get("target_x"),
            arguments.get("target_y"),
            arguments.get("grasp_point_x"),
            arguments.get("grasp_point_y"),
            descend_when_aligned=arguments.get("descend_when_aligned", True),
            descent_step_cm=arguments.get(
                "descent_step_cm", DEFAULT_DESCENT_RECALIBRATION_CM
            ),
            step_duration_s=arguments.get(
                "step_duration_s", DEFAULT_PIXEL_ALIGNMENT_STEP_DURATION_S
            ),
            speed_saturation_px=arguments.get(
                "speed_saturation_px", DEFAULT_PIXEL_ALIGNMENT_SPEED_SATURATION_PX
            ),
        )

    async def home(_arguments: Mapping[str, Any]) -> object:
        return await controller.go_home()

    async def stop(_arguments: Mapping[str, Any]) -> object:
        return await controller.stop_all()

    async def state(_arguments: Mapping[str, Any]) -> object:
        return await controller.state()

    return ToolRegistry(
        [
            ToolDefinition(
                name="move_jetarm_tcp",
                description=(
                    "Execute exactly one JetArm TCP movement through the camera-vector "
                    "runtime. up is grasp-point -> camera, down is camera -> grasp-point, "
                    "forward decreases grasp-point XYZ Y, backward increases XYZ Y, "
                    "left decreases XYZ X, and right increases XYZ X. distance_cm must be strictly less than 2 cm. For a longer user "
                    "request, the Agent must use a fresh RGB frame to decide only the current "
                    "movement, wait for status=ok, then capture a new frame before deciding "
                    "the next call. The controller never splits a long command."
                ),
                parameters={
                    "type": "object",
                    "properties": {
                        "direction": {
                            "type": "string",
                            "enum": [
                                "forward",
                                "backward",
                                "left",
                                "right",
                                "up",
                                "down",
                            ],
                        },
                        "distance_cm": {
                            "type": "number",
                            "exclusiveMinimum": 0,
                            "exclusiveMaximum": controller.config.max_distance_cm,
                        },
                        "speed_cm_s": {
                            "type": "number",
                            "minimum": controller.config.min_speed_cm_s,
                            "maximum": controller.config.max_speed_cm_s,
                            "default": controller.config.default_speed_cm_s,
                        },
                    },
                    "required": ["direction", "distance_cm"],
                    "additionalProperties": False,
                },
                handler=move,
            ),
            ToolDefinition(
                name="rotate_jetarm_wrist",
                description=(
                    "Rotate wrist joint J5 for a bounded duration, then stop it automatically."
                ),
                parameters={
                    "type": "object",
                    "properties": {
                        "direction": {
                            "type": "string",
                            "enum": ["clockwise", "counterclockwise"],
                        },
                        "duration_s": {
                            "type": "number",
                            "exclusiveMinimum": 0,
                            "maximum": controller.config.max_motor_duration_s,
                        },
                    },
                    "required": ["direction", "duration_s"],
                    "additionalProperties": False,
                },
                handler=wrist,
            ),
            ToolDefinition(
                name="control_jetarm_gripper",
                description=(
                    "Control J6 gripper. open/close stop automatically; grip_lock keeps "
                    "gripping until release_lock or stop."
                ),
                parameters={
                    "type": "object",
                    "properties": {
                        "action": {
                            "type": "string",
                            "enum": [
                                "open",
                                "close",
                                "grip_lock",
                                "release_lock",
                                "stop",
                            ],
                        },
                        "duration_s": {
                            "type": "number",
                            "exclusiveMinimum": 0,
                            "maximum": controller.config.max_motor_duration_s,
                        },
                    },
                    "required": ["action"],
                    "additionalProperties": False,
                },
                handler=gripper,
            ),
            ToolDefinition(
                name="set_jetarm_gripper_position",
                description=(
                    "Move J6 to a raw position. The grasp workflow uses position 370 "
                    "to keep the gripper released before a successful grasp, and after "
                    "a failed grasp before retrying."
                ),
                parameters={
                    "type": "object",
                    "properties": {
                        "position": {
                            "type": "integer",
                            "minimum": controller.settings.position_limits("J6")[0],
                            "maximum": controller.settings.position_limits("J6")[1],
                            "default": DEFAULT_GRIPPER_RELEASE_POSITION,
                        },
                        "run_time_ms": {
                            "type": "integer",
                            "minimum": 1,
                            "maximum": 30000,
                            "default": DEFAULT_GRIPPER_POSITION_RUN_TIME_MS,
                        },
                    },
                    "additionalProperties": False,
                },
                handler=gripper_position,
            ),
            ToolDefinition(
                name="move_jetarm_by_pixel_error",
                description=(
                    "Low-level compatibility pixel-error step. Do not use this for the "
                    "current visual grasp workflow; use control_jetarm_to_target_pixel "
                    "so the controller owns movement decisions. This tool moves one "
                    "small image-plane step from the grasp-point pixel toward the "
                    "block-center pixel. Distance is abs(pixel_error) divided by the "
                    "current height-linear px/cm scale (2 cm -> 50 px/cm; 25 cm -> 18 px/cm), capped by "
                    "the per-command limit; speed remains in the 0.7..1.5 cm/s range."
                ),
                parameters={
                    "type": "object",
                    "properties": {
                        "block_center_x": {"type": "number"},
                        "block_center_y": {"type": "number"},
                        "grasp_point_x": {"type": "number"},
                        "grasp_point_y": {"type": "number"},
                        "tolerance_px": {
                            "type": "number",
                            "minimum": 0,
                            "default": DEFAULT_PIXEL_ALIGNMENT_TOLERANCE_PX,
                        },
                        "step_duration_s": {
                            "type": "number",
                            "exclusiveMinimum": 0,
                            "maximum": controller.config.max_distance_cm,
                            "default": DEFAULT_PIXEL_ALIGNMENT_STEP_DURATION_S,
                        },
                        "speed_saturation_px": {
                            "type": "number",
                            "exclusiveMinimum": DEFAULT_PIXEL_ALIGNMENT_TOLERANCE_PX,
                            "default": DEFAULT_PIXEL_ALIGNMENT_SPEED_SATURATION_PX,
                        },
                    },
                    "required": [
                        "block_center_x",
                        "block_center_y",
                        "grasp_point_x",
                        "grasp_point_y",
                    ],
                    "additionalProperties": False,
                },
                handler=pixel_align,
            ),
            ToolDefinition(
                name="control_jetarm_to_target_pixel",
                description=(
                    "Controller-owned target-pixel workflow. The Agent only parses the "
                    "command, finds the target point in the latest image, and supplies "
                    "target_x/target_y. The controller reads FK height from joint "
                    "feedback, chooses tolerance by height (40/25/13/8 px), decides front/back/left/right "
                    "alignment distance using a height-linear px/cm scale (2 cm -> 50; 25 cm -> 18), and descends 2 cm when "
                    "aligned."
                ),
                parameters={
                    "type": "object",
                    "properties": {
                        "target_x": {"type": "number"},
                        "target_y": {"type": "number"},
                        "grasp_point_x": {"type": "number"},
                        "grasp_point_y": {"type": "number"},
                        "descend_when_aligned": {"type": "boolean", "default": True},
                        "descent_step_cm": {
                            "type": "number",
                            "exclusiveMinimum": 0,
                            "maximum": controller.config.max_distance_cm,
                            "default": DEFAULT_DESCENT_RECALIBRATION_CM,
                        },
                        "step_duration_s": {
                            "type": "number",
                            "exclusiveMinimum": 0,
                            "maximum": controller.config.max_distance_cm,
                            "default": DEFAULT_PIXEL_ALIGNMENT_STEP_DURATION_S,
                        },
                        "speed_saturation_px": {
                            "type": "number",
                            "exclusiveMinimum": DEFAULT_PIXEL_ALIGNMENT_TOLERANCE_PX,
                            "default": DEFAULT_PIXEL_ALIGNMENT_SPEED_SATURATION_PX,
                        },
                    },
                    "required": [
                        "target_x",
                        "target_y",
                        "grasp_point_x",
                        "grasp_point_y",
                    ],
                    "additionalProperties": False,
                },
                handler=target_pixel_control,
            ),
            ToolDefinition(
                name="move_jetarm_home",
                description="Move J1-J5 to the configured home pose and leave J6 unchanged.",
                parameters={"type": "object", "properties": {}},
                handler=home,
            ),
            ToolDefinition(
                name="stop_jetarm",
                description="Immediately stop Cartesian motion plus J5 and J6 motor motion.",
                parameters={"type": "object", "properties": {}},
                handler=stop,
            ),
            ToolDefinition(
                name="get_jetarm_state",
                description=(
                    "Read current joints, grasp-point coordinates, camera angle/pose, and "
                    "the arm kinematic, joint, Home, control, and coordinate parameters."
                ),
                parameters={"type": "object", "properties": {}},
                handler=state,
            ),
        ]
    )


def looks_like_arm_command(text: str) -> bool:
    """Conservatively identify commands that must produce a real arm tool call."""

    if looks_like_grasp_workflow_command(text):
        return True

    try:
        parse_compact_arm_command(text)
        return True
    except ArmControlError:
        pass
    normalized = text.strip().lower().replace(" ", "")
    phrases = (
        "向前",
        "前进",
        "向后",
        "后退",
        "向左",
        "向右",
        "向上",
        "向下",
        "上升",
        "下降",
        "抬高",
        "降低",
        "顺时针",
        "逆时针",
        "夹爪",
        "夹紧",
        "抓紧",
        "松开",
        "张开",
        "回到home",
        "回home",
        "回零",
        "机械臂停止",
        "停止机械臂",
        "机械臂状态",
        "机械臂参数",
        "关节限位",
        "home位置",
        "连杆尺寸",
        "抓取点坐标",
        "机械臂姿态",
        "相机夹角",
        "movearm",
        "moveforward",
        "movebackward",
        "gripper",
        "gohome",
        "stoparm",
    )
    return any(phrase in normalized for phrase in phrases)


def required_mcp_tool_for_command(text: str) -> str | None:
    """Map explicit natural-language actions to the MCP tool that must run."""

    if looks_like_grasp_workflow_command(text):
        return None

    if looks_like_camera_command(text):
        return "get_rgb_camera_frame"

    try:
        parse_compact_arm_command(text)
        return "move_jetarm"
    except ArmControlError:
        pass
    normalized = text.strip().lower().replace(" ", "")
    if any(
        phrase in normalized
        for phrase in (
            "向前",
            "前进",
            "向后",
            "后退",
            "向左",
            "向右",
            "向上",
            "向下",
            "上升",
            "下降",
            "抬高",
            "降低",
            "moveforward",
            "movebackward",
        )
    ):
        return "move_jetarm"
    if any(phrase in normalized for phrase in ("夹爪", "夹紧", "抓紧", "松开", "张开", "gripper")):
        return "control_jetarm_gripper"
    if any(phrase in normalized for phrase in ("顺时针", "逆时针")):
        return "rotate_jetarm_wrist"
    if any(phrase in normalized for phrase in ("回到home", "回home", "回零", "gohome")):
        return "move_jetarm_home"
    if any(phrase in normalized for phrase in ("机械臂停止", "停止机械臂", "stoparm")):
        return "stop_jetarm"
    if any(
        phrase in normalized
        for phrase in (
            "机械臂状态",
            "机械臂参数",
            "关节限位",
            "home位置",
            "连杆尺寸",
            "抓取点坐标",
            "机械臂姿态",
            "相机夹角",
        )
    ):
        return "get_jetarm_state"
    return None


def looks_like_camera_command(text: str) -> bool:
    """Return whether the user explicitly requests the current RGB view."""

    normalized = text.strip().lower().replace(" ", "")
    camera_terms = ("相机", "摄像头", "摄像机", "rgb", "画面", "图像")
    request_terms = (
        "看看",
        "看一下",
        "查看",
        "读取",
        "拍",
        "获取",
        "描述",
        "识别",
        "分析",
        "有什么",
        "看到了什么",
    )
    return any(term in normalized for term in camera_terms) and any(
        term in normalized for term in request_terms
    )


def looks_like_grasp_workflow_command(text: str) -> bool:
    """Return whether the user is asking for visual block grasping, not just J6."""

    normalized = text.strip().lower().replace(" ", "")
    grasp_terms = ("抓取", "夹取", "拿取", "拾取", "grasp", "pick")
    object_terms = ("物块", "方块", "积木", "目标", "物体", "block", "cube", "object")
    return any(term in normalized for term in grasp_terms) and (
        any(term in normalized for term in object_terms)
        or normalized in {"抓取", "夹取", "拿取", "grasp", "pick"}
    )


ARM_TOOL_SYSTEM_PROMPT = """
机械臂工具规则：
1. 只有用户明确要求移动或操作夹爪时才调用会改变机械臂状态的工具，禁止自行追加动作；读取状态和参数可在回答或执行任务确有需要时调用get_jetarm_state。
2. “上/下”使用camera_vector控制系：上=抓取点到摄像头方向，下=摄像头到抓取点方向；“前/后/左/右”使用抓取点XYZ水平坐标：前=Y减小，后=Y增大，左=X减小，右=X增大。水平运动时把目标换算到XYZ的X/Y方向，Z不参与移动目标并保持当前高度；运动过程中保持摄像头-抓取点姿态尽量不变，距离必须保持用户给出的厘米数。
3. 用户没有给出距离时先询问，不得猜测。未指定速度时使用1.5cm/s，速度只能在1到5cm/s。
4. 视觉抓取目标时，Agent只解析用户命令，并在最新RGB图像中寻找目标点像素target_x/target_y；不得决定前后左右方向、下降距离或运动速度。
5. get_rgb_camera_frame会返回camera.grasp_point_pixel。视觉抓取时调用control_jetarm_to_target_pixel并只提供目标点像素，抓取点像素和运动决策由控制程序负责。
6. control_jetarm_to_target_pixel会根据关节反馈/FK解算抓取点高度，按高度选择像素容差：>15cm为40px，>10且<=15cm为25px，>5且<=10cm为13px，<=5cm为8px。
7. 目标点与抓取点未重合时，控制程序自行决定前后左右移动；移动距离按像素误差除以当前高度对应的线性比例计算：高度2cm时50px/cm，高度25cm时18px/cm，其他高度线性插值/外推。
8. 已重合时，控制程序以2cm/s向下运动，并在下降过程中持续基于关节角度/FK解算抓取点位置。
9. 每下降2cm或任一机械臂移动返回status=ok后，旧图像立即失效；必须重新调用get_rgb_camera_frame，再由Agent重新寻找目标点像素。
10. 普通手动move_jetarm移动仍必须先取图，每条距离严格小于2cm，推荐最多1.9cm；不得一次生成后续动作序列。
11. 取图失败或任一移动命令失败后立即停止后续移动，不得沿用旧图像，也不得声称动作完成；存在运动风险时调用stop_jetarm。
12. 发生错误、方向不明确或用户要求停止时调用stop_jetarm。
13. 当前只使用单路RGB相机，不得请求或声称使用深度流。
14. 用户要求查看、描述、识别或分析相机画面时，也必须调用get_rgb_camera_frame；只有收到真实图像后才能描述画面。
15. 需要机械臂参数时调用get_jetarm_state并读取arm_parameters，禁止猜测关节限位、Home、几何尺寸或坐标系。
""".strip()
