"""Safe AI tool adapter for the existing Ubuntu JetArm operation terminal."""

from __future__ import annotations

import asyncio
import importlib
import math
import re
from collections.abc import Awaitable, Callable, Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

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
SleepFunction = Callable[[float], Awaitable[None]]


class ArmControlError(RuntimeError):
    """Raised when a requested arm action violates safety or cannot execute."""


@dataclass(frozen=True)
class ArmControlConfig:
    mode: str = "dry-run"
    serial_port: str | None = None
    terminal_config_path: Path = DEFAULT_TERMINAL_CONFIG
    max_distance_cm: float = MAX_AGENT_MOVE_COMMAND_CM
    default_speed_cm_s: float = DEFAULT_TCP_SPEED_CM_S
    min_speed_cm_s: float = MIN_TCP_SPEED_CM_S
    max_speed_cm_s: float = MAX_TCP_SPEED_CM_S
    max_motor_duration_s: float = 2.0

    def validate(self) -> None:
        if self.mode not in {"dry-run", "hardware"}:
            raise ArmControlError(f"机械臂模式必须是dry-run或hardware，收到: {self.mode}")
        if self.max_distance_cm <= 0:
            raise ArmControlError("max_distance_cm必须大于0")
        if self.max_distance_cm > MAX_AGENT_MOVE_COMMAND_CM:
            raise ArmControlError(
                f"max_distance_cm不能超过{MAX_AGENT_MOVE_COMMAND_CM:g}；"
                f"Agent每条移动命令必须严格小于{MAX_AGENT_MOVE_COMMAND_CM:g}cm"
            )
        if not 0 < self.min_speed_cm_s <= self.default_speed_cm_s <= self.max_speed_cm_s:
            raise ArmControlError("TCP速度配置必须满足0 < 最小值 <= 默认值 <= 最大值")
        if self.max_motor_duration_s <= 0:
            raise ArmControlError("max_motor_duration_s必须大于0")


def _load_terminal_module() -> Any:
    import_error: Exception | None = None
    for module_name in (
        "ubuntu22_04_operation_terminal.jetarm_terminal",
        "project.ubuntu22_04_operation_terminal.jetarm_terminal",
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

        terminal = _load_terminal_module()

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

        self.runtime = terminal.ManualServoRuntime(
            self.controller, self.settings, logger=self.logger
        )
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
        base_directions = {
            "forward": (1.0, 0.0, 0.0),
            "backward": (-1.0, 0.0, 0.0),
            "left": (0.0, 1.0, 0.0),
            "right": (0.0, -1.0, 0.0),
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

    def _set_cartesian_direction(
        self, direction: str
    ) -> tuple[float, tuple[float, float, float]]:
        self.runtime.set_vertical_direction(0)
        self.runtime.center_joystick()
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

    async def _move_tcp_segment(
        self, direction: str, distance_cm: float, speed_cm_s: float
    ) -> dict[str, Any]:
        """Execute one physical Cartesian segment no longer than max_segment_cm."""

        self._refresh_hardware_positions()
        start_tcp = self.runtime.model.tcp(self.runtime.positions).copy()
        requested_speed_m_s = speed_cm_s / 100.0
        planner_speed_m_s, direction_unit = self._set_cartesian_direction(direction)
        camera_reference_before_move = self._camera_pose()

        planner_time_s = (distance_cm / 100.0) / planner_speed_m_s
        remaining_s = planner_time_s
        steps = 0
        try:
            while remaining_s > 1e-9:
                planner_dt = min(self.settings.tick_s, remaining_s)
                execution_dt = planner_dt * planner_speed_m_s / requested_speed_m_s
                if not self.runtime.step_cartesian(
                    planner_dt, run_time_s=execution_dt
                ):
                    raise ArmControlError(
                        f"向{direction}运动在第{steps + 1}步无法继续，"
                        "可能已到关节限位或当前姿态不可达"
                    )
                steps += 1
                remaining_s -= planner_dt
                if self.config.mode == "hardware":
                    await self.sleep(execution_dt)
        finally:
            self._stop_cartesian()

        if self.config.mode == "hardware":
            await self.sleep(0.05)
            self._refresh_hardware_positions()
        end_tcp = self.runtime.model.tcp(self.runtime.positions).copy()
        delta_cm = (end_tcp - start_tcp) * 100.0
        estimated_distance = sum(
            float(delta_cm[index]) * direction_unit[index] for index in range(3)
        )
        return {
            "status": "ok",
            "mode": self.config.mode,
            "direction": direction,
            "requested_distance_cm": distance_cm,
            "speed_cm_s": speed_cm_s,
            "estimated_distance_cm": round(estimated_distance, 3),
            "estimated_delta_cm": {
                "forward_x": round(float(delta_cm[0]), 3),
                "left_y": round(float(delta_cm[1]), 3),
                "up_z": round(float(delta_cm[2]), 3),
            },
            "direction_reference": (
                "camera_view" if direction in {"up", "down"} else "base"
            ),
            "direction_unit_base": {
                "forward_x": round(direction_unit[0], 6),
                "left_y": round(direction_unit[1], 6),
                "up_z": round(direction_unit[2], 6),
            },
            "camera_pose_before_move": camera_reference_before_move,
            "steps": steps,
            "joint_positions": dict(self.runtime.positions),
        }

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
        return {
            "mount": "eye_in_hand_near_j5_j6",
            "home_line_of_sight_angle_from_vertical_deg": 0.0,
            "line_of_sight_angle_from_vertical_deg": round(
                abs(math.degrees(tilt_rad)), 3
            ),
            "signed_pitch_from_home_deg": round(math.degrees(tilt_rad), 3),
            "signed_pitch_positive_direction": "toward_arm_radial_forward",
            "view_up_unit_base": {
                "forward_x": round(up[0], 6),
                "left_y": round(up[1], 6),
                "up_z": round(up[2], 6),
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
            "camera": self._camera_pose(),
        }

    def arm_parameters(self) -> dict[str, Any]:
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
            "agent_direction_frames": {
                "forward_backward_left_right": "base",
                "up_down": "camera_view",
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
                "home_angle_from_vertical_deg": 0.0,
                "signed_pitch_formula": "pitch(J2+J3+J4)-home_pitch(J2+J3+J4)",
            },
        }

    async def pose(self) -> dict[str, Any]:
        self._refresh_hardware_positions()
        return self._arm_pose()

    async def state(self) -> dict[str, Any]:
        self._refresh_hardware_positions()
        arm_pose = self._arm_pose()
        return {
            "status": "ok",
            "mode": self.config.mode,
            "serial_port": self.serial_port,
            "joint_positions": arm_pose["joint_positions"],
            "tcp_cm": arm_pose["grasp_point_base_cm"],
            "arm_pose": arm_pose,
            "arm_parameters": self.arm_parameters(),
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
                    "Execute exactly one JetArm TCP movement. forward/backward/left/right "
                    "use the robot-base frame; up/down use the current camera-view frame "
                    "calibrated to base +Z at Home. distance_cm must be strictly less than "
                    "2 cm. For a longer user "
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


ARM_TOOL_SYSTEM_PROMPT = """
机械臂工具规则：
1. 只有用户明确要求移动或操作夹爪时才调用会改变机械臂状态的工具，禁止自行追加动作；读取状态和参数可在回答或执行任务确有需要时调用get_jetarm_state。
2. “前/后/左/右”使用机械臂基座坐标系；“上/下”使用当前相机视角坐标，Home时相机视角的上等于基座+Z。距离必须保持用户给出的厘米数。
3. 用户没有给出距离时先询问，不得猜测。未指定速度时使用1.5cm/s，速度只能在1到5cm/s。
4. 每次移动前必须调用get_rgb_camera_frame；该工具会在同一个结果中返回最新RGB图像、抓取点基座坐标和相机视线相对竖直方向的夹角。只有图像和对应姿态都已传给Agent，才允许Agent决定本次动作。
5. Agent每次只根据当前最新图像下发一条move_jetarm命令，距离必须严格小于2cm，推荐最多1.9cm；不得一次生成后续动作序列。
6. 本条移动返回status=ok后，必须重新调用get_rgb_camera_frame，把动作后的新图像传给Agent，再决定是否及如何移动下一条。
7. 控制器不会替Agent切分长距离。长距离目标必须通过“取图→单步移动→重新取图”的视觉闭环逐步完成。
8. 取图失败或任一移动命令失败后立即停止后续移动，不得沿用旧图像，也不得声称动作完成；存在运动风险时调用stop_jetarm。
9. 发生错误、方向不明确或用户要求停止时调用stop_jetarm。
10. 当前只使用单路RGB相机，不得请求或声称使用深度流。
11. 用户要求查看、描述、识别或分析相机画面时，也必须调用get_rgb_camera_frame；只有收到真实图像后才能描述画面。
12. 每张RGB图像只授权紧随其后的一条移动命令；机械臂位置改变后旧图像立即失效，禁止基于旧图连续移动。
13. 需要机械臂参数时调用get_jetarm_state并读取arm_parameters，禁止猜测关节限位、Home、几何尺寸或坐标系。
""".strip()
