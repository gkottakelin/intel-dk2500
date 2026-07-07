"""Safe AI tool adapter for the existing Ubuntu JetArm operation terminal."""

from __future__ import annotations

import asyncio
import importlib
import math
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
SleepFunction = Callable[[float], Awaitable[None]]


class ArmControlError(RuntimeError):
    """Raised when a requested arm action violates safety or cannot execute."""


@dataclass(frozen=True)
class ArmControlConfig:
    mode: str = "dry-run"
    serial_port: str | None = None
    terminal_config_path: Path = DEFAULT_TERMINAL_CONFIG
    max_distance_cm: float = 10.0
    max_motor_duration_s: float = 2.0

    def validate(self) -> None:
        if self.mode not in {"dry-run", "hardware"}:
            raise ArmControlError(f"机械臂模式必须是dry-run或hardware，收到: {self.mode}")
        if self.max_distance_cm <= 0:
            raise ArmControlError("max_distance_cm必须大于0")
        if self.max_motor_duration_s <= 0:
            raise ArmControlError("max_motor_duration_s必须大于0")


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

        terminal = None
        import_error: Exception | None = None
        for module_name in (
            "ubuntu22_04_operation_terminal.jetarm_terminal",
            "project.ubuntu22_04_operation_terminal.jetarm_terminal",
        ):
            try:
                terminal = importlib.import_module(module_name)
                break
            except (ImportError, ModuleNotFoundError) as exc:
                import_error = exc
        if terminal is None:
            raise ArmControlError(
                "机械臂工具依赖加载失败，请执行: "
                "python -m pip install -r requirements-ai.txt"
            ) from import_error

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

    def _set_cartesian_direction(self, direction: str) -> float:
        self.runtime.set_vertical_direction(0)
        self.runtime.center_joystick()
        if direction == "forward":
            self.runtime.set_joystick(0.0, -1.0)
            return self.settings.max_horizontal_speed_m_s
        if direction == "backward":
            self.runtime.set_joystick(0.0, 1.0)
            return self.settings.max_horizontal_speed_m_s
        if direction == "left":
            self.runtime.set_joystick(-1.0, 0.0)
            return self.settings.max_horizontal_speed_m_s
        if direction == "right":
            self.runtime.set_joystick(1.0, 0.0)
            return self.settings.max_horizontal_speed_m_s
        if direction == "up":
            self.runtime.set_vertical_direction(1)
            return self.settings.vertical_speed_m_s
        if direction == "down":
            self.runtime.set_vertical_direction(-1)
            return self.settings.vertical_speed_m_s
        raise ArmControlError(f"不支持的TCP方向: {direction}")

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

    async def move_tcp(self, direction: str, distance_cm: object) -> dict[str, Any]:
        """Move TCP in the robot-base frame through bounded incremental IK steps."""

        distance = self._validate_positive_number(
            distance_cm, "distance_cm", self.config.max_distance_cm
        )
        self._refresh_hardware_positions()
        start_tcp = self.runtime.model.tcp(self.runtime.positions).copy()
        speed_m_s = self._set_cartesian_direction(direction)
        if speed_m_s <= 0:
            self._stop_cartesian()
            raise ArmControlError("配置的TCP速度必须大于0")

        total_time_s = (distance / 100.0) / speed_m_s
        remaining_s = total_time_s
        steps = 0
        try:
            while remaining_s > 1e-9:
                dt = min(self.settings.tick_s, remaining_s)
                if not self.runtime.step_cartesian(dt):
                    raise ArmControlError(
                        f"向{direction}运动在第{steps + 1}步无法继续，"
                        "可能已到关节限位或当前姿态不可达"
                    )
                steps += 1
                remaining_s -= dt
                if self.config.mode == "hardware":
                    await self.sleep(dt)
        finally:
            self._stop_cartesian()

        if self.config.mode == "hardware":
            await self.sleep(0.05)
            self._refresh_hardware_positions()
        end_tcp = self.runtime.model.tcp(self.runtime.positions).copy()
        delta_cm = (end_tcp - start_tcp) * 100.0
        axis = {
            "forward": (0, 1.0),
            "backward": (0, -1.0),
            "left": (1, 1.0),
            "right": (1, -1.0),
            "up": (2, 1.0),
            "down": (2, -1.0),
        }[direction]
        estimated_distance = float(delta_cm[axis[0]]) * axis[1]
        return {
            "status": "ok",
            "mode": self.config.mode,
            "direction": direction,
            "requested_distance_cm": distance,
            "estimated_distance_cm": round(estimated_distance, 3),
            "estimated_delta_cm": {
                "forward_x": round(float(delta_cm[0]), 3),
                "left_y": round(float(delta_cm[1]), 3),
                "up_z": round(float(delta_cm[2]), 3),
            },
            "steps": steps,
            "joint_positions": dict(self.runtime.positions),
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

    async def state(self) -> dict[str, Any]:
        self._refresh_hardware_positions()
        tcp_cm = self.runtime.model.tcp(self.runtime.positions) * 100.0
        return {
            "status": "ok",
            "mode": self.config.mode,
            "serial_port": self.serial_port,
            "joint_positions": dict(self.runtime.positions),
            "tcp_cm": {
                "forward_x": round(float(tcp_cm[0]), 3),
                "left_y": round(float(tcp_cm[1]), 3),
                "up_z": round(float(tcp_cm[2]), 3),
            },
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
            str(arguments.get("direction", "")), arguments.get("distance_cm")
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
                    "Move the JetArm TCP by an explicit distance in the robot-base "
                    "coordinate frame. Use only when the user explicitly requests motion."
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
                            "maximum": controller.config.max_distance_cm,
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
                description="Stop J5/J6 and move all JetArm joints to the configured home pose.",
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
                description="Read current J1-J4 positions and estimated TCP coordinates.",
                parameters={"type": "object", "properties": {}},
                handler=state,
            ),
        ]
    )


def looks_like_arm_command(text: str) -> bool:
    """Conservatively identify commands that must produce a real arm tool call."""

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
        "movearm",
        "moveforward",
        "movebackward",
        "gripper",
        "gohome",
        "stoparm",
    )
    return any(phrase in normalized for phrase in phrases)


ARM_TOOL_SYSTEM_PROMPT = """
机械臂工具规则：
1. 只有用户明确要求移动或操作夹爪时才调用机械臂工具，禁止自行追加动作。
2. “前/后/左/右/上/下”使用机械臂基座坐标系；距离必须保持用户给出的厘米数。
3. 用户没有给出距离时先询问，不得猜测。单次距离超过工具上限时必须拒绝并解释。
4. 每个动作完成后读取工具结果；只有status=ok时才能声称动作完成。
5. 发生错误、方向不明确或用户要求停止时调用stop_jetarm。
6. 当前没有相机反馈，不能声称看见物体或完成视觉闭环夹取。
""".strip()
