"""Local stdio MCP server for JetArm hardware, adapted from robot_MCP."""

from __future__ import annotations

import argparse
import atexit
import asyncio
import base64
import json
import logging
import math
import sys
from pathlib import Path
from typing import Any, Callable, Literal, Optional

from .arm_control import (
    DEFAULT_DESCENT_RECALIBRATION_CM,
    DEFAULT_GRIPPER_POSITION_RUN_TIME_MS,
    DEFAULT_GRIPPER_RELEASE_POSITION,
    DEFAULT_PIXEL_ALIGNMENT_SPEED_SATURATION_PX,
    DEFAULT_PIXEL_ALIGNMENT_STEP_DURATION_S,
    DEFAULT_PIXEL_ALIGNMENT_TOLERANCE_PX,
    MAX_AGENT_MOVE_COMMAND_CM,
    PIXEL_ALIGNMENT_SCALE_FAR_HEIGHT_CM,
    PIXEL_ALIGNMENT_SCALE_FAR_PX_PER_CM,
    PIXEL_ALIGNMENT_SCALE_NEAR_HEIGHT_CM,
    PIXEL_ALIGNMENT_SCALE_NEAR_PX_PER_CM,
    ArmControlConfig,
    JetArmToolController,
    parse_compact_arm_command,
)
from .device_config import (
    DEFAULT_DEVICE_CONFIG_PATH,
    PROJECT_ROOT,
    RuntimeDeviceConfig,
    validate_device_interfaces,
)
from .rgb_camera import RGBJpegFrame, capture_rgb_jpeg


DEFAULT_WORKFLOW_PATH = PROJECT_ROOT / "workflows" / "jetarm_mcp_workflow.md"
AGENT_VISUAL_WORKFLOW_MAX_DISTANCE_CM = 100.0
LOGGER = logging.getLogger("jetarm_mcp")


class JetArmMCPService:
    """Hardware service exposed through MCP tools."""

    def __init__(
        self,
        devices: RuntimeDeviceConfig,
        *,
        controller_factory: Callable[[ArmControlConfig], JetArmToolController] = JetArmToolController,
        camera_capture: Callable[[str], RGBJpegFrame] = capture_rgb_jpeg,
        workflow_path: str | Path = DEFAULT_WORKFLOW_PATH,
        max_distance_cm: float = MAX_AGENT_MOVE_COMMAND_CM,
    ) -> None:
        self.devices = devices
        self.controller_factory = controller_factory
        self.camera_capture = camera_capture
        self.workflow_path = Path(workflow_path)
        self.max_distance_cm = max_distance_cm
        self._controller: JetArmToolController | None = None
        configured_grasp_point = (
            {
                "x": round(float(devices.grasp_point_x), 3),
                "y": round(float(devices.grasp_point_y), 3),
                "source": "device_config",
            }
            if devices.grasp_point_x is not None
            and devices.grasp_point_y is not None
            else None
        )
        self._configured_grasp_point_pixel: dict[str, Any] | None = (
            dict(configured_grasp_point) if configured_grasp_point else None
        )
        self._last_grasp_point_pixel: dict[str, Any] | None = (
            dict(configured_grasp_point) if configured_grasp_point else None
        )
        self._last_rgb_frame_size: tuple[int, int] | None = None
        self._grasp_final_phase = False
        self._gripper_prepared_for_grasp = False
        self._awaiting_grasp_visual_confirmation = False
        self._grasp_step_records: list[dict[str, Any]] = []

    def _terminal_config_path(self) -> Path:
        path = Path(self.devices.arm_terminal_config)
        return path if path.is_absolute() else (PROJECT_ROOT / path).resolve()

    def controller(self) -> JetArmToolController:
        if self.devices.arm_mode == "off":
            raise RuntimeError("机械臂模式为off，请先运行设备配置程序")
        if self._controller is None:
            LOGGER.info("Initializing JetArm controller in %s mode", self.devices.arm_mode)
            self._controller = self.controller_factory(
                ArmControlConfig(
                    mode=self.devices.arm_mode,
                    serial_port=self.devices.arm_port or None,
                    terminal_config_path=self._terminal_config_path(),
                    max_distance_cm=AGENT_VISUAL_WORKFLOW_MAX_DISTANCE_CM,
                    allow_extended_distance=True,
                    default_speed_cm_s=1.5,
                    min_speed_cm_s=1.0,
                    max_speed_cm_s=5.0,
                    camera_vector_version="v2",
                    manual_progress_check_enabled=False,
                )
            )
            LOGGER.info("JetArm controller initialized")
        return self._controller

    def initial_instructions(self) -> str:
        try:
            return self.workflow_path.read_text(encoding="utf-8")
        except FileNotFoundError:
            return "JetArm MCP工作流文件缺失。"

    async def capture_rgb(self) -> RGBJpegFrame:
        camera = self.devices.rgb_camera.strip()
        if not camera:
            raise RuntimeError("未配置Orbbec相机序列号/UID，请先运行设备配置程序")
        LOGGER.info("Capturing RGB frame from %s", camera)
        return await asyncio.to_thread(self.camera_capture, camera)

    async def observation_arm_pose(self) -> dict[str, Any]:
        """Read the pose paired with a camera observation without forcing arm use."""

        if self.devices.arm_mode == "off":
            return {
                "status": "unavailable",
                "reason": "机械臂模式为off，无法读取抓取点坐标和相机姿态",
            }
        try:
            return {"status": "ok", **(await self.controller().pose())}
        except Exception as exc:
            return {"status": "error", "error": str(exc)}

    def device_status(self) -> dict[str, Any]:
        errors = validate_device_interfaces(self.devices)
        return {
            "status": "ok" if not errors else "error",
            "arm_mode": self.devices.arm_mode,
            "arm_port": self.devices.arm_port or None,
            "rgb_camera": self.devices.rgb_camera or None,
            "rgb_camera_name": self.devices.rgb_camera_name or None,
            "errors": errors,
        }

    async def move(self, command: str, speed_cm_s: float = 1.5) -> dict[str, Any]:
        LOGGER.info("Executing compact command %s at %.3f cm/s", command, speed_cm_s)
        _direction, distance_cm = parse_compact_arm_command(command)
        if distance_cm >= self.max_distance_cm:
            raise RuntimeError(
                f"Agent普通移动单次必须小于{self.max_distance_cm:g}cm"
            )
        result = await self.controller().execute_compact_command(command, speed_cm_s)
        result["mcp"] = "move_jetarm"
        LOGGER.info("Compact command completed with status=%s", result.get("status"))
        return result

    async def state(self) -> dict[str, Any]:
        LOGGER.info("Reading JetArm state")
        result = await self.controller().state()
        result["mcp"] = "get_jetarm_state"
        LOGGER.info("JetArm state read complete")
        return result

    async def home(self) -> dict[str, Any]:
        result = await self.controller().go_home()
        result["mcp"] = "move_jetarm_home"
        return result

    async def stop(self) -> dict[str, Any]:
        result = await self.controller().stop_all()
        result["mcp"] = "stop_jetarm"
        return result

    async def wrist(self, direction: str, duration_s: float) -> dict[str, Any]:
        result = await self.controller().rotate_wrist(direction, duration_s)
        result["mcp"] = "rotate_jetarm_wrist"
        return result

    async def gripper(self, action: str, duration_s: float = 0.5) -> dict[str, Any]:
        result = await self.controller().control_gripper(action, duration_s)
        result["mcp"] = "control_jetarm_gripper"
        return result

    def set_grasp_point_pixel(self, x: float, y: float) -> dict[str, Any]:
        """Set the user-measured grasp pixel before an Agent grasp workflow."""

        try:
            resolved_x = float(x)
            resolved_y = float(y)
        except (TypeError, ValueError) as exc:
            raise RuntimeError("抓取点像素必须是两个数字") from exc
        if (
            not math.isfinite(resolved_x)
            or not math.isfinite(resolved_y)
            or resolved_x < 0.0
            or resolved_y < 0.0
        ):
            raise RuntimeError("抓取点像素必须是大于等于0的有限数字")
        point = {
            "x": round(resolved_x, 3),
            "y": round(resolved_y, 3),
            "source": "user_input_before_agent_grasp",
        }
        self._configured_grasp_point_pixel = dict(point)
        self._last_grasp_point_pixel = dict(point)
        self._grasp_final_phase = False
        self._gripper_prepared_for_grasp = False
        self._awaiting_grasp_visual_confirmation = False
        self._grasp_step_records.clear()
        return {
            "status": "ok",
            "mcp": "set_jetarm_grasp_point_pixel",
            "grasp_point_pixel": point,
            "workflow_reset": True,
        }

    def grasp_point_pixel_for_frame(
        self, width: int, height: int
    ) -> dict[str, Any] | None:
        point = self._configured_grasp_point_pixel
        if point is None:
            return None
        x = float(point["x"])
        y = float(point["y"])
        if not 0.0 <= x < float(width) or not 0.0 <= y < float(height):
            raise RuntimeError(
                "调用前输入的抓取点像素超出当前RGB图像范围："
                f"point=({x:g}, {y:g}), image={width}x{height}"
            )
        return dict(point)

    @staticmethod
    def _pixel_vertical_relation(target_y: float, grasp_y: float) -> str:
        if target_y < grasp_y - 2.0:
            return "above"
        if target_y > grasp_y + 2.0:
            return "below"
        return "same_y"

    def _normalize_agent_target_y(
        self,
        target_y: float,
        grasp_y: float,
        declared_relation: str | None,
    ) -> tuple[float, dict[str, Any]]:
        received_y = float(target_y)
        if declared_relation is None:
            return received_y, {
                "coordinate_origin": "top_left",
                "y_axis_direction": "down",
                "declared_vertical_relation": None,
                "normalization": "compatibility_no_relation_check",
                "received_target_y": received_y,
                "normalized_target_y": received_y,
            }
        relation = str(declared_relation).strip().lower()
        if relation not in {"above", "below", "same_y"}:
            raise RuntimeError(
                "target_vertical_relation必须是above、below或same_y"
            )
        direct_relation = self._pixel_vertical_relation(received_y, grasp_y)
        if direct_relation == relation:
            return received_y, {
                "coordinate_origin": "top_left",
                "y_axis_direction": "down",
                "declared_vertical_relation": relation,
                "numeric_vertical_relation": direct_relation,
                "normalization": "none_top_left_y_down_confirmed",
                "received_target_y": received_y,
                "normalized_target_y": received_y,
            }
        if self._last_rgb_frame_size is None:
            raise RuntimeError(
                "没有最新RGB原图尺寸，无法校验Agent目标Y坐标方向"
            )
        _width, height = self._last_rgb_frame_size
        candidates = [float(height) - received_y, float(height - 1) - received_y]
        for candidate in candidates:
            if (
                0.0 <= candidate < float(height)
                and self._pixel_vertical_relation(candidate, grasp_y) == relation
            ):
                return candidate, {
                    "coordinate_origin": "top_left",
                    "y_axis_direction": "down",
                    "declared_vertical_relation": relation,
                    "numeric_vertical_relation_before": direct_relation,
                    "numeric_vertical_relation_after": relation,
                    "normalization": "bottom_origin_y_up_converted_to_top_left_y_down",
                    "received_target_y": received_y,
                    "normalized_target_y": candidate,
                    "image_height": height,
                }
        raise RuntimeError(
            "Agent目标Y坐标与其声明的上下关系矛盾，且无法按原图高度安全转换："
            f"target_y={received_y:g}, grasp_y={grasp_y:g}, relation={relation}"
        )

    async def set_gripper_position(
        self,
        position: int = DEFAULT_GRIPPER_RELEASE_POSITION,
        run_time_ms: int = DEFAULT_GRIPPER_POSITION_RUN_TIME_MS,
    ) -> dict[str, Any]:
        result = await self.controller().set_gripper_position(position, run_time_ms)
        result["mcp"] = "set_jetarm_gripper_position"
        return result

    async def pixel_align(
        self,
        block_center_x: float,
        block_center_y: float,
        grasp_point_x: float,
        grasp_point_y: float,
        tolerance_px: float = DEFAULT_PIXEL_ALIGNMENT_TOLERANCE_PX,
        step_duration_s: float = DEFAULT_PIXEL_ALIGNMENT_STEP_DURATION_S,
        speed_saturation_px: float = DEFAULT_PIXEL_ALIGNMENT_SPEED_SATURATION_PX,
    ) -> dict[str, Any]:
        result = await self.controller().move_by_pixel_error(
            block_center_x,
            block_center_y,
            grasp_point_x,
            grasp_point_y,
            tolerance_px=tolerance_px,
            step_duration_s=step_duration_s,
            speed_saturation_px=speed_saturation_px,
        )
        result["mcp"] = "move_jetarm_by_pixel_error"
        return result

    @staticmethod
    def _camera_grasp_angle_deg(result: dict[str, Any]) -> float | None:
        angle = result.get("v2_returned_camera_line_angle_deg")
        if isinstance(angle, (int, float)):
            return round(float(angle), 3)
        hold = result.get("camera_line_angle_hold")
        if isinstance(hold, dict):
            angle = hold.get("actual_after_deg")
            if isinstance(angle, (int, float)):
                return round(float(angle), 3)
        return None

    def _record_grasp_step(
        self,
        target_x: float,
        target_y: float,
        result: dict[str, Any],
        *,
        motion_step: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        source = motion_step or result
        original = source.get(
            "original_grasp_point_xyz_cm",
            result.get("grasp_point_xyz_before_cm"),
        )
        expected = source.get(
            "expected_grasp_point_xyz_cm",
            result.get("grasp_point_xyz_expected_cm", original),
        )
        actual = source.get(
            "actual_grasp_point_xyz_cm",
            result.get("grasp_point_xyz_after_cm", original),
        )
        direction = source.get("direction", result.get("direction"))
        distance_cm = source.get(
            "requested_distance_cm", result.get("requested_distance_cm")
        )
        if direction is None:
            direction = "none"
        if not isinstance(distance_cm, (int, float)):
            distance_cm = 0.0 if direction == "none" else None
        plan = {
            "direction": str(direction),
            "distance_cm": (
                round(float(distance_cm), 3) if distance_cm is not None else None
            ),
        }
        angle = source.get("camera_grasp_vertical_angle_deg")
        if not isinstance(angle, (int, float)):
            angle = self._camera_grasp_angle_deg(result)
        record = {
            "step": len(self._grasp_step_records) + 1,
            "target_pixel": {
                "x": round(float(target_x), 3),
                "y": round(float(target_y), 3),
            },
            "original_grasp_point_xyz_cm": original,
            "motion_plan": plan,
            "expected_grasp_point_xyz_cm": expected,
            "actual_grasp_point_xyz_cm": actual,
            "camera_grasp_vertical_angle_deg": angle,
        }
        self._grasp_step_records.append(record)
        return record

    def _attach_grasp_step_records(
        self, result: dict[str, Any], new_records: list[dict[str, Any]]
    ) -> dict[str, Any]:
        result["grasp_step_record"] = new_records[-1] if new_records else None
        result["new_grasp_step_records"] = list(new_records)
        # Return only records produced by this tool call. Earlier calls already
        # remain in the Agent/tool history; repeating the full history here
        # would grow the model context quadratically during a long closed loop.
        result["grasp_step_records"] = list(new_records)
        result["grasp_step_record_count_total"] = len(self._grasp_step_records)
        result["grasp_step_record_format"] = [
            "target_pixel",
            "original_grasp_point_xyz_cm",
            "motion_plan",
            "expected_grasp_point_xyz_cm",
            "actual_grasp_point_xyz_cm",
            "camera_grasp_vertical_angle_deg",
        ]
        return result

    async def _complete_final_grasp(
        self, target_x: float, target_y: float, alignment: dict[str, Any]
    ) -> dict[str, Any]:
        from .arm_control import FINAL_GRASP_HEIGHT_CM

        final_descent = await self.controller().descend_to_height(
            FINAL_GRASP_HEIGHT_CM
        )
        new_records: list[dict[str, Any]] = []
        raw_steps = final_descent.get("motion_steps")
        if isinstance(raw_steps, list):
            for raw_step in raw_steps:
                if isinstance(raw_step, dict):
                    new_records.append(
                        self._record_grasp_step(
                            target_x,
                            target_y,
                            final_descent,
                            motion_step=raw_step,
                        )
                    )
        if not new_records:
            new_records.append(
                self._record_grasp_step(target_x, target_y, final_descent)
            )
        final_height = final_descent.get(
            "height_after_cm", final_descent.get("height_cm")
        )
        tolerance = float(final_descent.get("target_tolerance_cm", 0.0))
        safely_reached = (
            final_descent.get("status") == "ok"
            and isinstance(final_height, (int, float))
            and float(final_height) <= FINAL_GRASP_HEIGHT_CM + tolerance
        )
        if not safely_reached:
            result = {
                **alignment,
                "status": "error",
                "mcp": "control_jetarm_to_target_pixel",
                "controller_decision": "final_descent_failed",
                "error": final_descent.get(
                    "error", "最终下降未安全到达抓取高度"
                ),
                "final_descent": final_descent,
                "grasp_completed": False,
                "requires_new_target_pixel": False,
            }
            return self._attach_grasp_step_records(result, new_records)

        grip = await self.controller().control_gripper("grip_lock")
        if grip.get("status") != "ok":
            result = {
                **alignment,
                "status": "error",
                "mcp": "control_jetarm_to_target_pixel",
                "controller_decision": "gripper_failed",
                "error": grip.get("error", "夹取动作失败"),
                "final_descent": final_descent,
                "gripper": grip,
                "grasp_completed": False,
                "requires_new_target_pixel": False,
            }
            return self._attach_grasp_step_records(result, new_records)
        home = await self.controller().go_home()
        if home.get("status") != "ok":
            result = {
                **alignment,
                "status": "error",
                "mcp": "control_jetarm_to_target_pixel",
                "controller_decision": "home_failed_after_grip",
                "error": home.get("error", "夹取后返回Home失败"),
                "final_descent": final_descent,
                "gripper": grip,
                "home": home,
                "grasp_completed": False,
                "requires_new_target_pixel": False,
            }
            return self._attach_grasp_step_records(result, new_records)
        self._grasp_final_phase = False
        self._gripper_prepared_for_grasp = False
        self._awaiting_grasp_visual_confirmation = True
        result = {
            **alignment,
            "status": "ok",
            "mcp": "control_jetarm_to_target_pixel",
            "controller_decision": "grasp_complete",
            "final_descent": final_descent,
            "gripper": grip,
            "home": home,
            "mechanical_grasp_sequence_completed": True,
            "grasp_completed": False,
            "grasp_completion_status": "awaiting_visual_verification",
            "visual_verification_required": True,
            "requires_new_target_pixel": False,
        }
        return self._attach_grasp_step_records(result, new_records)

    def confirm_grasp_result(self, success: bool) -> dict[str, Any]:
        if not self._awaiting_grasp_visual_confirmation:
            raise RuntimeError("当前没有等待Agent图像确认的抓取结果")
        if not isinstance(success, bool):
            raise RuntimeError("success必须是布尔值")
        self._awaiting_grasp_visual_confirmation = False
        if success:
            return {
                "status": "ok",
                "mcp": "confirm_jetarm_grasp_result",
                "grasp_completed": True,
                "visual_verification": "success",
                "grasp_step_record_count_total": len(self._grasp_step_records),
            }
        self._grasp_final_phase = False
        self._gripper_prepared_for_grasp = False
        return {
            "status": "ok",
            "mcp": "confirm_jetarm_grasp_result",
            "grasp_completed": False,
            "visual_verification": "failed",
            "retry_required": True,
            "instruction": "使用当前最新RGB图像重新识别同一目标中心并继续闭环",
            "grasp_step_record_count_total": len(self._grasp_step_records),
        }

    async def control_to_target_pixel(
        self,
        target_x: float,
        target_y: float,
        grasp_point_x: float | None = None,
        grasp_point_y: float | None = None,
        descend_when_aligned: bool = True,
        descent_step_cm: float = DEFAULT_DESCENT_RECALIBRATION_CM,
        step_duration_s: float = DEFAULT_PIXEL_ALIGNMENT_STEP_DURATION_S,
        speed_saturation_px: float = DEFAULT_PIXEL_ALIGNMENT_SPEED_SATURATION_PX,
        final_alignment_threshold_cm: float | None = None,
        final_grasp_height_cm: float | None = None,
        target_vertical_relation: str | None = None,
    ) -> dict[str, Any]:
        if self._awaiting_grasp_visual_confirmation:
            raise RuntimeError(
                "必须先根据Home后最新RGB图像调用confirm_jetarm_grasp_result"
            )
        grasp_pixel = self._last_grasp_point_pixel
        resolved_grasp_x = (
            grasp_point_x
            if grasp_point_x is not None
            else (grasp_pixel or {}).get("x")
        )
        resolved_grasp_y = (
            grasp_point_y
            if grasp_point_y is not None
            else (grasp_pixel or {}).get("y")
        )
        if resolved_grasp_x is None or resolved_grasp_y is None:
            raise RuntimeError(
                "未输入抓取点像素；请在Agent抓取调用前先设置抓取点x/y"
            )
        normalized_target_y, coordinate_validation = self._normalize_agent_target_y(
            target_y,
            float(resolved_grasp_y),
            target_vertical_relation,
        )
        if not self._gripper_prepared_for_grasp:
            await self.controller().set_gripper_position(
                DEFAULT_GRIPPER_RELEASE_POSITION,
                DEFAULT_GRIPPER_POSITION_RUN_TIME_MS,
            )
            self._gripper_prepared_for_grasp = True
        kwargs: dict[str, Any] = dict(
            descend_when_aligned=(
                False if self._grasp_final_phase else descend_when_aligned
            ),
            descent_step_cm=descent_step_cm,
            step_duration_s=step_duration_s,
            speed_saturation_px=speed_saturation_px,
        )
        if final_alignment_threshold_cm is not None:
            kwargs["final_alignment_threshold_cm"] = float(final_alignment_threshold_cm)
        if final_grasp_height_cm is not None:
            kwargs["final_grasp_height_cm"] = float(final_grasp_height_cm)
        result = await self.controller().control_to_target_pixel(
            target_x,
            normalized_target_y,
            resolved_grasp_x,
            resolved_grasp_y,
            **kwargs,
        )
        result["mcp"] = "control_jetarm_to_target_pixel"
        result["grasp_point_pixel_source"] = (
            (grasp_pixel or {}).get("source")
            if grasp_point_x is None or grasp_point_y is None
            else "tool_arguments"
        )
        result["agent_grasp_workflow"] = "manual_pixel_test_v2"
        result["camera_vector_version"] = "v2"
        result["progress_check_enabled"] = False
        result["agent_target_pixel_received"] = {
            "x": round(float(target_x), 3),
            "y": round(float(target_y), 3),
        }
        result["target_coordinate_validation"] = coordinate_validation
        record = self._record_grasp_step(target_x, normalized_target_y, result)
        if result.get("status") != "ok":
            return self._attach_grasp_step_records(result, [record])
        if result.get("controller_decision") != "aligned_hold":
            return self._attach_grasp_step_records(result, [record])
        if not self._grasp_final_phase:
            self._grasp_final_phase = True
            result["final_alignment_phase"] = True
            result["requires_new_target_pixel"] = True
            return self._attach_grasp_step_records(result, [record])
        return await self._complete_final_grasp(
            target_x, normalized_target_y, result
        )

    def close(self) -> None:
        if self._controller is not None:
            self._controller.close()
            self._controller = None


def create_mcp_server(service: JetArmMCPService) -> Any:
    try:
        from mcp.server.fastmcp import FastMCP
        from mcp.types import CallToolResult, ImageContent, TextContent
    except ImportError as exc:
        raise RuntimeError(
            "缺少MCP SDK，请执行: python -m pip install -r requirements-ai.txt"
        ) from exc

    mcp = FastMCP("JetArm robot controller", json_response=True)

    def content_result(
        result: dict[str, Any], frame: RGBJpegFrame | None = None
    ) -> CallToolResult:
        content: list[Any] = [
            TextContent(
                type="text",
                text=json.dumps(result, ensure_ascii=False),
            )
        ]
        if frame is not None:
            content.append(
                ImageContent(
                    type="image",
                    data=base64.b64encode(frame.data).decode("ascii"),
                    mimeType=frame.mime_type,
                )
            )
        return CallToolResult(
            content=content,
            structuredContent=result,
            isError=result.get("status") == "error",
        )

    async def with_rgb_image(
        result: dict[str, Any], *, camera_required: bool = False
    ) -> Any:
        try:
            frame = await service.capture_rgb()
        except Exception as exc:
            LOGGER.error("RGB capture failed: %s", exc)
            if camera_required:
                return content_result(
                    {
                        "status": "error",
                        "mcp": "get_rgb_camera_frame",
                        "error": str(exc),
                    }
                )
            if service.devices.rgb_camera:
                result["camera"] = {"status": "error", "error": str(exc)}
            return content_result(result)

        try:
            grasp_point_pixel = service.grasp_point_pixel_for_frame(
                frame.width, frame.height
            )
        except RuntimeError as exc:
            return content_result(
                {
                    "status": "error",
                    "mcp": "get_rgb_camera_frame",
                    "error": str(exc),
                }
            )
        service._last_grasp_point_pixel = (
            dict(grasp_point_pixel) if grasp_point_pixel is not None else None
        )
        service._last_rgb_frame_size = (int(frame.width), int(frame.height))
        result["camera"] = {
            "status": "ok",
            "device": service.devices.rgb_camera,
            "name": service.devices.rgb_camera_name or None,
            "width": frame.width,
            "height": frame.height,
            "mime_type": frame.mime_type,
            "grasp_point_pixel": grasp_point_pixel,
            "grasp_point_pixel_required_before_grasp": (
                grasp_point_pixel is None
            ),
        }
        # One MCP result carries both the pixels and the pose used to interpret
        # them.  Movement is blocked when an enabled arm cannot provide pose.
        arm_pose = await service.observation_arm_pose()
        result["arm_pose"] = arm_pose
        if camera_required and arm_pose.get("status") == "error":
            result["status"] = "error"
            result["error"] = "RGB图像已采集，但机械臂姿态读取失败，禁止据此移动"
        return content_result(result, frame)

    @mcp.tool(description="读取JetArm工作流规范；首次控制机械臂前必须读取。")
    def get_initial_instructions() -> str:
        return service.initial_instructions()

    @mcp.tool(description="检查机械臂串口和Orbbec相机序列号/UID配置。")
    def get_device_status() -> dict[str, Any]:
        return service.device_status()

    @mcp.tool(
        description=(
            "通过Orbbec SDK从已配置序列号/UID的Gemini相机采集最新彩色画面并返回JPEG。"
            "同一结果还返回抓取点基座坐标、关节位置、相机视线与竖直方向夹角及相机视角上方向。"
            "每次视觉定位和机械臂移动前必须调用；抓取点像素来自用户调用前输入。"
            "视觉抓取时Agent只返回目标物品中心像素，"
            "运动由control_jetarm_to_target_pixel决策；不启动深度流。"
        ),
        structured_output=False,
    )
    async def get_rgb_camera_frame() -> Any:
        return await with_rgb_image(
            {"status": "ok", "mcp": "get_rgb_camera_frame"},
            camera_required=True,
        )

    @mcp.tool(
        description=(
            "内部宿主工具：在Agent抓取调用前保存用户输入的抓取点像素。"
            "该工具不会暴露给模型。"
        )
    )
    def set_jetarm_grasp_point_pixel(x: float, y: float) -> dict[str, Any]:
        return service.set_grasp_point_pixel(x, y)

    @mcp.tool(
        description=(
            "执行一条JetArm末端移动命令。command格式为前1.9、后1、左0.5、右1.5、上1或下0.8；"
            "数字单位为厘米，每条命令的距离必须严格小于2cm。未指定速度时使用1.5cm/s；"
            "允许1到5cm/s。所有方向使用camera-vector控制系：上为抓取点到摄像头，"
            "下为摄像头到抓取点；V2前使实际抓取点Y减小，后使Y增加，"
            "左使X减小，右使X增加；"
            "水平运动只换算到XYZ的X/Y目标，Z不参与并保持当前高度，同时保持摄像头-抓取点姿态。"
            "调用前必须把最新RGB图像和配套机械臂姿态传给Agent，每次只调用一条；"
            "收到status=ok后必须重新取图；视觉抓取应改用control_jetarm_to_target_pixel，"
            "由控制程序根据目标点像素决策运动。控制器不会自动切分。"
            " Camera-vector frame is authoritative: up is grasp-point to camera, "
            "down is camera to grasp-point; forward decreases actual grasp-point "
            "XYZ Y, backward increases Y, left decreases X, and right increases X."
        ),
        structured_output=False,
    )
    async def move_jetarm(command: str, speed_cm_s: float = 1.5) -> Any:
        return content_result(await service.move(command, speed_cm_s))

    @mcp.tool(
        description=(
            "读取JetArm关节位置、抓取点坐标、相机姿态，以及关节限位、Home、"
            "几何尺寸、控制速度和坐标系等机械臂参数。"
        ),
        structured_output=False,
    )
    async def get_jetarm_state() -> Any:
        return await with_rgb_image(await service.state())

    @mcp.tool(
        description="让JetArm返回配置的home位姿。",
        structured_output=False,
    )
    async def move_jetarm_home() -> Any:
        return await with_rgb_image(await service.home())

    @mcp.tool(description="立即停止JetArm笛卡尔运动、J5和J6。")
    async def stop_jetarm() -> dict[str, Any]:
        return await service.stop()

    @mcp.tool(
        description="按clockwise或counterclockwise旋转J5，最长2秒。",
        structured_output=False,
    )
    async def rotate_jetarm_wrist(direction: str, duration_s: float) -> Any:
        return await with_rgb_image(await service.wrist(direction, duration_s))

    @mcp.tool(
        description="控制J6夹爪：open、close、grip_lock、release_lock或stop。",
        structured_output=False,
    )
    async def control_jetarm_gripper(
        action: str, duration_s: float = 0.5
    ) -> Any:
        return await with_rgb_image(await service.gripper(action, duration_s))

    @mcp.tool(
        description=(
            "Move J6 to a raw position. The grasp workflow uses position 370 to "
            "keep the gripper released before success, and after failed visual "
            "success checks before retrying."
        ),
        structured_output=False,
    )
    async def set_jetarm_gripper_position(
        position: int = DEFAULT_GRIPPER_RELEASE_POSITION,
        run_time_ms: int = DEFAULT_GRIPPER_POSITION_RUN_TIME_MS,
    ) -> Any:
        return await with_rgb_image(
            await service.set_gripper_position(position, run_time_ms)
        )

    @mcp.tool(
        description=(
            "Low-level compatibility pixel-error step. Do not use this for the "
            "current visual grasp workflow; use control_jetarm_to_target_pixel so "
            "the controller owns movement decisions. This tool moves one small "
            "image-plane step using explicit block-center and grasp-point pixels. "
            "Distance is abs(pixel_error) divided by the current height-linear "
            f"px/cm scale ({PIXEL_ALIGNMENT_SCALE_NEAR_HEIGHT_CM:g} cm -> {PIXEL_ALIGNMENT_SCALE_NEAR_PX_PER_CM:g}; "
            f"{PIXEL_ALIGNMENT_SCALE_FAR_HEIGHT_CM:g} cm -> {PIXEL_ALIGNMENT_SCALE_FAR_PX_PER_CM:g}) capped by the command limit; speed "
            "is limited to 0.7..1.5 cm/s."
        ),
        structured_output=False,
    )
    async def move_jetarm_by_pixel_error(
        block_center_x: float,
        block_center_y: float,
        grasp_point_x: float,
        grasp_point_y: float,
        tolerance_px: float = DEFAULT_PIXEL_ALIGNMENT_TOLERANCE_PX,
        step_duration_s: float = DEFAULT_PIXEL_ALIGNMENT_STEP_DURATION_S,
        speed_saturation_px: float = DEFAULT_PIXEL_ALIGNMENT_SPEED_SATURATION_PX,
    ) -> Any:
        return content_result(
            await service.pixel_align(
                block_center_x,
                block_center_y,
                grasp_point_x,
                grasp_point_y,
                tolerance_px,
                step_duration_s,
                speed_saturation_px,
            )
        )

    @mcp.tool(
        description=(
            "Manual-pixel-test V2 grasp workflow with progress detection disabled. "
            "The Agent must supply the center pixel target_x/target_y of the requested "
            "object using top-left origin, X-right, Y-down original-image coordinates, "
            "plus target_vertical_relation=above/below/same_y for validation. The controller uses the "
            "user-entered grasp-point pixel, reads joint feedback/FK height, chooses "
            "height-based tolerance (40/25/13/8 px), performs V2 front/back/left/right "
            "alignment with a height-linear px/cm scale "
            f"({PIXEL_ALIGNMENT_SCALE_NEAR_HEIGHT_CM:g} cm -> {PIXEL_ALIGNMENT_SCALE_NEAR_PX_PER_CM:g}; "
            f"{PIXEL_ALIGNMENT_SCALE_FAR_HEIGHT_CM:g} cm -> {PIXEL_ALIGNMENT_SCALE_FAR_PX_PER_CM:g}), descends in 2 cm stages, and requests a "
            "fresh target pixel after every move. On final alignment it automatically "
            "descends to grasp height, grips, and returns Home. The Agent must then "
            "verify the fresh image with confirm_jetarm_grasp_result."
        ),
        structured_output=False,
    )
    async def control_jetarm_to_target_pixel(
        target_x: float,
        target_y: float,
        target_vertical_relation: Literal["above", "below", "same_y"],
    ) -> Any:
        return content_result(
            await service.control_to_target_pixel(
                target_x,
                target_y,
                target_vertical_relation=target_vertical_relation,
            )
        )

    @mcp.tool(
        description=(
            "After control_jetarm_to_target_pixel completes the mechanical grasp and "
            "the session automatically returns a fresh Home-position RGB image, the "
            "Agent must inspect that image and call this tool with success=true only "
            "when the requested object was actually picked up. Use success=false to "
            "continue the target-pixel closed loop from the current image."
        )
    )
    def confirm_jetarm_grasp_result(success: bool) -> dict[str, Any]:
        return service.confirm_grasp_result(success)

    return mcp


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="JetArm本地MCP服务器")
    parser.add_argument("--device-config", default=str(DEFAULT_DEVICE_CONFIG_PATH))
    parser.add_argument("--arm-mode", choices=("off", "dry-run", "hardware"))
    parser.add_argument("--arm-port")
    parser.add_argument("--arm-config")
    parser.add_argument(
        "--arm-max-distance-cm",
        type=float,
        default=MAX_AGENT_MOVE_COMMAND_CM,
        help="单条Agent移动命令的排他上限，不能超过2cm",
    )
    return parser


def main(argv: Optional[list[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    devices = RuntimeDeviceConfig.load(args.device_config, required=False)
    if (
        args.arm_mode is not None
        or args.arm_port is not None
        or args.arm_config is not None
    ):
        devices = RuntimeDeviceConfig(
            arm_mode=args.arm_mode or devices.arm_mode,
            arm_port=args.arm_port if args.arm_port is not None else devices.arm_port,
            arm_terminal_config=args.arm_config or devices.arm_terminal_config,
            rgb_camera=devices.rgb_camera,
            rgb_camera_name=devices.rgb_camera_name,
            grasp_point_x=devices.grasp_point_x,
            grasp_point_y=devices.grasp_point_y,
        )
        devices.validate()
    service = JetArmMCPService(devices, max_distance_cm=args.arm_max_distance_cm)
    atexit.register(service.close)
    if devices.arm_mode != "off":
        # Open and validate hardware before the stdio protocol loop starts.
        # This also keeps heavy driver imports out of an active MCP request.
        service.controller()
    server = create_mcp_server(service)
    LOGGER.info("Starting JetArm MCP server over stdio")
    server.run(transport="stdio")
    return 0


if __name__ == "__main__":
    # The interactive Agent prints structured motion records itself. Keep the
    # stdio child quiet unless a genuine server failure needs attention.
    logging.basicConfig(level=logging.ERROR, stream=sys.stderr)
    raise SystemExit(main())
