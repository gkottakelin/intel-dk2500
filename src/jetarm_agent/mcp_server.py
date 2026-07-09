"""Local stdio MCP server for JetArm hardware, adapted from robot_MCP."""

from __future__ import annotations

import argparse
import atexit
import asyncio
import base64
import json
import logging
import sys
from pathlib import Path
from typing import Any, Callable, Optional

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
)
from .device_config import (
    DEFAULT_DEVICE_CONFIG_PATH,
    PROJECT_ROOT,
    RuntimeDeviceConfig,
    validate_device_interfaces,
)
from .rgb_camera import RGBJpegFrame, capture_rgb_jpeg


DEFAULT_WORKFLOW_PATH = PROJECT_ROOT / "workflows" / "jetarm_mcp_workflow.md"
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
        self._last_grasp_point_pixel: dict[str, Any] | None = None

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
                    max_distance_cm=self.max_distance_cm,
                    default_speed_cm_s=1.5,
                    min_speed_cm_s=1.0,
                    max_speed_cm_s=5.0,
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

    @staticmethod
    def grasp_point_pixel_for_frame(width: int, height: int) -> dict[str, Any]:
        return {
            "x": round(float(width) / 2.0, 3),
            "y": round(float(height) / 2.0, 3),
            "source": "image_center_default",
        }

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
    ) -> dict[str, Any]:
        from .arm_control import FINAL_ALIGNMENT_THRESHOLD_CM, FINAL_GRASP_HEIGHT_CM

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
                "missing grasp point pixel; call get_rgb_camera_frame first or pass grasp_point_x/grasp_point_y"
            )
        kwargs: dict[str, Any] = dict(
            descend_when_aligned=descend_when_aligned,
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
            target_y,
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
        return result

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

        grasp_point_pixel = service.grasp_point_pixel_for_frame(frame.width, frame.height)
        service._last_grasp_point_pixel = grasp_point_pixel
        result["camera"] = {
            "status": "ok",
            "device": service.devices.rgb_camera,
            "name": service.devices.rgb_camera_name or None,
            "width": frame.width,
            "height": frame.height,
            "mime_type": frame.mime_type,
            "grasp_point_pixel": grasp_point_pixel,
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
            "每次视觉定位和机械臂移动前必须调用；视觉抓取时Agent只返回目标点像素，"
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
            "执行一条JetArm末端移动命令。command格式为前1.9、后1、左0.5、右1.5、上1或下0.8；"
            "数字单位为厘米，每条命令的距离必须严格小于2cm。未指定速度时使用1.5cm/s；"
            "允许1到5cm/s。所有方向使用camera-vector控制系：上为抓取点到摄像头，"
            "下为摄像头到抓取点；前=抓取点XYZ的Y减小，后=Y增大，左=X减小，右=X增大；"
            "运动过程中保持摄像头-抓取点连线与竖直方向夹角不变。"
            "调用前必须把最新RGB图像和配套机械臂姿态传给Agent，每次只调用一条；"
            "收到status=ok后必须重新取图；视觉抓取应改用control_jetarm_to_target_pixel，"
            "由控制程序根据目标点像素决策运动。控制器不会自动切分。"
            " Camera-vector frame is authoritative: up is grasp-point to camera, "
            "down is camera to grasp-point; forward/backward/left/right use the "
            "grasp-point XYZ horizontal axes."
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
            "Controller-owned target-pixel workflow. The Agent only returns the "
            "target pixel from the latest RGB image. The controller uses the "
            "latest grasp-point pixel, reads joint feedback/FK height, chooses "
            "height-based tolerance (40/25/13/8 px), performs front/back/left/right "
            "alignment with a height-linear px/cm scale "
            f"({PIXEL_ALIGNMENT_SCALE_NEAR_HEIGHT_CM:g} cm -> {PIXEL_ALIGNMENT_SCALE_NEAR_PX_PER_CM:g}; "
            f"{PIXEL_ALIGNMENT_SCALE_FAR_HEIGHT_CM:g} cm -> {PIXEL_ALIGNMENT_SCALE_FAR_PX_PER_CM:g}). When aligned, descends 2 cm. When one "
            "more descent step would reach or pass the final-alignment threshold "
            "(2 cm), the controller returns aligned_hold instead; the caller "
            "should request a final alignment then descend to final_grasp_height_cm."
        ),
        structured_output=False,
    )
    async def control_jetarm_to_target_pixel(
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
    ) -> Any:
        return content_result(
            await service.control_to_target_pixel(
                target_x,
                target_y,
                grasp_point_x,
                grasp_point_y,
                descend_when_aligned,
                descent_step_cm,
                step_duration_s,
                speed_saturation_px,
                final_alignment_threshold_cm,
                final_grasp_height_cm,
            )
        )

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
    logging.basicConfig(level=logging.INFO, stream=sys.stderr)
    raise SystemExit(main())
