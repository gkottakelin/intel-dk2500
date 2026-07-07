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

from .arm_control import ArmControlConfig, JetArmToolController
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
        max_distance_cm: float = 10.0,
    ) -> None:
        self.devices = devices
        self.controller_factory = controller_factory
        self.camera_capture = camera_capture
        self.workflow_path = Path(workflow_path)
        self.max_distance_cm = max_distance_cm
        self._controller: JetArmToolController | None = None

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
                    max_segment_cm=3.0,
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

        result["camera"] = {
            "status": "ok",
            "device": service.devices.rgb_camera,
            "name": service.devices.rgb_camera_name or None,
            "width": frame.width,
            "height": frame.height,
            "mime_type": frame.mime_type,
        }
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
            "用户要求查看、描述、识别或分析相机画面时调用；不启动深度流。"
        )
    )
    async def get_rgb_camera_frame() -> Any:
        return await with_rgb_image(
            {"status": "ok", "mcp": "get_rgb_camera_frame"},
            camera_required=True,
        )

    @mcp.tool(
        description=(
            "按紧凑中文命令移动JetArm末端。command格式为前5、后2、左1.5、右3、上2或下1；"
            "数字单位为厘米。未指定速度时使用1.5cm/s；允许1到5cm/s。控制器自动拆分为"
            "每段不超过3cm，并在全部分段执行完成后返回status=ok。"
        )
    )
    async def move_jetarm(command: str, speed_cm_s: float = 1.5) -> Any:
        return await with_rgb_image(await service.move(command, speed_cm_s))

    @mcp.tool(description="读取JetArm关节位置和估算TCP坐标。")
    async def get_jetarm_state() -> Any:
        return await with_rgb_image(await service.state())

    @mcp.tool(description="让JetArm返回配置的home位姿。")
    async def move_jetarm_home() -> Any:
        return await with_rgb_image(await service.home())

    @mcp.tool(description="立即停止JetArm笛卡尔运动、J5和J6。")
    async def stop_jetarm() -> dict[str, Any]:
        return await service.stop()

    @mcp.tool(description="按clockwise或counterclockwise旋转J5，最长2秒。")
    async def rotate_jetarm_wrist(direction: str, duration_s: float) -> Any:
        return await with_rgb_image(await service.wrist(direction, duration_s))

    @mcp.tool(description="控制J6夹爪：open、close、grip_lock、release_lock或stop。")
    async def control_jetarm_gripper(
        action: str, duration_s: float = 0.5
    ) -> Any:
        return await with_rgb_image(await service.gripper(action, duration_s))

    return mcp


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="JetArm本地MCP服务器")
    parser.add_argument("--device-config", default=str(DEFAULT_DEVICE_CONFIG_PATH))
    parser.add_argument("--arm-mode", choices=("off", "dry-run", "hardware"))
    parser.add_argument("--arm-port")
    parser.add_argument("--arm-config")
    parser.add_argument("--arm-max-distance-cm", type=float, default=10.0)
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
