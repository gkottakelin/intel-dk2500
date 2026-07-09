"""Interactive command-line interface for API chat."""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import os
import sys
from contextlib import AsyncExitStack
from pathlib import Path
from typing import Optional

from .arm_control import (
    ARM_TOOL_SYSTEM_PROMPT,
    ArmControlConfig,
    DEFAULT_TERMINAL_CONFIG,
    DEFAULT_GRIPPER_RELEASE_POSITION,
    MAX_AGENT_MOVE_COMMAND_CM,
    JetArmToolController,
    choose_arm_serial_port,
    looks_like_arm_command,
    looks_like_camera_command,
    looks_like_grasp_workflow_command,
    required_mcp_tool_for_command,
)
from .config import AgentSettings, ConfigurationError, DEFAULT_CONFIG_PATH
from .device_config import DEFAULT_DEVICE_CONFIG_PATH, RuntimeDeviceConfig
from .mcp_client import MCPClientError, MCPRobotBridge
from .mcp_server import DEFAULT_WORKFLOW_PATH
from .openai_compatible import APIClientError, OpenAICompatibleClient
from .roundtrip_test import run_counter_roundtrip_test
from .session import ChatSession
from .tool_agent import MAX_VISUAL_CLOSED_LOOP_ROUNDS, ToolCallingSession


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="JetArm AI command-line dialogue")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG_PATH), help="AI JSON配置文件")
    parser.add_argument("--base-url", default=None, help="覆盖API base URL")
    parser.add_argument("--model", default=None, help="覆盖模型名称")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--once", default=None, help="发送一条消息后退出")
    mode.add_argument(
        "--tool-test",
        action="store_true",
        help="运行程序->AI->计数工具->AI贯通测试",
    )
    mode.add_argument(
        "--manual-pixel-test",
        action="store_true",
        help="不调用API，由人工输入目标点像素来模拟Agent视觉抓取闭环",
    )
    parser.add_argument("--env-file", default=None, help="可选.env文件路径")
    parser.add_argument(
        "--device-config",
        default=str(DEFAULT_DEVICE_CONFIG_PATH),
        help="机械臂和RGB相机接口配置文件",
    )
    parser.add_argument(
        "--arm-mode",
        choices=("off", "dry-run", "hardware"),
        default=None,
        help="机械臂工具模式；默认off，hardware会打开真实串口",
    )
    parser.add_argument("--arm-port", default=None, help="机械臂串口，如/dev/ttyUSB0")
    parser.add_argument("--arm-config", default=None, help="Ubuntu终端terminal.json路径")
    parser.add_argument(
        "--arm-max-distance-cm",
        type=float,
        default=None,
        help="Agent单条TCP移动命令的排他上限，默认2cm且不能超过2cm",
    )
    parser.add_argument(
        "--manual-image-width",
        type=int,
        default=640,
        help="manual-pixel-test使用的模拟RGB图像宽度，默认640",
    )
    parser.add_argument(
        "--manual-image-height",
        type=int,
        default=480,
        help="manual-pixel-test使用的模拟RGB图像高度，默认480",
    )
    parser.add_argument(
        "--manual-grasp-x",
        type=float,
        default=None,
        help="manual-pixel-test使用的抓取点像素x，默认图像中心",
    )
    parser.add_argument(
        "--manual-grasp-y",
        type=float,
        default=None,
        help="manual-pixel-test使用的抓取点像素y，默认图像中心",
    )
    return parser


def _load_env_file(path: Optional[str]) -> None:
    try:
        from dotenv import load_dotenv
    except ImportError:
        if path:
            raise ConfigurationError(
                "使用--env-file需要python-dotenv，请先安装requirements-ai.txt"
            )
        return
    if path:
        env_path = Path(path)
        if not env_path.is_file():
            raise ConfigurationError(f".env文件不存在: {env_path}")
        load_dotenv(env_path)
    else:
        load_dotenv()


def _print_help(*, arm_enabled: bool = False, camera_enabled: bool = False) -> None:
    print("可用命令:")
    print("  /help     显示帮助")
    print("  /clear    清空当前对话上下文")
    print("  /history  显示当前上下文")
    print("  /config   显示非敏感API配置")
    print("  /tool-test 运行AI调用本地代码的3秒贯通测试")
    if arm_enabled:
        print("  /arm-status 读取机械臂关节和TCP状态")
        print("  /arm-home   直接回到home位姿")
        print("  /arm-stop   立即停止J5/J6和笛卡尔运动")
        print("  /workflow   显示JetArm MCP工作流规范")
    if camera_enabled:
        print("  /camera     采集当前RGB画面并让AI描述")
    print("  /exit     退出")


def _print_history(session: object) -> None:
    history = getattr(session, "history", [])
    if not history:
        print("当前没有对话记录。")
        return
    labels = {"user": "你", "assistant": "AI", "tool": "工具"}
    for index, message in enumerate(history, 1):
        label = labels.get(message.get("role"), str(message.get("role", "?")))
        content = message.get("content", "")
        if isinstance(content, list):
            text_parts = [
                str(part.get("text", ""))
                for part in content
                if isinstance(part, dict) and part.get("type") == "text"
            ]
            image_count = sum(
                1
                for part in content
                if isinstance(part, dict) and part.get("type") == "image_url"
            )
            content = " ".join(filter(None, text_parts))
            if image_count:
                content = f"{content} [RGB图像×{image_count}]".strip()
        print(f"{index:02d} {label}: {content}")


async def _send(session: ChatSession, text: str) -> str:
    print("AI: ", end="", flush=True)
    answer = await session.ask(text, on_token=lambda token: print(token, end="", flush=True))
    print()
    return answer


async def _send_with_tools(session: ToolCallingSession, text: str) -> str:
    required_tool = required_mcp_tool_for_command(text)
    is_arm_command = looks_like_arm_command(text)
    camera_request = required_tool == "get_rgb_camera_frame"
    movement_request = required_tool == "move_jetarm"
    grasp_workflow_request = looks_like_grasp_workflow_command(text)
    if is_arm_command:
        print(f"[工作流 1/5] 接收自然语言: {text}")
        print("[工作流 2/5] Agent解析意图并生成MCP工具调用")
    result = await session.ask(
        text,
        require_any_tool=required_tool is not None,
        required_tool_name=required_tool,
        required_tool_retries=1,
        preselected_tool_name=(
            "get_rgb_camera_frame"
            if camera_request or movement_request or grasp_workflow_request
            else None
        ),
        preselected_tool_arguments=(
            {} if camera_request or movement_request or grasp_workflow_request else None
        ),
        first_tool_choice="none" if camera_request else "auto",
        allow_additional_tools=not camera_request,
    )
    for call in result.tool_calls:
        prefix = "[工作流 3/5] MCP调用" if is_arm_command else "[MCP调用]"
        print(f"{prefix}: {call.name} {json.dumps(call.arguments, ensure_ascii=False)}")
        prefix = "[工作流 4/5] MCP结果" if is_arm_command else "[MCP结果]"
        print(f"{prefix}: {call.name} {json.dumps(call.result, ensure_ascii=False)}")
        if call.images:
            print(f"[MCP图像]: {len(call.images)}张RGB JPEG已传给Agent")
    if is_arm_command:
        print("[工作流 5/5] Agent读取MCP结果并生成总结报告")
    print(f"AI: {result.text}")
    return result.text


async def _run_tool_test(
    settings: AgentSettings, client: OpenAICompatibleClient
) -> None:
    result = await run_counter_roundtrip_test(
        settings,
        client,
        on_status=lambda status: print(f"[tool-test] {status}"),
    )
    print(
        f"[tool-test] 通过：工具调用{result.tool_call_count}次，"
        f"计数器={result.counter}"
    )


def _workflow_text() -> str:
    try:
        return DEFAULT_WORKFLOW_PATH.read_text(encoding="utf-8")
    except FileNotFoundError as exc:
        raise ConfigurationError(f"MCP工作流文件不存在: {DEFAULT_WORKFLOW_PATH}") from exc


def _print_workflow_summary() -> None:
    print("MCP执行工作流:")
    print("  1. MCP采集最新RGB图像并传给Agent")
    print("  2. Agent只解析命令并寻找目标点像素，不决定机械臂运动")
    print("  3. MCP控制程序根据目标点和抓取点像素执行对准或下降")
    print("  4. 每移动或下降2cm后重新采集RGB图像，再让Agent重新找目标点")
    print("  5. 达到抓取高度后执行夹取、复位，并让Agent按新图检查结果")


def _parse_manual_target_pixel(text: str) -> tuple[float, float] | None:
    normalized = text.strip().lower()
    if normalized in {"q", "quit", "exit", "退出"}:
        return None
    normalized = normalized.replace(",", " ").replace("，", " ")
    parts = normalized.split()
    if len(parts) != 2:
        raise ValueError("请输入两个像素坐标，例如: 450 230")
    try:
        x = float(parts[0])
        y = float(parts[1])
    except ValueError as exc:
        raise ValueError("像素坐标必须是数字，例如: 450 230") from exc
    if not math.isfinite(x) or not math.isfinite(y):
        raise ValueError("像素坐标必须是有限数字")
    return x, y


def _extract_camera_grasp_vertical_angle(result: dict[str, object]) -> float | None:
    """Extract the camera-grasp line angle from vertical from a move result dict."""
    for key in ("camera_pose_after_move", "camera_pose_before_move"):
        camera_pose = result.get(key)
        if isinstance(camera_pose, dict):
            angle = camera_pose.get("line_of_sight_angle_from_vertical_deg")
            if isinstance(angle, (int, float)):
                return float(angle)
    camera_line = result.get("camera_line_angle_hold")
    if isinstance(camera_line, dict):
        angle = camera_line.get("actual_after_deg")
        if isinstance(angle, (int, float)):
            return float(angle)
    return None


def _format_camera_angle(angle: float | None) -> str:
    if angle is None:
        return ""
    return f"摄像头-抓取点与竖直夹角: {angle:.1f}°"


def _print_manual_pixel_result(result: dict[str, object]) -> None:
    decision = result.get("controller_decision")
    error = result.get("pixel_error", {})
    tolerance = result.get("dynamic_tolerance_px")
    grasp_xyz_before = result.get("grasp_point_xyz_before_cm")
    grasp_xyz_after = result.get("grasp_point_xyz_after_cm")
    angle = _extract_camera_grasp_vertical_angle(result)
    angle_text = _format_camera_angle(angle)
    angle_segment = f" | {angle_text}" if angle_text else ""
    if decision == "horizontal_align":
        print(
            "控制结果: 水平对准 | "
            f"方向={result.get('direction')} "
            f"像素比例步长={result.get('requested_distance_cm')}cm "
            f"速度={result.get('speed_cm_s')}cm/s "
            f"误差={error} 容差={tolerance}px "
            f"抓取点XYZ={grasp_xyz_before} -> {grasp_xyz_after}"
            f"{angle_segment}"
        )
        return
    if decision == "descend_after_alignment":
        print(
            "控制结果: 已对准，下降 | "
            f"下降={result.get('requested_distance_cm')}cm "
            f"速度={result.get('speed_cm_s')}cm/s "
            f"高度={result.get('height_before_cm')}cm -> {result.get('height_after_cm')}cm "
            f"误差={error} 容差={tolerance}px "
            f"抓取点XYZ={grasp_xyz_before} -> {grasp_xyz_after}"
            f"{angle_segment}"
        )
        return
    if decision == "aligned_hold":
        remaining = result.get("remaining_descent_to_final_cm")
        print(
            "控制结果: 已对准，接近目标高度 | "
            f"当前高度={result.get('height_cm')}cm "
            f"误差={error} 容差={tolerance}px "
            + (f"距最终抓取高度还剩={remaining}cm" if remaining is not None else "")
            + angle_segment
        )
        return
    print(f"控制结果: {json.dumps(result, ensure_ascii=False)}")


def _resolve_manual_pixel_arm_config(args: argparse.Namespace) -> ArmControlConfig:
    device_path = Path(args.device_config)
    saved_devices = RuntimeDeviceConfig.load(device_path, required=False)
    saved_has_config = device_path.is_file()
    if args.arm_mode == "off":
        raise ConfigurationError("manual-pixel-test需要dry-run或hardware，不能使用off")
    arm_mode = (
        args.arm_mode
        or (
            saved_devices.arm_mode
            if saved_has_config and saved_devices.arm_mode != "off"
            else ""
        )
        or os.getenv("JETARM_ARM_MODE", "").strip()
        or "dry-run"
    )
    if arm_mode not in {"dry-run", "hardware"}:
        raise ConfigurationError("manual-pixel-test只支持dry-run或hardware")
    arm_port = (
        args.arm_port
        or (saved_devices.arm_port if saved_has_config else "")
        or os.getenv("JETARM_ARM_PORT", "").strip()
        or None
    )
    arm_config_path = Path(
        args.arm_config
        or os.getenv("JETARM_ARM_CONFIG", "")
        or saved_devices.arm_terminal_config
        or DEFAULT_TERMINAL_CONFIG
    )
    return ArmControlConfig(
        mode=arm_mode,
        serial_port=arm_port,
        terminal_config_path=arm_config_path,
        max_distance_cm=100.0,
        allow_extended_distance=True,
    )


async def _run_manual_pixel_test(args: argparse.Namespace) -> int:
    if args.manual_image_width <= 0 or args.manual_image_height <= 0:
        raise ConfigurationError("manual image width/height必须大于0")
    grasp_x = (
        float(args.manual_grasp_x)
        if args.manual_grasp_x is not None
        else args.manual_image_width / 2.0
    )
    grasp_y = (
        float(args.manual_grasp_y)
        if args.manual_grasp_y is not None
        else args.manual_image_height / 2.0
    )
    if not math.isfinite(grasp_x) or not math.isfinite(grasp_y):
        raise ConfigurationError("manual grasp point必须是有限数字")

    arm_config = _resolve_manual_pixel_arm_config(args)
    if arm_config.mode == "hardware":
        print("硬件手动像素闭环测试：会真实控制机械臂运动。")
        print(
            "串口: "
            + (
                arm_config.serial_port
                if arm_config.serial_port
                else "未手动指定，将复用ubuntu22_04_operation_terminal自动识别"
            )
        )
        confirmation = input("确认机械臂周围安全后输入 RUN 开始，其他输入将退出: ").strip()
        if confirmation != "RUN":
            print("已取消硬件手动像素测试。")
            return 0

    controller = JetArmToolController(arm_config)
    try:
        release = await controller.set_gripper_position(DEFAULT_GRIPPER_RELEASE_POSITION)
        state = await controller.state()
        grasp_params = state["arm_parameters"]["vision_guided_grasp"]
        final_alignment_threshold_cm = grasp_params["final_alignment_threshold_cm"]
        final_grasp_height_cm = grasp_params["final_grasp_height_cm"]
        mode_text = (
            "hardware，真实机械臂运行"
            if arm_config.mode == "hardware"
            else "dry-run，仅模拟"
        )
        print(f"手动像素闭环测试（{mode_text}；不调用API，不接相机）")
        print(
            f"模拟图像: {args.manual_image_width}x{args.manual_image_height}, "
            f"固定抓取点像素=({grasp_x:g}, {grasp_y:g})"
        )
        print(
            f"J6已保持松开: {release['target_position']}；"
            f"初始抓取点XYZ={state['grasp_point_xyz_cm']}；"
            f"最终对准触发高度={final_alignment_threshold_cm}cm；"
            f"最终抓取高度={final_grasp_height_cm}cm"
        )
        print(
            "手动像素修正: 抓取点像素固定；水平移动比例按当前高度线性计算（2cm=50px/cm，25cm=18px/cm）；"
            f"下降每次2cm；高度≤{final_alignment_threshold_cm:g}cm时触发最终对准，"
            f"对准后下降至{final_grasp_height_cm:g}cm抓取。"
        )
        print("输入目标点像素，格式如: 450 230 或 450,230。输入 q 退出。")

        round_index = 1
        in_final_phase = False
        while True:
            state = await controller.state()
            print(
                f"\n[第{round_index}轮] 当前抓取点XYZ={state['grasp_point_xyz_cm']}，"
                "旧图像视为已失效，请输入新图中的目标点像素。"
            )
            try:
                raw = input("目标点像素 x y: ")
            except EOFError:
                print()
                return 0
            try:
                parsed = _parse_manual_target_pixel(raw)
            except ValueError as exc:
                print(f"输入错误: {exc}")
                continue
            if parsed is None:
                print("已退出手动像素测试。")
                return 0
            target_x, target_y = parsed

            if in_final_phase:
                # Align one last time without descending, then descend to
                # final_grasp_height_cm and execute the grasp.
                result = await controller.control_to_target_pixel(
                    target_x, target_y, grasp_x, grasp_y,
                    descend_when_aligned=False,
                )
                _print_manual_pixel_result(result)
                decision = result.get("controller_decision")
                if decision != "aligned_hold":
                    print("最终对准未完成，请重新输入目标点像素。")
                    continue
                print(
                    f"最终对准完成，下降至{final_grasp_height_cm:g}cm…"
                )
                descend_result = await controller.descend_to_height(
                    final_grasp_height_cm
                )
                print(
                    f"下降结果: 高度={descend_result.get('height_after_cm')}cm "
                    f"步数={descend_result.get('steps')}"
                )
                grip = await controller.control_gripper("grip_lock")
                home = await controller.go_home()
                print(
                    "已完成抓取并复位 | "
                    f"夹爪={grip['action']} Home状态={home['status']}"
                )
                return 0

            result = await controller.control_to_target_pixel(
                target_x, target_y, grasp_x, grasp_y,
            )
            _print_manual_pixel_result(result)
            decision = result.get("controller_decision")

            if decision == "aligned_hold":
                # Height is already near or below the final-alignment threshold
                # and the controller declined to descend. Enter the final phase.
                remaining = result.get("remaining_descent_to_final_cm")
                height_now = result.get("height_cm")
                print(
                    f"高度{height_now}cm已达到最终对准阈值，"
                    f"距抓取高度剩余{remaining}cm。请输入新目标点像素完成最终对准。"
                )
                in_final_phase = True
                round_index += 1
                continue

            if decision == "descend_after_alignment":
                height_after = result.get("height_after_cm")
                if (
                    isinstance(height_after, (int, float))
                    and height_after <= final_alignment_threshold_cm
                ):
                    print(
                        f"下降后高度{height_after}cm已达到最终对准阈值，"
                        "下一轮将触发最终对准。"
                    )
                    in_final_phase = True

            round_index += 1
    finally:
        controller.close()


async def run(args: argparse.Namespace) -> int:
    if args.manual_pixel_test:
        return await _run_manual_pixel_test(args)

    _load_env_file(args.env_file)
    device_path = Path(args.device_config)
    has_device_config = device_path.is_file()
    saved_devices = RuntimeDeviceConfig.load(device_path, required=False)
    arm_mode = (
        args.arm_mode
        or (saved_devices.arm_mode if has_device_config else "")
        or os.getenv("JETARM_ARM_MODE", "").strip()
        or "off"
    )
    arm_port = (
        args.arm_port
        or (saved_devices.arm_port if has_device_config else "")
        or os.getenv("JETARM_ARM_PORT")
        or None
    )
    arm_config_path = Path(
        args.arm_config
        or os.getenv("JETARM_ARM_CONFIG", "")
        or saved_devices.arm_terminal_config
        or DEFAULT_TERMINAL_CONFIG
    )
    configured_max_distance_cm = (
        args.arm_max_distance_cm
        if args.arm_max_distance_cm is not None
        else float(
            os.getenv(
                "JETARM_ARM_MAX_DISTANCE_CM",
                str(MAX_AGENT_MOVE_COMMAND_CM),
            )
        )
    )
    if configured_max_distance_cm <= 0:
        raise ConfigurationError("Agent单条移动命令上限必须大于0")
    max_distance_cm = min(
        configured_max_distance_cm,
        MAX_AGENT_MOVE_COMMAND_CM,
    )
    if configured_max_distance_cm > MAX_AGENT_MOVE_COMMAND_CM:
        print(
            f"提示: 旧的机械臂距离上限{configured_max_distance_cm:g}cm已自动限制为"
            f"{MAX_AGENT_MOVE_COMMAND_CM:g}cm；实际每条命令必须严格小于该值。"
        )

    if arm_mode == "hardware" and arm_port is None:
        print("正在打开机械臂串口选择窗口...")
        arm_port = choose_arm_serial_port()
        if arm_port is None:
            print("已取消串口选择，Agent未启动。")
            return 0

    effective_devices = RuntimeDeviceConfig(
        arm_mode=arm_mode,
        arm_port=arm_port or "",
        arm_terminal_config=str(arm_config_path),
        rgb_camera=saved_devices.rgb_camera,
        rgb_camera_name=saved_devices.rgb_camera_name,
    )
    effective_devices.validate()

    settings = AgentSettings.from_sources(
        args.config,
        base_url=args.base_url,
        model=args.model,
    )
    client = OpenAICompatibleClient(settings)
    chat_session = ChatSession(settings, client)

    print("JetArm AI 对话终端（MCP）")
    print(f"API: {settings.base_url}")
    print(f"模型: {settings.model}")
    print(f"设备配置: {device_path}")
    print(f"RGB相机: {effective_devices.rgb_camera or '未配置'}")

    if args.tool_test:
        try:
            await _run_tool_test(settings, client)
            return 0
        finally:
            await client.close()

    arm_session: ToolCallingSession | None = None
    bridge: MCPRobotBridge | None = None
    try:
        async with AsyncExitStack() as stack:
            if arm_mode != "off" or bool(effective_devices.rgb_camera):
                bridge = await stack.enter_async_context(
                    MCPRobotBridge(
                        device_config=device_path,
                        arm_mode=arm_mode,
                        arm_port=arm_port,
                        arm_config=arm_config_path,
                        max_distance_cm=max_distance_cm,
                    )
                )
                registry = await bridge.registry()
                workflow = _workflow_text()
                arm_session = ToolCallingSession(
                    settings,
                    client,
                    registry,
                    system_prompt=(
                        f"{settings.system_prompt}\n\n{ARM_TOOL_SYSTEM_PROMPT}"
                        f"\n\n以下是必须遵守的JetArm MCP工作流：\n{workflow}"
                    ),
                    max_rounds=MAX_VISUAL_CLOSED_LOOP_ROUNDS,
                )
                print(f"机械臂MCP: {arm_mode} ({arm_port or '未启用'})")
                print(f"可用MCP工具: {', '.join(registry.names())}")
                _print_workflow_summary()

            if args.once:
                if looks_like_arm_command(args.once) and arm_mode == "off":
                    raise ConfigurationError("检测到机械臂指令，但设备配置中的arm_mode为off")
                if looks_like_camera_command(args.once) and not effective_devices.rgb_camera:
                    raise ConfigurationError("检测到视觉指令，但未配置RGB相机")
                if arm_session is not None:
                    await _send_with_tools(arm_session, args.once)
                elif looks_like_arm_command(args.once):
                    raise ConfigurationError("检测到机械臂指令，但设备配置中的arm_mode为off")
                else:
                    await _send(chat_session, args.once)
                return 0

            _print_help(
                arm_enabled=bridge is not None and arm_mode != "off",
                camera_enabled=bool(effective_devices.rgb_camera),
            )
            while True:
                try:
                    text = input("\n你: ").strip()
                except EOFError:
                    print()
                    return 0
                if not text:
                    continue
                command = text.lower()
                if command in {"/exit", "/quit", "exit", "quit"}:
                    return 0
                if command == "/help":
                    _print_help(
                        arm_enabled=bridge is not None and arm_mode != "off",
                        camera_enabled=bool(effective_devices.rgb_camera),
                    )
                    continue
                if command == "/clear":
                    chat_session.clear()
                    if arm_session is not None:
                        arm_session.clear()
                    print("对话上下文已清空。")
                    continue
                if command == "/history":
                    _print_history(arm_session or chat_session)
                    continue
                if command == "/config":
                    summary = settings.public_summary()
                    summary["devices"] = {
                        "arm_mode": arm_mode,
                        "arm_port": arm_port,
                        "arm_config": str(arm_config_path),
                        "rgb_camera": effective_devices.rgb_camera or None,
                        "max_distance_cm": max_distance_cm,
                        "default_speed_cm_s": 1.5,
                        "distance_limit_rule": (
                            f"每条move_jetarm命令严格小于{max_distance_cm:g}cm"
                        ),
                        "controller_auto_split": False,
                    }
                    print(json.dumps(summary, ensure_ascii=False, indent=2))
                    continue
                if command == "/tool-test":
                    try:
                        await _run_tool_test(settings, client)
                    except (APIClientError, RuntimeError, ValueError) as exc:
                        print(f"错误: {exc}", file=sys.stderr)
                    continue
                if command == "/workflow":
                    print(_workflow_text())
                    continue
                if command == "/camera":
                    if not effective_devices.rgb_camera or arm_session is None:
                        print("错误: 未配置RGB相机或相机MCP未启动。", file=sys.stderr)
                        continue
                    text = "请读取当前RGB相机画面，并只根据实际画面简要描述你看到的内容。"
                    command = text.lower()
                if command in {"/arm-status", "/arm-home", "/arm-stop"}:
                    if bridge is None or arm_mode == "off":
                        print("错误: 机械臂MCP未启用。", file=sys.stderr)
                        continue
                    tool_name = {
                        "/arm-status": "get_jetarm_state",
                        "/arm-home": "move_jetarm_home",
                        "/arm-stop": "stop_jetarm",
                    }[command]
                    result = await bridge.call_tool(tool_name, {})
                    print(json.dumps(result, ensure_ascii=False, indent=2))
                    continue
                if arm_session is None and looks_like_arm_command(text):
                    print(
                        "错误: 检测到机械臂指令，但机械臂MCP未启用。"
                        "请先运行 python3 -m src.jetarm_agent.device_config。",
                        file=sys.stderr,
                    )
                    continue
                if arm_mode == "off" and looks_like_arm_command(text):
                    print("错误: 机械臂MCP未启用，请先把arm_mode配置为hardware。", file=sys.stderr)
                    continue
                if looks_like_camera_command(text) and not effective_devices.rgb_camera:
                    print("错误: 未配置RGB相机，请先运行设备配置程序。", file=sys.stderr)
                    continue
                try:
                    if arm_session is not None:
                        await _send_with_tools(arm_session, text)
                    else:
                        await _send(chat_session, text)
                except (APIClientError, MCPClientError, RuntimeError, ValueError) as exc:
                    print(f"\n错误: {exc}", file=sys.stderr)
    finally:
        await client.close()


def main(argv: Optional[list[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        return asyncio.run(run(args))
    except KeyboardInterrupt:
        print("\n已退出。")
        return 130
    except (ConfigurationError, APIClientError, RuntimeError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
