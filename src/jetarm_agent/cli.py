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
    mode.add_argument(
        "--manual-pixel-test-v2",
        action="store_true",
        help="人工像素闭环V2；复用原工作流并使用camera-vector V2运动程序",
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
        help="Agent单条TCP移动命令的排他上限，默认100cm",
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
        help="人工像素测试抓取点x；V1默认图像中心，V2默认320",
    )
    parser.add_argument(
        "--manual-grasp-y",
        type=float,
        default=None,
        help="人工像素测试抓取点y；V1默认图像中心，V2默认147",
    )
    parser.add_argument(
        "--manual-progress-check",
        choices=("on", "off"),
        default="on",
        help="人工像素测试有效进展检测开关，默认on；off时只记录进展异常",
    )
    parser.add_argument(
        "--agent-grasp-x",
        type=float,
        default=None,
        help="Agent抓取调用前由用户输入的抓取点像素x",
    )
    parser.add_argument(
        "--agent-grasp-y",
        type=float,
        default=None,
        help="Agent抓取调用前由用户输入的抓取点像素y",
    )
    parser.add_argument(
        "--red-block-grasp",
        action="store_true",
        help="提示Agent优先使用detect_red_block_target自动检测红色物块，而非zoom_rgb_target_tile分块定位",
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
        print("  /arm-init   回Home后将J6张开到400，重置抓取闭环")
        print("  /arm-stop   立即停止J5/J6和笛卡尔运动")
        print("  /workflow   显示JetArm MCP工作流规范")
        print("  /grasp-point x y  临时覆盖配置文件中的抓取点像素")
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
    is_grasp_workflow = looks_like_grasp_workflow_command(text)
    camera_request = required_tool == "get_rgb_camera_frame"
    movement_request = required_tool == "move_jetarm"
    streamed_call_ids: set[str] = set()

    def print_completed_grasp_step(call: object) -> None:
        result = getattr(call, "result", None)
        if not isinstance(result, dict) or not _has_agent_grasp_step_records(result):
            return
        _print_agent_grasp_step_records(result)
        call_id = getattr(call, "call_id", None)
        if isinstance(call_id, str):
            streamed_call_ids.add(call_id)
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
            if camera_request or movement_request
            else None
        ),
        preselected_tool_arguments=(
            {} if camera_request or movement_request else None
        ),
        first_tool_choice="none" if camera_request else "auto",
        allow_additional_tools=not camera_request,
        on_tool_call=print_completed_grasp_step,
    )
    for call in result.tool_calls:
        if is_grasp_workflow:
            if call.call_id not in streamed_call_ids and isinstance(call.result, dict):
                _print_agent_grasp_step_records(call.result)
            if isinstance(call.result, dict) and call.result.get("status") == "error":
                print(f"抓取步骤失败 | 工具={call.name} | 原因={call.result.get('error')}")
            continue
        prefix = "[工作流 3/5] MCP调用" if is_arm_command else "[MCP调用]"
        print(f"{prefix}: {call.name} {json.dumps(call.arguments, ensure_ascii=False)}")
        prefix = "[工作流 4/5] MCP结果" if is_arm_command else "[MCP结果]"
        print(f"{prefix}: {call.name} {json.dumps(call.result, ensure_ascii=False)}")
        if isinstance(call.result, dict):
            _print_agent_grasp_step_records(call.result)
        if call.images:
            print(f"[MCP图像]: {len(call.images)}张RGB JPEG已传给Agent")
    if is_arm_command:
        print("[工作流 5/5] Agent读取MCP结果并生成总结报告")
    print(f"AI: {result.text}")
    return result.text


def _has_agent_grasp_step_records(result: dict[str, object]) -> bool:
    records = result.get("new_grasp_step_records")
    return bool(records) if isinstance(records, list) else isinstance(
        result.get("grasp_step_record"), dict
    )


def _print_agent_grasp_step_records(result: dict[str, object]) -> None:
    coordinate_validation = result.get("target_coordinate_validation")
    if isinstance(coordinate_validation, dict) and coordinate_validation.get(
        "normalization"
    ) == "bottom_origin_y_up_converted_to_top_left_y_down":
        print(
            "Agent目标Y坐标已修正 | "
            f"收到={coordinate_validation.get('received_target_y')} | "
            f"修正后={coordinate_validation.get('normalized_target_y')} | "
            "原因=检测到左下角原点/Y向上坐标，已转换为左上角原点/Y向下"
        )
    raw_records = result.get("new_grasp_step_records")
    if not isinstance(raw_records, list):
        record = result.get("grasp_step_record")
        raw_records = [record] if isinstance(record, dict) else []
    for record in raw_records:
        if not isinstance(record, dict):
            continue
        plan = record.get("motion_plan")
        plan = plan if isinstance(plan, dict) else {}
        distance_cm = plan.get("distance_cm")
        distance_text = "无" if distance_cm is None else f"{distance_cm} cm"
        print(f"\n========== 抓取步骤 {record.get('step', '-')} ==========")
        print(f"目标点像素坐标：{record.get('target_pixel')}")
        print(f"原抓取点实际坐标：{record.get('original_grasp_point_xyz_cm')}")
        print(
            "运动规划："
            f"方向={plan.get('direction', 'none')}，距离={distance_text}"
        )
        print(f"预计抓取点坐标：{record.get('expected_grasp_point_xyz_cm')}")
        print(f"实际抓取点坐标：{record.get('actual_grasp_point_xyz_cm')}")
        print(f"夹角：{record.get('camera_grasp_vertical_angle_deg')}°")
        print("================================")


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


def _print_workflow_summary(*, red_block_mode: bool = False) -> None:
    if red_block_mode:
        print("MCP执行工作流（红色物块检测模式）:")
        print("  1. 从接口与抓取点配置读取固定抓取点像素")
        print("  2. Agent调用detect_red_block_target自动检测红色物块中心")
        print("  3. 控制端以检测到的坐标执行V2动作")
        print("  4. 每次动作结束后重新取图并重新检测")
        print("  5. 最终下降、夹取，确认J6稳定后Home；Agent用新图确认")
    else:
        print("MCP执行工作流:")
        print("  1. 从接口与抓取点配置读取固定抓取点像素")
        print("  2. Agent识别目标，使用数据层3x3四级分块得到原图中心像素")
        print("  3. 控制端强制采用分块坐标，以人工测试V2执行一次动作")
        print("  4. 每次动作结束后重新取图并清空旧分块路径，再次定位")
        print("  5. 最终下降、夹取，确认J6稳定后Home；Agent用新图确认")


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


def _agent_grasp_point_from_args(
    args: argparse.Namespace,
) -> tuple[float, float] | None:
    x = getattr(args, "agent_grasp_x", None)
    y = getattr(args, "agent_grasp_y", None)
    if x is None and y is None:
        return None
    if x is None or y is None:
        raise ConfigurationError("--agent-grasp-x和--agent-grasp-y必须同时提供")
    point = _parse_manual_target_pixel(f"{x} {y}")
    if point is None or point[0] < 0.0 or point[1] < 0.0:
        raise ConfigurationError("Agent抓取点像素必须是大于等于0的有限数字")
    return point


def _extract_camera_grasp_vertical_angle(result: dict[str, object]) -> float | None:
    """Extract the camera-grasp line angle from vertical from a move result dict."""
    v2_angle = result.get("v2_returned_camera_line_angle_deg")
    if isinstance(v2_angle, (int, float)):
        return float(v2_angle)
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


def _print_motion_step_diagnostics(result: dict[str, object]) -> None:
    raw_steps = result.get("motion_steps")
    if isinstance(raw_steps, list):
        steps = [step for step in raw_steps if isinstance(step, dict)]
    else:
        progress_judgement = result.get("progress_judgement")
        reasons = (
            list(progress_judgement.get("no_progress_reasons", []))
            if isinstance(progress_judgement, dict)
            else []
        )
        steps = [
            {
                "step": 1,
                "original_grasp_point_xyz_cm": result.get(
                    "grasp_point_xyz_before_cm"
                ),
                "expected_grasp_point_xyz_cm": result.get(
                    "grasp_point_xyz_expected_cm"
                ),
                "actual_grasp_point_xyz_cm": result.get(
                    "grasp_point_xyz_after_cm"
                ),
                "effective": (
                    progress_judgement.get("effective")
                    if isinstance(progress_judgement, dict)
                    else result.get("status") == "ok"
                ),
                "progress_check_enabled": (
                    progress_judgement.get("enabled")
                    if isinstance(progress_judgement, dict)
                    else True
                ),
                "progress_check_warning": (
                    progress_judgement.get("warning")
                    if isinstance(progress_judgement, dict)
                    else None
                ),
                "no_progress_reasons": reasons,
            }
        ]

    for step in steps:
        original = step.get("original_grasp_point_xyz_cm")
        expected = step.get("expected_grasp_point_xyz_cm")
        actual = step.get("actual_grasp_point_xyz_cm")
        if original is None and expected is None and actual is None:
            continue
        effective = bool(step.get("effective"))
        progress_check_enabled = step.get("progress_check_enabled", True) is not False
        progress_check_warning = step.get("progress_check_warning")
        reasons = [str(item) for item in step.get("no_progress_reasons", [])]
        if not progress_check_enabled:
            reason_text = "检测已关闭（本步不判定）"
            if progress_check_warning:
                reason_text += f"；记录={progress_check_warning}"
            if reasons:
                reason_text += "；安全/运动错误=" + "；".join(reasons)
        elif not effective and not reasons:
            fallback = result.get("error")
            reasons = [str(fallback or "运动结果未通过有效进展判定")]
            reason_text = "；".join(reasons)
        else:
            reason_text = "无（本步有效）" if effective else "；".join(reasons)
        print(
            f"运动步骤{step.get('step', 1)} | "
            f"原本抓取点XYZ={original} | "
            f"预计抓取点XYZ={expected} | "
            f"实际抓取点XYZ={actual} | "
            f"未取得有效进展原因={reason_text}"
        )


def _print_manual_pixel_result(result: dict[str, object]) -> None:
    decision = result.get("controller_decision")
    error = result.get("pixel_error", {})
    tolerance = result.get("dynamic_tolerance_px")
    if isinstance(tolerance, dict):
        tolerance_text = (
            f"X={tolerance.get('x')}px/Y={tolerance.get('y')}px"
        )
    else:
        tolerance_text = f"{tolerance}px"
    grasp_xyz_before = result.get("grasp_point_xyz_before_cm")
    grasp_xyz_after = result.get("grasp_point_xyz_after_cm")
    angle = _extract_camera_grasp_vertical_angle(result)
    angle_text = _format_camera_angle(angle)
    angle_segment = f" | {angle_text}" if angle_text else ""
    pose_segment = ""
    progress_segment = ""
    recovery_segment = ""
    low_z_recovery = result.get("v2_forward_low_z_recovery")
    if isinstance(low_z_recovery, dict) and low_z_recovery.get("used"):
        recovery_segment = (
            " | 低Z前移恢复=已触发"
            f"（Z={low_z_recovery.get('start_z_cm')}cm→"
            f"{low_z_recovery.get('target_z_cm')}cm；"
            "J1-J4绝对坐标重规划，J2未锁定）"
        )
    progress_judgement = result.get("progress_judgement")
    if (
        isinstance(progress_judgement, dict)
        and progress_judgement.get("enabled") is False
    ):
        progress_segment = " | 有效进展检测=关闭"
    pose_constraint = result.get("camera_pose_constraint")
    if isinstance(pose_constraint, dict) and pose_constraint.get("relaxed"):
        reason = pose_constraint.get("reason") or pose_constraint.get("reasons")
        relaxed_steps = pose_constraint.get("relaxed_step_count", 0)
        pose_segment = (
            " | 姿态约束=已放宽"
            f"（仅下降；原因={reason}；步数={relaxed_steps}）"
        )
    limit_fallback = result.get("v2_limit_fallback")
    if not isinstance(limit_fallback, dict) and isinstance(
        pose_constraint, dict
    ):
        candidate_fallback = pose_constraint.get("limit_fallback")
        if isinstance(candidate_fallback, dict):
            limit_fallback = candidate_fallback
    if isinstance(limit_fallback, dict) and limit_fallback.get("used"):
        pose_segment += (
            " | 限位状态=目标不可达，已前往最近可达位置"
            f"（可达比例={float(limit_fallback.get('reachable_fraction', 0.0)) * 100.0:.1f}%，"
            f"剩余距离={float(limit_fallback.get('remaining_distance_m', 0.0)) * 100.0:.2f}cm）"
        )
    progress_validation = result.get("horizontal_progress_validation")
    vertical_progress_validation = result.get("vertical_progress_validation")
    if not isinstance(progress_validation, dict):
        progress_validation = vertical_progress_validation
    if (
        isinstance(progress_validation, dict)
        and progress_validation.get("accepted")
        and (
            progress_validation.get("overrode_v2_error")
            or progress_validation.get("overrode_zero_direction_progress")
        )
    ):
        if progress_validation.get("rule") == "manual_v2_relaxed_vertical_progress":
            progress_segment += (
                " | 竖直进展=放宽接受"
                f"（|ΔZ|={progress_validation.get('z_change_cm')}cm，"
                f"XY={progress_validation.get('xy_change_cm')}cm）"
            )
        else:
            progress_segment += (
                " | 水平进展=放宽接受"
                f"（XY={progress_validation.get('xy_change_cm')}cm，"
                f"|ΔZ|={progress_validation.get('z_change_cm')}cm，"
                "夹角误差="
                f"{progress_validation.get('camera_line_angle_error_deg')}°）"
            )
    status_segment = (
        angle_segment + pose_segment + progress_segment + recovery_segment
    )
    if decision == "horizontal_align":
        print(
            "控制结果: 水平对准 | "
            f"方向={result.get('direction')} "
            f"像素比例步长={result.get('requested_distance_cm')}cm "
            f"速度={result.get('speed_cm_s')}cm/s "
            f"终端持续={result.get('terminal_hold_duration_s')}s "
            f"误差={error} 容差={tolerance_text} "
            f"抓取点XYZ={grasp_xyz_before} -> {grasp_xyz_after}"
            f"{status_segment}"
        )
        _print_motion_step_diagnostics(result)
        return
    if decision == "descend_after_alignment":
        print(
            "控制结果: 已对准，下降 | "
            f"下降={result.get('requested_distance_cm')}cm "
            f"速度={result.get('speed_cm_s')}cm/s "
            f"终端持续={result.get('terminal_hold_duration_s')}s "
            f"高度={result.get('height_before_cm')}cm -> {result.get('height_after_cm')}cm "
            f"误差={error} 容差={tolerance_text} "
            f"抓取点XYZ={grasp_xyz_before} -> {grasp_xyz_after}"
            f"{status_segment}"
        )
        _print_motion_step_diagnostics(result)
        return
    if decision == "aligned_hold":
        remaining = result.get("remaining_descent_to_final_cm")
        print(
            "控制结果: 已对准，接近目标高度 | "
            f"当前高度={result.get('height_cm')}cm "
            f"误差={error} 容差={tolerance_text} "
            + (f"距最终抓取高度还剩={remaining}cm" if remaining is not None else "")
            + status_segment
        )
        return
    print(f"控制结果: {json.dumps(result, ensure_ascii=False)}")
    _print_motion_step_diagnostics(result)


def _resolve_manual_pixel_arm_config(
    args: argparse.Namespace, *, camera_vector_version: str = "v1"
) -> ArmControlConfig:
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
        camera_vector_version=camera_vector_version,
        manual_progress_check_enabled=(
            getattr(args, "manual_progress_check", "on") == "on"
        ),
    )


async def _run_manual_pixel_test(
    args: argparse.Namespace,
    *,
    default_grasp_x: float | None = None,
    default_grasp_y: float | None = None,
    camera_vector_version: str = "v1",
    display_name: str = "手动像素闭环测试",
) -> int:
    if args.manual_image_width <= 0 or args.manual_image_height <= 0:
        raise ConfigurationError("manual image width/height必须大于0")
    grasp_x = (
        float(args.manual_grasp_x)
        if args.manual_grasp_x is not None
        else (
            float(default_grasp_x)
            if default_grasp_x is not None
            else args.manual_image_width / 2.0
        )
    )
    grasp_y = (
        float(args.manual_grasp_y)
        if args.manual_grasp_y is not None
        else (
            float(default_grasp_y)
            if default_grasp_y is not None
            else args.manual_image_height / 2.0
        )
    )
    if not math.isfinite(grasp_x) or not math.isfinite(grasp_y):
        raise ConfigurationError("manual grasp point必须是有限数字")

    arm_config = _resolve_manual_pixel_arm_config(
        args,
        camera_vector_version=camera_vector_version,
    )
    if arm_config.mode == "hardware":
        print(f"硬件{display_name}：会真实控制机械臂运动。")
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
            print(f"已取消硬件{display_name}。")
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
        print(f"{display_name}（{mode_text}；不调用API，不接相机）")
        print(
            "运动程序: "
            + (
                "camera_vector_terminal_v2 / CameraVectorV2Runtime"
                if camera_vector_version == "v2"
                else "camera_vector_terminal / CameraRelativeManualServoRuntime"
            )
        )
        print(
            f"模拟图像: {args.manual_image_width}x{args.manual_image_height}, "
            f"固定抓取点像素=({grasp_x:g}, {grasp_y:g})"
        )
        print(
            "有效进展检测: "
            + (
                "开启"
                if arm_config.manual_progress_check_enabled
                else "关闭（异常仍输出但不作为中止条件）"
            )
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
                if result.get("status") != "ok":
                    print(
                        "运动未取得有效进展，已停止流程："
                        f"{result.get('error', '目标可能不可达或已触及关节限位')}"
                    )
                    return 2
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
                _print_motion_step_diagnostics(descend_result)
                final_height = descend_result.get(
                    "height_after_cm",
                    descend_result.get("height_cm"),
                )
                print(
                    f"下降结果: 高度={final_height}cm "
                    f"目标={final_grasp_height_cm:g}cm "
                    "容差="
                    f"{descend_result.get('target_tolerance_cm', 0.0)}cm "
                    f"步数={descend_result.get('steps')}"
                )
                final_pose_constraint = descend_result.get(
                    "camera_pose_constraint"
                )
                if (
                    isinstance(final_pose_constraint, dict)
                    and final_pose_constraint.get("relaxed")
                ):
                    print(
                        "姿态约束状态: 最终下降期间已因限位放宽 | "
                        f"原因={final_pose_constraint.get('reasons')} "
                        "放宽步数="
                        f"{final_pose_constraint.get('relaxed_step_count')}"
                    )
                if (
                    descend_result.get("status") != "ok"
                    or not isinstance(final_height, (int, float))
                    or float(final_height)
                    > final_grasp_height_cm
                    + float(descend_result.get("target_tolerance_cm", 0.0))
                ):
                    print(
                        "最终下降未安全到达抓取高度，已停止流程且不会执行夹取。"
                    )
                    return 2
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
            if result.get("status") != "ok":
                print(
                    "运动未取得有效进展，已停止流程："
                    f"{result.get('error', '目标可能不可达或已触及关节限位')}"
                )
                return 2
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
    if args.manual_pixel_test_v2:
        from .manual_pixel_test_v2 import run_manual_pixel_test_v2

        return await run_manual_pixel_test_v2(args)
    if args.manual_pixel_test:
        return await _run_manual_pixel_test(args)

    agent_grasp_point_override = _agent_grasp_point_from_args(args)
    _load_env_file(args.env_file)
    device_path = Path(args.device_config)
    has_device_config = device_path.is_file()
    saved_devices = RuntimeDeviceConfig.load(device_path, required=False)
    agent_grasp_point = agent_grasp_point_override
    if (
        agent_grasp_point is None
        and saved_devices.grasp_point_x is not None
        and saved_devices.grasp_point_y is not None
    ):
        agent_grasp_point = (
            float(saved_devices.grasp_point_x),
            float(saved_devices.grasp_point_y),
        )
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
            f"提示: 机械臂距离上限{configured_max_distance_cm:g}cm已自动限制为"
            f"{MAX_AGENT_MOVE_COMMAND_CM:g}cm。"
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
        grasp_point_x=(
            agent_grasp_point[0] if agent_grasp_point is not None else None
        ),
        grasp_point_y=(
            agent_grasp_point[1] if agent_grasp_point is not None else None
        ),
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
    red_block_mode = bool(getattr(args, "red_block_grasp", False))
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
                if agent_grasp_point_override is not None:
                    configured_point = await bridge.call_tool(
                        "set_jetarm_grasp_point_pixel",
                        {
                            "x": agent_grasp_point_override[0],
                            "y": agent_grasp_point_override[1],
                        },
                    )
                    print(
                        "Agent调用前抓取点已设置: "
                        f"{configured_point.get('grasp_point_pixel')}"
                    )
                registry = await bridge.registry()
                workflow = _workflow_text()
                if red_block_mode:
                    workflow += (
                        "\n\n**当前模式提示：优先使用 detect_red_block_target "
                        "自动检测红色物块中心作为目标点像素，"
                        "而不是 zoom_rgb_target_tile 分块定位。"
                        "每次视觉闭环回合都必须重新调用 detect_red_block_target "
                        "获取最新检测坐标。**"
                    )
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
                print(
                    "Agent抓取点像素: "
                    + (
                        f"({agent_grasp_point[0]:g}, {agent_grasp_point[1]:g})"
                        if agent_grasp_point is not None
                        else "未设置；抓取前请先输入 /grasp-point x y"
                    )
                )
                _print_workflow_summary(red_block_mode=red_block_mode)

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
                        "agent_grasp_point_pixel": (
                            {
                                "x": agent_grasp_point[0],
                                "y": agent_grasp_point[1],
                            }
                            if agent_grasp_point is not None
                            else None
                        ),
                        "agent_grasp_workflow": "manual_pixel_test_v2",
                        "agent_grasp_progress_check_enabled": False,
                        "agent_target_pixel_localization": (
                            "red_block_detector"
                            if red_block_mode
                            else "hierarchical_data_layer_tiles_3x3_depth4"
                        ),
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
                if command == "/grasp-point" or command.startswith(
                    "/grasp-point "
                ):
                    if bridge is None or arm_mode == "off":
                        print("错误: 机械臂MCP未启用。", file=sys.stderr)
                        continue
                    raw_point = text[len("/grasp-point") :].strip()
                    if not raw_point:
                        try:
                            raw_point = input("调用Agent抓取前输入抓取点像素 x y: ")
                        except EOFError:
                            print()
                            continue
                    try:
                        point = _parse_manual_target_pixel(raw_point)
                        if point is None or point[0] < 0.0 or point[1] < 0.0:
                            raise ValueError("抓取点像素必须大于等于0")
                        configured_point = await bridge.call_tool(
                            "set_jetarm_grasp_point_pixel",
                            {"x": point[0], "y": point[1]},
                        )
                    except (ValueError, MCPClientError) as exc:
                        print(f"错误: {exc}", file=sys.stderr)
                        continue
                    agent_grasp_point = point
                    print(
                        "Agent调用前抓取点已设置并重置抓取闭环: "
                        f"{configured_point.get('grasp_point_pixel')}"
                    )
                    continue
                if command == "/camera":
                    if not effective_devices.rgb_camera or arm_session is None:
                        print("错误: 未配置RGB相机或相机MCP未启动。", file=sys.stderr)
                        continue
                    text = "请读取当前RGB相机画面，并只根据实际画面简要描述你看到的内容。"
                    command = text.lower()
                if command in {"/arm-status", "/arm-home", "/arm-init", "/arm-stop"}:
                    if bridge is None or arm_mode == "off":
                        print("错误: 机械臂MCP未启用。", file=sys.stderr)
                        continue
                    tool_name = {
                        "/arm-status": "get_jetarm_state",
                        "/arm-home": "move_jetarm_home",
                        "/arm-init": "initialize_jetarm",
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
