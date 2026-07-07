"""Interactive command-line interface for API chat."""

from __future__ import annotations

import argparse
import asyncio
import json
import os
import sys
from contextlib import AsyncExitStack
from pathlib import Path
from typing import Optional

from .arm_control import (
    ARM_TOOL_SYSTEM_PROMPT,
    DEFAULT_TERMINAL_CONFIG,
    choose_arm_serial_port,
    looks_like_arm_command,
    required_mcp_tool_for_command,
)
from .config import AgentSettings, ConfigurationError, DEFAULT_CONFIG_PATH
from .device_config import DEFAULT_DEVICE_CONFIG_PATH, RuntimeDeviceConfig
from .mcp_client import MCPClientError, MCPRobotBridge
from .mcp_server import DEFAULT_WORKFLOW_PATH
from .openai_compatible import APIClientError, OpenAICompatibleClient
from .roundtrip_test import run_counter_roundtrip_test
from .session import ChatSession
from .tool_agent import ToolCallingSession


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
        help="AI单次TCP移动距离上限，默认10cm",
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


def _print_help(*, arm_enabled: bool = False) -> None:
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
    print("  /exit     退出")


def _print_history(session: object) -> None:
    history = getattr(session, "history", [])
    if not history:
        print("当前没有对话记录。")
        return
    labels = {"user": "你", "assistant": "AI", "tool": "工具"}
    for index, message in enumerate(history, 1):
        label = labels.get(message.get("role"), str(message.get("role", "?")))
        print(f"{index:02d} {label}: {message.get('content', '')}")


async def _send(session: ChatSession, text: str) -> str:
    print("AI: ", end="", flush=True)
    answer = await session.ask(text, on_token=lambda token: print(token, end="", flush=True))
    print()
    return answer


async def _send_with_tools(session: ToolCallingSession, text: str) -> str:
    required_tool = required_mcp_tool_for_command(text)
    is_arm_command = looks_like_arm_command(text)
    if is_arm_command:
        print(f"[工作流 1/5] 接收自然语言: {text}")
        print("[工作流 2/5] Agent解析意图并生成MCP工具调用")
    result = await session.ask(
        text,
        require_any_tool=looks_like_arm_command(text),
        required_tool_name=required_tool,
        required_tool_retries=1,
    )
    for call in result.tool_calls:
        prefix = "[工作流 3/5] MCP调用" if is_arm_command else "[MCP调用]"
        print(f"{prefix}: {call.name} {json.dumps(call.arguments, ensure_ascii=False)}")
        prefix = "[工作流 4/5] MCP结果" if is_arm_command else "[MCP结果]"
        print(f"{prefix}: {call.name} {json.dumps(call.result, ensure_ascii=False)}")
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
    print("  1. Agent解析自然语言和距离/速度")
    print("  2. Agent调用本地JetArm MCP工具")
    print("  3. 控制器把动作拆成每段不超过3cm")
    print("  4. 全部分段完成后MCP返回status=ok")
    print("  5. Agent读取结果并生成总结报告")


async def run(args: argparse.Namespace) -> int:
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
    max_distance_cm = (
        args.arm_max_distance_cm
        if args.arm_max_distance_cm is not None
        else float(os.getenv("JETARM_ARM_MAX_DISTANCE_CM", "10"))
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
            if arm_mode != "off":
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
                    max_rounds=8,
                )
                print(f"机械臂MCP: {arm_mode} ({arm_port or '模拟控制器'})")
                _print_workflow_summary()

            if args.once:
                if arm_session is not None:
                    await _send_with_tools(arm_session, args.once)
                elif looks_like_arm_command(args.once):
                    raise ConfigurationError("检测到机械臂指令，但设备配置中的arm_mode为off")
                else:
                    await _send(chat_session, args.once)
                return 0

            _print_help(arm_enabled=bridge is not None)
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
                    _print_help(arm_enabled=bridge is not None)
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
                        "max_segment_cm": 3.0,
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
                if command in {"/arm-status", "/arm-home", "/arm-stop"}:
                    if bridge is None:
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
