"""Interactive command-line interface for API chat."""

from __future__ import annotations

import argparse
import asyncio
import json
import os
import sys
from pathlib import Path
from typing import Optional

from .arm_control import (
    ARM_TOOL_SYSTEM_PROMPT,
    DEFAULT_TERMINAL_CONFIG,
    ArmControlConfig,
    JetArmToolController,
    build_arm_tool_registry,
    looks_like_arm_command,
)
from .config import AgentSettings, ConfigurationError, DEFAULT_CONFIG_PATH
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
    result = await session.ask(
        text,
        require_any_tool=looks_like_arm_command(text),
        required_tool_retries=1,
    )
    for call in result.tool_calls:
        print(
            f"[arm-tool] {call.name} "
            f"{json.dumps(call.result, ensure_ascii=False)}"
        )
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


async def run(args: argparse.Namespace) -> int:
    _load_env_file(args.env_file)
    settings = AgentSettings.from_sources(
        args.config,
        base_url=args.base_url,
        model=args.model,
    )
    client = OpenAICompatibleClient(settings)
    chat_session = ChatSession(settings, client)

    print("JetArm AI 对话终端")
    print(f"API: {settings.base_url}")
    print(f"模型: {settings.model}")

    if args.tool_test:
        try:
            await _run_tool_test(settings, client)
            return 0
        finally:
            await client.close()

    arm_mode = args.arm_mode or os.getenv("JETARM_ARM_MODE", "off").strip()
    arm_port = args.arm_port or os.getenv("JETARM_ARM_PORT") or None
    arm_config_path = Path(
        args.arm_config
        or os.getenv("JETARM_ARM_CONFIG", "")
        or DEFAULT_TERMINAL_CONFIG
    )
    max_distance_cm = (
        args.arm_max_distance_cm
        if args.arm_max_distance_cm is not None
        else float(os.getenv("JETARM_ARM_MAX_DISTANCE_CM", "10"))
    )

    arm_controller: JetArmToolController | None = None
    arm_session: ToolCallingSession | None = None
    if arm_mode != "off":
        arm_controller = JetArmToolController(
            ArmControlConfig(
                mode=arm_mode,
                serial_port=arm_port,
                terminal_config_path=arm_config_path,
                max_distance_cm=max_distance_cm,
            ),
            logger=lambda message: print(f"[arm] {message}"),
        )
        arm_session = ToolCallingSession(
            settings,
            client,
            build_arm_tool_registry(arm_controller),
            system_prompt=f"{settings.system_prompt}\n\n{ARM_TOOL_SYSTEM_PROMPT}",
            max_rounds=8,
        )
        port_text = arm_controller.serial_port or "模拟控制器"
        print(f"机械臂工具: {arm_mode} ({port_text})")

    try:
        if args.once:
            if arm_session is not None:
                await _send_with_tools(arm_session, args.once)
            elif looks_like_arm_command(args.once):
                raise ConfigurationError(
                    "检测到机械臂指令，但机械臂工具未启用。"
                    "请使用--arm-mode dry-run测试，或使用"
                    "--arm-mode hardware --arm-port <串口>连接硬件。"
                )
            else:
                await _send(chat_session, args.once)
            return 0

        _print_help(arm_enabled=arm_controller is not None)
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
                _print_help(arm_enabled=arm_controller is not None)
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
                summary["arm"] = {
                    "mode": arm_mode,
                    "serial_port": arm_port,
                    "config": str(arm_config_path),
                    "max_distance_cm": max_distance_cm,
                }
                print(json.dumps(summary, ensure_ascii=False, indent=2))
                continue
            if command == "/tool-test":
                try:
                    await _run_tool_test(settings, client)
                except (APIClientError, RuntimeError, ValueError) as exc:
                    print(f"错误: {exc}", file=sys.stderr)
                continue
            if command in {"/arm-status", "/arm-home", "/arm-stop"}:
                if arm_controller is None:
                    print("错误: 机械臂工具未启用，请使用--arm-mode。", file=sys.stderr)
                    continue
                try:
                    if command == "/arm-status":
                        result = await arm_controller.state()
                    elif command == "/arm-home":
                        result = await arm_controller.go_home()
                    else:
                        result = await arm_controller.stop_all()
                    print(json.dumps(result, ensure_ascii=False, indent=2))
                except (RuntimeError, ValueError) as exc:
                    print(f"错误: {exc}", file=sys.stderr)
                continue
            if arm_session is None and looks_like_arm_command(text):
                print(
                    "错误: 检测到机械臂指令，但机械臂工具未启用。\n"
                    "请退出后使用 --arm-mode dry-run 测试；确认无误后使用 "
                    "--arm-mode hardware --arm-port <串口>。",
                    file=sys.stderr,
                )
                continue
            try:
                if arm_session is not None:
                    await _send_with_tools(arm_session, text)
                else:
                    await _send(chat_session, text)
            except (APIClientError, RuntimeError, ValueError) as exc:
                print(f"\n错误: {exc}", file=sys.stderr)
    finally:
        if arm_controller is not None:
            arm_controller.close()
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
