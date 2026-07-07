"""Interactive command-line interface for API chat."""

from __future__ import annotations

import argparse
import asyncio
import json
import sys
from pathlib import Path
from typing import Optional

from .config import AgentSettings, ConfigurationError, DEFAULT_CONFIG_PATH
from .openai_compatible import APIClientError, OpenAICompatibleClient
from .session import ChatSession


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="JetArm AI command-line dialogue")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG_PATH), help="AI JSON配置文件")
    parser.add_argument("--base-url", default=None, help="覆盖API base URL")
    parser.add_argument("--model", default=None, help="覆盖模型名称")
    parser.add_argument("--once", default=None, help="发送一条消息后退出")
    parser.add_argument("--env-file", default=None, help="可选.env文件路径")
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


def _print_help() -> None:
    print("可用命令:")
    print("  /help     显示帮助")
    print("  /clear    清空当前对话上下文")
    print("  /history  显示当前上下文")
    print("  /config   显示非敏感API配置")
    print("  /exit     退出")


def _print_history(session: ChatSession) -> None:
    if not session.history:
        print("当前没有对话记录。")
        return
    for index, message in enumerate(session.history, 1):
        label = "你" if message["role"] == "user" else "AI"
        print(f"{index:02d} {label}: {message['content']}")


async def _send(session: ChatSession, text: str) -> str:
    print("AI: ", end="", flush=True)
    answer = await session.ask(text, on_token=lambda token: print(token, end="", flush=True))
    print()
    return answer


async def run(args: argparse.Namespace) -> int:
    _load_env_file(args.env_file)
    settings = AgentSettings.from_sources(
        args.config,
        base_url=args.base_url,
        model=args.model,
    )
    client = OpenAICompatibleClient(settings)
    session = ChatSession(settings, client)

    print("JetArm AI 对话终端")
    print(f"API: {settings.base_url}")
    print(f"模型: {settings.model}")

    if args.once:
        await _send(session, args.once)
        return 0

    _print_help()
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
            _print_help()
            continue
        if command == "/clear":
            session.clear()
            print("对话上下文已清空。")
            continue
        if command == "/history":
            _print_history(session)
            continue
        if command == "/config":
            print(json.dumps(settings.public_summary(), ensure_ascii=False, indent=2))
            continue
        try:
            await _send(session, text)
        except (APIClientError, RuntimeError, ValueError) as exc:
            print(f"\n错误: {exc}", file=sys.stderr)


def main(argv: Optional[list[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        return asyncio.run(run(args))
    except KeyboardInterrupt:
        print("\n已退出。")
        return 130
    except (ConfigurationError, APIClientError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
