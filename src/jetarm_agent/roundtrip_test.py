"""Live program -> AI -> local program -> AI integration test."""

from __future__ import annotations

import asyncio
from collections.abc import Awaitable, Callable
from dataclasses import dataclass

from .config import AgentSettings
from .openai_compatible import OpenAICompatibleClient
from .tool_agent import ToolAgentResult, ToolCallingSession
from .tooling import TestCounter, ToolRegistry


SleepFunction = Callable[[float], Awaitable[None]]
StatusCallback = Callable[[str], None]


@dataclass(frozen=True)
class RoundTripTestResult:
    counter: int
    answer: str
    tool_call_count: int


async def run_counter_roundtrip_test(
    settings: AgentSettings,
    client: OpenAICompatibleClient,
    *,
    delay_s: float = 3.0,
    sleep: SleepFunction = asyncio.sleep,
    on_status: StatusCallback | None = None,
) -> RoundTripTestResult:
    """Wait, send ``ok`` to the model, execute its counter call, then call AI again."""

    if delay_s < 0:
        raise ValueError("delay_s不能小于0")

    counter = TestCounter()
    registry = ToolRegistry([counter.definition()])
    session = ToolCallingSession(
        settings,
        client,
        registry,
        system_prompt=(
            "This is a deterministic bidirectional integration test. When the user "
            "message is exactly 'ok', call increment_test_counter exactly once with "
            "amount=1. After receiving the tool result, do not call another tool; "
            "report the returned count briefly."
        ),
        max_rounds=3,
    )

    if on_status is not None:
        on_status(f"程序等待{delay_s:g}秒后向AI发送ok")
    await sleep(delay_s)
    if on_status is not None:
        on_status("程序 -> AI: ok")

    required_counter_call = {
        "type": "function",
        "function": {"name": TestCounter.TOOL_NAME},
    }
    result: ToolAgentResult = await session.ask(
        "ok",
        first_tool_choice=required_counter_call,
        allow_additional_tools=False,
    )

    if counter.value != 1:
        raise RuntimeError(f"贯通测试失败：计数器期望1，实际{counter.value}")
    if on_status is not None:
        on_status(f"AI -> 程序: {TestCounter.TOOL_NAME}，计数={counter.value}")
        on_status(f"程序 -> AI: 工具结果ok，AI最终回复: {result.text}")

    return RoundTripTestResult(
        counter=counter.value,
        answer=result.text,
        tool_call_count=len(result.tool_calls),
    )
