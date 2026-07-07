"""Model -> local tool -> model execution loop."""

from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Any

from .config import AgentSettings
from .openai_compatible import OpenAICompatibleClient, ToolModelResponse
from .tooling import ToolExecutionError, ToolRegistry


@dataclass(frozen=True)
class ExecutedToolCall:
    call_id: str
    name: str
    arguments: dict[str, Any]
    result: object


@dataclass(frozen=True)
class ToolAgentResult:
    text: str
    tool_calls: tuple[ExecutedToolCall, ...]


class ToolCallingSession:
    """Run bounded tool calls while preserving valid conversation messages."""

    def __init__(
        self,
        settings: AgentSettings,
        client: OpenAICompatibleClient,
        registry: ToolRegistry,
        *,
        system_prompt: str | None = None,
        max_rounds: int = 8,
    ) -> None:
        if max_rounds < 1:
            raise ValueError("max_rounds必须大于0")
        self.settings = settings
        self.client = client
        self.registry = registry
        self.system_prompt = system_prompt or settings.system_prompt
        self.max_rounds = max_rounds
        self.history: list[dict[str, Any]] = []

    async def ask(
        self,
        text: str,
        *,
        first_tool_choice: object = "auto",
        allow_additional_tools: bool = True,
    ) -> ToolAgentResult:
        user_text = text.strip()
        if not user_text:
            raise ValueError("对话内容不能为空")

        turn: list[dict[str, Any]] = [{"role": "user", "content": user_text}]
        executed: list[ExecutedToolCall] = []
        tool_choice = first_tool_choice

        for _ in range(self.max_rounds):
            messages = [
                {"role": "system", "content": self.system_prompt},
                *self.history,
                *turn,
            ]
            response = await self.client.complete_with_tools(
                messages,
                self.registry.schemas(),
                tool_choice=tool_choice,
            )
            turn.append(response.assistant_message())

            if not response.tool_calls:
                answer = response.content.strip()
                if not answer:
                    raise RuntimeError("API返回了空回复且没有工具调用")
                self.history.extend(turn)
                self._trim_history()
                return ToolAgentResult(answer, tuple(executed))

            for tool_call in response.tool_calls:
                arguments, result = await self._execute(tool_call.name, tool_call.arguments)
                executed.append(
                    ExecutedToolCall(
                        call_id=tool_call.call_id,
                        name=tool_call.name,
                        arguments=arguments,
                        result=result,
                    )
                )
                turn.append(
                    {
                        "role": "tool",
                        "tool_call_id": tool_call.call_id,
                        "name": tool_call.name,
                        "content": json.dumps(result, ensure_ascii=False),
                    }
                )

            tool_choice = "auto" if allow_additional_tools else "none"

        raise RuntimeError(f"工具调用超过最大轮数: {self.max_rounds}")

    async def _execute(
        self, name: str, raw_arguments: str
    ) -> tuple[dict[str, Any], object]:
        try:
            parsed = json.loads(raw_arguments or "{}")
            if not isinstance(parsed, dict):
                raise ToolExecutionError("工具参数必须是JSON对象")
            result = await self.registry.execute(name, parsed)
            return parsed, result
        except (json.JSONDecodeError, ToolExecutionError, ValueError) as exc:
            arguments = parsed if "parsed" in locals() and isinstance(parsed, dict) else {}
            return arguments, {"status": "error", "error": str(exc)}
        except Exception as exc:
            return {}, {"status": "error", "error": f"工具执行异常: {exc}"}

    def _trim_history(self) -> None:
        limit = self.settings.max_history_messages
        while len(self.history) > limit:
            next_user = next(
                (
                    index
                    for index, message in enumerate(self.history[1:], 1)
                    if message.get("role") == "user"
                ),
                len(self.history),
            )
            del self.history[:next_user]
