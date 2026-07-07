"""Model -> local tool -> model execution loop."""

from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Any

from .config import AgentSettings
from .openai_compatible import OpenAICompatibleClient, ToolModelResponse
from .tooling import (
    ToolExecutionError,
    ToolExecutionPayload,
    ToolImage,
    ToolRegistry,
)


@dataclass(frozen=True)
class ExecutedToolCall:
    call_id: str
    name: str
    arguments: dict[str, Any]
    result: object
    images: tuple[ToolImage, ...] = ()


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

    def clear(self) -> None:
        self.history.clear()

    async def ask(
        self,
        text: str,
        *,
        first_tool_choice: object = "auto",
        allow_additional_tools: bool = True,
        require_any_tool: bool = False,
        required_tool_name: str | None = None,
        required_tool_retries: int = 1,
        preselected_tool_arguments: dict[str, Any] | None = None,
    ) -> ToolAgentResult:
        user_text = text.strip()
        if not user_text:
            raise ValueError("对话内容不能为空")

        turn: list[dict[str, Any]] = [{"role": "user", "content": user_text}]
        executed: list[ExecutedToolCall] = []
        tool_choice = first_tool_choice
        retry_count = 0

        if preselected_tool_arguments is not None:
            if required_tool_name is None:
                raise ValueError("预选工具必须同时提供required_tool_name")
            call_id = f"local-{required_tool_name}-{len(self.history)}"
            raw_arguments = json.dumps(preselected_tool_arguments, ensure_ascii=False)
            turn.append(
                {
                    "role": "assistant",
                    "content": None,
                    "tool_calls": [
                        {
                            "id": call_id,
                            "type": "function",
                            "function": {
                                "name": required_tool_name,
                                "arguments": raw_arguments,
                            },
                        }
                    ],
                }
            )
            arguments, result, images = await self._execute(
                required_tool_name, raw_arguments
            )
            executed.append(
                ExecutedToolCall(
                    call_id=call_id,
                    name=required_tool_name,
                    arguments=arguments,
                    result=result,
                    images=images,
                )
            )
            turn.append(
                {
                    "role": "tool",
                    "tool_call_id": call_id,
                    "name": required_tool_name,
                    "content": json.dumps(result, ensure_ascii=False),
                }
            )
            if images:
                self._append_latest_images(turn, images)

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
                required_name_executed = required_tool_name is None or any(
                    call.name == required_tool_name for call in executed
                )
                any_tool_executed = not require_any_tool or bool(executed)
                if not required_name_executed or not any_tool_executed:
                    if retry_count >= required_tool_retries:
                        required = required_tool_name or "任一已注册工具"
                        raise RuntimeError(f"AI没有调用必需工具: {required}")
                    retry_count += 1
                    required = required_tool_name or "适合当前指令的已注册工具"
                    turn.append(
                        {
                            "role": "user",
                            "content": (
                                f"本次指令必须调用{required}，"
                                "请现在返回tool_calls，不要只回复文字。"
                            ),
                        }
                    )
                    tool_choice = "auto"
                    continue
                answer = response.content.strip()
                if not answer:
                    raise RuntimeError("API返回了空回复且没有工具调用")
                self.history.extend(turn)
                self._trim_history()
                return ToolAgentResult(answer, tuple(executed))

            latest_images: tuple[ToolImage, ...] = ()
            for tool_call in response.tool_calls:
                arguments, result, images = await self._execute(
                    tool_call.name, tool_call.arguments
                )
                executed.append(
                    ExecutedToolCall(
                        call_id=tool_call.call_id,
                        name=tool_call.name,
                        arguments=arguments,
                        result=result,
                        images=images,
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
                if images:
                    latest_images = images

            if latest_images:
                self._append_latest_images(turn, latest_images)

            tool_choice = "auto" if allow_additional_tools else "none"

        raise RuntimeError(f"工具调用超过最大轮数: {self.max_rounds}")

    async def _execute(
        self, name: str, raw_arguments: str
    ) -> tuple[dict[str, Any], object, tuple[ToolImage, ...]]:
        try:
            parsed = json.loads(raw_arguments or "{}")
            if not isinstance(parsed, dict):
                raise ToolExecutionError("工具参数必须是JSON对象")
            raw_result = await self.registry.execute(name, parsed)
            if isinstance(raw_result, ToolExecutionPayload):
                return parsed, raw_result.value, raw_result.images
            return parsed, raw_result, ()
        except (json.JSONDecodeError, ToolExecutionError, ValueError) as exc:
            arguments = parsed if "parsed" in locals() and isinstance(parsed, dict) else {}
            return arguments, {"status": "error", "error": str(exc)}, ()
        except Exception as exc:
            return {}, {"status": "error", "error": f"工具执行异常: {exc}"}, ()

    @staticmethod
    def _remove_images(messages: list[dict[str, Any]]) -> None:
        """Keep text/tool history while removing older base64 camera frames."""

        retained: list[dict[str, Any]] = []
        for message in messages:
            content = message.get("content")
            if not isinstance(content, list):
                retained.append(message)
                continue
            filtered = [
                part
                for part in content
                if not (
                    isinstance(part, dict)
                    and part.get("type") in {"image", "image_url"}
                )
            ]
            if filtered:
                retained.append({**message, "content": filtered})
        messages[:] = retained

    def _append_latest_images(
        self, turn: list[dict[str, Any]], images: tuple[ToolImage, ...]
    ) -> None:
        self._remove_images(self.history)
        self._remove_images(turn)
        turn.append(
            {
                "role": "user",
                "content": [
                    {
                        "type": "text",
                        "text": "这是JetArm单路RGB相机刚刚返回的最新画面。",
                    },
                    *(image.openai_content_part() for image in images),
                ],
            }
        )

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
