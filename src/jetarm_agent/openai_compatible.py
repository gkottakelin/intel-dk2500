"""OpenAI-compatible streaming chat provider.

This keeps the provider boundary used by ``robot_MCP`` while adding an explicit
``base_url`` so hosted gateways and self-hosted compatible APIs can be used.
"""

from __future__ import annotations

from collections.abc import AsyncIterator, Sequence
from dataclasses import dataclass
from typing import Any, Optional

from .config import AgentSettings


class APIClientError(RuntimeError):
    """A user-facing API initialization or request error."""


@dataclass(frozen=True)
class FunctionToolCall:
    call_id: str
    name: str
    arguments: str

    def as_message_item(self) -> dict[str, Any]:
        return {
            "id": self.call_id,
            "type": "function",
            "function": {"name": self.name, "arguments": self.arguments},
        }


@dataclass(frozen=True)
class ToolModelResponse:
    content: str
    tool_calls: tuple[FunctionToolCall, ...]

    def assistant_message(self) -> dict[str, Any]:
        message: dict[str, Any] = {
            "role": "assistant",
            "content": self.content or None,
        }
        if self.tool_calls:
            message["tool_calls"] = [call.as_message_item() for call in self.tool_calls]
        return message


class OpenAICompatibleClient:
    def __init__(self, settings: AgentSettings, *, client: Optional[Any] = None) -> None:
        self.settings = settings
        if client is not None:
            self._client = client
            return

        try:
            from openai import AsyncOpenAI
            import httpx
        except ImportError as exc:
            raise APIClientError(
                "缺少openai/httpx依赖，请先执行: "
                "python -m pip install -r requirements-ai.txt"
            ) from exc

        try:
            # Git may need a shell proxy, while Kimi should connect directly.
            # Never inherit HTTP_PROXY/HTTPS_PROXY/ALL_PROXY in the Agent.
            self._http_client = httpx.AsyncClient(
                trust_env=False,
                timeout=settings.timeout_s,
            )
            self._client = AsyncOpenAI(
                api_key=settings.resolve_api_key(),
                base_url=settings.base_url,
                timeout=settings.timeout_s,
                http_client=self._http_client,
            )
        except Exception as exc:
            detail = str(exc)
            raise APIClientError(f"API客户端初始化失败: {detail}") from exc

    async def close(self) -> None:
        close = getattr(self._client, "close", None)
        if callable(close):
            result = close()
            if hasattr(result, "__await__"):
                await result

    def _generation_options(self) -> dict[str, Any]:
        options: dict[str, Any] = {}
        if self.settings.temperature is not None:
            options["temperature"] = self.settings.temperature
        if self.settings.extra_body:
            options["extra_body"] = dict(self.settings.extra_body)
        return options

    async def stream_chat(self, messages: Sequence[dict[str, str]]) -> AsyncIterator[str]:
        """Yield text deltas from one chat-completions request."""

        request = {
            "model": self.settings.model,
            "messages": list(messages),
            "max_tokens": self.settings.max_tokens,
            "stream": True,
            **self._generation_options(),
        }
        try:
            stream = await self._client.chat.completions.create(**request)
            async for chunk in stream:
                choices = getattr(chunk, "choices", None)
                if not choices:
                    continue
                delta = getattr(choices[0], "delta", None)
                text = getattr(delta, "content", None) if delta is not None else None
                if text:
                    yield str(text)
        except Exception as exc:
            raise APIClientError(f"API请求失败: {exc}") from exc

    async def complete_with_tools(
        self,
        messages: Sequence[dict[str, Any]],
        tools: Sequence[dict[str, Any]],
        *,
        tool_choice: object = "auto",
    ) -> ToolModelResponse:
        """Request one non-streaming response that may contain function calls."""

        request = {
            "model": self.settings.model,
            "messages": list(messages),
            "max_tokens": self.settings.max_tokens,
            "tools": list(tools),
            "tool_choice": tool_choice,
            "stream": False,
            **self._generation_options(),
        }
        try:
            response = await self._client.chat.completions.create(**request)
            choices = getattr(response, "choices", None)
            if not choices:
                raise RuntimeError("API没有返回choices")
            message = getattr(choices[0], "message", None)
            if message is None:
                raise RuntimeError("API没有返回assistant message")

            calls: list[FunctionToolCall] = []
            for call in getattr(message, "tool_calls", None) or []:
                function = getattr(call, "function", None)
                name = getattr(function, "name", None) if function is not None else None
                if not name:
                    raise RuntimeError("API返回的工具调用缺少函数名称")
                calls.append(
                    FunctionToolCall(
                        call_id=str(getattr(call, "id", "") or ""),
                        name=str(name),
                        arguments=str(getattr(function, "arguments", "{}") or "{}"),
                    )
                )
            if any(not call.call_id for call in calls):
                raise RuntimeError("API返回的工具调用缺少call id")

            content = getattr(message, "content", None)
            return ToolModelResponse(
                content=str(content) if content is not None else "",
                tool_calls=tuple(calls),
            )
        except APIClientError:
            raise
        except Exception as exc:
            raise APIClientError(f"API工具调用请求失败: {exc}") from exc
