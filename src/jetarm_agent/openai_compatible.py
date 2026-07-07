"""OpenAI-compatible streaming chat provider.

This keeps the provider boundary used by ``robot_MCP`` while adding an explicit
``base_url`` so hosted gateways and self-hosted compatible APIs can be used.
"""

from __future__ import annotations

from collections.abc import AsyncIterator, Sequence
from typing import Any, Optional

from .config import AgentSettings


class APIClientError(RuntimeError):
    """A user-facing API initialization or request error."""


class OpenAICompatibleClient:
    def __init__(self, settings: AgentSettings, *, client: Optional[Any] = None) -> None:
        self.settings = settings
        if client is not None:
            self._client = client
            return

        try:
            from openai import AsyncOpenAI
        except ImportError as exc:
            raise APIClientError(
                "缺少openai依赖，请先执行: python -m pip install -r requirements-ai.txt"
            ) from exc

        try:
            self._client = AsyncOpenAI(
                api_key=settings.resolve_api_key(),
                base_url=settings.base_url,
                timeout=settings.timeout_s,
            )
        except Exception as exc:
            detail = str(exc)
            if "proxy URL" in detail and "socks" in detail.lower():
                raise APIClientError(
                    "SOCKS代理配置无效：请把代理地址的socks://改为socks5://，"
                    "并执行 python -m pip install -r requirements-ai.txt；"
                    f"原始错误: {detail}"
                ) from exc
            raise APIClientError(f"API客户端初始化失败: {detail}") from exc

    async def stream_chat(self, messages: Sequence[dict[str, str]]) -> AsyncIterator[str]:
        """Yield text deltas from one chat-completions request."""

        request = {
            "model": self.settings.model,
            "messages": list(messages),
            "temperature": self.settings.temperature,
            "max_tokens": self.settings.max_tokens,
            "stream": True,
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
