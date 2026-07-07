"""Multi-turn conversation state for the JetArm AI terminal."""

from __future__ import annotations

from collections.abc import Callable
from typing import Optional

from .config import AgentSettings
from .openai_compatible import OpenAICompatibleClient


TokenCallback = Callable[[str], None]


class ChatSession:
    def __init__(self, settings: AgentSettings, client: OpenAICompatibleClient) -> None:
        self.settings = settings
        self.client = client
        self.history: list[dict[str, str]] = []

    def clear(self) -> None:
        self.history.clear()

    def request_messages(self) -> list[dict[str, str]]:
        return [
            {"role": "system", "content": self.settings.system_prompt},
            *self.history,
        ]

    async def ask(self, text: str, *, on_token: Optional[TokenCallback] = None) -> str:
        user_text = text.strip()
        if not user_text:
            raise ValueError("对话内容不能为空")

        self.history.append({"role": "user", "content": user_text})
        chunks: list[str] = []
        try:
            async for token in self.client.stream_chat(self.request_messages()):
                chunks.append(token)
                if on_token is not None:
                    on_token(token)
        except Exception:
            # A failed request must not poison the next turn's context.
            self.history.pop()
            raise

        answer = "".join(chunks).strip()
        if not answer:
            self.history.pop()
            raise RuntimeError("API返回了空回复")

        self.history.append({"role": "assistant", "content": answer})
        self._trim_history()
        return answer

    def _trim_history(self) -> None:
        limit = self.settings.max_history_messages
        if len(self.history) <= limit:
            return
        self.history = self.history[-limit:]
        # Avoid starting the retained context with an orphan assistant reply.
        if self.history and self.history[0]["role"] == "assistant":
            self.history.pop(0)
