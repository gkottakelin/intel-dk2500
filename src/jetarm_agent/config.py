"""Configuration loading for the JetArm AI terminal."""

from __future__ import annotations

import json
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Optional


DEFAULT_CONFIG_PATH = Path(__file__).resolve().parents[2] / "config" / "ai_agent.json"


class ConfigurationError(ValueError):
    """Raised when the AI terminal configuration is incomplete or invalid."""


@dataclass(frozen=True)
class AgentSettings:
    provider: str
    base_url: str
    model: str
    api_key_env: str
    timeout_s: float
    extra_body: dict[str, Any]
    temperature: Optional[float]
    max_tokens: int
    max_history_messages: int
    system_prompt: str

    @classmethod
    def from_sources(
        cls,
        config_path: str | Path = DEFAULT_CONFIG_PATH,
        *,
        environ: Optional[Mapping[str, str]] = None,
        base_url: Optional[str] = None,
        model: Optional[str] = None,
    ) -> "AgentSettings":
        """Load JSON configuration, then apply environment and CLI overrides.

        Precedence is CLI argument, environment variable, then JSON file.
        API keys are never read from the JSON file.
        """

        path = Path(config_path)
        try:
            with path.open("r", encoding="utf-8") as file:
                payload = json.load(file)
        except FileNotFoundError as exc:
            raise ConfigurationError(f"AI配置文件不存在: {path}") from exc
        except json.JSONDecodeError as exc:
            raise ConfigurationError(f"AI配置文件不是有效JSON: {path}: {exc}") from exc

        env = environ if environ is not None else os.environ
        api = payload.get("api", {})
        conversation = payload.get("conversation", {})
        raw_extra_body = api.get("extra_body", {})
        if not isinstance(raw_extra_body, dict):
            raise ConfigurationError("api.extra_body必须是JSON对象")
        raw_temperature = conversation.get("temperature", 0.2)

        resolved_base_url = (
            base_url
            or env.get("JETARM_API_BASE_URL")
            or str(api.get("base_url", ""))
        ).strip().rstrip("/")
        resolved_model = (
            model
            or env.get("JETARM_API_MODEL")
            or str(api.get("model", ""))
        ).strip()

        settings = cls(
            provider=str(api.get("provider", "openai_compatible")).strip(),
            base_url=resolved_base_url,
            model=resolved_model,
            api_key_env=str(api.get("api_key_env", "JETARM_API_KEY")).strip(),
            timeout_s=float(api.get("timeout_s", 60)),
            extra_body=dict(raw_extra_body),
            temperature=(
                None if raw_temperature is None else float(raw_temperature)
            ),
            max_tokens=int(conversation.get("max_tokens", 2048)),
            max_history_messages=int(conversation.get("max_history_messages", 40)),
            system_prompt=str(conversation.get("system_prompt", "")).strip(),
        )
        settings.validate()
        return settings

    def validate(self) -> None:
        if self.provider != "openai_compatible":
            raise ConfigurationError(
                f"当前阶段只支持provider=openai_compatible，收到: {self.provider!r}"
            )
        if not self.base_url:
            raise ConfigurationError("缺少API base_url")
        if not self.model:
            raise ConfigurationError("缺少API model")
        if not self.api_key_env:
            raise ConfigurationError("api_key_env不能为空")
        if self.timeout_s <= 0:
            raise ConfigurationError("timeout_s必须大于0")
        if self.temperature is not None and not 0 <= self.temperature <= 2:
            raise ConfigurationError("temperature必须在0..2之间")
        if self.max_tokens <= 0:
            raise ConfigurationError("max_tokens必须大于0")
        if self.max_history_messages < 2:
            raise ConfigurationError("max_history_messages不能小于2")
        if not self.system_prompt:
            raise ConfigurationError("system_prompt不能为空")

    def resolve_api_key(self, environ: Optional[Mapping[str, str]] = None) -> str:
        env = environ if environ is not None else os.environ
        value = str(env.get(self.api_key_env, "")).strip()
        if not value:
            raise ConfigurationError(
                f"缺少API Key，请设置环境变量 {self.api_key_env}"
            )
        return value

    def public_summary(self) -> dict[str, object]:
        """Return safe configuration values suitable for terminal display."""

        return {
            "provider": self.provider,
            "base_url": self.base_url,
            "model": self.model,
            "api_key_env": self.api_key_env,
            "timeout_s": self.timeout_s,
            "extra_body": self.extra_body,
            "temperature": self.temperature,
            "max_tokens": self.max_tokens,
            "max_history_messages": self.max_history_messages,
        }
