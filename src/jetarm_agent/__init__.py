"""JetArm command-line AI client.

The first migration stage contains only API access and multi-turn dialogue.
Camera, MCP, and robot-control tools are intentionally added in later stages.
"""

from .config import AgentSettings, ConfigurationError
from .openai_compatible import APIClientError, OpenAICompatibleClient
from .session import ChatSession

__all__ = [
    "APIClientError",
    "AgentSettings",
    "ChatSession",
    "ConfigurationError",
    "OpenAICompatibleClient",
]
