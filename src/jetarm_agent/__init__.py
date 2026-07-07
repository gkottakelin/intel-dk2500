"""JetArm command-line AI client and safe local tool-calling runtime."""

from .arm_control import (
    ArmControlConfig,
    ArmControlError,
    JetArmToolController,
    build_arm_tool_registry,
)
from .config import AgentSettings, ConfigurationError
from .openai_compatible import APIClientError, OpenAICompatibleClient
from .roundtrip_test import RoundTripTestResult, run_counter_roundtrip_test
from .session import ChatSession
from .tool_agent import ToolAgentResult, ToolCallingSession
from .tooling import TestCounter, ToolDefinition, ToolExecutionError, ToolRegistry

__all__ = [
    "APIClientError",
    "AgentSettings",
    "ArmControlConfig",
    "ArmControlError",
    "ChatSession",
    "ConfigurationError",
    "OpenAICompatibleClient",
    "RoundTripTestResult",
    "TestCounter",
    "ToolAgentResult",
    "ToolCallingSession",
    "ToolDefinition",
    "ToolExecutionError",
    "ToolRegistry",
    "JetArmToolController",
    "build_arm_tool_registry",
    "run_counter_roundtrip_test",
]
