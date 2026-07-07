"""Safe local tool registration and execution for the AI agent."""

from __future__ import annotations

from collections.abc import Awaitable, Callable, Mapping
from dataclasses import dataclass
from typing import Any


ToolHandler = Callable[[Mapping[str, Any]], Awaitable[object]]


class ToolExecutionError(RuntimeError):
    """Raised when a model requests an unknown or invalid local tool call."""


@dataclass(frozen=True)
class ToolImage:
    """Base64 image returned by a local or MCP tool."""

    data: str
    mime_type: str

    def openai_content_part(self) -> dict[str, Any]:
        return {
            "type": "image_url",
            "image_url": {
                "url": f"data:{self.mime_type};base64,{self.data}",
                "detail": "low",
            },
        }


@dataclass(frozen=True)
class ToolExecutionPayload:
    """JSON-compatible tool result plus optional visual observations."""

    value: object
    images: tuple[ToolImage, ...] = ()


@dataclass(frozen=True)
class ToolDefinition:
    name: str
    description: str
    parameters: dict[str, Any]
    handler: ToolHandler

    def api_schema(self) -> dict[str, Any]:
        return {
            "type": "function",
            "function": {
                "name": self.name,
                "description": self.description,
                "parameters": self.parameters,
            },
        }


class ToolRegistry:
    """Explicit allow-list of functions the model may request."""

    def __init__(self, tools: list[ToolDefinition] | None = None) -> None:
        self._tools: dict[str, ToolDefinition] = {}
        for tool in tools or []:
            self.register(tool)

    def register(self, tool: ToolDefinition) -> None:
        if not tool.name:
            raise ValueError("工具名称不能为空")
        if tool.name in self._tools:
            raise ValueError(f"工具已注册: {tool.name}")
        self._tools[tool.name] = tool

    def schemas(self) -> list[dict[str, Any]]:
        return [tool.api_schema() for tool in self._tools.values()]

    def names(self) -> tuple[str, ...]:
        return tuple(self._tools)

    async def execute(self, name: str, arguments: Mapping[str, Any]) -> object:
        tool = self._tools.get(name)
        if tool is None:
            raise ToolExecutionError(f"模型请求了未注册工具: {name}")
        if not isinstance(arguments, Mapping):
            raise ToolExecutionError(f"工具{name}的参数必须是JSON对象")
        return await tool.handler(arguments)


class TestCounter:
    """Stateful no-hardware tool used to verify the complete agent loop."""

    TOOL_NAME = "increment_test_counter"

    def __init__(self) -> None:
        self.value = 0

    def definition(self) -> ToolDefinition:
        return ToolDefinition(
            name=self.TOOL_NAME,
            description=(
                "Increment the local integration-test counter exactly once after the "
                "program sends the signal 'ok'. This is a safe test tool and does not "
                "control robot hardware."
            ),
            parameters={
                "type": "object",
                "properties": {
                    "amount": {
                        "type": "integer",
                        "enum": [1],
                        "description": "Always use 1 for this integration test.",
                    }
                },
                "required": ["amount"],
                "additionalProperties": False,
            },
            handler=self.increment,
        )

    async def increment(self, arguments: Mapping[str, Any]) -> dict[str, object]:
        amount = arguments.get("amount")
        if isinstance(amount, bool) or amount != 1:
            raise ToolExecutionError("increment_test_counter的amount必须为1")
        self.value += 1
        return {"status": "ok", "count": self.value}
