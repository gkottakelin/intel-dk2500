"""stdio MCP client bridge used by the Kimi/OpenAI-compatible agent."""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path
from typing import Any

from .device_config import DEFAULT_DEVICE_CONFIG_PATH, PROJECT_ROOT
from .tooling import ToolDefinition, ToolRegistry


class MCPClientError(RuntimeError):
    """Raised when the local JetArm MCP server cannot be used."""


class MCPRobotBridge:
    def __init__(
        self,
        *,
        device_config: str | Path = DEFAULT_DEVICE_CONFIG_PATH,
        arm_mode: str | None = None,
        arm_port: str | None = None,
        arm_config: str | Path | None = None,
        max_distance_cm: float = 10.0,
    ) -> None:
        self.device_config = Path(device_config)
        self.arm_mode = arm_mode
        self.arm_port = arm_port
        self.arm_config = Path(arm_config) if arm_config is not None else None
        self.max_distance_cm = max_distance_cm
        self.session: Any = None
        self._stdio_context: Any = None
        self._session_context: Any = None

    async def __aenter__(self) -> "MCPRobotBridge":
        try:
            from mcp import ClientSession, StdioServerParameters
            from mcp.client.stdio import stdio_client
        except ImportError as exc:
            raise MCPClientError(
                "缺少MCP SDK，请执行: python -m pip install -r requirements-ai.txt"
            ) from exc

        args = [
            "-m",
            "src.jetarm_agent.mcp_server",
            "--device-config",
            str(self.device_config),
        ]
        if self.arm_mode is not None:
            args.extend(["--arm-mode", self.arm_mode])
        if self.arm_port is not None:
            args.extend(["--arm-port", self.arm_port])
        if self.arm_config is not None:
            args.extend(["--arm-config", str(self.arm_config)])
        args.extend(["--arm-max-distance-cm", str(self.max_distance_cm)])
        env = dict(os.environ)
        env["PYTHONUNBUFFERED"] = "1"
        parameters = StdioServerParameters(
            command=sys.executable,
            args=args,
            cwd=str(PROJECT_ROOT),
            env=env,
        )
        try:
            self._stdio_context = stdio_client(parameters)
            read_stream, write_stream = await self._stdio_context.__aenter__()
            self._session_context = ClientSession(read_stream, write_stream)
            self.session = await self._session_context.__aenter__()
            await self.session.initialize()
        except Exception as exc:
            await self._close_contexts()
            raise MCPClientError(f"无法启动本地JetArm MCP服务器: {exc}") from exc
        return self

    async def __aexit__(self, exc_type: Any, exc: Any, traceback: Any) -> None:
        await self._close_contexts(exc_type, exc, traceback)

    async def _close_contexts(
        self, exc_type: Any = None, exc: Any = None, traceback: Any = None
    ) -> None:
        if self._session_context is not None:
            await self._session_context.__aexit__(exc_type, exc, traceback)
            self._session_context = None
            self.session = None
        if self._stdio_context is not None:
            await self._stdio_context.__aexit__(exc_type, exc, traceback)
            self._stdio_context = None

    async def registry(self) -> ToolRegistry:
        if self.session is None:
            raise MCPClientError("MCP客户端尚未连接")
        response = await self.session.list_tools()
        definitions: list[ToolDefinition] = []
        for tool in response.tools:
            name = str(tool.name)
            description = str(tool.description or "")
            parameters = dict(tool.inputSchema or {"type": "object", "properties": {}})

            async def handler(arguments: Any, tool_name: str = name) -> object:
                return await self.call_tool(tool_name, dict(arguments))

            definitions.append(
                ToolDefinition(
                    name=name,
                    description=description,
                    parameters=parameters,
                    handler=handler,
                )
            )
        return ToolRegistry(definitions)

    async def call_tool(self, name: str, arguments: dict[str, Any]) -> object:
        if self.session is None:
            raise MCPClientError("MCP客户端尚未连接")
        result = await self.session.call_tool(name, arguments)
        if getattr(result, "isError", False):
            texts = [
                str(item.text)
                for item in getattr(result, "content", [])
                if getattr(item, "text", None) is not None
            ]
            return {"status": "error", "error": "\n".join(texts) or "MCP工具失败"}
        structured = getattr(result, "structuredContent", None)
        if structured is not None:
            return structured

        texts = [
            str(item.text)
            for item in getattr(result, "content", [])
            if getattr(item, "text", None) is not None
        ]
        if len(texts) == 1:
            try:
                return json.loads(texts[0])
            except json.JSONDecodeError:
                return {"status": "ok", "message": texts[0]}
        return {"status": "ok", "content": texts}
