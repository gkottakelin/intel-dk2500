import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

try:
    from project.src.jetarm_agent.config import AgentSettings
    from project.src.jetarm_agent.device_config import (
        RuntimeDeviceConfig,
        discover_rgb_cameras,
        validate_device_interfaces,
    )
    from project.src.jetarm_agent.mcp_client import MCPRobotBridge
    from project.src.jetarm_agent.mcp_server import (
        DEFAULT_WORKFLOW_PATH,
        JetArmMCPService,
    )
    from project.src.jetarm_agent.openai_compatible import (
        FunctionToolCall,
        ToolModelResponse,
    )
    from project.src.jetarm_agent.tool_agent import ToolCallingSession
except ModuleNotFoundError:
    from src.jetarm_agent.config import AgentSettings
    from src.jetarm_agent.device_config import (
        RuntimeDeviceConfig,
        discover_rgb_cameras,
        validate_device_interfaces,
    )
    from src.jetarm_agent.mcp_client import MCPRobotBridge
    from src.jetarm_agent.mcp_server import DEFAULT_WORKFLOW_PATH, JetArmMCPService
    from src.jetarm_agent.openai_compatible import FunctionToolCall, ToolModelResponse
    from src.jetarm_agent.tool_agent import ToolCallingSession


PROJECT_ROOT = Path(__file__).resolve().parents[1]


class FakeModelClient:
    def __init__(self, responses):
        self.responses = list(responses)

    async def complete_with_tools(self, messages, tools, *, tool_choice="auto"):
        return self.responses.pop(0)


class DeviceConfigTest(unittest.TestCase):
    def test_device_config_roundtrip_and_orbbec_sdk_discovery(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config_path = root / "devices.json"
            camera = SimpleNamespace(
                name="SV1301S_U3",
                serial_number="4-1.2-11",
                uid="4-1.2-11",
                selection_key="4-1.2-11",
                vid=0x2BC5,
                pid=0x0614,
                label="SV1301S_U3 | SN: 4-1.2-11 | 2bc5:0614",
            )

            config = RuntimeDeviceConfig(
                arm_mode="dry-run",
                rgb_camera=camera.selection_key,
                rgb_camera_name=camera.name,
            )
            config.save(config_path)
            loaded = RuntimeDeviceConfig.load(config_path)
            cameras = discover_rgb_cameras(lambda: [camera])

            self.assertEqual(loaded, config)
            self.assertEqual(cameras[0].selection_key, "4-1.2-11")
            self.assertEqual(cameras[0].name, "SV1301S_U3")
            self.assertEqual(
                validate_device_interfaces(
                    config, camera_discover=lambda: [camera]
                ),
                [],
            )


class MCPServiceTest(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.service = JetArmMCPService(
            RuntimeDeviceConfig(
                arm_mode="dry-run",
                arm_terminal_config=str(
                    PROJECT_ROOT
                    / "ubuntu22_04_operation_terminal"
                    / "config"
                    / "terminal.json"
                ),
            )
        )

    async def asyncTearDown(self):
        self.service.close()

    async def test_mcp_service_executes_front_five_as_two_segments(self):
        result = await self.service.move("前5")

        self.assertEqual(result["status"], "ok")
        self.assertEqual(result["mcp"], "move_jetarm")
        self.assertEqual(result["speed_cm_s"], 1.5)
        self.assertEqual(result["segment_count"], 2)
        self.assertEqual(
            [segment["distance_cm"] for segment in result["segments"]],
            [3, 2],
        )

    async def test_markdown_workflow_is_loaded(self):
        instructions = self.service.initial_instructions()
        self.assertTrue(DEFAULT_WORKFLOW_PATH.is_file())
        self.assertIn("前3 → 前2", instructions)
        self.assertIn("status=ok", instructions)

    async def test_model_mcp_controller_model_roundtrip(self):
        service = self.service

        class FakeMCPSession:
            async def list_tools(self):
                return SimpleNamespace(
                    tools=[
                        SimpleNamespace(
                            name="move_jetarm",
                            description="move",
                            inputSchema={
                                "type": "object",
                                "properties": {
                                    "command": {"type": "string"},
                                    "speed_cm_s": {"type": "number"},
                                },
                                "required": ["command"],
                            },
                        )
                    ]
                )

            async def call_tool(self, name, arguments):
                result = await service.move(
                    arguments["command"], arguments.get("speed_cm_s", 1.5)
                )
                return SimpleNamespace(structuredContent=result, content=[], isError=False)

        bridge = MCPRobotBridge()
        bridge.session = FakeMCPSession()
        registry = await bridge.registry()
        settings = AgentSettings.from_sources(
            PROJECT_ROOT / "config" / "ai_agent.json", environ={}
        )
        model = FakeModelClient(
            [
                ToolModelResponse(
                    content="",
                    tool_calls=(
                        FunctionToolCall(
                            call_id="mcp-1",
                            name="move_jetarm",
                            arguments=json.dumps(
                                {"command": "前5", "speed_cm_s": 1.5},
                                ensure_ascii=False,
                            ),
                        ),
                    ),
                ),
                ToolModelResponse(content="已完成向前5厘米。", tool_calls=()),
            ]
        )
        session = ToolCallingSession(settings, model, registry)

        result = await session.ask(
            "向前5厘米",
            require_any_tool=True,
            required_tool_name="move_jetarm",
        )

        self.assertEqual(result.text, "已完成向前5厘米。")
        self.assertEqual(result.tool_calls[0].result["status"], "ok")
        self.assertEqual(result.tool_calls[0].result["segment_count"], 2)


if __name__ == "__main__":
    unittest.main()
