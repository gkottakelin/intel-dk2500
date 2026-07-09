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
    from project.src.jetarm_agent.tool_agent import (
        MAX_VISUAL_CLOSED_LOOP_ROUNDS,
        ToolCallingSession,
    )
    from project.src.jetarm_agent.tooling import ToolRegistry
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
    from src.jetarm_agent.tool_agent import (
        MAX_VISUAL_CLOSED_LOOP_ROUNDS,
        ToolCallingSession,
    )
    from src.jetarm_agent.tooling import ToolRegistry


PROJECT_ROOT = Path(__file__).resolve().parents[1]


class FakeModelClient:
    def __init__(self, responses):
        self.responses = list(responses)
        self.requests = []

    async def complete_with_tools(self, messages, tools, *, tool_choice="auto"):
        self.requests.append(
            {"messages": list(messages), "tools": list(tools), "tool_choice": tool_choice}
        )
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

    async def test_mcp_service_executes_one_sub_two_cm_command(self):
        result = await self.service.move("前1.9")

        self.assertEqual(result["status"], "ok")
        self.assertEqual(result["mcp"], "move_jetarm")
        self.assertEqual(result["speed_cm_s"], 1.5)
        self.assertEqual(result["requested_distance_cm"], 1.9)
        self.assertEqual(result["motion_command_count"], 1)
        self.assertNotIn("segments", result)

    async def test_mcp_service_exposes_grasp_helpers(self):
        release = await self.service.set_gripper_position(370)
        aligned = await self.service.pixel_align(104, 96, 100, 100)
        moved = await self.service.pixel_align(
            220,
            100,
            100,
            100,
            tolerance_px=10,
            step_duration_s=0.4,
            speed_saturation_px=120,
        )
        with self.assertRaisesRegex(RuntimeError, "get_rgb_camera_frame"):
            await self.service.control_to_target_pixel(100, 100)
        self.service._last_grasp_point_pixel = {
            "x": 100.0,
            "y": 100.0,
            "source": "test_frame",
        }
        target_aligned = await self.service.control_to_target_pixel(
            100,
            100,
            descend_when_aligned=False,
        )
        target_moved = await self.service.control_to_target_pixel(
            220,
            100,
            descend_when_aligned=False,
            step_duration_s=0.4,
            speed_saturation_px=120,
        )

        self.assertEqual(release["status"], "ok")
        self.assertEqual(release["mcp"], "set_jetarm_gripper_position")
        self.assertEqual(release["target_position"], 370)
        self.assertTrue(aligned["aligned"])
        self.assertEqual(aligned["mcp"], "move_jetarm_by_pixel_error")
        self.assertFalse(moved["aligned"])
        self.assertEqual(moved["direction"], "right")
        self.assertGreaterEqual(moved["speed_cm_s"], 0.7)
        self.assertLessEqual(moved["speed_cm_s"], 1.5)
        self.assertEqual(target_aligned["mcp"], "control_jetarm_to_target_pixel")
        self.assertEqual(target_aligned["agent_role"], "target_pixel_only")
        self.assertEqual(target_aligned["controller_decision"], "aligned_hold")
        self.assertEqual(target_aligned["grasp_point_pixel_source"], "test_frame")
        self.assertEqual(target_moved["controller_decision"], "horizontal_align")
        self.assertEqual(target_moved["direction"], "right")

    async def test_visual_closed_loop_limit_is_two_hundred_rounds(self):
        settings = AgentSettings.from_sources(
            PROJECT_ROOT / "config" / "ai_agent.json", environ={}
        )
        session = ToolCallingSession(settings, FakeModelClient([]), ToolRegistry())

        self.assertEqual(MAX_VISUAL_CLOSED_LOOP_ROUNDS, 200)
        self.assertEqual(session.max_rounds, 200)

    async def test_mcp_service_rejects_long_command_instead_of_splitting(self):
        with self.assertRaisesRegex(Exception, "单次"):
            await self.service.move("前5")

    async def test_markdown_workflow_is_loaded(self):
        instructions = self.service.initial_instructions()
        self.assertTrue(DEFAULT_WORKFLOW_PATH.is_file())
        self.assertIn("get_rgb_camera_frame", instructions)
        self.assertIn("control_jetarm_to_target_pixel", instructions)
        self.assertIn("Agent 只负责解析用户命令", instructions)
        self.assertIn("camera.grasp_point_pixel", instructions)
        self.assertIn("set_jetarm_gripper_position(position=370)", instructions)
        self.assertIn("每下降 `2 cm`", instructions)
        self.assertIn("高度 `>15 cm`", instructions)
        self.assertIn("status=error", instructions)

    async def test_model_mcp_controller_model_roundtrip(self):
        service = self.service

        class FakeMCPSession:
            async def list_tools(self):
                return SimpleNamespace(
                    tools=[
                        SimpleNamespace(
                            name="get_rgb_camera_frame",
                            description="capture RGB",
                            inputSchema={"type": "object", "properties": {}},
                        ),
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
                if name == "get_rgb_camera_frame":
                    return SimpleNamespace(
                        structuredContent={
                            "status": "ok",
                            "mcp": "get_rgb_camera_frame",
                        },
                        content=[
                            SimpleNamespace(data="anBlZw==", mimeType="image/jpeg")
                        ],
                        isError=False,
                    )
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
                                {"command": "前1.9", "speed_cm_s": 1.5},
                                ensure_ascii=False,
                            ),
                        ),
                    ),
                ),
                ToolModelResponse(
                    content="",
                    tool_calls=(
                        FunctionToolCall(
                            call_id="mcp-2",
                            name="move_jetarm",
                            arguments=json.dumps(
                                {"command": "前1.9", "speed_cm_s": 1.5},
                                ensure_ascii=False,
                            ),
                        ),
                    ),
                ),
                ToolModelResponse(
                    content="",
                    tool_calls=(
                        FunctionToolCall(
                            call_id="mcp-3",
                            name="move_jetarm",
                            arguments=json.dumps(
                                {"command": "前1.2", "speed_cm_s": 1.5},
                                ensure_ascii=False,
                            ),
                        ),
                    ),
                ),
                ToolModelResponse(
                    content="已通过三条MCP命令完成向前5厘米。",
                    tool_calls=(),
                ),
            ]
        )
        session = ToolCallingSession(settings, model, registry)

        result = await session.ask(
            "向前5厘米",
            require_any_tool=True,
            required_tool_name="move_jetarm",
            preselected_tool_name="get_rgb_camera_frame",
            preselected_tool_arguments={},
        )

        self.assertEqual(result.text, "已通过三条MCP命令完成向前5厘米。")
        self.assertEqual(
            [
                call.arguments["command"]
                for call in result.tool_calls
                if call.name == "move_jetarm"
            ],
            ["前1.9", "前1.9", "前1.2"],
        )
        self.assertEqual(
            [call.name for call in result.tool_calls],
            [
                "get_rgb_camera_frame",
                "move_jetarm",
                "get_rgb_camera_frame",
                "move_jetarm",
                "get_rgb_camera_frame",
                "move_jetarm",
                "get_rgb_camera_frame",
            ],
        )
        self.assertTrue(all(call.result["status"] == "ok" for call in result.tool_calls))
        self.assertTrue(
            all(
                any(
                    isinstance(message.get("content"), list)
                    and any(
                        isinstance(part, dict) and part.get("type") == "image_url"
                        for part in message["content"]
                    )
                    for message in request["messages"]
                )
                for request in model.requests
            )
        )

    async def test_agent_runtime_rejects_motion_without_rgb_visible_to_model(self):
        class FakeMCPSession:
            def __init__(self):
                self.motion_calls = 0

            async def list_tools(self):
                return SimpleNamespace(
                    tools=[
                        SimpleNamespace(
                            name="get_rgb_camera_frame",
                            description="capture RGB",
                            inputSchema={"type": "object", "properties": {}},
                        ),
                        SimpleNamespace(
                            name="move_jetarm",
                            description="move",
                            inputSchema={
                                "type": "object",
                                "properties": {"command": {"type": "string"}},
                                "required": ["command"],
                            },
                        ),
                    ]
                )

            async def call_tool(self, name, arguments):
                if name == "move_jetarm":
                    self.motion_calls += 1
                return SimpleNamespace(
                    structuredContent={"status": "ok"},
                    content=[],
                    isError=False,
                )

        mcp_session = FakeMCPSession()
        bridge = MCPRobotBridge()
        bridge.session = mcp_session
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
                            call_id="unsafe-move",
                            name="move_jetarm",
                            arguments=json.dumps(
                                {"command": "下1.9"}, ensure_ascii=False
                            ),
                        ),
                    ),
                ),
                ToolModelResponse(
                    content="未取得最新RGB图像，未执行移动。",
                    tool_calls=(),
                ),
            ]
        )
        session = ToolCallingSession(settings, model, registry)

        result = await session.ask(
            "向下移动1.9厘米",
            require_any_tool=True,
            required_tool_name="move_jetarm",
        )

        self.assertEqual(mcp_session.motion_calls, 0)
        self.assertEqual(result.tool_calls[0].result["status"], "error")
        self.assertIn("没有最新RGB图像", result.tool_calls[0].result["error"])


if __name__ == "__main__":
    unittest.main()
