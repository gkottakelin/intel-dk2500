import unittest
from types import SimpleNamespace

import numpy as np

try:
    from project.src.jetarm_agent.config import AgentSettings
    from project.src.jetarm_agent.device_config import RuntimeDeviceConfig
    from project.src.jetarm_agent.mcp_client import MCPRobotBridge
    from project.src.jetarm_agent.mcp_server import JetArmMCPService, create_mcp_server
    from project.src.jetarm_agent.openai_compatible import ToolModelResponse
    from project.src.jetarm_agent.rgb_camera import RGBJpegFrame, capture_rgb_jpeg
    from project.src.jetarm_agent.tool_agent import ToolCallingSession
except ModuleNotFoundError:
    from src.jetarm_agent.config import AgentSettings
    from src.jetarm_agent.device_config import RuntimeDeviceConfig
    from src.jetarm_agent.mcp_client import MCPRobotBridge
    from src.jetarm_agent.mcp_server import JetArmMCPService, create_mcp_server
    from src.jetarm_agent.openai_compatible import ToolModelResponse
    from src.jetarm_agent.rgb_camera import RGBJpegFrame, capture_rgb_jpeg
    from src.jetarm_agent.tool_agent import ToolCallingSession


class FakeCapture:
    def __init__(self, frame):
        self.frame = frame
        self.released = False

    def isOpened(self):
        return True

    def set(self, _key, _value):
        return True

    def read(self):
        return True, self.frame

    def release(self):
        self.released = True


class FakeCV2:
    CAP_V4L2 = 200
    CAP_PROP_FRAME_WIDTH = 3
    CAP_PROP_FRAME_HEIGHT = 4
    CAP_PROP_BUFFERSIZE = 38
    IMWRITE_JPEG_QUALITY = 1

    def __init__(self):
        self.capture = FakeCapture(np.zeros((24, 32, 3), dtype=np.uint8))

    def VideoCapture(self, device, backend):
        self.opened = (device, backend)
        return self.capture

    @staticmethod
    def imencode(extension, frame, options):
        assert extension == ".jpg"
        assert frame.shape == (24, 32, 3)
        assert options == [1, 85]
        return True, np.frombuffer(b"jpeg-bytes", dtype=np.uint8)


class RGBCaptureTest(unittest.TestCase):
    def test_v4l2_frame_is_encoded_and_camera_is_released(self):
        fake_cv2 = FakeCV2()

        frame = capture_rgb_jpeg("/dev/video2", cv2_module=fake_cv2)

        self.assertEqual(fake_cv2.opened, ("/dev/video2", fake_cv2.CAP_V4L2))
        self.assertEqual(frame.data, b"jpeg-bytes")
        self.assertEqual((frame.width, frame.height), (32, 24))
        self.assertTrue(fake_cv2.capture.released)


class CameraMCPAgentTest(unittest.IsolatedAsyncioTestCase):
    async def test_service_uses_configured_rgb_camera(self):
        seen = []

        def fake_capture(device):
            seen.append(device)
            return RGBJpegFrame(b"jpeg", 640, 480)

        service = JetArmMCPService(
            RuntimeDeviceConfig(arm_mode="off", rgb_camera="/dev/video4"),
            camera_capture=fake_capture,
        )

        frame = await service.capture_rgb()

        self.assertEqual(seen, ["/dev/video4"])
        self.assertEqual(frame.data, b"jpeg")

    async def test_fastmcp_returns_json_and_jpeg_content_blocks(self):
        try:
            from mcp.types import ImageContent, TextContent
        except ImportError:
            self.skipTest("MCP SDK is not installed in this test environment")

        service = JetArmMCPService(
            RuntimeDeviceConfig(arm_mode="off", rgb_camera="/dev/video4"),
            camera_capture=lambda _device: RGBJpegFrame(b"jpeg", 640, 480),
        )
        server = create_mcp_server(service)

        content = await server.call_tool("get_rgb_camera_frame", {})

        self.assertTrue(any(isinstance(item, TextContent) for item in content))
        image = next(item for item in content if isinstance(item, ImageContent))
        self.assertEqual(image.mimeType, "image/jpeg")
        self.assertEqual(image.data, "anBlZw==")

    async def test_mcp_image_is_forwarded_to_the_next_kimi_request(self):
        class FakeMCPSession:
            async def list_tools(self):
                return SimpleNamespace(
                    tools=[
                        SimpleNamespace(
                            name="get_rgb_camera_frame",
                            description="capture RGB",
                            inputSchema={"type": "object", "properties": {}},
                        )
                    ]
                )

            async def call_tool(self, name, arguments):
                self.called = (name, arguments)
                return SimpleNamespace(
                    structuredContent={"status": "ok", "camera": {"width": 640}},
                    content=[
                        SimpleNamespace(data="anBlZw==", mimeType="image/jpeg")
                    ],
                    isError=False,
                )

        class RecordingModel:
            def __init__(self):
                self.requests = []
                self.responses = [
                    ToolModelResponse(content="画面中有一个机械臂。", tool_calls=()),
                ]

            async def complete_with_tools(self, messages, tools, *, tool_choice="auto"):
                self.requests.append(
                    {"messages": list(messages), "tools": list(tools), "tool_choice": tool_choice}
                )
                return self.responses.pop(0)

        bridge = MCPRobotBridge()
        bridge.session = FakeMCPSession()
        registry = await bridge.registry()
        settings = AgentSettings(
            provider="openai_compatible",
            base_url="https://api.moonshot.cn/v1",
            model="kimi-k2.6",
            api_key_env="MOONSHOT_API_KEY",
            timeout_s=60,
            extra_body={},
            temperature=None,
            max_tokens=1024,
            max_history_messages=20,
            system_prompt="test",
        )
        model = RecordingModel()
        session = ToolCallingSession(settings, model, registry)

        result = await session.ask(
            "查看相机画面",
            first_tool_choice="none",
            allow_additional_tools=False,
            require_any_tool=True,
            required_tool_name="get_rgb_camera_frame",
            preselected_tool_arguments={},
        )

        self.assertEqual(result.text, "画面中有一个机械臂。")
        self.assertEqual(len(result.tool_calls[0].images), 1)
        request = model.requests[0]
        self.assertEqual(request["tool_choice"], "none")
        messages = request["messages"]
        self.assertEqual(messages[-2]["role"], "tool")
        image_message = messages[-1]
        self.assertEqual(image_message["role"], "user")
        self.assertEqual(image_message["content"][1]["type"], "image_url")
        self.assertEqual(
            image_message["content"][1]["image_url"]["url"],
            "data:image/jpeg;base64,anBlZw==",
        )


if __name__ == "__main__":
    unittest.main()
