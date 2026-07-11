import base64
import json
import unittest

import cv2
import numpy as np

try:
    from project.src.jetarm_agent.config import AgentSettings
    from project.src.jetarm_agent.openai_compatible import (
        FunctionToolCall,
        ToolModelResponse,
    )
    from project.src.jetarm_agent.tool_agent import ToolCallingSession
    from project.src.jetarm_agent.tooling import (
        ToolDefinition,
        ToolExecutionPayload,
        ToolImage,
        ToolRegistry,
    )
    from project.src.jetarm_agent.visual_tiles import VisualTileLocator
except ModuleNotFoundError:
    from src.jetarm_agent.config import AgentSettings
    from src.jetarm_agent.openai_compatible import FunctionToolCall, ToolModelResponse
    from src.jetarm_agent.tool_agent import ToolCallingSession
    from src.jetarm_agent.tooling import (
        ToolDefinition,
        ToolExecutionPayload,
        ToolImage,
        ToolRegistry,
    )
    from src.jetarm_agent.visual_tiles import VisualTileLocator


def jpeg_image(width: int = 640, height: int = 480) -> ToolImage:
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    ok, encoded = cv2.imencode(".jpg", frame)
    if not ok:
        raise AssertionError("test JPEG encoding failed")
    return ToolImage(
        data=base64.b64encode(encoded.tobytes()).decode("ascii"),
        mime_type="image/jpeg",
    )


class VisualTileLocatorTest(unittest.IsolatedAsyncioTestCase):
    async def test_four_3x3_levels_map_back_to_original_pixel(self):
        locator = VisualTileLocator()
        locator.set_frame(jpeg_image())

        selections = [(0, 1), (1, 1), (2, 1), (0, 2)]
        payload = None
        for row, column in selections:
            payload = await locator.zoom({"row": row, "column": column})

        self.assertIsNotNone(payload)
        self.assertTrue(locator.ready)
        summary = locator.summary()
        self.assertEqual(summary["depth"], 4)
        bounds = summary["original_bounds_inclusive"]
        self.assertGreaterEqual(bounds["x_min"], 0)
        self.assertLess(bounds["x_max"], 640)
        self.assertGreaterEqual(bounds["y_min"], 0)
        self.assertLess(bounds["y_max"], 480)
        self.assertLessEqual(summary["maximum_quantization_error_px"]["x"], 4)
        self.assertLessEqual(summary["maximum_quantization_error_px"]["y"], 3)
        target_x, target_y = locator.target_pixel()
        self.assertGreaterEqual(target_x, bounds["x_min"])
        self.assertLessEqual(target_x, bounds["x_max"])
        self.assertGreaterEqual(target_y, bounds["y_min"])
        self.assertLessEqual(target_y, bounds["y_max"])
        self.assertEqual(len(payload.images), 1)

    async def test_new_frame_resets_previous_tile_path(self):
        locator = VisualTileLocator(required_depth=2)
        locator.set_frame(jpeg_image())
        await locator.zoom({"row": 1, "column": 2})
        self.assertEqual(locator.depth, 1)

        locator.set_frame(jpeg_image())

        self.assertEqual(locator.depth, 0)
        self.assertFalse(locator.ready)
        self.assertEqual(
            locator.summary()["original_bounds_inclusive"],
            {"x_min": 0, "y_min": 0, "x_max": 639, "y_max": 479},
        )

    async def test_session_forces_final_tile_center_into_v2_control_call(self):
        frame = jpeg_image()
        received_control_arguments = []

        async def camera(_arguments):
            return ToolExecutionPayload(
                value={
                    "status": "ok",
                    "camera": {"width": 640, "height": 480},
                },
                images=(frame,),
            )

        async def control(arguments):
            received_control_arguments.append(dict(arguments))
            return {"status": "error", "error": "test stops before hardware motion"}

        registry = ToolRegistry(
            [
                ToolDefinition(
                    name="get_rgb_camera_frame",
                    description="test camera",
                    parameters={"type": "object", "properties": {}},
                    handler=camera,
                ),
                ToolDefinition(
                    name="control_jetarm_to_target_pixel",
                    description="test V2 controller",
                    parameters={
                        "type": "object",
                        "properties": {
                            "target_x": {"type": "number"},
                            "target_y": {"type": "number"},
                        },
                        "required": ["target_x", "target_y"],
                    },
                    handler=control,
                ),
            ]
        )

        def call(index, name, arguments):
            return ToolModelResponse(
                content="",
                tool_calls=(
                    FunctionToolCall(
                        call_id=f"call-{index}",
                        name=name,
                        arguments=json.dumps(arguments),
                    ),
                ),
            )

        responses = [
            call(0, "get_rgb_camera_frame", {}),
            call(1, "zoom_rgb_target_tile", {"row": 0, "column": 1}),
            call(2, "zoom_rgb_target_tile", {"row": 1, "column": 1}),
            call(3, "zoom_rgb_target_tile", {"row": 2, "column": 1}),
            call(4, "zoom_rgb_target_tile", {"row": 0, "column": 2}),
            call(
                5,
                "control_jetarm_to_target_pixel",
                {"target_x": 1, "target_y": 2},
            ),
            ToolModelResponse(content="测试结束", tool_calls=()),
        ]

        class FakeClient:
            def __init__(self):
                self.requests = []

            async def complete_with_tools(self, messages, tools, *, tool_choice="auto"):
                self.requests.append((messages, tools, tool_choice))
                return responses.pop(0)

        settings = AgentSettings(
            provider="openai_compatible",
            base_url="https://example.invalid/v1",
            model="test",
            api_key_env="TEST_KEY",
            timeout_s=1,
            extra_body={},
            temperature=None,
            max_tokens=100,
            max_history_messages=40,
            system_prompt="test",
        )
        session = ToolCallingSession(settings, FakeClient(), registry)

        result = await session.ask("抓取红色物块")

        self.assertEqual(result.text, "测试结束")
        self.assertEqual(len(received_control_arguments), 1)
        actual = received_control_arguments[0]
        final_tile_result = result.tool_calls[4].result
        expected = final_tile_result["visual_tile_localization"][
            "estimated_target_pixel"
        ]
        self.assertEqual(actual["target_x"], expected["x"])
        self.assertEqual(actual["target_y"], expected["y"])
        self.assertNotEqual(actual["target_x"], 1)
        self.assertNotEqual(actual["target_y"], 2)


if __name__ == "__main__":
    unittest.main()
