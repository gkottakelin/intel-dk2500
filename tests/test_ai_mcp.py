import json
import io
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import AsyncMock

try:
    from project.src.jetarm_agent.cli import (
        _agent_grasp_point_from_args,
        _print_agent_grasp_step_records,
        _send_with_tools,
        build_parser,
    )
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
    from project.src.jetarm_agent.tooling import ToolImage, ToolRegistry
except ModuleNotFoundError:
    from src.jetarm_agent.cli import (
        _agent_grasp_point_from_args,
        _print_agent_grasp_step_records,
        _send_with_tools,
        build_parser,
    )
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
    from src.jetarm_agent.tooling import ToolImage, ToolRegistry


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
                grasp_point_x=320.0,
                grasp_point_y=147.0,
            )
            config.save(config_path)
            loaded = RuntimeDeviceConfig.load(config_path)
            cameras = discover_rgb_cameras(lambda: [camera])

            self.assertEqual(loaded, config)
            self.assertEqual(
                (loaded.grasp_point_x, loaded.grasp_point_y), (320.0, 147.0)
            )
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

    async def test_mcp_service_executes_one_long_command(self):
        result = await self.service.move("前5")

        self.assertEqual(result["status"], "ok")
        self.assertEqual(result["mcp"], "move_jetarm")
        self.assertEqual(result["speed_cm_s"], 1.5)
        self.assertEqual(result["requested_distance_cm"], 5.0)
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
        with self.assertRaisesRegex(RuntimeError, "输入抓取点像素"):
            await self.service.control_to_target_pixel(100, 100)
        configured = self.service.set_grasp_point_pixel(100, 100)
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
        self.assertEqual(
            target_aligned["grasp_point_pixel_source"],
            "user_input_before_agent_grasp",
        )
        self.assertEqual(target_moved["controller_decision"], "horizontal_align")
        self.assertEqual(target_moved["direction"], "right")
        self.assertEqual(configured["grasp_point_pixel"]["x"], 100.0)
        self.assertEqual(target_moved["camera_vector_version"], "v2")
        self.assertFalse(target_moved["progress_check_enabled"])
        self.assertIsNotNone(
            target_aligned["grasp_step_record"][
                "camera_grasp_vertical_angle_deg"
            ]
        )
        self.assertEqual(
            target_moved["grasp_step_record_format"],
            [
                "target_pixel",
                "original_grasp_point_xyz_cm",
                "motion_plan",
                "expected_grasp_point_xyz_cm",
                "actual_grasp_point_xyz_cm",
                "camera_grasp_vertical_angle_deg",
            ],
        )

    async def test_agent_controller_uses_manual_v2_with_progress_check_disabled(self):
        controller = self.service.controller()

        self.assertEqual(controller.config.camera_vector_version, "v2")
        self.assertFalse(controller.config.manual_progress_check_enabled)
        self.assertEqual(controller.config.max_distance_cm, 100.0)
        self.assertTrue(controller.config.allow_extended_distance)

    async def test_user_grasp_point_is_fixed_across_frames(self):
        self.assertIsNone(self.service.grasp_point_pixel_for_frame(640, 480))

        self.service.set_grasp_point_pixel(320, 147)

        first = self.service.grasp_point_pixel_for_frame(640, 480)
        second = self.service.grasp_point_pixel_for_frame(640, 480)
        self.assertEqual(first, second)
        self.assertEqual(first["source"], "user_input_before_agent_grasp")
        with self.assertRaisesRegex(RuntimeError, "超出当前RGB图像范围"):
            self.service.grasp_point_pixel_for_frame(100, 100)

    async def test_service_loads_grasp_point_from_interface_config(self):
        service = JetArmMCPService(
            RuntimeDeviceConfig(
                arm_mode="dry-run",
                arm_terminal_config=str(
                    PROJECT_ROOT
                    / "ubuntu22_04_operation_terminal"
                    / "config"
                    / "terminal.json"
                ),
                grasp_point_x=320.0,
                grasp_point_y=147.0,
            )
        )
        self.addAsyncCleanup(self._close_service, service)

        point = service.grasp_point_pixel_for_frame(640, 480)

        self.assertEqual(
            point,
            {"x": 320.0, "y": 147.0, "source": "device_config"},
        )

    async def _close_service(self, service):
        service.close()

    async def test_agent_bottom_origin_y_is_converted_before_v2_motion(self):
        self.service.set_grasp_point_pixel(320, 147)
        self.service._last_rgb_frame_size = (640, 480)

        result = await self.service.control_to_target_pixel(
            320,
            410,
            target_vertical_relation="above",
        )

        self.assertEqual(result["direction"], "forward")
        validation = result["target_coordinate_validation"]
        self.assertEqual(
            validation["normalization"],
            "bottom_origin_y_up_converted_to_top_left_y_down",
        )
        self.assertEqual(validation["received_target_y"], 410.0)
        self.assertEqual(validation["normalized_target_y"], 70.0)
        self.assertEqual(result["grasp_step_record"]["target_pixel"]["y"], 70.0)

    async def test_agent_top_left_y_is_kept_without_conversion(self):
        self.service.set_grasp_point_pixel(320, 147)
        self.service._last_rgb_frame_size = (640, 480)

        result = await self.service.control_to_target_pixel(
            320,
            70,
            target_vertical_relation="above",
        )

        self.assertEqual(result["direction"], "forward")
        self.assertEqual(
            result["target_coordinate_validation"]["normalization"],
            "none_top_left_y_down_confirmed",
        )

    async def test_agent_only_needs_top_left_target_xy(self):
        self.service.set_grasp_point_pixel(320, 147)
        self.service._last_rgb_frame_size = (640, 480)

        result = await self.service.control_to_target_pixel(320, 70)

        self.assertEqual(result["direction"], "forward")
        self.assertEqual(
            result["target_coordinate_validation"]["normalization"],
            "compatibility_no_relation_check",
        )

    async def test_agent_target_pixel_outside_latest_original_image_is_rejected(self):
        self.service.set_grasp_point_pixel(320, 147)
        self.service._last_rgb_frame_size = (640, 480)

        with self.assertRaisesRegex(RuntimeError, "超出最新RGB原图范围"):
            await self.service.control_to_target_pixel(640, 100)

    async def test_unresolvable_agent_y_relation_is_rejected_before_motion(self):
        self.service.set_grasp_point_pixel(320, 147)
        self.service._last_rgb_frame_size = (640, 480)

        with self.assertRaisesRegex(RuntimeError, "无法按原图高度安全转换"):
            await self.service.control_to_target_pixel(
                320,
                300,
                target_vertical_relation="same_y",
            )

    async def test_agent_receives_high_detail_top_left_coordinate_contract(self):
        image_part = ToolImage("anBlZw==", "image/jpeg").openai_content_part()
        instruction = ToolCallingSession._rgb_coordinate_instruction(
            {
                "camera": {
                    "width": 640,
                    "height": 480,
                    "grasp_point_pixel": {"x": 320, "y": 147},
                }
            }
        )

        self.assertEqual(image_part["image_url"]["detail"], "high")
        self.assertIn("左上角(0,0)", instruction)
        self.assertIn("Y向下增大", instruction)
        self.assertIn("320", instruction)
        self.assertIn("右下角=(639,479)", instruction)
        self.assertIn("只提交目标物品中心的target_x/target_y", instruction)

    async def test_final_alignment_automatically_descends_grips_and_homes(self):
        sequence = []
        fake = SimpleNamespace(
            set_gripper_position=AsyncMock(return_value={"status": "ok"}),
            control_to_target_pixel=AsyncMock(
                side_effect=[
                    {
                        "status": "ok",
                        "controller_decision": "aligned_hold",
                        "motion_command_count": 0,
                        "grasp_point_xyz_before_cm": {"x": 0, "y": -20, "z": 3},
                        "grasp_point_xyz_after_cm": {"x": 0, "y": -20, "z": 3},
                    },
                    {
                        "status": "ok",
                        "controller_decision": "aligned_hold",
                        "motion_command_count": 0,
                        "grasp_point_xyz_before_cm": {"x": 0, "y": -20, "z": 3},
                        "grasp_point_xyz_after_cm": {"x": 0, "y": -20, "z": 3},
                    },
                ]
            ),
            descend_to_height=AsyncMock(
                return_value={
                    "status": "ok",
                    "height_after_cm": 1.0,
                    "target_tolerance_cm": 0.4,
                    "motion_steps": [
                        {
                            "direction": "down",
                            "requested_distance_cm": 2.0,
                            "original_grasp_point_xyz_cm": {"x": 0, "y": -20, "z": 3},
                            "expected_grasp_point_xyz_cm": {"x": 0, "y": -20, "z": 1},
                            "actual_grasp_point_xyz_cm": {"x": 0, "y": -20, "z": 1},
                            "motion_plan": {"accepted": True},
                            "camera_grasp_vertical_angle_deg": 12.5,
                        }
                    ],
                }
            ),
            control_gripper=AsyncMock(
                side_effect=lambda _action: (
                    sequence.append("j6_stable"),
                    {
                        "status": "ok",
                        "action": "grip_lock",
                        "j6_stability": {"status": "ok", "stable": True},
                    },
                )[1]
            ),
            go_home=AsyncMock(
                side_effect=lambda: (
                    sequence.append("home"),
                    {"status": "ok"},
                )[1]
            ),
            close=lambda: None,
        )
        self.service._controller = fake
        self.service.set_grasp_point_pixel(320, 147)

        first = await self.service.control_to_target_pixel(330, 155)
        final = await self.service.control_to_target_pixel(328, 150)

        self.assertTrue(first["final_alignment_phase"])
        self.assertTrue(first["requires_new_target_pixel"])
        self.assertEqual(final["controller_decision"], "grasp_complete")
        self.assertFalse(final["grasp_completed"])
        self.assertEqual(
            final["grasp_completion_status"], "awaiting_visual_verification"
        )
        with self.assertRaisesRegex(RuntimeError, "confirm_jetarm_grasp_result"):
            await self.service.control_to_target_pixel(328, 150)
        confirmed = self.service.confirm_grasp_result(True)
        self.assertTrue(confirmed["grasp_completed"])
        fake.descend_to_height.assert_awaited_once_with(1.0)
        fake.control_gripper.assert_awaited_once_with("grip_lock")
        fake.go_home.assert_awaited_once()
        self.assertEqual(sequence, ["j6_stable", "home"])
        record = final["new_grasp_step_records"][-1]
        self.assertEqual(record["target_pixel"], {"x": 328.0, "y": 150.0})
        self.assertEqual(
            record["motion_plan"], {"direction": "down", "distance_cm": 2.0}
        )
        self.assertEqual(record["camera_grasp_vertical_angle_deg"], 12.5)

    async def test_agent_initialize_resets_workflow_after_home_and_j6_open(self):
        fake = SimpleNamespace(
            initialize_for_agent=AsyncMock(
                return_value={
                    "status": "ok",
                    "action": "agent_initialize",
                    "sequence": ["home", "open_j6_to_400"],
                    "j6_target_position": 400,
                }
            ),
            close=lambda: None,
        )
        self.service._controller = fake
        self.service._grasp_final_phase = True
        self.service._gripper_prepared_for_grasp = True

        result = await self.service.initialize_agent()

        self.assertEqual(result["mcp"], "initialize_jetarm")
        self.assertEqual(result["j6_target_position"], 400)
        self.assertTrue(result["grasp_workflow_reset"])
        self.assertFalse(self.service._grasp_final_phase)
        self.assertFalse(self.service._gripper_prepared_for_grasp)

    async def test_unstable_j6_blocks_home_after_grasp(self):
        fake = SimpleNamespace(
            descend_to_height=AsyncMock(
                return_value={
                    "status": "ok",
                    "height_after_cm": 1.0,
                    "target_tolerance_cm": 0.4,
                    "motion_steps": [],
                }
            ),
            control_gripper=AsyncMock(
                return_value={
                    "status": "error",
                    "action": "grip_lock",
                    "error": "J6在10秒内未稳定，禁止回Home",
                    "j6_stability": {"status": "error", "stable": False},
                }
            ),
            go_home=AsyncMock(return_value={"status": "ok"}),
            close=lambda: None,
        )
        self.service._controller = fake

        result = await self.service._complete_final_grasp(
            320,
            147,
            {"status": "ok"},
        )

        self.assertEqual(result["status"], "error")
        self.assertEqual(result["controller_decision"], "gripper_failed")
        self.assertIn("禁止回Home", result["error"])
        fake.go_home.assert_not_awaited()

    async def test_failed_visual_confirmation_reopens_grasp_loop(self):
        self.service._awaiting_grasp_visual_confirmation = True
        self.service._gripper_prepared_for_grasp = True

        result = self.service.confirm_grasp_result(False)

        self.assertFalse(result["grasp_completed"])
        self.assertTrue(result["retry_required"])
        self.assertFalse(self.service._awaiting_grasp_visual_confirmation)
        self.assertFalse(self.service._gripper_prepared_for_grasp)

    async def test_visual_closed_loop_limit_is_two_hundred_rounds(self):
        settings = AgentSettings.from_sources(
            PROJECT_ROOT / "config" / "ai_agent.json", environ={}
        )
        session = ToolCallingSession(settings, FakeModelClient([]), ToolRegistry())

        self.assertEqual(MAX_VISUAL_CLOSED_LOOP_ROUNDS, 200)
        self.assertEqual(session.max_rounds, 200)

    async def test_agent_grasp_point_cli_requires_a_pair(self):
        args = build_parser().parse_args(
            ["--agent-grasp-x", "320", "--agent-grasp-y", "147"]
        )
        self.assertEqual(_agent_grasp_point_from_args(args), (320.0, 147.0))

        incomplete = build_parser().parse_args(["--agent-grasp-x", "320"])
        with self.assertRaisesRegex(ValueError, "必须同时提供"):
            _agent_grasp_point_from_args(incomplete)

    async def test_agent_decides_grasp_intent_without_local_camera_preselection(self):
        class FakeSession:
            async def ask(self, text, **kwargs):
                self.text = text
                self.kwargs = kwargs
                return SimpleNamespace(text="由Agent判断", tool_calls=())

        session = FakeSession()

        with redirect_stdout(io.StringIO()):
            await _send_with_tools(session, "请抓取红色物块")

        self.assertIsNone(session.kwargs["preselected_tool_name"])
        self.assertIsNone(session.kwargs["preselected_tool_arguments"])

    async def test_grasp_step_terminal_record_uses_requested_field_order(self):
        result = {
            "new_grasp_step_records": [
                {
                    "target_pixel": {"x": 330, "y": 150},
                    "original_grasp_point_xyz_cm": {"x": 0, "y": -20, "z": 10},
                    "motion_plan": {"direction": "forward", "distance_cm": 1.0},
                    "expected_grasp_point_xyz_cm": {"x": 0, "y": -21, "z": 10},
                    "actual_grasp_point_xyz_cm": {"x": 0, "y": -20.9, "z": 10},
                    "camera_grasp_vertical_angle_deg": 12.5,
                }
            ]
        }
        output = io.StringIO()

        with redirect_stdout(output):
            _print_agent_grasp_step_records(result)

        text = output.getvalue()
        labels = [
            "目标点像素坐标：",
            "原抓取点实际坐标：",
            "运动规划：",
            "预计抓取点坐标：",
            "实际抓取点坐标：",
            "夹角：",
        ]
        positions = [text.index(label) for label in labels]
        self.assertEqual(positions, sorted(positions))

    async def test_grasp_step_record_is_printed_as_soon_as_tool_returns(self):
        record_result = {
            "status": "ok",
            "new_grasp_step_records": [
                {
                    "target_pixel": {"x": 330, "y": 150},
                    "original_grasp_point_xyz_cm": {"x": 0, "y": -20, "z": 10},
                    "motion_plan": {"direction": "forward", "distance_cm": 1.0},
                    "expected_grasp_point_xyz_cm": {"x": 0, "y": -21, "z": 10},
                    "actual_grasp_point_xyz_cm": {"x": 0, "y": -20.9, "z": 10},
                    "camera_grasp_vertical_angle_deg": 12.5,
                }
            ],
        }

        class FakeSession:
            async def ask(self, _text, **kwargs):
                call = SimpleNamespace(
                    call_id="target-step-1",
                    name="control_jetarm_to_target_pixel",
                    arguments={"target_x": 330, "target_y": 150},
                    result=record_result,
                    images=(),
                )
                kwargs["on_tool_call"](call)
                self.output_at_return = output.getvalue()
                return SimpleNamespace(text="继续闭环", tool_calls=(call,))

        output = io.StringIO()
        session = FakeSession()
        with redirect_stdout(output):
            await _send_with_tools(session, "尝试抓取红色物块")

        self.assertIn("========== 抓取步骤", session.output_at_return)
        self.assertEqual(output.getvalue().count("========== 抓取步骤"), 1)
        self.assertIn("运动规划：方向=forward，距离=1.0 cm", output.getvalue())
        self.assertNotIn("[工作流 4/5] MCP结果", output.getvalue())

    async def test_internal_grasp_setup_and_legacy_pixel_tool_are_hidden_from_agent(self):
        class FakeMCPSession:
            async def list_tools(self):
                return SimpleNamespace(
                    tools=[
                        SimpleNamespace(
                            name="set_jetarm_grasp_point_pixel",
                            description="internal",
                            inputSchema={"type": "object", "properties": {}},
                        ),
                        SimpleNamespace(
                            name="move_jetarm_by_pixel_error",
                            description="legacy",
                            inputSchema={"type": "object", "properties": {}},
                        ),
                        SimpleNamespace(
                            name="control_jetarm_to_target_pixel",
                            description="target only",
                            inputSchema={
                                "type": "object",
                                "properties": {
                                    "target_x": {"type": "number"},
                                    "target_y": {"type": "number"},
                                },
                            },
                        ),
                    ]
                )

        bridge = MCPRobotBridge()
        bridge.session = FakeMCPSession()

        registry = await bridge.registry()

        self.assertEqual(registry.names(), ("control_jetarm_to_target_pixel",))

    async def test_mcp_service_rejects_only_at_configured_100cm_guard(self):
        result = await self.service.move("前5")
        self.assertEqual(result["status"], "ok")
        with self.assertRaisesRegex(Exception, "小于100"):
            await self.service.move("前100")

    async def test_markdown_workflow_is_loaded(self):
        instructions = self.service.initial_instructions()
        self.assertTrue(DEFAULT_WORKFLOW_PATH.is_file())
        self.assertIn("get_rgb_camera_frame", instructions)
        self.assertIn("control_jetarm_to_target_pixel", instructions)
        self.assertIn("Agent 根据用户自然语言自行判断", instructions)
        self.assertIn("camera.grasp_point_pixel", instructions)
        self.assertIn("/grasp-point 320 147", instructions)
        self.assertIn("有效进展检测固定关闭", instructions)
        self.assertIn("confirm_jetarm_grasp_result", instructions)
        self.assertIn("目标点像素坐标", instructions)
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
                                {"command": "前5", "speed_cm_s": 1.5},
                                ensure_ascii=False,
                            ),
                        ),
                    ),
                ),
                ToolModelResponse(
                    content="已通过一条MCP命令完成向前5厘米。",
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

        self.assertEqual(result.text, "已通过一条MCP命令完成向前5厘米。")
        self.assertEqual(
            [
                call.arguments["command"]
                for call in result.tool_calls
                if call.name == "move_jetarm"
            ],
            ["前5"],
        )
        self.assertEqual(
            [call.name for call in result.tool_calls],
            [
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
