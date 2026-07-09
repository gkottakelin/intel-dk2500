import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

try:
    from project.src.jetarm_agent.arm_control import (
        MAX_AGENT_MOVE_COMMAND_CM,
        ArmControlConfig,
        ArmControlError,
        JetArmToolController,
        build_arm_tool_registry,
        choose_arm_serial_port,
        format_compact_arm_command,
        looks_like_arm_command,
        looks_like_camera_command,
        looks_like_grasp_workflow_command,
        parse_compact_arm_command,
        required_mcp_tool_for_command,
    )
    from project.src.jetarm_agent.config import AgentSettings, ConfigurationError
    from project.src.jetarm_agent.cli import _parse_manual_target_pixel
    from project.src.jetarm_agent.cli import _resolve_manual_pixel_arm_config
    from project.src.jetarm_agent.device_config import RuntimeDeviceConfig
    from project.src.jetarm_agent.openai_compatible import (
        FunctionToolCall,
        ToolModelResponse,
    )
    from project.src.jetarm_agent.tool_agent import ToolCallingSession
except ModuleNotFoundError:
    from src.jetarm_agent.arm_control import (
        MAX_AGENT_MOVE_COMMAND_CM,
        ArmControlConfig,
        ArmControlError,
        JetArmToolController,
        build_arm_tool_registry,
        choose_arm_serial_port,
        format_compact_arm_command,
        looks_like_arm_command,
        looks_like_camera_command,
        looks_like_grasp_workflow_command,
        parse_compact_arm_command,
        required_mcp_tool_for_command,
    )
    from src.jetarm_agent.config import AgentSettings, ConfigurationError
    from src.jetarm_agent.cli import _parse_manual_target_pixel
    from src.jetarm_agent.cli import _resolve_manual_pixel_arm_config
    from src.jetarm_agent.device_config import RuntimeDeviceConfig
    from src.jetarm_agent.openai_compatible import FunctionToolCall, ToolModelResponse
    from src.jetarm_agent.tool_agent import ToolCallingSession


PROJECT_ROOT = Path(__file__).resolve().parents[1]


class FakeToolModelClient:
    def __init__(self, responses):
        self.responses = list(responses)
        self.requests = []

    async def complete_with_tools(self, messages, tools, *, tool_choice="auto"):
        self.requests.append(
            {"messages": list(messages), "tools": list(tools), "tool_choice": tool_choice}
        )
        return self.responses.pop(0)


class ArmControlDryRunTest(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.controller = JetArmToolController(ArmControlConfig(mode="dry-run"))

    async def asyncTearDown(self):
        self.controller.close()

    async def test_executes_one_agent_command_without_controller_splitting(self):
        result = await self.controller.move_tcp("forward", 1.9)

        self.assertEqual(result["status"], "ok")
        self.assertEqual(result["requested_distance_cm"], 1.9)
        self.assertEqual(result["command"], "前1.9")
        self.assertEqual(result["speed_cm_s"], 1.5)
        self.assertEqual(result["command_limit_cm_exclusive"], 2)
        self.assertEqual(result["motion_command_count"], 1)
        self.assertNotIn("segments", result)
        self.assertGreater(result["estimated_distance_cm"], 1)
        self.assertTrue(self.controller.controller.move_calls)
        self.assertTrue(
            all(
                servo_id in {1, 2, 3, 4}
                for servo_id, _target, _run_time in self.controller.controller.move_calls
            )
        )

    async def test_compact_command_and_speed_bounds(self):
        self.assertEqual(parse_compact_arm_command("前5厘米"), ("forward", 5.0))
        self.assertEqual(format_compact_arm_command("up", 1.5), "上1.5")
        result = await self.controller.execute_compact_command("右1.5", 2.0)
        self.assertEqual(result["status"], "ok")
        self.assertEqual(result["speed_cm_s"], 2.0)
        with self.assertRaisesRegex(ArmControlError, "1到5"):
            await self.controller.execute_compact_command("前1", 0.9)
        with self.assertRaisesRegex(ArmControlError, "1到5"):
            await self.controller.execute_compact_command("前1", 5.1)

    async def test_default_speed_changes_servo_execution_time(self):
        await self.controller.execute_compact_command("前1")
        run_times = [item[2] for item in self.controller.controller.move_calls]
        self.assertTrue(run_times)
        self.assertGreater(max(run_times), 80)

    async def test_rejects_two_centimeters_or_more_without_splitting(self):
        with self.assertRaisesRegex(ArmControlError, "单次必须小于2"):
            await self.controller.move_tcp("forward", 2)
        with self.assertRaisesRegex(ArmControlError, "单次不能超过2"):
            await self.controller.move_tcp("forward", 5)
        self.assertEqual(self.controller.controller.move_calls, [])

    async def test_wrist_and_gripper_motor_actions_always_stop(self):
        await self.controller.rotate_wrist("clockwise", 0.5)
        await self.controller.control_gripper("open", 0.5)
        await self.controller.control_gripper("grip_lock")
        await self.controller.control_gripper("release_lock")

        calls = self.controller.controller.motor_calls
        self.assertEqual(calls[0:2], [(5, 100), (5, 0)])
        self.assertEqual(calls[2:4], [(10, -100), (10, 0)])
        self.assertEqual(calls[4:6], [(10, 300), (10, 0)])

    async def test_gripper_release_position_and_pixel_alignment_tool(self):
        release = await self.controller.set_gripper_position(370)
        aligned = await self.controller.move_by_pixel_error(104, 96, 100, 100)
        moved = await self.controller.move_by_pixel_error(
            220,
            100,
            100,
            100,
            tolerance_px=10,
            step_duration_s=0.4,
            speed_saturation_px=120,
        )

        self.assertEqual(release["status"], "ok")
        self.assertEqual(release["target_position"], 370)
        self.assertEqual(self.controller.controller.move_calls[0], (10, 370, 500))
        self.assertTrue(aligned["aligned"])
        self.assertEqual(aligned["motion_command_count"], 0)
        self.assertFalse(moved["aligned"])
        self.assertEqual(moved["direction"], "right")
        self.assertGreaterEqual(moved["speed_cm_s"], 0.7)
        self.assertLessEqual(moved["speed_cm_s"], 1.5)
        self.assertEqual(moved["requested_distance_cm"], 2)
        self.assertEqual(moved["pixel_to_motion_scale_px_per_cm"], 16.0)

    async def test_pixel_difference_maps_to_centimeters_at_sixteen_px_per_cm(self):
        moved = await self.controller.move_by_pixel_error(
            68,
            100,
            100,
            100,
            tolerance_px=10,
        )

        self.assertFalse(moved["aligned"])
        self.assertEqual(moved["direction"], "left")
        self.assertEqual(moved["pixel_error"], {"dx": -32.0, "dy": 0.0})
        self.assertEqual(moved["pixel_to_motion_scale_px_per_cm"], 16.0)
        self.assertAlmostEqual(
            moved["requested_distance_cm"], 2.0, places=6
        )

    async def test_manual_extended_pixel_distance_is_not_capped_at_two_cm(self):
        controller = JetArmToolController(
            ArmControlConfig(
                mode="dry-run",
                max_distance_cm=100,
                allow_extended_distance=True,
            )
        )
        self.addAsyncCleanup(self._close_controller, controller)

        moved = await controller.move_by_pixel_error(
            165,
            100,
            100,
            100,
            tolerance_px=10,
        )

        self.assertEqual(moved["direction"], "right")
        self.assertAlmostEqual(
            moved["requested_distance_cm"], 65.0 / 16.0, places=6
        )
        self.assertEqual(moved["pixel_error"], {"dx": 65.0, "dy": 0.0})

    async def _close_controller(self, controller):
        controller.close()

    async def test_controller_owned_target_pixel_workflow(self):
        aligned_hold = await self.controller.control_to_target_pixel(
            100,
            100,
            100,
            100,
            descend_when_aligned=False,
        )
        moved = await self.controller.control_to_target_pixel(
            220,
            100,
            100,
            100,
            descend_when_aligned=False,
            step_duration_s=0.4,
            speed_saturation_px=120,
        )
        descended = await self.controller.control_to_target_pixel(
            100,
            100,
            100,
            100,
        )

        self.assertEqual(aligned_hold["agent_role"], "target_pixel_only")
        self.assertEqual(aligned_hold["controller_decision"], "aligned_hold")
        self.assertTrue(aligned_hold["aligned"])
        self.assertFalse(aligned_hold["requires_new_target_pixel"])
        self.assertEqual(aligned_hold["height_source"], "joint_feedback_fk")
        self.assertIn("grasp_point_before_cm", aligned_hold)
        self.assertIn("grasp_point_after_cm", aligned_hold)
        self.assertIn(aligned_hold["dynamic_tolerance_px"], {18.0, 15.0, 10.0, 8.0})

        self.assertEqual(moved["controller_decision"], "horizontal_align")
        self.assertFalse(moved["aligned"])
        self.assertEqual(moved["direction"], "right")
        self.assertTrue(moved["requires_new_target_pixel"])
        self.assertEqual(moved["target_pixel"], {"x": 220.0, "y": 100.0})
        self.assertEqual(moved["grasp_point_pixel"], {"x": 100.0, "y": 100.0})
        self.assertEqual(moved["pixel_to_motion_scale_px_per_cm"], 16.0)
        self.assertIn("grasp_point_before_cm", moved)
        self.assertIn("grasp_point_after_cm", moved)
        self.assertLessEqual(moved["speed_cm_s"], 1.5)

        self.assertEqual(descended["controller_decision"], "descend_after_alignment")
        self.assertTrue(descended["aligned"])
        self.assertEqual(descended["direction"], "down")
        self.assertEqual(descended["speed_cm_s"], 2.0)
        self.assertEqual(descended["descent_recalibration_interval_cm"], 2.0)
        self.assertTrue(descended["requires_new_target_pixel"])
        self.assertTrue(descended["tcp_samples_cm"])
        self.assertEqual(descended["tcp_samples_cm"][0]["source"], "joint_feedback_fk")
        self.assertIn("grasp_point_before_cm", descended)
        self.assertIn("grasp_point_after_cm", descended)

    def test_manual_pixel_input_parser(self):
        self.assertEqual(_parse_manual_target_pixel("450 230"), (450.0, 230.0))
        self.assertEqual(_parse_manual_target_pixel("450,230"), (450.0, 230.0))
        self.assertEqual(_parse_manual_target_pixel("450，230"), (450.0, 230.0))
        self.assertIsNone(_parse_manual_target_pixel("q"))
        with self.assertRaisesRegex(ValueError, "两个像素坐标"):
            _parse_manual_target_pixel("450")
        with self.assertRaisesRegex(ValueError, "必须是数字"):
            _parse_manual_target_pixel("x y")

    def test_manual_pixel_arm_config_uses_hardware_device_config(self):
        with tempfile.TemporaryDirectory() as directory:
            config_path = Path(directory) / "devices.json"
            RuntimeDeviceConfig(
                arm_mode="hardware",
                arm_port="/dev/ttyUSB0",
                arm_terminal_config=str(PROJECT_ROOT / "ubuntu22_04_operation_terminal" / "config" / "terminal.json"),
            ).save(config_path)
            args = SimpleNamespace(
                device_config=str(config_path),
                arm_mode=None,
                arm_port=None,
                arm_config=None,
            )

            config = _resolve_manual_pixel_arm_config(args)

        self.assertEqual(config.mode, "hardware")
        self.assertEqual(config.serial_port, "/dev/ttyUSB0")
        self.assertEqual(config.max_distance_cm, 100.0)
        self.assertTrue(config.allow_extended_distance)

    def test_manual_pixel_hardware_without_port_delegates_to_terminal_discovery(self):
        args = SimpleNamespace(
            device_config=str(PROJECT_ROOT / "missing-devices.json"),
            arm_mode="hardware",
            arm_port=None,
            arm_config=None,
        )

        config = _resolve_manual_pixel_arm_config(args)

        self.assertEqual(config.mode, "hardware")
        self.assertIsNone(config.serial_port)

    def test_default_arm_config_still_rejects_over_two_cm_limit(self):
        with self.assertRaisesRegex(ArmControlError, "不能超过2"):
            ArmControlConfig(max_distance_cm=100).validate()

    def test_manual_pixel_arm_config_rejects_off_mode(self):
        args = SimpleNamespace(
            device_config=str(PROJECT_ROOT / "missing-devices.json"),
            arm_mode="off",
            arm_port=None,
            arm_config=None,
        )

        with self.assertRaisesRegex(ConfigurationError, "不能使用off"):
            _resolve_manual_pixel_arm_config(args)

    async def test_home_stop_and_state_cover_original_terminal_actions(self):
        home = await self.controller.go_home()
        stopped = await self.controller.stop_all()
        state = await self.controller.state()

        self.assertEqual(home["joint_positions"], {"J1": 500, "J2": 478, "J3": 641, "J4": 890})
        self.assertEqual(stopped["action"], "stop_all")
        self.assertIn("tcp_cm", state)
        self.assertGreater(
            state["arm_pose"]["camera"][
                "line_of_sight_angle_from_vertical_deg"
            ],
            0.0,
        )
        self.assertEqual(state["arm_pose"]["camera"]["control_frame"], "camera_vector")
        self.assertEqual(
            state["arm_pose"]["grasp_point_base_cm"], state["tcp_cm"]
        )
        self.assertEqual(
            state["arm_parameters"]["agent_direction_frames"]["up_down"],
            "camera_grasp_line",
        )
        self.assertEqual(
            state["arm_parameters"]["agent_direction_frames"][
                "forward_backward_left_right"
            ],
            "camera_vector_plane",
        )
        self.assertEqual(
            state["arm_parameters"]["joints"]["J2"]["servo_id"], 2
        )
        self.assertEqual(
            state["arm_parameters"]["vision_guided_grasp"][
                "j6_release_position_before_success"
            ],
            370,
        )
        self.assertEqual(
            state["arm_parameters"]["vision_guided_grasp"][
                "pixel_recalculation_descent_interval_cm"
            ],
            2.0,
        )
        self.assertEqual(
            [
                band["tolerance_px"]
                for band in state["arm_parameters"]["vision_guided_grasp"][
                    "height_tolerance_bands_px"
                ]
            ],
            [40, 25, 13, 8],
        )
        home_servo_ids = {
            servo_id
            for servo_id, _target, _run_time in self.controller.controller.move_calls
        }
        self.assertEqual(home_servo_ids, {1, 2, 3, 4, 5})

    async def test_camera_relative_up_rotates_with_current_pose(self):
        self.controller.runtime.positions["J4"] -= 100

        pose = await self.controller.pose()
        result = await self.controller.move_tcp("up", 1.9)

        self.assertGreater(
            pose["camera"]["line_of_sight_angle_from_vertical_deg"], 20
        )
        self.assertEqual(result["direction_reference"], "camera_vector")
        self.assertNotEqual(result["direction_unit_base"]["forward_x"], 0.0)
        self.assertLess(result["direction_unit_base"]["up_z"], 1.0)

    async def test_tool_registry_exposes_only_bounded_arm_functions(self):
        schemas = build_arm_tool_registry(self.controller).schemas()
        names = {schema["function"]["name"] for schema in schemas}

        self.assertEqual(
            names,
            {
                "move_jetarm_tcp",
                "rotate_jetarm_wrist",
                "control_jetarm_gripper",
                "set_jetarm_gripper_position",
                "move_jetarm_by_pixel_error",
                "control_jetarm_to_target_pixel",
                "move_jetarm_home",
                "stop_jetarm",
                "get_jetarm_state",
            },
        )
        move_schema = next(
            schema for schema in schemas if schema["function"]["name"] == "move_jetarm_tcp"
        )
        self.assertEqual(
            move_schema["function"]["parameters"]["properties"]["distance_cm"][
                "exclusiveMaximum"
            ],
            2,
        )
        speed_schema = move_schema["function"]["parameters"]["properties"]["speed_cm_s"]
        self.assertEqual(speed_schema["minimum"], 1)
        self.assertEqual(speed_schema["maximum"], 5)
        self.assertEqual(speed_schema["default"], 1.5)
        pixel_schema = next(
            schema for schema in schemas if schema["function"]["name"] == "move_jetarm_by_pixel_error"
        )
        self.assertEqual(
            pixel_schema["function"]["parameters"]["properties"]["tolerance_px"]["default"],
            10,
        )
        target_schema = next(
            schema for schema in schemas if schema["function"]["name"] == "control_jetarm_to_target_pixel"
        )
        self.assertEqual(
            target_schema["function"]["parameters"]["properties"]["descent_step_cm"]["default"],
            2.0,
        )

    async def test_arm_command_detection_does_not_require_model_guessing(self):
        self.assertTrue(looks_like_grasp_workflow_command("抓取物块"))
        self.assertTrue(looks_like_arm_command("抓取物块"))
        self.assertIsNone(required_mcp_tool_for_command("抓取物块"))
        self.assertTrue(looks_like_arm_command("向前移动5厘米"))
        self.assertTrue(looks_like_arm_command("前5"))
        self.assertTrue(looks_like_arm_command("夹紧夹爪"))
        self.assertFalse(looks_like_arm_command("请介绍一下机械臂的结构"))
        self.assertEqual(required_mcp_tool_for_command("向前移动5厘米"), "move_jetarm")
        self.assertEqual(required_mcp_tool_for_command("前5"), "move_jetarm")
        self.assertEqual(
            required_mcp_tool_for_command("读取机械臂参数和关节限位"),
            "get_jetarm_state",
        )
        self.assertTrue(looks_like_camera_command("查看摄像头画面"))
        self.assertEqual(
            required_mcp_tool_for_command("描述当前RGB图像"),
            "get_rgb_camera_frame",
        )

    async def test_serial_chooser_reuses_ubuntu_terminal_dialog(self):
        class FakeRoot:
            def __init__(self):
                self.withdrawn = False
                self.destroyed = False

            def withdraw(self):
                self.withdrawn = True

            def destroy(self):
                self.destroyed = True

        root = FakeRoot()
        calls = []

        def choose(fake_root, initial_port):
            calls.append((fake_root, initial_port))
            return "/dev/ttyUSB0"

        terminal = SimpleNamespace(
            tk=SimpleNamespace(Tk=lambda: root),
            choose_serial_port_dialog=choose,
        )
        patch_target = f"{choose_arm_serial_port.__module__}._load_terminal_module"
        with patch(patch_target, return_value=terminal):
            selected = choose_arm_serial_port()

        self.assertEqual(selected, "/dev/ttyUSB0")
        self.assertEqual(calls, [(root, None)])
        self.assertTrue(root.withdrawn)
        self.assertTrue(root.destroyed)

    async def test_ai_tool_call_executes_distance_planner_and_returns_result(self):
        fake = FakeToolModelClient(
            [
                ToolModelResponse(content="好的。", tool_calls=()),
                ToolModelResponse(
                    content="",
                    tool_calls=(
                        FunctionToolCall(
                            call_id="arm-call-1",
                            name="move_jetarm_tcp",
                            arguments=json.dumps(
                                {"direction": "forward", "distance_cm": 1}
                            ),
                        ),
                    ),
                ),
                ToolModelResponse(content="已向前移动1厘米。", tool_calls=()),
            ]
        )
        settings = AgentSettings.from_sources(
            PROJECT_ROOT / "config" / "ai_agent.json", environ={}
        )
        session = ToolCallingSession(
            settings,
            fake,
            build_arm_tool_registry(self.controller),
        )

        command = "向前移动1厘米"
        result = await session.ask(
            command,
            require_any_tool=looks_like_arm_command(command),
            required_tool_retries=1,
        )

        self.assertEqual(result.text, "已向前移动1厘米。")
        self.assertEqual(result.tool_calls[0].name, "move_jetarm_tcp")
        self.assertEqual(result.tool_calls[0].result["status"], "ok")
        self.assertTrue(self.controller.controller.move_calls)
        self.assertEqual(len(fake.requests), 3)
        self.assertIn("必须调用", fake.requests[1]["messages"][-1]["content"])
        tool_result_message = fake.requests[2]["messages"][-1]
        self.assertEqual(tool_result_message["role"], "tool")
        self.assertEqual(tool_result_message["tool_call_id"], "arm-call-1")

    async def test_agent_executes_long_request_as_sequential_sub_two_cm_calls(self):
        fake = FakeToolModelClient(
            [
                ToolModelResponse(
                    content="",
                    tool_calls=(
                        FunctionToolCall(
                            call_id="arm-call-1",
                            name="move_jetarm_tcp",
                            arguments=json.dumps(
                                {"direction": "forward", "distance_cm": 1.9}
                            ),
                        ),
                    ),
                ),
                ToolModelResponse(
                    content="",
                    tool_calls=(
                        FunctionToolCall(
                            call_id="arm-call-2",
                            name="move_jetarm_tcp",
                            arguments=json.dumps(
                                {"direction": "forward", "distance_cm": 1.9}
                            ),
                        ),
                    ),
                ),
                ToolModelResponse(
                    content="",
                    tool_calls=(
                        FunctionToolCall(
                            call_id="arm-call-3",
                            name="move_jetarm_tcp",
                            arguments=json.dumps(
                                {"direction": "forward", "distance_cm": 1.2}
                            ),
                        ),
                    ),
                ),
                ToolModelResponse(
                    content="已按三条MCP命令向前移动5厘米。",
                    tool_calls=(),
                ),
            ]
        )
        settings = AgentSettings.from_sources(
            PROJECT_ROOT / "config" / "ai_agent.json", environ={}
        )
        session = ToolCallingSession(
            settings,
            fake,
            build_arm_tool_registry(self.controller),
        )

        result = await session.ask(
            "向前移动5厘米",
            require_any_tool=True,
            required_tool_name="move_jetarm_tcp",
        )

        self.assertEqual(
            [call.arguments["distance_cm"] for call in result.tool_calls],
            [1.9, 1.9, 1.2],
        )
        self.assertTrue(all(call.result["status"] == "ok" for call in result.tool_calls))
        self.assertAlmostEqual(
            sum(call.arguments["distance_cm"] for call in result.tool_calls),
            5.0,
        )

    async def test_only_first_motion_call_in_same_model_round_is_executed(self):
        fake = FakeToolModelClient(
            [
                ToolModelResponse(
                    content="",
                    tool_calls=(
                        FunctionToolCall(
                            call_id="arm-call-1",
                            name="move_jetarm_tcp",
                            arguments=json.dumps(
                                {"direction": "forward", "distance_cm": 1.9}
                            ),
                        ),
                        FunctionToolCall(
                            call_id="arm-call-2",
                            name="move_jetarm_tcp",
                            arguments=json.dumps(
                                {"direction": "forward", "distance_cm": 1.9}
                            ),
                        ),
                    ),
                ),
                ToolModelResponse(content="已执行第一条，第二条未下发。", tool_calls=()),
            ]
        )
        settings = AgentSettings.from_sources(
            PROJECT_ROOT / "config" / "ai_agent.json", environ={}
        )
        session = ToolCallingSession(
            settings,
            fake,
            build_arm_tool_registry(self.controller),
        )

        result = await session.ask(
            "向前移动3.8厘米",
            require_any_tool=True,
            required_tool_name="move_jetarm_tcp",
        )

        self.assertEqual(result.tool_calls[0].result["status"], "ok")
        self.assertEqual(result.tool_calls[1].result["status"], "error")
        self.assertIn("同一轮只允许", result.tool_calls[1].result["error"])


if __name__ == "__main__":
    unittest.main()
