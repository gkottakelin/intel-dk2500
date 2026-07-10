import io
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import AsyncMock, patch

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
        manual_v2_horizontal_progress_validation,
        manual_v2_vertical_progress_validation,
        parse_compact_arm_command,
        pixel_alignment_px_per_cm_for_height,
        required_mcp_tool_for_command,
    )
    from project.src.jetarm_agent.config import AgentSettings, ConfigurationError
    from project.src.jetarm_agent.cli import (
        _parse_manual_target_pixel,
        _print_manual_pixel_result,
        _resolve_manual_pixel_arm_config,
        build_parser,
    )
    from project.src.jetarm_agent.device_config import RuntimeDeviceConfig
    from project.src.jetarm_agent.manual_pixel_test_v2 import (
        CAMERA_VECTOR_VERSION,
        DEFAULT_MANUAL_GRASP_X,
        DEFAULT_MANUAL_GRASP_Y,
        run_manual_pixel_test_v2,
    )
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
        manual_v2_horizontal_progress_validation,
        manual_v2_vertical_progress_validation,
        parse_compact_arm_command,
        pixel_alignment_px_per_cm_for_height,
        required_mcp_tool_for_command,
    )
    from src.jetarm_agent.config import AgentSettings, ConfigurationError
    from src.jetarm_agent.cli import (
        _parse_manual_target_pixel,
        _print_manual_pixel_result,
        _resolve_manual_pixel_arm_config,
        build_parser,
    )
    from src.jetarm_agent.device_config import RuntimeDeviceConfig
    from src.jetarm_agent.manual_pixel_test_v2 import (
        CAMERA_VECTOR_VERSION,
        DEFAULT_MANUAL_GRASP_X,
        DEFAULT_MANUAL_GRASP_Y,
        run_manual_pixel_test_v2,
    )
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
        self.assertTrue(release["position_mode_enabled"])
        self.assertEqual(self.controller.controller.servo_mode_calls[0], 10)
        self.assertEqual(self.controller.controller.move_calls[0], (10, 370, 500))
        self.assertTrue(aligned["aligned"])
        self.assertEqual(aligned["motion_command_count"], 0)
        self.assertFalse(moved["aligned"])
        self.assertEqual(moved["direction"], "right")
        self.assertGreaterEqual(moved["speed_cm_s"], 0.7)
        self.assertLessEqual(moved["speed_cm_s"], 1.5)
        self.assertEqual(moved["requested_distance_cm"], 2)
        self.assertAlmostEqual(
            moved["pixel_to_motion_scale_px_per_cm"],
            pixel_alignment_px_per_cm_for_height(
                moved["pixel_to_motion_scale_height_cm"]
            ),
            places=6,
        )

    def test_pixel_scale_is_linear_by_height(self):
        self.assertEqual(pixel_alignment_px_per_cm_for_height(2), 50.0)
        self.assertEqual(pixel_alignment_px_per_cm_for_height(25), 18.0)
        self.assertAlmostEqual(pixel_alignment_px_per_cm_for_height(13.5), 34.0)

    async def test_pixel_difference_maps_to_centimeters_by_current_height(self):
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
        scale = pixel_alignment_px_per_cm_for_height(
            moved["pixel_to_motion_scale_height_cm"]
        )
        self.assertAlmostEqual(moved["pixel_to_motion_scale_px_per_cm"], scale, places=6)
        self.assertAlmostEqual(
            moved["requested_distance_cm"], 32.0 / scale, places=6
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
        scale = pixel_alignment_px_per_cm_for_height(
            moved["pixel_to_motion_scale_height_cm"]
        )
        self.assertAlmostEqual(
            moved["requested_distance_cm"], 65.0 / scale, places=6
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
        self.assertIn(aligned_hold["dynamic_tolerance_px"], {40.0, 25.0, 13.0, 8.0})

        self.assertEqual(moved["controller_decision"], "horizontal_align")
        self.assertFalse(moved["aligned"])
        self.assertEqual(moved["direction"], "right")
        self.assertTrue(moved["requires_new_target_pixel"])
        self.assertEqual(moved["target_pixel"], {"x": 220.0, "y": 100.0})
        self.assertEqual(moved["grasp_point_pixel"], {"x": 100.0, "y": 100.0})
        self.assertAlmostEqual(
            moved["pixel_to_motion_scale_px_per_cm"],
            pixel_alignment_px_per_cm_for_height(
                moved["pixel_to_motion_scale_height_cm"]
            ),
            places=6,
        )
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
        self.assertEqual(descended["tcp_samples_cm"][0]["source"], "command_integrated_fk")
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

    def test_manual_pixel_output_marks_relaxed_downward_pose(self):
        output = io.StringIO()
        result = {
            "controller_decision": "descend_after_alignment",
            "requested_distance_cm": 2.0,
            "speed_cm_s": 2.0,
            "height_before_cm": 8.0,
            "height_after_cm": 6.0,
            "pixel_error": {"dx": 0.0, "dy": 0.0},
            "dynamic_tolerance_px": 13.0,
            "v2_returned_camera_line_angle_deg": 12.3,
            "camera_pose_after_move": {
                "line_of_sight_angle_from_vertical_deg": 99.0,
            },
            "camera_pose_constraint": {
                "relaxed": True,
                "reason": "V2严格姿态目标超出工作空间或关节限位",
                "relaxed_step_count": 3,
            },
        }

        with patch("sys.stdout", output):
            _print_manual_pixel_result(result)

        text = output.getvalue()
        self.assertIn("姿态约束=已放宽", text)
        self.assertIn("仅下降", text)
        self.assertIn("关节限位", text)
        self.assertIn("步数=3", text)
        self.assertIn("12.3°", text)
        self.assertNotIn("99.0°", text)

    def test_manual_pixel_output_marks_relaxed_horizontal_progress(self):
        output = io.StringIO()
        result = {
            "controller_decision": "horizontal_align",
            "horizontal_progress_validation": {
                "accepted": True,
                "overrode_v2_error": True,
                "xy_change_cm": 0.8,
                "z_change_cm": 0.5,
                "camera_line_angle_error_deg": 2.5,
            },
        }

        with patch("sys.stdout", output):
            _print_manual_pixel_result(result)

        text = output.getvalue()
        self.assertIn("水平进展=放宽接受", text)
        self.assertIn("XY=0.8cm", text)
        self.assertIn("|ΔZ|=0.5cm", text)
        self.assertIn("夹角误差=2.5°", text)

    def test_manual_pixel_output_marks_relaxed_vertical_progress(self):
        output = io.StringIO()
        result = {
            "controller_decision": "descend_after_alignment",
            "vertical_progress_validation": {
                "rule": "manual_v2_relaxed_vertical_progress",
                "accepted": True,
                "overrode_v2_error": True,
                "xy_change_cm": 0.5,
                "z_change_cm": 1.9,
            },
        }

        with patch("sys.stdout", output):
            _print_manual_pixel_result(result)

        text = output.getvalue()
        self.assertIn("竖直进展=放宽接受", text)
        self.assertIn("|ΔZ|=1.9cm", text)
        self.assertIn("XY=0.5cm", text)

    def test_manual_pixel_output_reports_step_coordinates_and_failure_reason(self):
        output = io.StringIO()
        result = {
            "status": "error",
            "controller_decision": "horizontal_align",
            "grasp_point_xyz_before_cm": {"x": 0.0, "y": -20.0, "z": 10.0},
            "grasp_point_xyz_expected_cm": {"x": 1.0, "y": -20.0, "z": 10.0},
            "grasp_point_xyz_after_cm": {"x": 0.2, "y": -20.0, "z": 11.1},
            "progress_judgement": {
                "effective": False,
                "no_progress_reasons": [
                    "|ΔZ|=1.100cm 未小于1.000cm",
                    "|ΔZ|=1.100cm 未小于XY变化=0.200cm",
                ],
            },
        }

        with patch("sys.stdout", output):
            _print_manual_pixel_result(result)

        text = output.getvalue()
        self.assertIn("原本抓取点XYZ={'x': 0.0, 'y': -20.0, 'z': 10.0}", text)
        self.assertIn("预计抓取点XYZ={'x': 1.0, 'y': -20.0, 'z': 10.0}", text)
        self.assertIn("实际抓取点XYZ={'x': 0.2, 'y': -20.0, 'z': 11.1}", text)
        self.assertIn("未取得有效进展原因=|ΔZ|=1.100cm 未小于1.000cm", text)

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

    def test_manual_pixel_v2_has_independent_mode_and_fixed_default_grasp_point(self):
        args = build_parser().parse_args(["--manual-pixel-test-v2"])

        self.assertTrue(args.manual_pixel_test_v2)
        self.assertFalse(args.manual_pixel_test)
        self.assertEqual(DEFAULT_MANUAL_GRASP_X, 320.0)
        self.assertEqual(DEFAULT_MANUAL_GRASP_Y, 147.0)
        self.assertEqual(CAMERA_VECTOR_VERSION, "v2")

    def test_manual_pixel_v2_arm_config_selects_v2_runtime(self):
        args = SimpleNamespace(
            device_config=str(PROJECT_ROOT / "missing-devices.json"),
            arm_mode="dry-run",
            arm_port=None,
            arm_config=None,
        )

        config = _resolve_manual_pixel_arm_config(
            args,
            camera_vector_version="v2",
        )

        self.assertEqual(config.camera_vector_version, "v2")
        self.assertEqual(config.max_distance_cm, 100.0)
        self.assertTrue(config.allow_extended_distance)

    async def test_v2_controller_reuses_pixel_parameters_with_v2_motion_runtime(self):
        controller = JetArmToolController(
            ArmControlConfig(
                mode="dry-run",
                max_distance_cm=100.0,
                allow_extended_distance=True,
                camera_vector_version="v2",
            )
        )
        try:
            state = await controller.state()
            parameters = state["arm_parameters"]["vision_guided_grasp"]
            terminal_executor = controller.terminal.execute_terminal_motion
            with patch.object(
                controller.terminal,
                "execute_terminal_motion",
                new=AsyncMock(wraps=terminal_executor),
            ) as execute_spy:
                result = await controller.control_to_target_pixel(
                    370,
                    147,
                    DEFAULT_MANUAL_GRASP_X,
                    DEFAULT_MANUAL_GRASP_Y,
                )

            self.assertEqual(controller.terminal.__name__, "camera_vector_terminal_v2")
            self.assertEqual(type(controller.runtime).__name__, "CameraVectorV2Runtime")
            self.assertEqual(parameters["height_tolerance_bands_px"][0]["tolerance_px"], 40)
            self.assertEqual(parameters["pixel_alignment_min_speed_cm_s"], 0.7)
            self.assertEqual(parameters["pixel_alignment_max_speed_cm_s"], 1.5)
            self.assertEqual(parameters["pixel_recalculation_descent_interval_cm"], 2.0)
            self.assertEqual(result["controller_decision"], "horizontal_align")
            self.assertEqual(result["direction"], "right")
            self.assertEqual(
                result["motion_loop"],
                "camera_vector_terminal_v2_absolute_grasp_target",
            )
            self.assertEqual(result["terminal_input"]["direction"], "right")
            self.assertIn("grasp_point_xyz_expected_cm", result)
            self.assertTrue(result["progress_judgement"]["effective"])
            self.assertEqual(
                result["progress_judgement"]["no_progress_reasons"], []
            )
            execute_spy.assert_awaited_once()
            self.assertIs(
                execute_spy.await_args.args[0],
                controller.runtime,
            )
            self.assertEqual(execute_spy.await_args.kwargs["direction"], "right")
            self.assertAlmostEqual(
                execute_spy.await_args.kwargs["duration_s"],
                result["requested_distance_cm"] / result["speed_cm_s"],
            )
            self.assertAlmostEqual(
                result["v2_returned_camera_line_angle_deg"],
                result["camera_line_angle_hold"]["actual_after_deg"],
                places=3,
            )
            self.assertAlmostEqual(
                result["terminal_hold_duration_s"],
                result["requested_distance_cm"] / result["speed_cm_s"],
            )
            self.assertEqual(
                state["arm_parameters"]["agent_direction_frames"]["implementation"],
                "ubuntu22_04_operation_terminal.camera_vector_terminal_v2",
            )
            self.assertEqual(
                state["arm_parameters"]["agent_direction_frames"]["forward"],
                "grasp_to_camera_line_xy_projection",
            )
        finally:
            controller.close()

    def test_manual_v2_relaxed_horizontal_progress_rule(self):
        accepted = manual_v2_horizontal_progress_validation(
            0.8,
            0.6,
            0.5,
            2.9,
        )
        self.assertTrue(accepted["accepted"])
        self.assertEqual(accepted["xy_change_cm"], 1.0)

        rejected_cases = (
            manual_v2_horizontal_progress_validation(2.0, 0.0, 1.0, 2.0),
            manual_v2_horizontal_progress_validation(0.4, 0.0, 0.4, 2.0),
            manual_v2_horizontal_progress_validation(1.0, 0.0, 0.2, 3.0),
        )
        for result in rejected_cases:
            self.assertFalse(result["accepted"])

    def test_manual_v2_relaxed_vertical_progress_rule(self):
        accepted = manual_v2_vertical_progress_validation(0.3, 0.4, -1.9)
        self.assertTrue(accepted["accepted"])
        self.assertEqual(accepted["xy_change_cm"], 0.5)
        self.assertEqual(accepted["z_change_cm"], 1.9)

        rejected_cases = (
            manual_v2_vertical_progress_validation(0.0, 0.0, 1.8),
            manual_v2_vertical_progress_validation(1.0, 0.0, 2.0),
        )
        for result in rejected_cases:
            self.assertFalse(result["accepted"])

    async def test_manual_v2_accepts_relaxed_horizontal_feedback(self):
        controller = JetArmToolController(
            ArmControlConfig(
                mode="dry-run",
                max_distance_cm=100.0,
                allow_extended_distance=True,
                camera_vector_version="v2",
            )
        )
        try:
            start = [
                float(value)
                for value in controller.runtime.model.tcp(
                    controller.runtime.positions
                )
            ]
            reference_angle = (
                controller.runtime.camera_pose_reference_status()[
                    "camera_line_angle_from_vertical_deg"
                ]
            )
            fake_terminal_result = {
                "status": "error",
                "error": "forced strict tolerance failure",
                "execution_path": "camera_vector_terminal_v2_absolute_grasp_target",
                "workflow": ["terminal_input", "absolute_grasp_target"],
                "terminal_input": {
                    "direction": "right",
                    "direction_unit_base": [0.0, 1.0, 0.0],
                },
                "steps": 1,
                "tcp_samples_m": [],
                "start_grasp_point_m": start,
                "actual_grasp_point_m": [
                    start[0],
                    start[1] + 0.008,
                    start[2] + 0.005,
                ],
                "motion_plan": {"target_joint_positions": dict(controller.runtime.positions)},
                "target_camera_line_angle_deg": reference_angle,
                "actual_camera_line_angle_deg": reference_angle + 2.5,
                "feedback_corrections": 0,
            }
            with patch.object(
                controller.terminal,
                "execute_terminal_motion",
                new=AsyncMock(return_value=fake_terminal_result),
            ):
                result = await controller.control_to_target_pixel(
                    370,
                    147,
                    DEFAULT_MANUAL_GRASP_X,
                    DEFAULT_MANUAL_GRASP_Y,
                )

            validation = result["horizontal_progress_validation"]
            self.assertEqual(result["status"], "ok")
            self.assertTrue(validation["accepted"])
            self.assertTrue(validation["overrode_v2_error"])
            self.assertEqual(validation["z_change_cm"], 0.5)
            self.assertEqual(validation["xy_change_cm"], 0.8)
            self.assertEqual(validation["camera_line_angle_error_deg"], 2.5)
        finally:
            controller.close()

    async def test_manual_v2_accepts_relaxed_vertical_feedback(self):
        controller = JetArmToolController(
            ArmControlConfig(
                mode="dry-run",
                max_distance_cm=100.0,
                allow_extended_distance=True,
                camera_vector_version="v2",
            )
        )
        try:
            start = [
                float(value)
                for value in controller.runtime.model.tcp(
                    controller.runtime.positions
                )
            ]
            reference_angle = (
                controller.runtime.camera_pose_reference_status()[
                    "camera_line_angle_from_vertical_deg"
                ]
            )
            fake_terminal_result = {
                "status": "error",
                "error": "forced strict vertical tolerance failure",
                "execution_path": "camera_vector_terminal_v2_absolute_grasp_target",
                "workflow": ["terminal_input", "absolute_grasp_target"],
                "terminal_input": {
                    "direction": "down",
                    "direction_unit_base": [0.0, 0.0, -1.0],
                },
                "steps": 1,
                "tcp_samples_m": [],
                "start_grasp_point_m": start,
                "actual_grasp_point_m": [
                    start[0] + 0.003,
                    start[1] + 0.004,
                    start[2] - 0.019,
                ],
                "motion_plan": {
                    "target_joint_positions": dict(controller.runtime.positions)
                },
                "target_camera_line_angle_deg": reference_angle,
                "actual_camera_line_angle_deg": reference_angle,
                "feedback_corrections": 0,
            }
            with patch.object(
                controller.terminal,
                "execute_terminal_motion",
                new=AsyncMock(return_value=fake_terminal_result),
            ):
                result = await controller.control_to_target_pixel(
                    DEFAULT_MANUAL_GRASP_X,
                    DEFAULT_MANUAL_GRASP_Y,
                    DEFAULT_MANUAL_GRASP_X,
                    DEFAULT_MANUAL_GRASP_Y,
                )

            validation = result["vertical_progress_validation"]
            self.assertEqual(result["status"], "ok")
            self.assertTrue(validation["accepted"])
            self.assertTrue(validation["overrode_v2_error"])
            self.assertEqual(validation["z_change_cm"], 1.9)
            self.assertEqual(validation["xy_change_cm"], 0.5)
        finally:
            controller.close()

    async def test_manual_v2_motion_matches_direct_v2_executor(self):
        config = ArmControlConfig(
            mode="dry-run",
            max_distance_cm=100.0,
            allow_extended_distance=True,
            camera_vector_version="v2",
        )
        manual = JetArmToolController(config)
        direct = JetArmToolController(config)
        try:
            manual_result = await manual.control_to_target_pixel(
                370,
                147,
                DEFAULT_MANUAL_GRASP_X,
                DEFAULT_MANUAL_GRASP_Y,
            )
            direct_result = await direct.terminal.execute_terminal_motion(
                direct.runtime,
                direction=manual_result["direction"],
                speed_cm_s=manual_result["speed_cm_s"],
                duration_s=manual_result["terminal_hold_duration_s"],
                real_time=False,
            )

            self.assertEqual(
                manual_result["v2_motion_plan"]["target_grasp_point_m"],
                direct_result["target_grasp_point_m"],
            )
            self.assertEqual(
                manual_result["v2_motion_plan"]["target_joint_positions"],
                direct_result["target_joint_positions"],
            )
            self.assertAlmostEqual(
                manual_result["v2_returned_camera_line_angle_deg"],
                direct_result["actual_camera_line_angle_deg"],
                places=9,
            )
        finally:
            manual.close()
            direct.close()

    async def test_v2_zero_progress_is_reported_as_error(self):
        controller = JetArmToolController(
            ArmControlConfig(
                mode="dry-run",
                max_distance_cm=100.0,
                allow_extended_distance=True,
                camera_vector_version="v2",
            )
        )
        try:
            rejected_plan = {
                "accepted": False,
                "strict_pose": True,
                "relaxed": False,
                "message": "forced no progress",
                "target_joint_positions": dict(controller.runtime.positions),
                "target_camera_line_angle_deg": (
                    controller.runtime.camera_line_vertical_angle_deg()
                ),
            }
            with patch.object(
                controller.runtime,
                "plan_grasp_target",
                return_value=rejected_plan,
            ):
                result = await controller.move_tcp("forward", 0.1, 1.0)

            self.assertEqual(result["status"], "error")
            self.assertEqual(result["estimated_distance_cm"], 0.0)
            self.assertIn("forced no progress", result["error"])
        finally:
            controller.close()

    async def test_final_descent_stops_after_no_height_progress(self):
        controller = JetArmToolController(
            ArmControlConfig(
                mode="dry-run",
                max_distance_cm=100.0,
                allow_extended_distance=True,
                camera_vector_version="v2",
            )
        )
        try:
            with patch.object(
                controller,
                "_move_tcp_segment",
                new=AsyncMock(return_value={"status": "ok"}),
            ):
                result = await controller.descend_to_height(1.0)

            self.assertEqual(result["status"], "error")
            self.assertIn("下降无有效进展", result["error"])
            self.assertEqual(result["steps"], 1)
        finally:
            controller.close()

    async def test_v2_descent_relaxes_at_limit_and_reports_status(self):
        controller = JetArmToolController(
            ArmControlConfig(
                mode="dry-run",
                max_distance_cm=100.0,
                allow_extended_distance=True,
                camera_vector_version="v2",
            )
        )
        try:
            relaxed_segment = None
            aligned_hold = None
            for _index in range(20):
                result = await controller.control_to_target_pixel(
                    DEFAULT_MANUAL_GRASP_X,
                    DEFAULT_MANUAL_GRASP_Y,
                    DEFAULT_MANUAL_GRASP_X,
                    DEFAULT_MANUAL_GRASP_Y,
                )
                pose_status = result.get("camera_pose_constraint")
                if isinstance(pose_status, dict) and pose_status.get("relaxed"):
                    relaxed_segment = result
                if result.get("controller_decision") == "aligned_hold":
                    aligned_hold = result
                    break

            self.assertIsNotNone(relaxed_segment)
            self.assertIsNotNone(aligned_hold)
            self.assertEqual(relaxed_segment["status"], "ok")
            self.assertEqual(
                relaxed_segment["camera_pose_constraint"]["reason_code"],
                "strict_pose_unreachable_or_joint_limit",
            )

            final_result = await controller.descend_to_height(1.0)

            self.assertEqual(final_result["status"], "ok")
            self.assertTrue(final_result["camera_pose_constraint"]["relaxed"])
            self.assertGreater(
                final_result["camera_pose_constraint"]["relaxed_step_count"],
                0,
            )
            self.assertLessEqual(
                final_result["height_after_cm"],
                final_result["target_height_cm"]
                + final_result["target_tolerance_cm"],
            )
            self.assertTrue(final_result["motion_steps"])
            for step in final_result["motion_steps"]:
                self.assertIn("original_grasp_point_xyz_cm", step)
                self.assertIn("expected_grasp_point_xyz_cm", step)
                self.assertIn("actual_grasp_point_xyz_cm", step)
                self.assertIn("no_progress_reasons", step)
        finally:
            controller.close()

    async def test_manual_pixel_v2_workflow_reports_default_grasp_and_runtime(self):
        args = build_parser().parse_args(
            ["--manual-pixel-test-v2", "--arm-mode", "dry-run"]
        )
        output = io.StringIO()

        with patch("builtins.input", return_value="q"), patch(
            "sys.stdout", output
        ):
            exit_code = await run_manual_pixel_test_v2(args)

        self.assertEqual(exit_code, 0)
        text = output.getvalue()
        self.assertIn("固定抓取点像素=(320, 147)", text)
        self.assertIn("camera_vector_terminal_v2 / CameraVectorV2Runtime", text)
        self.assertIn("2cm=50px/cm，25cm=18px/cm", text)

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

        self.assertEqual(home["joint_positions"], {"J1": 500, "J2": 410, "J3": 800, "J4": 800})
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
            "base_horizontal_xy",
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
