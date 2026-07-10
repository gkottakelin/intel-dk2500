import asyncio
import math
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np


APP_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP_ROOT))

from camera_vector_terminal import CameraLineConfig  # noqa: E402
from camera_vector_terminal_v2 import (  # noqa: E402
    INITIAL_J6_POSITION,
    CameraVectorV2Runtime,
    HorizontalDirectionUndefined,
    _unit,
    apply_terminal_motion_input,
    build_camera_vector_v2_frame,
    execute_terminal_motion,
    release_terminal_motion_input,
)
from jetarm_terminal import DryRunServoController, TerminalSettings  # noqa: E402


class CameraVectorTerminalV2Test(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.settings = TerminalSettings.from_file(APP_ROOT / "config" / "terminal.json")
        cls.camera_config = CameraLineConfig.from_file(APP_ROOT / "config" / "terminal.json")

    def make_runtime(self):
        controller = DryRunServoController(self.settings)
        runtime = CameraVectorV2Runtime(
            controller,
            self.settings,
            camera_config=self.camera_config,
        )
        runtime.initialize(use_home_positions=True)
        return runtime, controller

    def test_forward_is_horizontal_camera_to_grasp_projection(self):
        runtime, _controller = self.make_runtime()
        frame = runtime.camera_relative_frame()
        expected = frame.up.copy()
        expected[2] = 0.0
        expected /= np.linalg.norm(expected)

        np.testing.assert_allclose(frame.forward, -expected)
        np.testing.assert_allclose(frame.backward, expected)
        np.testing.assert_allclose(frame.left, np.cross((0.0, 0.0, 1.0), expected))
        np.testing.assert_allclose(frame.right, -frame.left)
        self.assertEqual(float(frame.forward[2]), 0.0)
        self.assertEqual(float(frame.left[2]), 0.0)
        self.assertAlmostEqual(float(np.dot(frame.forward, frame.left)), 0.0, places=9)

    def test_planar_frame_rotates_with_camera_line_not_fixed_base_xy(self):
        runtime, _controller = self.make_runtime()
        before = runtime.camera_relative_frame().forward
        runtime.positions["J1"] += 100
        after = runtime.camera_relative_frame().forward

        self.assertFalse(np.allclose(before, after))
        self.assertAlmostEqual(float(before[2]), 0.0, places=9)
        self.assertAlmostEqual(float(after[2]), 0.0, places=9)

    def test_terminal_input_applies_requested_speed_to_swapped_forward(self):
        runtime, _controller = self.make_runtime()
        frame = runtime.camera_relative_frame()

        status = apply_terminal_motion_input(runtime, "forward", 1.0)

        self.assertAlmostEqual(status["input_ratio"], 0.2)
        self.assertAlmostEqual(runtime.joystick_x, 0.0)
        self.assertAlmostEqual(runtime.joystick_y, -0.2)
        np.testing.assert_allclose(
            runtime.cartesian_velocity(),
            frame.forward * 0.01,
        )
        release_terminal_motion_input(runtime)
        self.assertEqual(runtime.joystick_x, 0.0)
        self.assertEqual(runtime.joystick_y, 0.0)

    def test_shared_terminal_executor_converts_speed_and_time_to_distance(self):
        runtime, _controller = self.make_runtime()
        before = runtime.model.tcp(runtime.positions)

        result = asyncio.run(
            execute_terminal_motion(
                runtime,
                direction="left",
                speed_cm_s=1.0,
                duration_s=2.5,
                real_time=False,
                collect_tcp_samples=True,
            )
        )

        after = runtime.model.tcp(runtime.positions)
        self.assertEqual(
            result["execution_path"],
            "camera_vector_terminal_v2_absolute_grasp_target",
        )
        self.assertEqual(result["requested_distance_cm"], 2.5)
        self.assertEqual(result["terminal_input"]["direction"], "left")
        self.assertEqual(result["terminal_input"]["speed_cm_s"], 1.0)
        self.assertTrue(result["tcp_samples_m"])
        self.assertEqual(result["steps"], 1)
        self.assertEqual(result["feedback_corrections"], 0)
        self.assertEqual(
            result["workflow"],
            [
                "terminal_input",
                "absolute_grasp_target",
                "best_pose_ik",
                "synchronized_joint_adjustment",
                "joint_feedback_verification",
            ],
        )
        self.assertLess(result["position_error_m"], 0.004)
        self.assertLess(result["camera_line_angle_error_deg"], 1.0)
        self.assertGreater(
            float(np.dot(after - before, runtime.camera_relative_frame().left)),
            0.0,
        )

    def test_one_action_is_solved_once_as_an_absolute_grasp_target(self):
        runtime, controller = self.make_runtime()
        before = runtime.model.tcp(runtime.positions).copy()
        frame = runtime.camera_relative_frame()

        with patch.object(
            runtime,
            "plan_grasp_target",
            wraps=runtime.plan_grasp_target,
        ) as planner:
            result = asyncio.run(
                execute_terminal_motion(
                    runtime,
                    direction="forward",
                    speed_cm_s=1.0,
                    duration_s=2.0,
                    real_time=False,
                )
            )

        planner.assert_called_once()
        expected_target = before + frame.forward * 0.02
        expected_target[2] = before[2]
        np.testing.assert_allclose(
            result["target_grasp_point_m"], expected_target, atol=1e-12
        )
        self.assertLessEqual(len(controller.move_calls), 4)
        self.assertTrue(
            all(run_time_ms == 2000 for _sid, _target, run_time_ms in controller.move_calls)
        )

    def test_feedback_error_does_not_replace_session_pose_reference(self):
        runtime, controller = self.make_runtime()
        reference = runtime.camera_pose_reference_status()

        asyncio.run(
            execute_terminal_motion(
                runtime,
                direction="left",
                speed_cm_s=1.0,
                duration_s=1.0,
                real_time=False,
            )
        )
        # Simulate a physical J4 tracking error before the next action.
        j4_id = self.settings.servo_id("J4")
        controller.positions[j4_id] = max(
            self.settings.position_limits("J4")[0],
            controller.positions[j4_id] - 25,
        )
        feedback_positions = {
            name: controller.read_position(self.settings.servo_id(name))
            for name in ("J1", "J2", "J3", "J4")
        }
        expected_feedback_tcp = runtime.model.tcp(feedback_positions)

        result = asyncio.run(
            execute_terminal_motion(
                runtime,
                direction="right",
                speed_cm_s=1.0,
                duration_s=1.0,
                real_time=False,
            )
        )

        self.assertAlmostEqual(
            result["target_camera_line_angle_deg"],
            reference["camera_line_angle_from_vertical_deg"],
            places=9,
        )
        np.testing.assert_allclose(
            result["start_grasp_point_m"], expected_feedback_tcp, atol=1e-12
        )
        self.assertLess(
            abs(
                result["actual_camera_line_angle_deg"]
                - reference["camera_line_angle_from_vertical_deg"]
            ),
            1.0,
        )
        self.assertEqual(runtime.camera_pose_reference_status(), reference)

    def test_real_time_feedback_correction_uses_final_joint_feedback(self):
        class OneCycleLagController(DryRunServoController):
            def __init__(self, settings):
                super().__init__(settings)
                self.attempts = {}

            def move_servo(self, servo_id, target_position, run_time_ms):
                self.move_calls.append((servo_id, target_position, run_time_ms))
                self.attempts[servo_id] = self.attempts.get(servo_id, 0) + 1
                if self.attempts[servo_id] >= 2:
                    self.positions[servo_id] = target_position

        async def no_sleep(_duration):
            return None

        controller = OneCycleLagController(self.settings)
        runtime = CameraVectorV2Runtime(
            controller,
            self.settings,
            camera_config=self.camera_config,
        )
        runtime.initialize(use_home_positions=True)

        with patch.object(
            runtime,
            "plan_grasp_target",
            wraps=runtime.plan_grasp_target,
        ) as planner:
            result = asyncio.run(
                execute_terminal_motion(
                    runtime,
                    direction="forward",
                    speed_cm_s=1.0,
                    duration_s=1.0,
                    real_time=True,
                    sleep=no_sleep,
                )
            )

        self.assertEqual(result["status"], "ok")
        self.assertEqual(result["feedback_corrections"], 1)
        self.assertEqual(planner.call_count, 2)
        np.testing.assert_allclose(
            planner.call_args_list[0].args[0],
            planner.call_args_list[1].args[0],
            atol=1e-12,
        )
        np.testing.assert_allclose(
            planner.call_args_list[0].args[0],
            result["target_grasp_point_m"],
            atol=1e-12,
        )
        self.assertAlmostEqual(
            result["actual_camera_line_angle_deg"],
            runtime.camera_line_vertical_angle_deg(),
            places=9,
        )
        self.assertLess(result["position_error_m"], 0.004)

    def test_persistent_feedback_error_stops_after_bounded_corrections(self):
        class PersistentLagController(DryRunServoController):
            def __init__(self, settings):
                super().__init__(settings)
                self.attempts = {}

            def move_servo(self, servo_id, target_position, run_time_ms):
                self.move_calls.append((servo_id, target_position, run_time_ms))
                self.attempts[servo_id] = self.attempts.get(servo_id, 0) + 1

        async def no_sleep(_duration):
            return None

        controller = PersistentLagController(self.settings)
        runtime = CameraVectorV2Runtime(
            controller,
            self.settings,
            camera_config=self.camera_config,
        )
        runtime.initialize(use_home_positions=True)

        result = asyncio.run(
            execute_terminal_motion(
                runtime,
                direction="forward",
                speed_cm_s=1.0,
                duration_s=1.0,
                real_time=True,
                sleep=no_sleep,
            )
        )

        self.assertEqual(result["status"], "error")
        self.assertEqual(result["feedback_corrections"], 2)
        self.assertGreater(result["position_error_m"], 0.004)

    def test_strict_feedback_correction_never_executes_relaxed_plan(self):
        class PersistentLagController(DryRunServoController):
            def __init__(self, settings):
                super().__init__(settings)
                self.attempts = {}

            def move_servo(self, servo_id, target_position, run_time_ms):
                self.move_calls.append((servo_id, target_position, run_time_ms))
                self.attempts[servo_id] = self.attempts.get(servo_id, 0) + 1

        async def no_sleep(_duration):
            return None

        controller = PersistentLagController(self.settings)
        runtime = CameraVectorV2Runtime(
            controller,
            self.settings,
            camera_config=self.camera_config,
        )
        runtime.initialize(use_home_positions=True)
        original_planner = runtime.plan_grasp_target
        plan_calls = 0

        def strict_then_relaxed(target_tcp):
            nonlocal plan_calls
            plan_calls += 1
            if plan_calls == 1:
                return original_planner(target_tcp)
            return {
                "accepted": True,
                "strict_pose": False,
                "target_joint_positions": {
                    **runtime.positions,
                    "J2": runtime.positions["J2"] + 1,
                },
                "target_camera_line_angle_deg": (
                    runtime.camera_pose_reference_status()[
                        "camera_line_angle_from_vertical_deg"
                    ]
                ),
            }

        with patch.object(
            runtime,
            "plan_grasp_target",
            side_effect=strict_then_relaxed,
        ):
            result = asyncio.run(
                execute_terminal_motion(
                    runtime,
                    direction="forward",
                    speed_cm_s=1.0,
                    duration_s=1.0,
                    real_time=True,
                    sleep=no_sleep,
                )
            )

        self.assertEqual(result["status"], "error")
        self.assertEqual(result["feedback_corrections"], 0)
        self.assertTrue(all(count == 1 for count in controller.attempts.values()))

    def test_planner_exception_releases_terminal_input(self):
        runtime, _controller = self.make_runtime()

        with patch.object(
            runtime,
            "plan_grasp_target",
            side_effect=RuntimeError("forced planner failure"),
        ):
            with self.assertRaisesRegex(RuntimeError, "forced planner failure"):
                asyncio.run(
                    execute_terminal_motion(
                        runtime,
                        direction="left",
                        speed_cm_s=1.0,
                        duration_s=1.0,
                        real_time=False,
                    )
                )

        self.assertEqual(runtime.joystick_x, 0.0)
        self.assertEqual(runtime.joystick_y, 0.0)
        self.assertEqual(runtime.vertical_direction, 0.0)

    def test_motion_start_feedback_exception_cannot_latch_input(self):
        runtime, _controller = self.make_runtime()

        with patch.object(
            runtime,
            "refresh_joint_positions_from_controller",
            side_effect=RuntimeError("forced feedback failure"),
        ):
            with self.assertRaisesRegex(RuntimeError, "forced feedback failure"):
                asyncio.run(
                    execute_terminal_motion(
                        runtime,
                        direction="left",
                        speed_cm_s=1.0,
                        duration_s=1.0,
                        real_time=False,
                    )
                )

        self.assertEqual(runtime.joystick_x, 0.0)
        self.assertEqual(runtime.joystick_y, 0.0)
        self.assertEqual(runtime.vertical_direction, 0.0)
        self.assertIsNone(runtime._locked_frame)
        self.assertIsNone(runtime._last_interactive_target_tcp)

    def test_horizontal_forward_holds_height_and_line_inclination(self):
        runtime, _controller = self.make_runtime()
        frame = runtime.camera_relative_frame()
        before_tcp = runtime.model.tcp(runtime.positions)
        before_angle = runtime._camera_line_vertical_angle_rad(runtime.positions)

        runtime.set_joystick(0.0, -1.0)
        self.assertTrue(runtime.step_cartesian(0.08))

        after_tcp = runtime.model.tcp(runtime.positions)
        after_angle = runtime._camera_line_vertical_angle_rad(runtime.positions)
        displacement = after_tcp - before_tcp
        self.assertGreater(float(np.dot(displacement, frame.forward)), 0.0)
        self.assertLess(abs(float(displacement[2])), 0.0015)
        self.assertLess(abs(after_angle - before_angle), math.radians(1.0))

    def test_horizontal_left_holds_height_and_line_inclination(self):
        runtime, _controller = self.make_runtime()
        frame = runtime.camera_relative_frame()
        before_tcp = runtime.model.tcp(runtime.positions)
        before_angle = runtime._camera_line_vertical_angle_rad(runtime.positions)

        runtime.set_joystick(-1.0, 0.0)
        self.assertTrue(runtime.step_cartesian(0.08))

        after_tcp = runtime.model.tcp(runtime.positions)
        after_angle = runtime._camera_line_vertical_angle_rad(runtime.positions)
        displacement = after_tcp - before_tcp
        self.assertGreater(float(np.dot(displacement, frame.left)), 0.0)
        self.assertLess(abs(float(displacement[2])), 0.0015)
        self.assertLess(abs(after_angle - before_angle), math.radians(1.0))

    def test_up_follows_grasp_to_camera_and_holds_inclination(self):
        runtime, _controller = self.make_runtime()
        frame = runtime.camera_relative_frame()
        before_tcp = runtime.model.tcp(runtime.positions)
        before_angle = runtime._camera_line_vertical_angle_rad(runtime.positions)

        runtime.set_vertical_direction(1.0)
        self.assertTrue(runtime.step_cartesian(0.08))

        after_tcp = runtime.model.tcp(runtime.positions)
        after_angle = runtime._camera_line_vertical_angle_rad(runtime.positions)
        displacement = after_tcp - before_tcp
        self.assertGreater(float(np.dot(displacement, frame.up)), 0.0)
        self.assertLess(abs(after_angle - before_angle), math.radians(1.0))

    def test_all_six_controls_move_in_declared_direction(self):
        controls = (
            ("up", lambda runtime: runtime.set_vertical_direction(1.0), "up", False),
            ("down", lambda runtime: runtime.set_vertical_direction(-1.0), "down", False),
            ("forward", lambda runtime: runtime.set_joystick(0.0, -1.0), "forward", True),
            ("backward", lambda runtime: runtime.set_joystick(0.0, 1.0), "backward", True),
            ("left", lambda runtime: runtime.set_joystick(-1.0, 0.0), "left", True),
            ("right", lambda runtime: runtime.set_joystick(1.0, 0.0), "right", True),
        )
        for label, activate, direction_name, horizontal in controls:
            with self.subTest(direction=label):
                runtime, _controller = self.make_runtime()
                frame = runtime.camera_relative_frame()
                expected_direction = getattr(frame, direction_name)
                before_tcp = runtime.model.tcp(runtime.positions)
                before_angle = runtime._camera_line_vertical_angle_rad(runtime.positions)

                activate(runtime)
                self.assertTrue(runtime.step_cartesian(0.08))

                after_tcp = runtime.model.tcp(runtime.positions)
                after_angle = runtime._camera_line_vertical_angle_rad(runtime.positions)
                displacement = after_tcp - before_tcp
                self.assertGreater(float(np.dot(displacement, expected_direction)), 0.0)
                self.assertLess(abs(after_angle - before_angle), math.radians(1.0))
                if horizontal:
                    self.assertLess(abs(float(displacement[2])), 0.0015)

    def test_repeated_horizontal_motion_does_not_accumulate_height_drift(self):
        runtime, _controller = self.make_runtime()
        before_tcp = runtime.model.tcp(runtime.positions)
        before_angle = runtime._camera_line_vertical_angle_rad(runtime.positions)

        runtime.set_joystick(0.0, -0.5)
        for _index in range(8):
            self.assertTrue(runtime.step_cartesian(0.08))

        after_tcp = runtime.model.tcp(runtime.positions)
        after_angle = runtime._camera_line_vertical_angle_rad(runtime.positions)
        self.assertLess(abs(float(after_tcp[2] - before_tcp[2])), 0.0015)
        self.assertLess(abs(after_angle - before_angle), math.radians(1.0))

    def test_best_pose_rank_prefers_joint_continuity(self):
        runtime, _controller = self.make_runtime()
        near = dict(runtime.positions)
        far = dict(runtime.positions)
        for name in ("J1", "J2", "J3", "J4"):
            low, high = self.settings.position_limits(name)
            near[name] = min(high, near[name] + 2)
            far[name] = high if abs(high - far[name]) >= abs(far[name] - low) else low

        self.assertLess(
            runtime._joint_continuity_rank(near),
            runtime._joint_continuity_rank(far),
        )

    def test_gui_release_correction_keeps_last_absolute_target(self):
        runtime, controller = self.make_runtime()
        initial_feedback = dict(controller.positions)
        reference = runtime.camera_pose_reference_status()
        runtime.set_joystick(0.0, -0.5)
        self.assertTrue(runtime.step_cartesian(0.4))
        last_target = runtime._last_interactive_target_tcp.copy()
        runtime.center_joystick()
        controller.positions.update(initial_feedback)

        with patch.object(
            runtime,
            "plan_grasp_target",
            wraps=runtime.plan_grasp_target,
        ) as planner:
            correcting = runtime.finish_interactive_motion()
            with self.assertRaisesRegex(RuntimeError, "correction is still running"):
                runtime.set_joystick(0.0, -0.5)
            result = runtime.finish_interactive_motion()

        self.assertEqual(correcting["status"], "correcting")
        self.assertEqual(result["status"], "ok")
        self.assertTrue(result["verified"])
        self.assertTrue(result["corrected"])
        planner.assert_called_once()
        np.testing.assert_allclose(
            planner.call_args.args[0], last_target, atol=1e-12
        )
        self.assertAlmostEqual(
            correcting["motion_plan"]["target_camera_line_angle_deg"],
            reference["camera_line_angle_from_vertical_deg"],
            places=9,
        )
        self.assertIsNone(runtime._last_interactive_target_tcp)

    def test_near_vertical_line_does_not_guess_horizontal_direction(self):
        with self.assertRaises(HorizontalDirectionUndefined):
            _unit(np.array((1e-8, -1e-8, 0.0), dtype=float))

    def test_gui_down_refuses_unverified_relaxed_pose(self):
        runtime, _controller = self.make_runtime()
        before = dict(runtime.positions)
        runtime.set_vertical_direction(-1.0)

        with patch.object(runtime, "_analytic_pose_candidates", return_value=[]):
            self.assertTrue(runtime.step_cartesian(0.08))

        status = runtime.pose_relaxation_status()
        self.assertEqual(runtime.positions, before)
        self.assertFalse(status["relaxed"])
        self.assertTrue(status["relaxed_candidate_planned"])
        self.assertEqual(status["mode"], "strict")
        self.assertEqual(
            status["reason_code"],
            "strict_pose_unreachable_or_joint_limit",
        )
        self.assertGreater(status["relaxed_step_count"], 0)

    def test_up_does_not_relax_inclination_when_strict_pose_is_unreachable(self):
        runtime, _controller = self.make_runtime()
        before = dict(runtime.positions)
        runtime.set_vertical_direction(1.0)

        with patch.object(runtime, "_analytic_pose_candidates", return_value=[]):
            self.assertTrue(runtime.step_cartesian(0.08))

        status = runtime.pose_relaxation_status()
        self.assertEqual(runtime.positions, before)
        self.assertFalse(status["relaxed"])
        self.assertEqual(status["mode"], "strict")

    def test_home_configuration_keeps_j6_out_of_normal_home(self):
        self.assertEqual(
            self.settings.home,
            {"J1": 500, "J2": 410, "J3": 800, "J4": 800, "J5": 500},
        )
        self.assertNotIn("J6", self.settings.home)

    def test_initialize_button_action_homes_arm_and_positions_j6(self):
        runtime, controller = self.make_runtime()
        runtime.toggle_grip_lock()
        controller.move_calls.clear()

        runtime.initialize_home_pose()

        self.assertFalse(runtime.j6_grip_locked)
        self.assertEqual(controller.servo_mode_calls[-1], self.settings.servo_id("J6"))
        self.assertEqual(
            controller.move_calls,
            [
                (1, 500, 1200),
                (2, 410, 1200),
                (3, 800, 1200),
                (4, 800, 1200),
                (5, 500, 1200),
                (10, INITIAL_J6_POSITION, 1200),
            ],
        )


if __name__ == "__main__":
    unittest.main()
