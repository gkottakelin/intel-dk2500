import sys
import unittest
from pathlib import Path

import numpy as np


APP_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP_ROOT))

from camera_vector_terminal import (  # noqa: E402
    CameraLineConfig,
    CameraRelativeFrame,
    CameraRelativeManualServoRuntime,
    _with_forward_continuity,
    build_camera_relative_frame,
    tk,
    ttk,
)
from jetarm_terminal import DryRunServoController, TerminalSettings  # noqa: E402


class CameraVectorTerminalTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.settings = TerminalSettings.from_file(APP_ROOT / "config" / "terminal.json")
        cls.camera_config = CameraLineConfig.from_file(APP_ROOT / "config" / "terminal.json")

    def make_runtime(self):
        controller = DryRunServoController(self.settings)
        runtime = CameraRelativeManualServoRuntime(
            controller,
            self.settings,
            camera_config=self.camera_config,
        )
        runtime.initialize(use_home_positions=True)
        return runtime, controller

    def assert_unit(self, vector):
        self.assertAlmostEqual(float(np.linalg.norm(vector)), 1.0, places=9)

    def test_terminal_reexports_tkinter_modules_for_device_config(self):
        self.assertTrue(hasattr(sys.modules["camera_vector_terminal"], "tk"))
        self.assertTrue(hasattr(sys.modules["camera_vector_terminal"], "ttk"))
        if tk is not None:
            self.assertIsNotNone(ttk)

    def test_camera_frame_is_orthonormal(self):
        runtime, _controller = self.make_runtime()

        frame = runtime.camera_relative_frame()

        self.assert_unit(frame.up)
        self.assert_unit(frame.forward)
        self.assert_unit(frame.left)
        self.assertAlmostEqual(float(np.dot(frame.up, frame.forward)), 0.0, places=9)
        self.assertAlmostEqual(float(np.dot(frame.up, frame.left)), 0.0, places=9)
        self.assertAlmostEqual(float(np.dot(frame.forward, frame.left)), 0.0, places=9)
        np.testing.assert_allclose(frame.down, -frame.up)
        np.testing.assert_allclose(frame.backward, -frame.forward)
        np.testing.assert_allclose(frame.right, -frame.left)

    def test_up_and_down_follow_grasp_camera_line(self):
        runtime, _controller = self.make_runtime()
        frame = runtime.camera_relative_frame()

        self.assertGreater(frame.up[2], 0.9)
        runtime.set_vertical_direction(1)
        up_velocity = runtime.cartesian_velocity()
        runtime.set_vertical_direction(-1)
        down_velocity = runtime.cartesian_velocity()

        np.testing.assert_allclose(up_velocity, frame.up * self.settings.vertical_speed_m_s)
        np.testing.assert_allclose(down_velocity, frame.down * self.settings.vertical_speed_m_s)

    def test_control_frame_locks_while_input_is_held(self):
        runtime, _controller = self.make_runtime()

        runtime.set_vertical_direction(1)
        held_velocity = runtime.cartesian_velocity()
        runtime.positions["J4"] -= 100
        still_held_velocity = runtime.cartesian_velocity()
        runtime.set_vertical_direction(0)
        released_frame = runtime.camera_relative_frame()

        np.testing.assert_allclose(still_held_velocity, held_velocity)
        self.assertFalse(np.allclose(released_frame.up, held_velocity / np.linalg.norm(held_velocity)))

    def test_forward_continuity_flips_reversed_plane(self):
        frame = CameraRelativeFrame(
            up=np.array((0.0, 0.0, 1.0), dtype=float),
            down=np.array((0.0, 0.0, -1.0), dtype=float),
            forward=np.array((-1.0, 0.0, 0.0), dtype=float),
            backward=np.array((1.0, 0.0, 0.0), dtype=float),
            left=np.array((0.0, 1.0, 0.0), dtype=float),
            right=np.array((0.0, -1.0, 0.0), dtype=float),
        )

        fixed = _with_forward_continuity(
            frame,
            np.array((1.0, 0.0, 0.0), dtype=float),
        )

        np.testing.assert_allclose(fixed.up, frame.up)
        np.testing.assert_allclose(fixed.forward, np.array((1.0, 0.0, 0.0)))
        np.testing.assert_allclose(fixed.backward, np.array((-1.0, 0.0, 0.0)))
        np.testing.assert_allclose(fixed.left, np.array((0.0, -1.0, 0.0)))
        np.testing.assert_allclose(fixed.right, np.array((0.0, 1.0, 0.0)))

    def test_motion_lock_uses_continuous_forward_direction(self):
        runtime, _controller = self.make_runtime()
        raw_frame = runtime.camera_relative_frame()
        runtime._last_forward = -raw_frame.forward.copy()

        runtime.set_joystick(0, -1)
        active_frame = runtime.active_camera_relative_frame()

        self.assertGreater(float(np.dot(active_frame.forward, -raw_frame.forward)), 0.99)
        self.assertLess(float(np.dot(active_frame.forward, raw_frame.forward)), -0.99)

    def test_line_angle_hold_keeps_camera_line_vertical_angle(self):
        runtime, _controller = self.make_runtime()
        before_angle = runtime._camera_line_vertical_angle_rad(runtime.positions)

        runtime.set_vertical_direction(1)
        for _index in range(5):
            self.assertTrue(runtime.step_cartesian(0.08))
        after_angle = runtime._camera_line_vertical_angle_rad(runtime.positions)

        self.assertLess(abs(after_angle - before_angle), 0.01)

    def test_joystick_moves_in_camera_plane(self):
        runtime, _controller = self.make_runtime()
        frame = runtime.camera_relative_frame()

        runtime.set_joystick(0, -1)
        forward_velocity = runtime.cartesian_velocity()
        runtime.set_joystick(1, 0)
        right_velocity = runtime.cartesian_velocity()

        np.testing.assert_allclose(
            forward_velocity,
            frame.forward * self.settings.max_horizontal_speed_m_s,
        )
        np.testing.assert_allclose(
            right_velocity,
            frame.right * self.settings.max_horizontal_speed_m_s,
        )
        self.assertAlmostEqual(float(np.dot(forward_velocity, frame.up)), 0.0, places=9)
        self.assertAlmostEqual(float(np.dot(right_velocity, frame.up)), 0.0, places=9)

    def test_step_cartesian_uses_camera_relative_velocity(self):
        runtime, controller = self.make_runtime()

        runtime.set_vertical_direction(1)
        moved = runtime.step_cartesian(0.08)

        self.assertTrue(moved)
        self.assertTrue(controller.move_calls)
        self.assertTrue(all(1 <= servo_id <= 4 for servo_id, _target, _time in controller.move_calls))

    def test_zero_camera_line_is_rejected(self):
        with self.assertRaises(ValueError):
            build_camera_relative_frame(
                self.settings,
                dict(self.settings.home),
                CameraLineConfig(
                    grasp_to_camera_along_tool_m=0.0,
                    grasp_to_camera_normal_m=0.0,
                    grasp_to_camera_lateral_m=0.0,
                ),
            )


if __name__ == "__main__":
    unittest.main()
