import math
import sys
import unittest
from pathlib import Path

import numpy as np


APP_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP_ROOT))

from camera_vector_terminal import CameraLineConfig  # noqa: E402
from camera_vector_terminal_v2 import (  # noqa: E402
    CameraVectorV2Runtime,
    HorizontalDirectionUndefined,
    _unit,
    build_camera_vector_v2_frame,
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
        expected = frame.down.copy()
        expected[2] = 0.0
        expected /= np.linalg.norm(expected)

        np.testing.assert_allclose(frame.forward, expected)
        np.testing.assert_allclose(frame.backward, -expected)
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

    def test_near_vertical_line_does_not_guess_horizontal_direction(self):
        with self.assertRaises(HorizontalDirectionUndefined):
            _unit(np.array((1e-8, -1e-8, 0.0), dtype=float))


if __name__ == "__main__":
    unittest.main()
