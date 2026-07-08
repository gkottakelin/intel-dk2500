import unittest

from project.src.arm_model import JetArmKinematicModel
from project.src.joint_controller import DEFAULT_CONFIG_PATH, load_joint_config
from project.src.operation_terminal import (
    DryRunServoController,
    ManualServoRuntime,
    OperationTerminalConfig,
)


class ManualServoRuntimeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config_data = load_joint_config(DEFAULT_CONFIG_PATH)
        cls.model = JetArmKinematicModel(cls.config_data)

    def make_runtime(self):
        terminal_config = OperationTerminalConfig()
        positions = {
            int(self.model.joints[joint_name]["servo_id"]): position
            for joint_name, position in terminal_config.home_positions.items()
        }
        controller = DryRunServoController(positions)
        runtime = ManualServoRuntime(controller, self.model, terminal_config)
        runtime.initialize(use_home_positions=True)
        return runtime, controller

    def test_j5_hold_controls_speed_and_release_stops(self):
        runtime, controller = self.make_runtime()

        runtime.rotate_j5_counterclockwise()
        runtime.stop_j5()
        runtime.rotate_j5_clockwise()
        runtime.stop_j5()

        self.assertEqual(controller.motor_calls, [(5, -100), (5, 0), (5, 100), (5, 0)])

    def test_j6_grip_lock_ignores_open_close_until_toggled_off(self):
        runtime, controller = self.make_runtime()

        self.assertTrue(runtime.toggle_grip_lock())
        self.assertEqual(controller.motor_calls[-1], (10, 300))
        before_ignored = list(controller.motor_calls)

        self.assertFalse(runtime.open_j6())
        self.assertFalse(runtime.close_j6())
        self.assertFalse(runtime.stop_j6())
        self.assertEqual(controller.motor_calls, before_ignored)

        self.assertFalse(runtime.toggle_grip_lock())
        self.assertEqual(controller.motor_calls[-1], (10, 0))
        self.assertTrue(runtime.open_j6())
        self.assertTrue(runtime.stop_j6())
        self.assertEqual(controller.motor_calls[-2:], [(10, -100), (10, 0)])

    def test_joystick_maps_screen_directions_to_robot_velocity(self):
        runtime, _controller = self.make_runtime()

        runtime.set_joystick(0, -1)
        self.assertAlmostEqual(runtime.cartesian_velocity()[0], 0.05)
        self.assertAlmostEqual(runtime.cartesian_velocity()[1], 0.0)

        runtime.set_joystick(1, 0)
        self.assertAlmostEqual(runtime.cartesian_velocity()[0], 0.0)
        self.assertAlmostEqual(runtime.cartesian_velocity()[1], -0.05)

        runtime.set_vertical_direction(-1)
        self.assertAlmostEqual(runtime.cartesian_velocity()[2], -0.05)

    def test_cartesian_step_sends_short_position_commands(self):
        runtime, controller = self.make_runtime()

        runtime.set_joystick(0, -1)
        moved = runtime.step_cartesian(0.08)

        self.assertTrue(moved)
        self.assertTrue(controller.move_calls)
        self.assertTrue(all(1 <= servo_id <= 4 for servo_id, _target, _run_time in controller.move_calls))
        self.assertTrue(all(run_time == 80 for _servo_id, _target, run_time in controller.move_calls))

    def test_stop_all_clears_grip_lock_and_stops_j5_j6(self):
        runtime, controller = self.make_runtime()

        runtime.toggle_grip_lock()
        runtime.stop_all()

        self.assertFalse(runtime.j6_grip_locked)
        self.assertEqual(controller.motor_calls[-2:], [(5, 0), (10, 0)])

    def test_go_home_leaves_j6_unchanged(self):
        runtime, controller = self.make_runtime()

        runtime.toggle_grip_lock()
        runtime.go_home()

        self.assertTrue(runtime.j6_grip_locked)
        self.assertEqual(controller.motor_calls[-2:], [(10, 300), (5, 0)])
        self.assertEqual(
            controller.move_calls[-5:],
            [(1, 485, 1200), (2, 478, 1200), (3, 641, 1200), (4, 890, 1200), (5, 500, 1200)],
        )
        self.assertNotIn((10, 360, 1200), controller.move_calls)


if __name__ == "__main__":
    unittest.main()
