import sys
import tempfile
import unittest
from pathlib import Path


APP_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP_ROOT))

from jetarm_terminal import (  # noqa: E402
    DryRunServoController,
    ManualServoRuntime,
    TerminalSettings,
    build_packet,
    discover_linux_serial_ports,
    select_linux_serial_port,
)


class UbuntuTerminalTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.settings = TerminalSettings.from_file(APP_ROOT / "config" / "terminal.json")

    def make_runtime(self):
        controller = DryRunServoController(self.settings)
        runtime = ManualServoRuntime(controller, self.settings)
        runtime.initialize(use_home_positions=True)
        return runtime, controller

    def test_bus_packet_matches_servo_protocol(self):
        packet = build_packet(1, 29, b"\x01\x00\x64\x00")
        self.assertEqual(packet, bytes.fromhex("55 55 01 07 1D 01 00 64 00 75"))

    def test_j5_hold_and_release(self):
        runtime, controller = self.make_runtime()
        runtime.rotate_j5_counterclockwise()
        runtime.stop_j5()
        runtime.rotate_j5_clockwise()
        runtime.stop_j5()
        self.assertEqual(controller.motor_calls, [(5, -100), (5, 0), (5, 100), (5, 0)])

    def test_j6_grip_lock_ignores_other_j6_commands(self):
        runtime, controller = self.make_runtime()
        self.assertTrue(runtime.toggle_grip_lock())
        self.assertEqual(controller.motor_calls[-1], (10, 300))
        previous = list(controller.motor_calls)
        self.assertFalse(runtime.open_j6())
        self.assertFalse(runtime.close_j6())
        self.assertFalse(runtime.stop_j6())
        self.assertEqual(controller.motor_calls, previous)
        self.assertFalse(runtime.toggle_grip_lock())
        self.assertEqual(controller.motor_calls[-1], (10, 0))

    def test_joystick_and_vertical_velocity_mapping(self):
        runtime, _controller = self.make_runtime()
        runtime.set_joystick(0, -1)
        self.assertAlmostEqual(runtime.cartesian_velocity()[0], 0.05)
        runtime.set_joystick(1, 0)
        self.assertAlmostEqual(runtime.cartesian_velocity()[1], -0.05)
        runtime.set_vertical_direction(1)
        self.assertAlmostEqual(runtime.cartesian_velocity()[2], 0.05)

    def test_cartesian_step_sends_j1_to_j4_commands(self):
        runtime, controller = self.make_runtime()
        runtime.set_joystick(0, -1)
        self.assertTrue(runtime.step_cartesian(0.08))
        self.assertTrue(controller.move_calls)
        self.assertTrue(all(1 <= servo_id <= 4 for servo_id, _target, _time in controller.move_calls))

    def test_vertical_step_generates_pitch_joint_commands(self):
        runtime, controller = self.make_runtime()
        runtime.set_vertical_direction(1)
        self.assertTrue(runtime.step_cartesian(0.08))
        self.assertTrue(controller.move_calls)
        self.assertTrue(all(servo_id in (2, 3, 4) for servo_id, _target, _time in controller.move_calls))

    def test_discovers_linux_usb_serial_patterns(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "ttyUSB0").touch()
            (root / "ttyACM1").touch()
            self.assertEqual(
                discover_linux_serial_ports(root),
                [str(root / "ttyUSB0"), str(root / "ttyACM1")],
            )

    def test_selects_single_serial_port_and_rejects_multiple(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            first = root / "ttyUSB0"
            first.touch()
            selected = select_linux_serial_port(None, device_root=root, access_check=lambda _path, _mode: True)
            self.assertEqual(selected, str(first))
            (root / "ttyACM0").touch()
            with self.assertRaises(RuntimeError):
                select_linux_serial_port(None, device_root=root, access_check=lambda _path, _mode: True)


if __name__ == "__main__":
    unittest.main()
