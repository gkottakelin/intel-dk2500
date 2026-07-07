import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace


APP_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP_ROOT))

from jetarm_terminal import (  # noqa: E402
    DryRunServoController,
    ManualServoRuntime,
    TerminalSettings,
    build_packet,
    discover_linux_serial_ports,
    discover_usb_serial_adapters,
    serial_discovery_diagnostic,
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

    def test_cartesian_step_can_use_a_longer_physical_run_time(self):
        runtime, controller = self.make_runtime()
        runtime.set_joystick(0, -1)
        self.assertTrue(runtime.step_cartesian(0.08, run_time_s=0.24))
        self.assertTrue(controller.move_calls)
        self.assertTrue(
            all(run_time_ms == 240 for _servo_id, _target, run_time_ms in controller.move_calls)
        )

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

    def test_discovers_nonstandard_device_reported_by_pyserial(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            port = root / "customSerial0"
            port.touch()
            provider = lambda: [
                SimpleNamespace(
                    device=str(port), vid=0x1A86, hwid="USB VID:PID=1A86:7523"
                )
            ]
            self.assertEqual(
                discover_linux_serial_ports(root, list_ports_provider=provider),
                [str(port)],
            )

    def test_ignores_legacy_ttys_reported_by_pyserial(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            legacy_port = root / "ttyS31"
            usb_port = root / "ttyUSB0"
            legacy_port.touch()
            usb_port.touch()
            provider = lambda: [
                SimpleNamespace(device=str(legacy_port), vid=None, hwid="PNP0501"),
                SimpleNamespace(
                    device=str(usb_port),
                    vid=0x1A86,
                    hwid="USB VID:PID=1A86:7523",
                ),
            ]
            self.assertEqual(
                discover_linux_serial_ports(root, list_ports_provider=provider),
                [str(usb_port)],
            )

    def test_detects_ch340_at_usb_layer_without_tty_node(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            device_root = root / "dev"
            sys_usb_root = root / "sys-usb"
            adapter = sys_usb_root / "1-1"
            device_root.mkdir()
            adapter.mkdir(parents=True)
            (adapter / "idVendor").write_text("1a86\n", encoding="utf-8")
            (adapter / "idProduct").write_text("7523\n", encoding="utf-8")
            (adapter / "manufacturer").write_text("QinHeng Electronics\n", encoding="utf-8")
            (adapter / "product").write_text("CH340 serial converter\n", encoding="utf-8")

            adapters = discover_usb_serial_adapters(sys_usb_root)
            self.assertEqual(len(adapters), 1)
            self.assertEqual(adapters[0].vendor_id, "1a86")
            diagnostic = serial_discovery_diagnostic(
                device_root=device_root,
                sys_usb_root=sys_usb_root,
                list_ports_provider=lambda: [],
            )
            self.assertIn("USB层已识别", diagnostic)
            self.assertIn("ch341", diagnostic)
            self.assertIn("brltty", diagnostic)

    def test_empty_discovery_reports_connection_checks(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            device_root = root / "dev"
            sys_usb_root = root / "sys-usb"
            device_root.mkdir()
            sys_usb_root.mkdir()
            diagnostic = serial_discovery_diagnostic(
                device_root=device_root,
                sys_usb_root=sys_usb_root,
                list_ports_provider=lambda: [],
            )
            self.assertIn("数据线", diagnostic)

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
