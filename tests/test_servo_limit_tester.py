import unittest

from project.src.servo_limit_tester import (
    PositionStabilityTracker,
    ServoLimitTester,
    default_speed,
)


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def sleep(self, duration):
        self.now += duration


class FakeController:
    def __init__(self, positions):
        self.positions = list(positions)
        self.last_position = self.positions[-1] if self.positions else 500
        self.motor_commands = []
        self.servo_mode_calls = []

    def read_position(self, servo_id):
        if self.positions:
            self.last_position = self.positions.pop(0)
        return self.last_position

    def set_motor_speed(self, servo_id, speed, *, fixed_speed_mode=False):
        self.motor_commands.append((servo_id, speed, fixed_speed_mode))

    def set_servo_mode(self, servo_id):
        self.servo_mode_calls.append(servo_id)


class ServoLimitTesterTest(unittest.TestCase):
    def make_tester(self, controller, clock, **kwargs):
        options = {
            "speed": 25,
            "stable_duration": 2.0,
            "stable_tolerance": 2,
            "poll_interval": 0.5,
            "max_duration": 10.0,
            "progress": None,
            "monotonic": clock.monotonic,
            "sleep": clock.sleep,
        }
        options.update(kwargs)
        return ServoLimitTester(controller, 1, **options)

    def test_stable_detection_after_two_seconds(self):
        clock = FakeClock()
        controller = FakeController([500, 500, 500, 500, 500, 500])
        tester = self.make_tester(controller, clock)

        result = tester.run_direction("positive")

        self.assertFalse(result.timed_out)
        self.assertEqual(result.reason, "stable_position")
        self.assertEqual(result.limit_position, 500)
        self.assertGreaterEqual(result.stable_s, 2.0)
        self.assertEqual(controller.motor_commands[0], (1, 25, False))
        self.assertEqual(controller.motor_commands[-1], (1, 0, False))
        self.assertEqual(controller.servo_mode_calls, [1])

    def test_movement_resets_stability_timer(self):
        clock = FakeClock()
        controller = FakeController([500, 500, 520, 520, 520, 520, 520])
        tester = self.make_tester(
            controller,
            clock,
            stable_duration=1.0,
            stable_tolerance=2,
            poll_interval=0.5,
        )

        result = tester.run_direction("positive")

        self.assertFalse(result.timed_out)
        self.assertEqual(result.limit_position, 520)
        self.assertGreaterEqual(result.elapsed_s, 1.5)

    def test_negative_direction_uses_negative_speed(self):
        clock = FakeClock()
        controller = FakeController([500, 500, 500, 500, 500, 500])
        tester = self.make_tester(controller, clock, speed=33)

        result = tester.run_direction("negative")

        self.assertEqual(result.signed_speed, -33)
        self.assertEqual(controller.motor_commands[0], (1, -33, False))
        self.assertEqual(controller.motor_commands[-1], (1, 0, False))

    def test_stop_is_called_on_timeout(self):
        clock = FakeClock()
        controller = FakeController([500, 510, 520, 530, 540, 550])
        tester = self.make_tester(
            controller,
            clock,
            stable_duration=2.0,
            stable_tolerance=0,
            poll_interval=0.5,
            max_duration=0.75,
        )

        result = tester.run_direction("positive")

        self.assertTrue(result.timed_out)
        self.assertEqual(result.reason, "max_duration")
        self.assertEqual(controller.motor_commands[0], (1, 25, False))
        self.assertEqual(controller.motor_commands[-1], (1, 0, False))
        self.assertEqual(controller.servo_mode_calls, [1])

    def test_position_stability_tracker_resets_on_motion(self):
        tracker = PositionStabilityTracker(500, 0.0, tolerance=2)

        self.assertEqual(tracker.update(501, 1.0), 1.0)
        self.assertEqual(tracker.update(506, 1.5), 0.0)
        self.assertEqual(tracker.reference_position, 506)
        self.assertEqual(tracker.update(507, 2.0), 0.5)

    def test_default_speed_depends_on_mode(self):
        self.assertEqual(default_speed(False, None), 80)
        self.assertEqual(default_speed(True, None), 10)
        self.assertEqual(default_speed(True, 12), 12)


if __name__ == "__main__":
    unittest.main()
