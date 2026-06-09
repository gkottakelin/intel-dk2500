import unittest

from project.src.joint_controller import (
    DEFAULT_CONFIG_PATH,
    JointAngleError,
    JointCommandExecutor,
    JointCommandPlanner,
    JointRangeError,
    MotorCommand,
    ServoMoveCommand,
    load_joint_config,
    main,
)


class FakeController:
    def __init__(self, positions=None):
        self.positions = {int(k): int(v) for k, v in (positions or {}).items()}
        self.position_sequences = {}
        self.move_calls = []
        self.motor_calls = []

    def read_position(self, servo_id):
        sequence = self.position_sequences.get(servo_id)
        if sequence:
            value = sequence.pop(0)
            self.positions[servo_id] = value
            return value
        return self.positions.get(servo_id, 500)

    def move_servo(self, servo_id, target_position, run_time_ms):
        self.move_calls.append((servo_id, target_position, run_time_ms))
        self.positions[servo_id] = target_position

    def set_motor_speed(self, servo_id, speed):
        self.motor_calls.append((servo_id, speed))


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def sleep(self, duration):
        self.now += duration


class JointControllerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = load_joint_config(DEFAULT_CONFIG_PATH)

    def setUp(self):
        self.planner = JointCommandPlanner(self.config)

    def test_loads_current_yaml_without_pyyaml(self):
        self.assertIn("metadata", self.config)
        self.assertEqual(self.config["metadata"]["default_com_port"], "COM15")
        self.assertEqual(self.config["joints"]["joint1_base_yaw"]["servo_id"], 1)
        self.assertEqual(self.config["joints"]["joint5_wrist_roll"]["control_mode"], "motor")

    def test_j2_segmented_run_time_profile(self):
        command = self.planner.plan_target("J2", 875, current_positions={"j2": 500})
        small = self.planner.plan_target("J2", 520, current_positions={"j2": 500})
        micro = self.planner.plan_target("J2", 508, current_positions={"j2": 500})

        self.assertIsInstance(command, ServoMoveCommand)
        self.assertEqual(command.servo_id, 2)
        self.assertEqual(command.target_position, 875)
        self.assertEqual(command.run_time_ms, 3750)
        self.assertEqual(small.run_time_ms, 4)
        self.assertEqual(micro.run_time_ms, 1)

    def test_j1_j3_j4_servo_run_time_rule(self):
        large = self.planner.plan_target("J1", 540, current_positions={"j1": 500})
        small = self.planner.plan_target("J3", 520, current_positions={"j3": 500})
        boundary = self.planner.plan_target("J4", 530, current_positions={"j4": 500})

        self.assertEqual(large.run_time_ms, 400)
        self.assertEqual(small.run_time_ms, 20)
        self.assertEqual(boundary.run_time_ms, 300)

    def test_rejects_out_of_range_targets(self):
        with self.assertRaises(JointRangeError):
            self.planner.plan_target("J2", 499, current_positions={"j2": 500})
        with self.assertRaises(JointRangeError):
            self.planner.plan_target("J5", 800, current_positions={"j5": 500})

    def test_maps_joint_angle_degrees_to_servo_position(self):
        j1 = self.planner.plan_angle_target("J1", 12.0, current_positions={"j1": 500})
        j2 = self.planner.plan_angle_target("J2", 90.0, current_positions={"j2": 500})
        j4 = self.planner.plan_angle_target("J4", 0.0, current_positions={"j4": 500})
        j5 = self.planner.plan_angle_target("J5", -60.0, current_positions={"j5": 500})

        self.assertEqual(j1.target_position, 550)
        self.assertEqual(j2.target_position, 875)
        self.assertEqual(j4.target_position, 500)
        self.assertEqual(j5.target_position, 250)

    def test_maps_joint_angle_radians_to_servo_position(self):
        command = self.planner.plan_radian_target("J1", 0.0, current_positions={"j1": 500})

        self.assertEqual(command.target_position, 500)

    def test_rejects_joint_angle_without_mapping_or_outside_range(self):
        with self.assertRaises(JointAngleError):
            self.planner.plan_angle_target("J6", 10.0, current_positions={"j6": 500})
        with self.assertRaises(JointAngleError):
            self.planner.plan_angle_target("J2", 91.0, current_positions={"j2": 500})

    def test_j5_motor_direction_selection(self):
        positive = self.planner.plan_target("J5", 650, current_positions={"j5": 500})
        negative = self.planner.plan_target("joint5_wrist_roll", 350, current_positions={"joint5_wrist_roll": 500})
        same = self.planner.plan_target("wrist_roll", 500, current_positions={"joint5_wrist_roll": 500})

        self.assertIsInstance(positive, MotorCommand)
        self.assertEqual(positive.speed, 100)
        self.assertEqual(negative.speed, -100)
        self.assertEqual(same.speed, 0)

    def test_j6_gripper_position_range_and_release_direction(self):
        close_force = self.planner.plan_target("J6", 1000, current_positions={"j6": 700})
        release = self.planner.plan_target("J6", 0, current_positions={"j6": 700})

        self.assertIsInstance(close_force, MotorCommand)
        self.assertEqual(close_force.speed, 100)
        self.assertEqual(close_force.target_position, 1000)
        self.assertEqual(release.speed, -100)
        self.assertEqual(release.target_position, 0)

    def test_executor_calls_fake_controller(self):
        controller = FakeController({1: 500, 5: 500})
        controller.position_sequences[5] = [520, 550, 595, 645]
        clock = FakeClock()
        executor = JointCommandExecutor(controller, monotonic=clock.monotonic, sleep=clock.sleep, poll_interval_s=0.1)

        commands = [
            ServoMoveCommand("joint1_base_yaw", 1, 540, 400),
            MotorCommand("joint5_wrist_roll", 5, 100, 650, stop_tolerance=5, timeout_s=2.0),
        ]
        executor.execute(commands)

        self.assertEqual(controller.move_calls, [(1, 540, 400)])
        self.assertEqual(controller.motor_calls[0], (5, 100))
        self.assertEqual(controller.motor_calls[-1], (5, 0))

    def test_dry_run_cli_does_not_open_hardware(self):
        rc = main(["--joint", "J1=520"])
        self.assertEqual(rc, 0)

    def test_dry_run_cli_accepts_angle_targets(self):
        rc = main(["--joint-deg", "J1=12", "--current", "J1=500"])
        self.assertEqual(rc, 0)


if __name__ == "__main__":
    unittest.main()
