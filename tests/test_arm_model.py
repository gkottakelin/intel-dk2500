import io
import math
import unittest
from contextlib import redirect_stdout

from project.src.joint_controller import JointRangeError
from project.src.arm_model import DEFAULT_CONFIG_PATH, JetArmKinematicModel, main


def assert_vector_close(testcase, actual, expected, places=9):
    testcase.assertEqual(len(actual), len(expected))
    for actual_value, expected_value in zip(actual, expected):
        testcase.assertAlmostEqual(actual_value, expected_value, places=places)


class ArmModelTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = JetArmKinematicModel.from_config_path(DEFAULT_CONFIG_PATH)

    def test_home_pose_matches_measured_grasp_point(self):
        result = self.model.forward_kinematics_from_positions(
            {"J1": 500, "J2": 500, "J3": 500, "J4": 500, "J5": 500, "J6": 0}
        )

        assert_vector_close(self, result.tcp_xyz, (0.0, 0.0, 0.527))
        self.assertEqual(result.gripper_position, 0)

    def test_raw_position_to_model_angle_uses_direction_sign(self):
        self.assertAlmostEqual(math.degrees(self.model.position_to_model_angle_rad("J1", 1000)), -120.0)
        self.assertAlmostEqual(math.degrees(self.model.position_to_model_angle_rad("J2", 875)), 90.0)
        self.assertEqual(self.model.model_angle_rad_to_servo_position("J1", math.radians(-120.0)), 1000)

    def test_j1_raw_increase_is_clockwise_in_model(self):
        left = self.model.forward_kinematics_from_positions({"J1": 0, "J2": 875})
        right = self.model.forward_kinematics_from_positions({"J1": 1000, "J2": 875})

        self.assertGreater(left.tcp_xyz[1], 0.0)
        self.assertLess(right.tcp_xyz[1], 0.0)

    def test_j2_position_increase_lowers_tcp_and_moves_forward(self):
        home = self.model.forward_kinematics_from_positions({"J2": 500})
        lowered = self.model.forward_kinematics_from_positions({"J2": 875})

        self.assertLess(lowered.tcp_xyz[2], home.tcp_xyz[2])
        self.assertGreater(lowered.tcp_xyz[0], home.tcp_xyz[0])

    def test_j5_roll_changes_orientation_not_tcp_position(self):
        roll_negative = self.model.forward_kinematics_from_positions({"J5": 250})
        roll_positive = self.model.forward_kinematics_from_positions({"J5": 750})

        assert_vector_close(self, roll_negative.tcp_xyz, roll_positive.tcp_xyz)
        self.assertNotEqual(roll_negative.rotation_matrix, roll_positive.rotation_matrix)

    def test_rejects_positions_outside_yaml_limits(self):
        with self.assertRaises(JointRangeError):
            self.model.forward_kinematics_from_positions({"J6": -1})

    def test_allows_gripper_force_increase_range(self):
        result = self.model.forward_kinematics_from_positions({"J6": 1000})

        assert_vector_close(self, result.tcp_xyz, (0.0, 0.0, 0.527))
        self.assertEqual(result.gripper_position, 1000)

    def test_forward_kinematics_from_model_angles(self):
        result = self.model.forward_kinematics_from_model_angles_deg({"J1": 90, "J2": 90})

        self.assertAlmostEqual(result.tcp_xyz[0], 0.0, places=9)
        self.assertGreater(result.tcp_xyz[1], 0.0)
        self.assertAlmostEqual(result.tcp_xyz[2], 0.105, places=9)

    def test_numeric_jacobian_shape_and_j5_position_column(self):
        jacobian = self.model.tcp_jacobian_from_positions({"J1": 500, "J2": 500, "J3": 500, "J4": 500, "J5": 500})

        self.assertEqual(len(jacobian), 3)
        self.assertTrue(all(len(row) == 5 for row in jacobian))
        j5_column = [row[4] for row in jacobian]
        assert_vector_close(self, j5_column, (0.0, 0.0, 0.0), places=7)

    def test_cli_dry_run_outputs_json_without_hardware(self):
        buffer = io.StringIO()
        with redirect_stdout(buffer):
            rc = main(
                [
                    "--position",
                    "J1=500",
                    "--position",
                    "J2=500",
                    "--position",
                    "J3=500",
                    "--position",
                    "J4=500",
                    "--position",
                    "J5=500",
                    "--position",
                    "J6=0",
                    "--json",
                ]
            )

        self.assertEqual(rc, 0)
        self.assertIn('"tcp_xyz"', buffer.getvalue())
        self.assertIn('"gripper_position": 0', buffer.getvalue())


if __name__ == "__main__":
    unittest.main()
