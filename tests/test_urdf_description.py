import math
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from project.src.arm_model import JetArmKinematicModel, DEFAULT_CONFIG_PATH


DESCRIPTION_ROOT = Path(__file__).resolve().parents[1] / "ros2_ws" / "src" / "jetarm_description"
XACRO_PATH = DESCRIPTION_ROOT / "urdf" / "jetarm.urdf.xacro"
GUI_XACRO_PATH = DESCRIPTION_ROOT / "urdf" / "jetarm_gui.urdf.xacro"
PACKAGE_XML_PATH = DESCRIPTION_ROOT / "package.xml"
CMAKE_PATH = DESCRIPTION_ROOT / "CMakeLists.txt"
LAUNCH_PATH = DESCRIPTION_ROOT / "launch" / "display.launch.py"
RVIZ_PATH = DESCRIPTION_ROOT / "rviz" / "jetarm.rviz"
MAPPER_PATH = DESCRIPTION_ROOT / "scripts" / "joint_state_mapper.py"
XACRO_NS = "{http://www.ros.org/wiki/xacro}"


def assert_vector_close(testcase, actual, expected, places=9):
    testcase.assertEqual(len(actual), len(expected))
    for actual_value, expected_value in zip(actual, expected):
        testcase.assertAlmostEqual(actual_value, expected_value, places=places)


def matmul(a, b):
    return [
        [sum(a[row][inner] * b[inner][col] for inner in range(4)) for col in range(4)]
        for row in range(4)
    ]


def translation(x, y, z):
    return [
        [1.0, 0.0, 0.0, x],
        [0.0, 1.0, 0.0, y],
        [0.0, 0.0, 1.0, z],
        [0.0, 0.0, 0.0, 1.0],
    ]


def rotation(axis, angle):
    x, y, z = axis
    c = math.cos(angle)
    s = math.sin(angle)
    if (x, y, z) == (0.0, 0.0, 1.0):
        return [
            [c, -s, 0.0, 0.0],
            [s, c, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    if (x, y, z) == (0.0, 1.0, 0.0):
        return [
            [c, 0.0, s, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [-s, 0.0, c, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    raise AssertionError(f"Unexpected axis: {axis}")


class XacroModel:
    def __init__(self, path):
        self.root = ET.parse(path).getroot()
        self.properties = self._load_properties()
        self.joints = {
            joint.attrib["name"]: joint
            for joint in self.root.findall("joint")
        }

    def _load_properties(self):
        properties = {}
        for node in self.root.findall(f"{XACRO_NS}property"):
            properties[node.attrib["name"]] = float(node.attrib["value"])
        return properties

    def value(self, token):
        token = token.strip()
        if token.startswith("${") and token.endswith("}"):
            expression = token[2:-1].strip()
            return float(eval(expression, {"__builtins__": {}}, self.properties))
        return float(token)

    def vector(self, text):
        return tuple(self.value(token) for token in text.split())

    def joint_origin(self, name):
        origin = self.joints[name].find("origin")
        if origin is None:
            return (0.0, 0.0, 0.0)
        return self.vector(origin.attrib.get("xyz", "0 0 0"))

    def joint_origin_rpy(self, name):
        origin = self.joints[name].find("origin")
        if origin is None:
            return (0.0, 0.0, 0.0)
        return self.vector(origin.attrib.get("rpy", "0 0 0"))

    def joint_axis(self, name):
        axis = self.joints[name].find("axis")
        if axis is None:
            return (0.0, 0.0, 0.0)
        return self.vector(axis.attrib["xyz"])

    def joint_parent(self, name):
        return self.joints[name].find("parent").attrib["link"]

    def joint_child(self, name):
        return self.joints[name].find("child").attrib["link"]

    def joint_limit(self, name):
        limit = self.joints[name].find("limit")
        return self.value(limit.attrib["lower"]), self.value(limit.attrib["upper"])

    def link_visual_rpy(self, link_name, visual_index=0):
        link = self.root.find(f"./link[@name='{link_name}']")
        if link is None:
            raise AssertionError(f"Missing link: {link_name}")
        visuals = link.findall("visual")
        origin = visuals[visual_index].find("origin")
        return self.vector(origin.attrib.get("rpy", "0 0 0"))


def urdf_tcp_from_model_angles(xacro_model, angles_rad):
    joint_order = [
        "joint1_base_yaw",
        "joint2_shoulder_pitch",
        "joint3_elbow_pitch",
        "joint4_wrist_pitch",
        "joint5_wrist_roll",
    ]
    transform = translation(0.0, 0.0, 0.0)
    for joint_name in joint_order:
        transform = matmul(transform, translation(*xacro_model.joint_origin(joint_name)))
        transform = matmul(transform, rotation(xacro_model.joint_axis(joint_name), angles_rad.get(joint_name, 0.0)))
    transform = matmul(transform, translation(*xacro_model.joint_origin("tcp_fixed_joint")))
    return (transform[0][3], transform[1][3], transform[2][3])


class UrdfDescriptionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.xacro = XacroModel(XACRO_PATH)
        cls.gui_xacro = XacroModel(GUI_XACRO_PATH)
        cls.model = JetArmKinematicModel.from_config_path(DEFAULT_CONFIG_PATH)

    def test_ros2_package_files_exist(self):
        self.assertTrue(PACKAGE_XML_PATH.exists())
        self.assertTrue(CMAKE_PATH.exists())
        self.assertTrue(XACRO_PATH.exists())
        self.assertTrue(GUI_XACRO_PATH.exists())
        self.assertTrue(LAUNCH_PATH.exists())
        self.assertTrue(RVIZ_PATH.exists())
        self.assertTrue(MAPPER_PATH.exists())

    def test_xacro_declares_expected_joints_and_axes(self):
        expected_axes = {
            "joint1_base_yaw": (0.0, 0.0, 1.0),
            "joint2_shoulder_pitch": (0.0, 1.0, 0.0),
            "joint3_elbow_pitch": (0.0, 1.0, 0.0),
            "joint4_wrist_pitch": (0.0, 1.0, 0.0),
            "joint5_wrist_roll": (0.0, 0.0, 1.0),
            "joint6_gripper": (1.0, 0.0, 0.0),
            "joint6_gripper_mimic": (1.0, 0.0, 0.0),
        }

        self.assertNotIn("left_gripper_finger_joint", self.xacro.joints)
        self.assertNotIn("right_gripper_finger_joint", self.xacro.joints)
        self.assertNotIn("left_gripper_inner_linkage_joint", self.xacro.joints)
        self.assertNotIn("right_gripper_inner_linkage_joint", self.xacro.joints)
        for joint_name, axis in expected_axes.items():
            self.assertIn(joint_name, self.xacro.joints)
            assert_vector_close(self, self.xacro.joint_axis(joint_name), axis)

    def test_xacro_geometry_matches_yaml_lengths(self):
        self.assertAlmostEqual(self.xacro.properties["base_to_joint2"], 0.105)
        self.assertAlmostEqual(self.xacro.properties["link_23"], 0.13)
        self.assertAlmostEqual(self.xacro.properties["link_34"], 0.13)
        self.assertAlmostEqual(self.xacro.properties["link_45"], 0.06)
        self.assertAlmostEqual(self.xacro.properties["link_5_tcp"], 0.102)

        assert_vector_close(self, self.xacro.joint_origin("joint2_shoulder_pitch"), (0.0, 0.0, 0.105))
        assert_vector_close(self, self.xacro.joint_origin("joint3_elbow_pitch"), (0.0, 0.0, 0.13))
        assert_vector_close(self, self.xacro.joint_origin("joint4_wrist_pitch"), (0.0, 0.0, 0.13))
        assert_vector_close(self, self.xacro.joint_origin("joint5_wrist_roll"), (0.0, 0.0, 0.06))
        assert_vector_close(self, self.xacro.joint_origin("tcp_fixed_joint"), (0.0, 0.0, 0.102))
        assert_vector_close(self, self.xacro.joint_origin("joint6_gripper"), (0.0, 0.0, 0.0))
        assert_vector_close(self, self.xacro.joint_origin("joint6_gripper_mimic"), (0.0, 0.0, 0.0))

    def test_gripper_is_two_scissor_arms_starting_at_j5(self):
        self.assertEqual(self.xacro.joint_parent("tcp_fixed_joint"), "tool_roll_link")
        self.assertEqual(self.xacro.joint_child("tcp_fixed_joint"), "tcp_link")
        self.assertNotIn("gripper_base_fixed_joint", self.xacro.joints)
        self.assertEqual(self.xacro.joint_parent("joint6_gripper"), "tool_roll_link")
        self.assertEqual(self.xacro.joint_parent("joint6_gripper_mimic"), "tool_roll_link")
        mimic = self.xacro.joints["joint6_gripper_mimic"].find("mimic")
        self.assertIsNotNone(mimic)
        self.assertEqual(mimic.attrib["joint"], "joint6_gripper")
        self.assertEqual(mimic.attrib["multiplier"], "-1.0")

    def test_gripper_zero_visual_pose_is_open(self):
        self.assertAlmostEqual(self.xacro.properties["j6_visual_closed_angle"], math.pi / 2.0)
        self.assertAlmostEqual(self.xacro.properties["j6_visual_open_angle"], math.pi / 2.0)

        left_rpy = self.xacro.joint_origin_rpy("joint6_gripper")
        right_rpy = self.xacro.joint_origin_rpy("joint6_gripper_mimic")

        assert_vector_close(self, left_rpy, (-math.pi / 2.0, 0.0, 0.0))
        assert_vector_close(self, right_rpy, (math.pi / 2.0, 0.0, 0.0))

    def test_xacro_limits_match_current_model_angles(self):
        assert_vector_close(
            self,
            self.xacro.joint_limit("joint2_shoulder_pitch"),
            (0.0, math.pi / 2.0),
        )
        assert_vector_close(
            self,
            self.xacro.joint_limit("joint5_wrist_roll"),
            (-math.radians(60.0), math.radians(60.0)),
        )
        assert_vector_close(
            self,
            self.xacro.joint_limit("joint6_gripper"),
            (0.0, math.pi / 2.0),
        )
        assert_vector_close(
            self,
            self.xacro.joint_limit("joint6_gripper_mimic"),
            (-math.pi / 2.0, 0.0),
        )

    def test_gui_uses_raw_j6_slider_range(self):
        assert_vector_close(
            self,
            self.gui_xacro.joint_limit("joint6_gripper"),
            (0.0, 1000.0),
        )

    def test_launch_maps_raw_joint_states_before_robot_state_publisher(self):
        launch_text = LAUNCH_PATH.read_text(encoding="utf-8")
        package_text = PACKAGE_XML_PATH.read_text(encoding="utf-8")
        cmake_text = CMAKE_PATH.read_text(encoding="utf-8")

        self.assertIn("jetarm_gui.urdf.xacro", launch_text)
        self.assertIn("ParameterValue", launch_text)
        self.assertIn("value_type=str", launch_text)
        self.assertIn("/joint_states_raw", launch_text)
        self.assertIn("joint_state_mapper.py", launch_text)
        self.assertIn("raw_closed_position", launch_text)
        self.assertIn("gripper_joint", launch_text)
        self.assertIn("visual_closed_angle_rad", launch_text)
        self.assertIn("1.5707963267948966", launch_text)
        self.assertIn("<exec_depend>rclpy</exec_depend>", package_text)
        self.assertIn("<exec_depend>sensor_msgs</exec_depend>", package_text)
        self.assertIn("scripts/joint_state_mapper.py", cmake_text)

    def test_mapper_keeps_a_single_j6_control_name(self):
        mapper_text = MAPPER_PATH.read_text(encoding="utf-8")

        self.assertIn("gripper_joint", mapper_text)
        self.assertNotIn("left_gripper_finger_joint", mapper_text)
        self.assertNotIn("right_gripper_finger_joint", mapper_text)
        self.assertNotIn("-visual_angle", mapper_text)
        self.assertNotIn("inner_linkage", mapper_text)

    def test_xacro_home_tcp_matches_python_fk(self):
        expected = self.model.forward_kinematics_from_model_angles_deg({}).tcp_xyz
        actual = urdf_tcp_from_model_angles(self.xacro, {})

        assert_vector_close(self, actual, expected)
        assert_vector_close(self, actual, (0.0, 0.0, 0.527))

    def test_xacro_nonzero_fk_matches_python_fk(self):
        angles_deg = {
            "J1": 35.0,
            "J2": 42.0,
            "J3": -18.0,
            "J4": 21.0,
            "J5": 55.0,
        }
        python_fk = self.model.forward_kinematics_from_model_angles_deg(angles_deg).tcp_xyz
        full_angles_rad = {
            self.model.resolve_joint_name(name): math.radians(value)
            for name, value in angles_deg.items()
        }
        xacro_fk = urdf_tcp_from_model_angles(self.xacro, full_angles_rad)

        assert_vector_close(self, xacro_fk, python_fk)


if __name__ == "__main__":
    unittest.main()
