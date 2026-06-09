from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_gui = LaunchConfiguration("use_gui")
    use_rviz = LaunchConfiguration("use_rviz")

    package_share = FindPackageShare("jetarm_description")
    xacro_file = PathJoinSubstitution([package_share, "urdf", "jetarm.urdf.xacro"])
    gui_xacro_file = PathJoinSubstitution([package_share, "urdf", "jetarm_gui.urdf.xacro"])
    rviz_file = PathJoinSubstitution([package_share, "rviz", "jetarm.rviz"])

    robot_description = {
        "robot_description": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", xacro_file]),
            value_type=str,
        )
    }
    gui_robot_description = {
        "robot_description": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", gui_xacro_file]),
            value_type=str,
        )
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_gui", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[robot_description],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                output="screen",
                parameters=[gui_robot_description],
                remappings=[("/joint_states", "/joint_states_raw")],
                condition=IfCondition(use_gui),
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                output="screen",
                parameters=[gui_robot_description],
                remappings=[("/joint_states", "/joint_states_raw")],
                condition=UnlessCondition(use_gui),
            ),
            Node(
                package="jetarm_description",
                executable="joint_state_mapper.py",
                name="jetarm_joint_state_mapper",
                output="screen",
                parameters=[
                    {
                        "raw_topic": "/joint_states_raw",
                        "mapped_topic": "/joint_states",
                        "gripper_joint": "joint6_gripper",
                        "raw_closed_position": 700.0,
                        "visual_closed_angle_rad": 1.5707963267948966,
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_file],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
