#!/usr/bin/env python3
"""Map the GUI raw J6 gripper slider to the visual gripper angle."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateMapper(Node):
    def __init__(self) -> None:
        super().__init__("jetarm_joint_state_mapper")
        self.declare_parameter("raw_topic", "/joint_states_raw")
        self.declare_parameter("mapped_topic", "/joint_states")
        self.declare_parameter("gripper_joint", "joint6_gripper")
        self.declare_parameter("raw_closed_position", 700.0)
        self.declare_parameter("visual_closed_angle_rad", 1.5707963267948966)

        raw_topic = self.get_parameter("raw_topic").get_parameter_value().string_value
        mapped_topic = self.get_parameter("mapped_topic").get_parameter_value().string_value

        self.gripper_joint = self.get_parameter("gripper_joint").get_parameter_value().string_value
        self.raw_closed_position = self.get_parameter("raw_closed_position").get_parameter_value().double_value
        self.visual_closed_angle_rad = (
            self.get_parameter("visual_closed_angle_rad").get_parameter_value().double_value
        )

        self.publisher = self.create_publisher(JointState, mapped_topic, 10)
        self.subscription = self.create_subscription(JointState, raw_topic, self.map_joint_state, 10)

    def map_joint_state(self, message: JointState) -> None:
        mapped = JointState()
        mapped.header = message.header

        for index, name in enumerate(message.name):
            if index >= len(message.position):
                continue
            position = message.position[index]
            if name == self.gripper_joint:
                position = self._raw_gripper_to_visual_angle(position)
            mapped.name.append(name)
            mapped.position.append(position)

        self.publisher.publish(mapped)

    def _raw_gripper_to_visual_angle(self, raw_position: float) -> float:
        bounded = min(max(raw_position, 0.0), self.raw_closed_position)
        return bounded / self.raw_closed_position * self.visual_closed_angle_rad


def main() -> None:
    rclpy.init()
    node = JointStateMapper()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
