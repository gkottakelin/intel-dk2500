"""Computable kinematic model for the current JetArm measurements.

This module is intentionally hardware-free. It reads ``joint_servo_map.yaml``
and computes an approximate forward kinematic model for J1-J5 plus the J6
gripper state. The first model treats J5 as wrist roll around the current tool
axis, so it affects orientation but not TCP position.
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Optional

try:
    from .joint_controller import (
        DEFAULT_CONFIG_PATH,
        JointConfigError,
        JointRangeError,
        JointCommandPlanner,
        assignments_to_dict,
        float_assignments_to_dict,
        load_joint_config,
        parse_assignment,
        parse_float_assignment,
    )
except ImportError:
    from joint_controller import (  # type: ignore
        DEFAULT_CONFIG_PATH,
        JointConfigError,
        JointRangeError,
        JointCommandPlanner,
        assignments_to_dict,
        float_assignments_to_dict,
        load_joint_config,
        parse_assignment,
        parse_float_assignment,
    )


ARM_JOINT_COUNT = 5
DEFAULT_JACOBIAN_DELTA_RAD = 1e-5


Vector3 = tuple[float, float, float]
Matrix3 = tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]


@dataclass(frozen=True)
class JointAngleState:
    joint_name: str
    servo_position: Optional[int]
    servo_angle_deg: Optional[float]
    model_angle_deg: float
    model_angle_rad: float

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class ForwardKinematicsResult:
    tcp_xyz: Vector3
    rotation_matrix: Matrix3
    joint_origins_xyz: dict[str, Vector3]
    joint_states: dict[str, JointAngleState]
    gripper_position: Optional[int]
    assumptions: tuple[str, ...]

    def to_dict(self) -> dict[str, Any]:
        data = asdict(self)
        data["joint_states"] = {name: state.to_dict() for name, state in self.joint_states.items()}
        return data


def _identity_matrix() -> Matrix3:
    return ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


def _matmul(a: Matrix3, b: Matrix3) -> Matrix3:
    return tuple(
        tuple(sum(a[row][inner] * b[inner][col] for inner in range(3)) for col in range(3))
        for row in range(3)
    )  # type: ignore[return-value]


def _matvec(matrix: Matrix3, vector: Vector3) -> Vector3:
    return tuple(sum(matrix[row][col] * vector[col] for col in range(3)) for row in range(3))  # type: ignore[return-value]


def _add(a: Vector3, b: Vector3) -> Vector3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _sub(a: Vector3, b: Vector3) -> Vector3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _scale(a: Vector3, value: float) -> Vector3:
    return (a[0] * value, a[1] * value, a[2] * value)


def _rz(angle_rad: float) -> Matrix3:
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)
    return ((cos_a, -sin_a, 0.0), (sin_a, cos_a, 0.0), (0.0, 0.0, 1.0))


def _ry(angle_rad: float) -> Matrix3:
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)
    return ((cos_a, 0.0, sin_a), (0.0, 1.0, 0.0), (-sin_a, 0.0, cos_a))


def _round_vector(vector: Vector3, digits: int = 12) -> Vector3:
    return tuple(0.0 if abs(value) < 10 ** -digits else round(value, digits) for value in vector)  # type: ignore[return-value]


def _round_matrix(matrix: Matrix3, digits: int = 12) -> Matrix3:
    return tuple(_round_vector(row, digits) for row in matrix)  # type: ignore[return-value]


class JetArmKinematicModel:
    """Approximate serial-chain model built from ``joint_servo_map.yaml``."""

    def __init__(self, config: dict[str, Any]) -> None:
        self.config = config
        self.planner = JointCommandPlanner(config)
        self.joints = self.planner.joints
        self.arm_joint_names = self._load_joint_group("arm", expected_count=ARM_JOINT_COUNT)
        self.gripper_joint_name = self._load_joint_group("gripper", expected_count=1)[0]
        self.geometry = self._load_geometry()
        self.assumptions = (
            "J1 is base yaw about +Z.",
            "J2-J4 form a vertical-home planar pitch chain in the base X/Z plane before yaw.",
            "J5 is treated as roll about the current tool axis and does not change TCP position.",
            "J6 is kept as gripper state only and does not change TCP pose.",
        )

    @classmethod
    def from_config_path(cls, path: str | Path = DEFAULT_CONFIG_PATH) -> "JetArmKinematicModel":
        return cls(load_joint_config(path))

    def _load_joint_group(self, group_name: str, *, expected_count: int) -> list[str]:
        groups = self.config.get("groups")
        if not isinstance(groups, dict) or not isinstance(groups.get(group_name), dict):
            raise JointConfigError(f"Missing joint group: {group_name}")
        joints = groups[group_name].get("joints")
        if not isinstance(joints, list) or len(joints) != expected_count:
            raise JointConfigError(f"Joint group {group_name} must contain {expected_count} joints")
        return [self.planner.resolve_joint_name(str(name)) for name in joints]

    def _load_geometry(self) -> dict[str, float]:
        geometry = self.config.get("robot_geometry")
        if not isinstance(geometry, dict):
            raise JointConfigError("Config requires robot_geometry")
        required = {
            "joint2_height": "base_to_joint2",
            "joint2_to_joint3_link_length": "link_23",
            "joint3_to_joint4_link_length": "link_34",
            "joint4_to_joint5_wrist_rotation_center": "link_45",
            "joint5_to_joint6_grasp_point": "link_5_tcp",
        }
        loaded: dict[str, float] = {}
        for yaml_key, model_key in required.items():
            if yaml_key not in geometry:
                raise JointConfigError(f"robot_geometry missing {yaml_key}")
            loaded[model_key] = float(geometry[yaml_key])
        return loaded

    def resolve_joint_name(self, name: str) -> str:
        return self.planner.resolve_joint_name(name)

    def _position_limits(self, joint_name: str) -> tuple[int, int]:
        position = self.joints[joint_name].get("position")
        if not isinstance(position, dict) or "min" not in position or "max" not in position:
            raise JointConfigError(f"{joint_name} is missing position min/max")
        return int(position["min"]), int(position["max"])

    def _angle_limits(self, joint_name: str) -> tuple[float, float]:
        angle = self.joints[joint_name].get("angle")
        if not isinstance(angle, dict) or "min_deg" not in angle or "max_deg" not in angle:
            raise JointConfigError(f"{joint_name} is missing angle min/max")
        return float(angle["min_deg"]), float(angle["max_deg"])

    def _home_position(self, joint_name: str) -> int:
        joint = self.joints[joint_name]
        if joint.get("home_position") is not None:
            return int(joint["home_position"])
        position = joint.get("position")
        if isinstance(position, dict) and position.get("center") is not None:
            return int(position["center"])
        raise JointConfigError(f"{joint_name} is missing home/center position")

    def _direction_sign(self, joint_name: str) -> float:
        value = self.joints[joint_name].get("direction_sign")
        if value is None:
            return 1.0
        return float(value)

    def position_to_servo_angle_deg(self, joint_ref: str, servo_position: int) -> float:
        joint_name = self.resolve_joint_name(joint_ref)
        min_pos, max_pos = self._position_limits(joint_name)
        min_deg, max_deg = self._angle_limits(joint_name)
        if not min_pos <= servo_position <= max_pos:
            raise JointRangeError(f"{joint_name} position {servo_position} outside {min_pos}..{max_pos}")
        ratio = (servo_position - min_pos) / (max_pos - min_pos)
        return min_deg + ratio * (max_deg - min_deg)

    def servo_angle_deg_to_position(self, joint_ref: str, servo_angle_deg: float) -> int:
        joint_name = self.resolve_joint_name(joint_ref)
        min_pos, max_pos = self._position_limits(joint_name)
        min_deg, max_deg = self._angle_limits(joint_name)
        if not min_deg <= servo_angle_deg <= max_deg:
            raise JointRangeError(f"{joint_name} angle {servo_angle_deg:g} outside {min_deg:g}..{max_deg:g}")
        ratio = (servo_angle_deg - min_deg) / (max_deg - min_deg)
        return int(round(min_pos + ratio * (max_pos - min_pos)))

    def position_to_model_angle_rad(self, joint_ref: str, servo_position: int) -> float:
        joint_name = self.resolve_joint_name(joint_ref)
        servo_angle_deg = self.position_to_servo_angle_deg(joint_name, servo_position)
        return math.radians(self._direction_sign(joint_name) * servo_angle_deg)

    def model_angle_rad_to_servo_position(self, joint_ref: str, model_angle_rad: float) -> int:
        joint_name = self.resolve_joint_name(joint_ref)
        servo_angle_deg = math.degrees(model_angle_rad) / self._direction_sign(joint_name)
        return self.servo_angle_deg_to_position(joint_name, servo_angle_deg)

    def _normalize_position_targets(self, positions: dict[str, int]) -> dict[str, int]:
        normalized: dict[str, int] = {}
        for raw_name, raw_position in positions.items():
            joint_name = self.resolve_joint_name(raw_name)
            value = int(raw_position)
            min_pos, max_pos = self._position_limits(joint_name)
            if not min_pos <= value <= max_pos:
                raise JointRangeError(f"{joint_name} position {value} outside {min_pos}..{max_pos}")
            normalized[joint_name] = value
        for joint_name in self.arm_joint_names:
            normalized.setdefault(joint_name, self._home_position(joint_name))
        return normalized

    def _normalize_angle_targets_rad(
        self,
        angles: dict[str, float],
        *,
        unit: str,
    ) -> dict[str, float]:
        normalized = {joint_name: 0.0 for joint_name in self.arm_joint_names}
        for raw_name, raw_angle in angles.items():
            joint_name = self.resolve_joint_name(raw_name)
            if joint_name not in self.arm_joint_names:
                raise JointConfigError(f"{joint_name} is not an arm pose joint")
            value = float(raw_angle)
            normalized[joint_name] = math.radians(value) if unit == "deg" else value
        return normalized

    def joint_states_from_positions(self, positions: dict[str, int]) -> dict[str, JointAngleState]:
        normalized = self._normalize_position_targets(positions)
        states: dict[str, JointAngleState] = {}
        for joint_name in self.arm_joint_names:
            servo_position = normalized[joint_name]
            servo_angle_deg = self.position_to_servo_angle_deg(joint_name, servo_position)
            model_angle_deg = self._direction_sign(joint_name) * servo_angle_deg
            states[joint_name] = JointAngleState(
                joint_name=joint_name,
                servo_position=servo_position,
                servo_angle_deg=servo_angle_deg,
                model_angle_deg=model_angle_deg,
                model_angle_rad=math.radians(model_angle_deg),
            )
        return states

    def joint_states_from_model_angles_rad(self, angles_rad: dict[str, float]) -> dict[str, JointAngleState]:
        normalized = self._normalize_angle_targets_rad(angles_rad, unit="rad")
        states: dict[str, JointAngleState] = {}
        for joint_name in self.arm_joint_names:
            model_angle_rad = normalized[joint_name]
            states[joint_name] = JointAngleState(
                joint_name=joint_name,
                servo_position=None,
                servo_angle_deg=None,
                model_angle_deg=math.degrees(model_angle_rad),
                model_angle_rad=model_angle_rad,
            )
        return states

    def joint_states_from_model_angles_deg(self, angles_deg: dict[str, float]) -> dict[str, JointAngleState]:
        return self.joint_states_from_model_angles_rad(self._normalize_angle_targets_rad(angles_deg, unit="deg"))

    def forward_kinematics_from_positions(self, positions: dict[str, int]) -> ForwardKinematicsResult:
        normalized = self._normalize_position_targets(positions)
        states = self.joint_states_from_positions(normalized)
        gripper_position = normalized.get(self.gripper_joint_name)
        return self._forward_kinematics(states, gripper_position=gripper_position)

    def forward_kinematics_from_model_angles_deg(
        self,
        angles_deg: dict[str, float],
        *,
        gripper_position: Optional[int] = None,
    ) -> ForwardKinematicsResult:
        states = self.joint_states_from_model_angles_deg(angles_deg)
        return self._forward_kinematics(states, gripper_position=gripper_position)

    def forward_kinematics_from_model_angles_rad(
        self,
        angles_rad: dict[str, float],
        *,
        gripper_position: Optional[int] = None,
    ) -> ForwardKinematicsResult:
        states = self.joint_states_from_model_angles_rad(angles_rad)
        return self._forward_kinematics(states, gripper_position=gripper_position)

    def _forward_kinematics(
        self,
        states: dict[str, JointAngleState],
        *,
        gripper_position: Optional[int],
    ) -> ForwardKinematicsResult:
        q = [states[joint_name].model_angle_rad for joint_name in self.arm_joint_names]
        q1, q2, q3, q4, q5 = q

        base_to_j2 = self.geometry["base_to_joint2"]
        link_23 = self.geometry["link_23"]
        link_34 = self.geometry["link_34"]
        link_45 = self.geometry["link_45"]
        link_5_tcp = self.geometry["link_5_tcp"]

        def pitch_vector(length: float, angle: float) -> Vector3:
            return (length * math.sin(angle), 0.0, length * math.cos(angle))

        plane_j1 = (0.0, 0.0, 0.0)
        plane_j2 = (0.0, 0.0, base_to_j2)
        plane_j3 = _add(plane_j2, pitch_vector(link_23, q2))
        plane_j4 = _add(plane_j3, pitch_vector(link_34, q2 + q3))
        plane_j5 = _add(plane_j4, pitch_vector(link_45, q2 + q3 + q4))
        plane_tcp = _add(plane_j5, pitch_vector(link_5_tcp, q2 + q3 + q4))

        yaw_rotation = _rz(q1)
        joint_origins = {
            self.arm_joint_names[0]: _round_vector(_matvec(yaw_rotation, plane_j1)),
            self.arm_joint_names[1]: _round_vector(_matvec(yaw_rotation, plane_j2)),
            self.arm_joint_names[2]: _round_vector(_matvec(yaw_rotation, plane_j3)),
            self.arm_joint_names[3]: _round_vector(_matvec(yaw_rotation, plane_j4)),
            self.arm_joint_names[4]: _round_vector(_matvec(yaw_rotation, plane_j5)),
            "tcp": _round_vector(_matvec(yaw_rotation, plane_tcp)),
        }

        pitch_rotation = _ry(q2 + q3 + q4)
        roll_rotation = _rz(q5)
        rotation = _round_matrix(_matmul(_matmul(yaw_rotation, pitch_rotation), roll_rotation))

        return ForwardKinematicsResult(
            tcp_xyz=joint_origins["tcp"],
            rotation_matrix=rotation,
            joint_origins_xyz=joint_origins,
            joint_states=states,
            gripper_position=gripper_position,
            assumptions=self.assumptions,
        )

    def tcp_jacobian_from_positions(
        self,
        positions: dict[str, int],
        *,
        delta_rad: float = DEFAULT_JACOBIAN_DELTA_RAD,
    ) -> list[list[float]]:
        states = self.joint_states_from_positions(self._normalize_position_targets(positions))
        angles = {name: state.model_angle_rad for name, state in states.items()}
        return self.tcp_jacobian_from_model_angles_rad(angles, delta_rad=delta_rad)

    def tcp_jacobian_from_model_angles_rad(
        self,
        angles_rad: dict[str, float],
        *,
        delta_rad: float = DEFAULT_JACOBIAN_DELTA_RAD,
    ) -> list[list[float]]:
        if delta_rad <= 0:
            raise ValueError("delta_rad must be positive")
        normalized = self._normalize_angle_targets_rad(angles_rad, unit="rad")
        columns: list[Vector3] = []
        for joint_name in self.arm_joint_names:
            plus = dict(normalized)
            minus = dict(normalized)
            plus[joint_name] += delta_rad
            minus[joint_name] -= delta_rad
            tcp_plus = self.forward_kinematics_from_model_angles_rad(plus).tcp_xyz
            tcp_minus = self.forward_kinematics_from_model_angles_rad(minus).tcp_xyz
            columns.append(_scale(_sub(tcp_plus, tcp_minus), 1.0 / (2.0 * delta_rad)))
        return [[columns[col][row] for col in range(len(columns))] for row in range(3)]


def _result_payload(result: ForwardKinematicsResult, jacobian: Optional[list[list[float]]] = None) -> dict[str, Any]:
    payload = result.to_dict()
    if jacobian is not None:
        payload["tcp_jacobian"] = jacobian
    return payload


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Compute JetArm forward kinematics from YAML geometry")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG_PATH), help="joint_servo_map.yaml path")
    parser.add_argument("--position", action="append", type=parse_assignment, help="raw servo position NAME=VALUE")
    parser.add_argument("--angle-deg", action="append", type=parse_float_assignment, help="model angle in degrees")
    parser.add_argument("--angle-rad", action="append", type=parse_float_assignment, help="model angle in radians")
    parser.add_argument("--jacobian", action="store_true", help="include numeric translational TCP Jacobian")
    parser.add_argument("--json", action="store_true", help="print JSON output")
    return parser


def main(argv: Optional[list[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    positions = assignments_to_dict(args.position)
    angles_deg = float_assignments_to_dict(args.angle_deg)
    angles_rad = float_assignments_to_dict(args.angle_rad)
    if sum(bool(value) for value in (positions, angles_deg, angles_rad)) > 1:
        parser.error("Use only one of --position, --angle-deg, or --angle-rad per run")

    try:
        model = JetArmKinematicModel.from_config_path(args.config)
        if angles_deg:
            result = model.forward_kinematics_from_model_angles_deg(angles_deg)
            jacobian = model.tcp_jacobian_from_model_angles_rad(
                {name: math.radians(value) for name, value in angles_deg.items()}
            ) if args.jacobian else None
        elif angles_rad:
            result = model.forward_kinematics_from_model_angles_rad(angles_rad)
            jacobian = model.tcp_jacobian_from_model_angles_rad(angles_rad) if args.jacobian else None
        else:
            result = model.forward_kinematics_from_positions(positions)
            jacobian = model.tcp_jacobian_from_positions(positions) if args.jacobian else None

        payload = _result_payload(result, jacobian=jacobian)
        if args.json:
            print(json.dumps(payload, indent=2))
        else:
            print(f"TCP xyz (m): {result.tcp_xyz}")
            print("Joint model angles (deg):")
            for joint_name, state in result.joint_states.items():
                print(f"- {joint_name}: {state.model_angle_deg:g}")
            if result.gripper_position is not None:
                print(f"Gripper position: {result.gripper_position}")
        return 0
    except Exception as exc:
        print(f"ERROR: {exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
