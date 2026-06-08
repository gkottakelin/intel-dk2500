"""YAML-driven joint command planning and execution for JetArm.

This module reads ``config/joint_servo_map.yaml`` and converts target joint
positions or joint angles into low-level bus-servo commands. It supports
dry-run planning without opening the serial port, and requires ``--confirm``
for hardware execution.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Callable, Optional

try:
    from .bus_servo import DEFAULT_BAUDRATE, BusServoController
except ImportError:
    from bus_servo import DEFAULT_BAUDRATE, BusServoController  # type: ignore


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG_PATH = PROJECT_ROOT / "config" / "joint_servo_map.yaml"
DEFAULT_MOTOR_TOLERANCE = 5
DEFAULT_MOTOR_TIMEOUT_S = 5.0


class JointControllerError(Exception):
    """Base exception for joint command planning/execution."""


class JointConfigError(JointControllerError):
    """Raised when the joint YAML is missing required data."""


class JointRangeError(JointControllerError):
    """Raised when a target position is outside configured limits."""


class JointAngleError(JointControllerError):
    """Raised when a target angle cannot be mapped to a servo position."""


@dataclass(frozen=True)
class ServoMoveCommand:
    joint_name: str
    servo_id: int
    target_position: int
    run_time_ms: int
    current_position: Optional[int] = None
    current_source: Optional[str] = None
    control_mode: str = "servo"

    def to_dict(self) -> dict[str, Any]:
        data = asdict(self)
        data["command_type"] = "servo_move"
        return data


@dataclass(frozen=True)
class MotorCommand:
    joint_name: str
    servo_id: int
    speed: int
    target_position: int
    current_position: Optional[int] = None
    current_source: Optional[str] = None
    stop_tolerance: int = DEFAULT_MOTOR_TOLERANCE
    timeout_s: float = DEFAULT_MOTOR_TIMEOUT_S
    control_mode: str = "motor"

    def to_dict(self) -> dict[str, Any]:
        data = asdict(self)
        data["command_type"] = "motor_until_position"
        return data


JointCommand = ServoMoveCommand | MotorCommand


def _strip_inline_comment(text: str) -> str:
    in_single = False
    in_double = False
    for index, char in enumerate(text):
        if char == "'" and not in_double:
            in_single = not in_single
        elif char == '"' and not in_single:
            in_double = not in_double
        elif char == "#" and not in_single and not in_double:
            if index == 0 or text[index - 1].isspace():
                return text[:index].rstrip()
    return text.rstrip()


def _parse_scalar(value: str) -> Any:
    value = value.strip()
    if value == "":
        return ""
    lowered = value.lower()
    if lowered in {"null", "none", "~"}:
        return None
    if lowered == "true":
        return True
    if lowered == "false":
        return False
    if value.startswith("[") and value.endswith("]"):
        inner = value[1:-1].strip()
        if not inner:
            return []
        return [_parse_scalar(part.strip()) for part in inner.split(",")]
    if (value.startswith('"') and value.endswith('"')) or (value.startswith("'") and value.endswith("'")):
        return value[1:-1]
    try:
        if any(char in value for char in (".", "e", "E")):
            return float(value)
        return int(value)
    except ValueError:
        return value


def _prepare_yaml_lines(text: str) -> list[tuple[int, str]]:
    prepared: list[tuple[int, str]] = []
    for raw in text.splitlines():
        if not raw.strip() or raw.lstrip().startswith("#"):
            continue
        indent = len(raw) - len(raw.lstrip(" "))
        content = _strip_inline_comment(raw.strip())
        if content:
            prepared.append((indent, content))
    return prepared


def _collect_block_scalar(lines: list[tuple[int, str]], start: int, parent_indent: int, folded: bool) -> tuple[str, int]:
    parts: list[str] = []
    index = start
    while index < len(lines):
        indent, content = lines[index]
        if indent <= parent_indent:
            break
        parts.append(content)
        index += 1
    separator = " " if folded else "\n"
    return separator.join(parts).strip(), index


def _parse_yaml_block(lines: list[tuple[int, str]], index: int, indent: int) -> tuple[Any, int]:
    if index >= len(lines):
        return {}, index

    is_list = lines[index][1].startswith("- ")
    if is_list:
        result: list[Any] = []
        while index < len(lines):
            current_indent, content = lines[index]
            if current_indent < indent or not content.startswith("- "):
                break
            if current_indent > indent:
                child, index = _parse_yaml_block(lines, index, current_indent)
                result.append(child)
                continue
            item = content[2:].strip()
            index += 1
            if item == "":
                if index < len(lines) and lines[index][0] > current_indent:
                    child, index = _parse_yaml_block(lines, index, lines[index][0])
                    result.append(child)
                else:
                    result.append(None)
            else:
                result.append(_parse_scalar(item))
        return result, index

    result: dict[str, Any] = {}
    while index < len(lines):
        current_indent, content = lines[index]
        if current_indent < indent:
            break
        if current_indent > indent:
            break
        if content.startswith("- "):
            break
        if ":" not in content:
            raise JointConfigError(f"Invalid YAML line: {content}")

        key, raw_value = content.split(":", 1)
        key = key.strip()
        raw_value = raw_value.strip()
        index += 1

        if raw_value in {">", "|"}:
            value, index = _collect_block_scalar(lines, index, current_indent, folded=(raw_value == ">"))
            result[key] = value
            continue

        if raw_value:
            result[key] = _parse_scalar(raw_value)
            continue

        if index < len(lines) and lines[index][0] > current_indent:
            child, index = _parse_yaml_block(lines, index, lines[index][0])
            result[key] = child
        else:
            result[key] = None
    return result, index


def load_yaml_subset(path: str | Path) -> dict[str, Any]:
    """Load the project's YAML subset without external dependencies."""

    text = Path(path).read_text(encoding="utf-8")
    lines = _prepare_yaml_lines(text)
    if not lines:
        return {}
    data, index = _parse_yaml_block(lines, 0, lines[0][0])
    if index != len(lines):
        raise JointConfigError(f"Could not parse all YAML lines in {path}")
    if not isinstance(data, dict):
        raise JointConfigError("Top-level YAML value must be a mapping")
    return data


def load_joint_config(path: str | Path = DEFAULT_CONFIG_PATH) -> dict[str, Any]:
    """Load joint config, preferring PyYAML when installed."""

    try:
        import yaml  # type: ignore
    except ImportError:
        return load_yaml_subset(path)

    with Path(path).open("r", encoding="utf-8") as file:
        data = yaml.safe_load(file)
    if not isinstance(data, dict):
        raise JointConfigError("Top-level YAML value must be a mapping")
    return data


def _joint_number_alias(joint_name: str) -> Optional[str]:
    if not joint_name.startswith("joint"):
        return None
    rest = joint_name[5:]
    digits = []
    for char in rest:
        if not char.isdigit():
            break
        digits.append(char)
    if not digits:
        return None
    return "j" + "".join(digits)


def _position_limits(joint: dict[str, Any]) -> tuple[int, int]:
    position = joint.get("position")
    if not isinstance(position, dict):
        raise JointConfigError("Joint is missing position limits")
    if "min" not in position or "max" not in position:
        raise JointConfigError("Joint position limits require min and max")
    return int(position["min"]), int(position["max"])


def _angle_limits(joint: dict[str, Any]) -> tuple[float, float]:
    angle = joint.get("angle")
    if not isinstance(angle, dict):
        raise JointAngleError("Joint is missing angle mapping")
    if "min_deg" not in angle or "max_deg" not in angle:
        raise JointAngleError("Joint angle mapping requires min_deg and max_deg")
    return float(angle["min_deg"]), float(angle["max_deg"])


def _default_current_position(joint: dict[str, Any]) -> tuple[Optional[int], Optional[str]]:
    if joint.get("home_position") is not None:
        return int(joint["home_position"]), "home_position_assumed"
    position = joint.get("position")
    if isinstance(position, dict) and position.get("center") is not None:
        return int(position["center"]), "center_position_assumed"
    return None, None


def _profile_band_matches(delta: int, band: dict[str, Any]) -> bool:
    minimum = band.get("min_delta_position_exclusive")
    maximum = band.get("max_delta_position_inclusive")
    if minimum is not None and delta <= float(minimum):
        return False
    if maximum is not None and delta > float(maximum):
        return False
    return minimum is not None or maximum is not None


def _threshold_from_motion_profile(profile: dict[str, Any], delta: int) -> Optional[float]:
    for name in ("micro_motion", "small_motion", "large_motion"):
        band = profile.get(name)
        if isinstance(band, dict) and _profile_band_matches(delta, band):
            return float(band.get("position_delta_over_run_time_ms_min", 1.0))

    if delta < 30 and isinstance(profile.get("small_motion"), dict):
        return float(profile["small_motion"].get("position_delta_over_run_time_ms_min", 1.0))
    if isinstance(profile.get("large_motion"), dict):
        return float(profile["large_motion"].get("position_delta_over_run_time_ms_min", 0.1))
    return None


class JointCommandPlanner:
    def __init__(self, config: dict[str, Any]) -> None:
        self.config = config
        joints = config.get("joints")
        if not isinstance(joints, dict):
            raise JointConfigError("Config requires a top-level joints mapping")
        self.joints: dict[str, dict[str, Any]] = joints
        self.aliases = self._build_aliases()

    def _build_aliases(self) -> dict[str, str]:
        aliases: dict[str, str] = {}
        for joint_name, joint in self.joints.items():
            candidates = {joint_name, joint_name.lower()}
            number_alias = _joint_number_alias(joint_name)
            if number_alias:
                candidates.add(number_alias)
                candidates.add(number_alias.upper())
                candidates.add("joint" + number_alias[1:])
            for field in ("function", "role"):
                value = joint.get(field)
                if isinstance(value, str):
                    candidates.add(value)
                    candidates.add(value.lower())
            if joint.get("servo_id") is not None:
                candidates.add(f"servo{joint['servo_id']}")
            for candidate in candidates:
                aliases[candidate.lower()] = joint_name
        return aliases

    def resolve_joint_name(self, name: str) -> str:
        key = name.strip().lower()
        if key in self.aliases:
            return self.aliases[key]
        raise JointConfigError(f"Unknown joint: {name}")

    def validate_position(self, joint_name: str, target_position: int) -> None:
        joint = self.joints[joint_name]
        minimum, maximum = _position_limits(joint)
        if not minimum <= target_position <= maximum:
            raise JointRangeError(
                f"{joint_name} target {target_position} outside configured range {minimum}..{maximum}"
            )

    def angle_deg_to_position(self, joint_name: str, target_angle_deg: float) -> int:
        joint = self.joints[joint_name]
        min_pos, max_pos = _position_limits(joint)
        min_deg, max_deg = _angle_limits(joint)
        if max_deg == min_deg:
            raise JointAngleError(f"{joint_name} has invalid zero-width angle mapping")
        if not min_deg <= target_angle_deg <= max_deg:
            raise JointAngleError(
                f"{joint_name} target angle {target_angle_deg:g} deg outside configured range "
                f"{min_deg:g}..{max_deg:g} deg"
            )

        ratio = (target_angle_deg - min_deg) / (max_deg - min_deg)
        return int(round(min_pos + ratio * (max_pos - min_pos)))

    def servo_run_time_ms(self, joint_name: str, current_position: int, target_position: int) -> int:
        joint = self.joints[joint_name]
        delta = abs(int(target_position) - int(current_position))
        if delta == 0:
            return 0

        constraints = joint.get("drive_constraints")
        if not isinstance(constraints, dict):
            return delta

        profile = constraints.get("motion_profile")
        if isinstance(profile, dict):
            threshold = _threshold_from_motion_profile(profile, delta)
            if threshold is None:
                raise JointConfigError(f"{joint_name} motion profile did not match delta {delta}")
            if threshold <= 0:
                raise JointConfigError(f"{joint_name} has non-positive motion profile threshold")
            return int(math.ceil(max(delta / threshold, float(constraints.get("run_time_ms_min", 1)))))

        if constraints.get("run_time_formula") == "max(abs(delta_position) / 20, 1)":
            return int(math.ceil(max(delta / 20.0, float(constraints.get("run_time_ms_min", 1)))))

        return delta

    def _current_position(
        self,
        joint_name: str,
        *,
        controller: Any = None,
        current_positions: Optional[dict[str, int]] = None,
    ) -> tuple[Optional[int], Optional[str]]:
        if current_positions:
            for key, value in current_positions.items():
                key_lower = key.lower()
                if key_lower == joint_name.lower() or self.aliases.get(key_lower) == joint_name:
                    return int(value), "provided_current_position"

        joint = self.joints[joint_name]
        if controller is not None:
            return int(controller.read_position(int(joint["servo_id"]))), "hardware_read"
        return _default_current_position(joint)

    def plan_target(
        self,
        joint_ref: str,
        target_position: int,
        *,
        controller: Any = None,
        current_positions: Optional[dict[str, int]] = None,
    ) -> JointCommand:
        joint_name = self.resolve_joint_name(joint_ref)
        self.validate_position(joint_name, target_position)
        joint = self.joints[joint_name]
        servo_id = int(joint["servo_id"])
        control_mode = str(joint.get("control_mode", "servo"))
        current, current_source = self._current_position(
            joint_name,
            controller=controller,
            current_positions=current_positions,
        )

        if control_mode in {"servo", "servo_only"}:
            if current is None:
                raise JointConfigError(f"{joint_name} needs current position to calculate run_time_ms")
            run_time_ms = self.servo_run_time_ms(joint_name, current, target_position)
            return ServoMoveCommand(
                joint_name=joint_name,
                servo_id=servo_id,
                target_position=int(target_position),
                run_time_ms=run_time_ms,
                current_position=current,
                current_source=current_source,
                control_mode=control_mode,
            )

        if control_mode in {"motor", "motor_for_grasp"}:
            if current is None:
                raise JointConfigError(f"{joint_name} needs current position to choose motor direction")
            constraints = joint.get("drive_constraints") if isinstance(joint.get("drive_constraints"), dict) else {}
            positive_speed = int(constraints.get("motor_speed_positive", 100))
            negative_speed = int(constraints.get("motor_speed_negative", -100))
            if target_position > current:
                speed = positive_speed
            elif target_position < current:
                speed = negative_speed
            else:
                speed = 0
            release = joint.get("release") if isinstance(joint.get("release"), dict) else {}
            return MotorCommand(
                joint_name=joint_name,
                servo_id=servo_id,
                speed=speed,
                target_position=int(target_position),
                current_position=current,
                current_source=current_source,
                stop_tolerance=int(release.get("stop_tolerance_position_units", DEFAULT_MOTOR_TOLERANCE)),
                timeout_s=float(release.get("timeout_s", DEFAULT_MOTOR_TIMEOUT_S)),
                control_mode=control_mode,
            )

        raise JointConfigError(f"Unsupported control_mode for {joint_name}: {control_mode}")

    def plan_angle_target(
        self,
        joint_ref: str,
        target_angle_deg: float,
        *,
        controller: Any = None,
        current_positions: Optional[dict[str, int]] = None,
    ) -> JointCommand:
        joint_name = self.resolve_joint_name(joint_ref)
        target_position = self.angle_deg_to_position(joint_name, target_angle_deg)
        return self.plan_target(
            joint_name,
            target_position,
            controller=controller,
            current_positions=current_positions,
        )

    def plan_radian_target(
        self,
        joint_ref: str,
        target_angle_rad: float,
        *,
        controller: Any = None,
        current_positions: Optional[dict[str, int]] = None,
    ) -> JointCommand:
        return self.plan_angle_target(
            joint_ref,
            math.degrees(target_angle_rad),
            controller=controller,
            current_positions=current_positions,
        )

    def plan_targets(
        self,
        targets: dict[str, int],
        *,
        controller: Any = None,
        current_positions: Optional[dict[str, int]] = None,
    ) -> list[JointCommand]:
        return [
            self.plan_target(name, target, controller=controller, current_positions=current_positions)
            for name, target in targets.items()
        ]

    def plan_angle_targets(
        self,
        targets_deg: dict[str, float],
        *,
        controller: Any = None,
        current_positions: Optional[dict[str, int]] = None,
    ) -> list[JointCommand]:
        return [
            self.plan_angle_target(name, target, controller=controller, current_positions=current_positions)
            for name, target in targets_deg.items()
        ]

    def plan_radian_targets(
        self,
        targets_rad: dict[str, float],
        *,
        controller: Any = None,
        current_positions: Optional[dict[str, int]] = None,
    ) -> list[JointCommand]:
        return [
            self.plan_radian_target(name, target, controller=controller, current_positions=current_positions)
            for name, target in targets_rad.items()
        ]


class JointCommandExecutor:
    def __init__(
        self,
        controller: Any,
        *,
        monotonic: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
        poll_interval_s: float = 0.05,
    ) -> None:
        self.controller = controller
        self.monotonic = monotonic
        self.sleep = sleep
        self.poll_interval_s = poll_interval_s

    def execute(self, commands: list[JointCommand]) -> None:
        for command in commands:
            self.execute_command(command)

    def execute_command(self, command: JointCommand) -> None:
        if isinstance(command, ServoMoveCommand):
            self.controller.move_servo(command.servo_id, command.target_position, command.run_time_ms)
            return
        self._execute_motor_until_target(command)

    def _execute_motor_until_target(self, command: MotorCommand) -> None:
        if command.speed == 0:
            self.controller.set_motor_speed(command.servo_id, 0)
            return

        start = self.monotonic()
        self.controller.set_motor_speed(command.servo_id, command.speed)
        try:
            while True:
                position = int(self.controller.read_position(command.servo_id))
                if command.speed > 0 and position >= command.target_position - command.stop_tolerance:
                    return
                if command.speed < 0 and position <= command.target_position + command.stop_tolerance:
                    return
                if self.monotonic() - start >= command.timeout_s:
                    raise JointControllerError(
                        f"{command.joint_name} motor command timed out before target {command.target_position}"
                    )
                self.sleep(self.poll_interval_s)
        finally:
            self.controller.set_motor_speed(command.servo_id, 0)


def parse_assignment(text: str) -> tuple[str, int]:
    if "=" not in text:
        raise argparse.ArgumentTypeError("Expected NAME=POSITION")
    name, value = text.split("=", 1)
    name = name.strip()
    if not name:
        raise argparse.ArgumentTypeError("Joint name is empty")
    try:
        position = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"Invalid position: {value}") from exc
    return name, position


def parse_float_assignment(text: str) -> tuple[str, float]:
    if "=" not in text:
        raise argparse.ArgumentTypeError("Expected NAME=VALUE")
    name, value = text.split("=", 1)
    name = name.strip()
    if not name:
        raise argparse.ArgumentTypeError("Joint name is empty")
    try:
        number = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"Invalid numeric value: {value}") from exc
    return name, number


def assignments_to_dict(assignments: Optional[list[tuple[str, int]]]) -> dict[str, int]:
    result: dict[str, int] = {}
    for name, position in assignments or []:
        result[name] = position
    return result


def float_assignments_to_dict(assignments: Optional[list[tuple[str, float]]]) -> dict[str, float]:
    result: dict[str, float] = {}
    for name, value in assignments or []:
        result[name] = value
    return result


def commands_to_dicts(commands: list[JointCommand]) -> list[dict[str, Any]]:
    return [command.to_dict() for command in commands]


def print_commands(commands: list[JointCommand]) -> None:
    print("Planned joint commands:")
    for command in commands:
        if isinstance(command, ServoMoveCommand):
            print(
                f"- {command.joint_name}: servo_id={command.servo_id} "
                f"move_servo target={command.target_position} run_time_ms={command.run_time_ms} "
                f"current={command.current_position} source={command.current_source}"
            )
        else:
            print(
                f"- {command.joint_name}: servo_id={command.servo_id} "
                f"set_motor_speed speed={command.speed} target={command.target_position} "
                f"current={command.current_position} source={command.current_source}"
            )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Plan or execute JetArm joint commands from YAML config")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG_PATH), help="joint_servo_map.yaml path")
    parser.add_argument("--joint", action="append", type=parse_assignment, help="target in form J1=520; repeatable")
    parser.add_argument(
        "--joint-deg",
        "--joint-angle",
        dest="joint_deg",
        action="append",
        type=parse_float_assignment,
        help="target angle in degrees, for example J1=12.5; repeatable",
    )
    parser.add_argument(
        "--joint-rad",
        action="append",
        type=parse_float_assignment,
        help="target angle in radians, for example J1=0.2; repeatable",
    )
    parser.add_argument("--current", action="append", type=parse_assignment, help="optional dry-run current position")
    parser.add_argument("--com-port", default=None, help="serial port; defaults to YAML metadata or COM15")
    parser.add_argument("--baudrate", type=int, default=None, help="baudrate; defaults to YAML metadata or 115200")
    parser.add_argument("--timeout", type=float, default=0.2)
    parser.add_argument("--json", action="store_true", help="print planned commands as JSON")
    parser.add_argument("--confirm", action="store_true", help="required to open serial port and move hardware")
    return parser


def main(argv: Optional[list[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    targets = assignments_to_dict(args.joint)
    angle_targets_deg = float_assignments_to_dict(args.joint_deg)
    angle_targets_rad = float_assignments_to_dict(args.joint_rad)
    current_positions = assignments_to_dict(args.current)
    if not targets and not angle_targets_deg and not angle_targets_rad:
        parser.error("At least one --joint, --joint-deg, or --joint-rad target is required")

    try:
        config = load_joint_config(args.config)
        planner = JointCommandPlanner(config)
        metadata = config.get("metadata") if isinstance(config.get("metadata"), dict) else {}
        com_port = args.com_port or metadata.get("default_com_port") or "COM15"
        baudrate = args.baudrate or int(metadata.get("baudrate", DEFAULT_BAUDRATE))

        if args.confirm:
            with BusServoController(com_port, baudrate=baudrate, timeout=args.timeout) as controller:
                commands = []
                commands.extend(planner.plan_targets(targets, controller=controller, current_positions=current_positions))
                commands.extend(
                    planner.plan_angle_targets(
                        angle_targets_deg,
                        controller=controller,
                        current_positions=current_positions,
                    )
                )
                commands.extend(
                    planner.plan_radian_targets(
                        angle_targets_rad,
                        controller=controller,
                        current_positions=current_positions,
                    )
                )
                print_commands(commands)
                JointCommandExecutor(controller).execute(commands)
        else:
            commands = []
            commands.extend(planner.plan_targets(targets, current_positions=current_positions))
            commands.extend(planner.plan_angle_targets(angle_targets_deg, current_positions=current_positions))
            commands.extend(planner.plan_radian_targets(angle_targets_rad, current_positions=current_positions))
            print_commands(commands)
            print("Dry run only. Add --confirm to open the serial port and execute.")

        if args.json:
            print(json.dumps(commands_to_dicts(commands), indent=2))
        return 0
    except Exception as exc:
        print(f"ERROR: {exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
