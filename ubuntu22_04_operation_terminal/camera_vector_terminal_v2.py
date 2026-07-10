"""Camera-line-relative JetArm terminal with pose-constrained analytic IK.

V2 defines the control frame from the current camera/grasp-point line:

- up: grasp point -> camera
- down: camera -> grasp point
- forward: the camera -> grasp line projected onto the base XY plane
- backward: opposite to forward
- left/right: horizontal axes perpendicular to forward

For each continuous input gesture the frame is captured once.  Horizontal
motion keeps the captured grasp-point height, while all Cartesian motion keeps
the captured camera-line inclination.  The horizontal azimuth of the line is
allowed to change because the four-axis arm needs J1 motion for arbitrary XY
translation.
"""

from __future__ import annotations

import argparse
import itertools
import math
import sys
from pathlib import Path
from typing import Any, Optional

import numpy as np

from camera_vector_terminal import (
    CameraLineConfig,
    CameraRelativeFrame,
    CameraRelativeManualServoRuntime,
    CameraVectorTerminalApp,
    build_camera_relative_frame,
)
from jetarm_terminal import (
    ARM_JOINTS,
    DEFAULT_CONFIG_PATH,
    BusServoController,
    DryRunServoController,
    ManualServoRuntime,
    TerminalSettings,
    choose_serial_port_dialog,
    discover_linux_serial_ports,
    select_linux_serial_port,
    serial_discovery_diagnostic,
    tk,
    ttk,
)


WORLD_UP = np.array((0.0, 0.0, 1.0), dtype=float)
HORIZONTAL_PROJECTION_EPSILON = 1e-4
IK_POSITION_TOLERANCE_M = 0.004
IK_INCLINATION_TOLERANCE_RAD = math.radians(1.0)
INITIAL_J6_POSITION = 400


class HorizontalDirectionUndefined(ValueError):
    """Raised when the camera/grasp line has no reliable XY projection."""


def _unit(vector: np.ndarray) -> np.ndarray:
    length = float(np.linalg.norm(vector))
    if length < HORIZONTAL_PROJECTION_EPSILON:
        raise HorizontalDirectionUndefined(
            "相机到抓取点连线接近竖直，无法可靠定义前后左右方向"
        )
    return vector / length


def build_camera_vector_v2_frame(
    settings: TerminalSettings,
    positions: dict[str, int],
    camera_config: CameraLineConfig,
) -> CameraRelativeFrame:
    """Build V2 directions from the camera/grasp line in base coordinates."""

    source = build_camera_relative_frame(settings, positions, camera_config)
    up = source.up
    down = -up
    horizontal_camera_to_grasp = np.array((down[0], down[1], 0.0), dtype=float)
    forward = _unit(horizontal_camera_to_grasp)
    left = _unit(np.cross(WORLD_UP, forward))
    return CameraRelativeFrame(
        up=up,
        down=down,
        forward=forward,
        backward=-forward,
        left=left,
        right=-left,
    )


def _wrap_angle(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


class CameraVectorV2Runtime(CameraRelativeManualServoRuntime):
    """Camera-relative runtime using fixed-inclination analytic pose IK."""

    position_target_tolerance_m = IK_POSITION_TOLERANCE_M

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._locked_pitch_rad: float | None = None
        self._locked_height_m: float | None = None
        self._pose_relaxation_used = False
        self._pose_relaxation_reason: str | None = None
        self._pose_relaxation_reason_code: str | None = None
        self._pose_relaxation_step_count = 0

    def camera_relative_frame(self) -> CameraRelativeFrame:
        return build_camera_vector_v2_frame(
            self.settings,
            self.positions,
            self.camera_config,
        )

    def _update_motion_lock(self) -> None:
        active = (
            abs(self.vertical_direction) > HORIZONTAL_PROJECTION_EPSILON
            or math.hypot(self.joystick_x, self.joystick_y)
            > HORIZONTAL_PROJECTION_EPSILON
        )
        if not active:
            self._clear_motion_lock()
            return

        horizontal_only = (
            abs(self.vertical_direction) <= HORIZONTAL_PROJECTION_EPSILON
            and math.hypot(self.joystick_x, self.joystick_y)
            > HORIZONTAL_PROJECTION_EPSILON
        )
        if self._locked_frame is None:
            self._locked_frame = self.continuous_camera_relative_frame()
            self._locked_line_angle_rad = self._camera_line_vertical_angle_rad(
                self.positions
            )
            self._locked_pitch_rad = self._tool_pitch_rad(self.positions)
            self._locked_height_m = float(self.model.tcp(self.positions)[2])
        self._height_hold_active = horizontal_only
        self.z_lock = horizontal_only

    def _clear_motion_lock(self, *, clear_forward: bool = False) -> None:
        super()._clear_motion_lock(clear_forward=clear_forward)
        self._locked_pitch_rad = None
        self._locked_height_m = None
        self.z_lock = False

    def reset_pose_relaxation_status(self) -> None:
        self._pose_relaxation_used = False
        self._pose_relaxation_reason = None
        self._pose_relaxation_reason_code = None
        self._pose_relaxation_step_count = 0

    def pose_relaxation_status(self) -> dict[str, object]:
        return {
            "constraint": "camera_grasp_line_inclination",
            "mode": (
                "relaxed_due_to_downward_limit"
                if self._pose_relaxation_used
                else "strict"
            ),
            "relaxed": self._pose_relaxation_used,
            "reason": self._pose_relaxation_reason,
            "reason_code": self._pose_relaxation_reason_code,
            "relaxed_step_count": self._pose_relaxation_step_count,
            "allowed_scope": "down_only",
        }

    def initialize_home_pose(self) -> None:
        """Return J1-J5 home and set J6 to its initialization position."""

        self.go_home()
        j6_id = self.settings.servo_id("J6")
        self.controller.set_servo_mode(j6_id)
        self.j6_grip_locked = False
        self.controller.move_servo(
            j6_id,
            INITIAL_J6_POSITION,
            self.settings.home_run_time_ms,
        )
        self.logger(
            f"初始化完成: J1-J5返回Home，J6位置={INITIAL_J6_POSITION}"
        )

    def _solve_next_positions(self, target_delta: np.ndarray) -> dict[str, int]:
        if self._locked_pitch_rad is None:
            return super()._solve_next_positions(target_delta)

        target_tcp = self.model.tcp(self.positions) + target_delta
        if self._height_hold_active and self._locked_height_m is not None:
            target_tcp[2] = self._locked_height_m

        candidates = self._analytic_pose_candidates(target_tcp, self._locked_pitch_rad)
        if not candidates:
            return self._relax_downward_pose_or_reject(
                target_delta,
                "strict_pose_unreachable_or_joint_limit",
                "V2严格姿态目标超出工作空间或关节限位",
            )

        target_inclination = self._locked_line_angle_rad
        if target_inclination is None:
            target_inclination = self._camera_line_vertical_angle_rad(self.positions)

        best = min(
            candidates,
            key=lambda item: self._pose_candidate_score(
                item,
                target_tcp,
                self._locked_pitch_rad,
                target_inclination,
            ),
        )
        position_error = float(np.linalg.norm(self.model.tcp(best) - target_tcp))
        inclination_error = abs(
            self._camera_line_vertical_angle_rad(best) - target_inclination
        )
        if (
            position_error > IK_POSITION_TOLERANCE_M
            or inclination_error > IK_INCLINATION_TOLERANCE_RAD
        ):
            return self._relax_downward_pose_or_reject(
                target_delta,
                "strict_pose_error_exceeded",
                "V2严格姿态解析逆解误差超限: "
                f"位置误差={position_error * 1000.0:.1f}mm, "
                f"倾角误差={math.degrees(inclination_error):.2f}°",
            )
        if best == self.positions:
            return self._relax_downward_pose_or_reject(
                target_delta,
                "strict_pose_no_progress",
                "V2严格姿态解析逆解无关节进展",
            )
        return best

    def _relax_downward_pose_or_reject(
        self,
        target_delta: np.ndarray,
        reason_code: str,
        strict_message: str,
    ) -> dict[str, int]:
        downward_only = (
            self.vertical_direction < -HORIZONTAL_PROJECTION_EPSILON
            and math.hypot(self.joystick_x, self.joystick_y)
            <= HORIZONTAL_PROJECTION_EPSILON
        )
        if not downward_only:
            self.logger(f"{strict_message}，已拒绝本步运动")
            return dict(self.positions)

        if not self._pose_relaxation_used:
            self.logger(
                f"{strict_message}；下降动作已放宽摄像头-抓取点连线倾角约束"
            )
        self._pose_relaxation_used = True
        if self._pose_relaxation_reason is None:
            self._pose_relaxation_reason = strict_message
            self._pose_relaxation_reason_code = reason_code
        self._pose_relaxation_step_count += 1
        return self._solve_relaxed_cartesian_positions(target_delta)

    def _solve_relaxed_cartesian_positions(
        self, target_delta: np.ndarray
    ) -> dict[str, int]:
        """Solve position only, bypassing V2 inclination constraints."""

        jacobian = self.model.jacobian(self.positions)
        damping = (self.settings.damping**2) * np.eye(3)
        try:
            dq = jacobian.T @ np.linalg.solve(
                jacobian @ jacobian.T + damping,
                target_delta,
            )
        except np.linalg.LinAlgError:
            dq = np.zeros(4)
        max_step = math.radians(self.settings.max_joint_step_deg)
        dq = np.clip(dq, -max_step, max_step)
        seed = dict(self.positions)
        for index, joint_name in enumerate(ARM_JOINTS):
            current = self.model.position_to_model_angle(
                joint_name, self.positions[joint_name]
            )
            target = self.model.clamp_model_angle(
                joint_name, current + float(dq[index])
            )
            seed[joint_name] = self.model.model_angle_to_position(
                joint_name, target
            )
        return ManualServoRuntime._local_refine(self, seed, target_delta)

    def _analytic_pose_candidates(
        self,
        target_tcp: np.ndarray,
        pitch_rad: float,
    ) -> list[dict[str, int]]:
        geometry = self.settings.geometry
        link_1 = float(geometry["joint2_to_joint3"])
        link_2 = float(geometry["joint3_to_joint4"])
        link_3 = float(geometry["joint4_to_joint5"] + geometry["joint5_to_tcp"])
        base_height = float(geometry["base_to_joint2"])

        x, y, z = (float(value) for value in target_tcp)
        radius = math.hypot(x, y)
        heading = math.atan2(y, x) if radius > 1e-12 else self._tool_yaw_rad(self.positions)
        radial_branches = (
            (_wrap_angle(heading), radius),
            (_wrap_angle(heading + math.pi), -radius),
        )
        candidates: list[dict[str, int]] = []
        for q1, signed_radius in radial_branches:
            wrist_r = signed_radius - link_3 * math.sin(pitch_rad)
            wrist_z = z - base_height - link_3 * math.cos(pitch_rad)
            denominator = 2.0 * link_1 * link_2
            cos_q3 = (
                wrist_r * wrist_r
                + wrist_z * wrist_z
                - link_1 * link_1
                - link_2 * link_2
            ) / denominator
            if cos_q3 < -1.0 - 1e-9 or cos_q3 > 1.0 + 1e-9:
                continue
            cos_q3 = max(-1.0, min(1.0, cos_q3))
            q3_magnitude = math.acos(cos_q3)
            for q3 in (q3_magnitude, -q3_magnitude):
                beta = math.atan2(
                    link_2 * math.sin(q3),
                    link_1 + link_2 * math.cos(q3),
                )
                q2 = math.atan2(wrist_r, wrist_z) - beta
                q4 = pitch_rad - q2 - q3
                angles = (q1, q2, q3, q4)
                if not self._angles_within_limits(angles):
                    continue
                seed = dict(self.positions)
                for joint_name, angle in zip(ARM_JOINTS, angles):
                    seed[joint_name] = self.model.model_angle_to_position(
                        joint_name, angle
                    )
                candidates.extend(self._discrete_neighbors(seed))
        return candidates

    def _angles_within_limits(self, angles: tuple[float, ...]) -> bool:
        for joint_name, angle in zip(ARM_JOINTS, angles):
            clamped = self.model.clamp_model_angle(joint_name, angle)
            if abs(clamped - angle) > 1e-8:
                return False
        return True

    def _discrete_neighbors(self, seed: dict[str, int]) -> list[dict[str, int]]:
        candidates: list[dict[str, int]] = []
        for offsets in itertools.product((-1, 0, 1), repeat=len(ARM_JOINTS)):
            candidate = dict(seed)
            valid = True
            for joint_name, offset in zip(ARM_JOINTS, offsets):
                low, high = self.settings.position_limits(joint_name)
                value = candidate[joint_name] + offset
                if value < low or value > high:
                    valid = False
                    break
                candidate[joint_name] = value
            if valid:
                candidates.append(candidate)
        return candidates

    def _pose_candidate_score(
        self,
        positions: dict[str, int],
        target_tcp: np.ndarray,
        target_pitch_rad: float,
        target_inclination_rad: float,
    ) -> float:
        position_error = float(np.linalg.norm(self.model.tcp(positions) - target_tcp))
        pitch_error = abs(
            _wrap_angle(self._tool_pitch_rad(positions) - target_pitch_rad)
        )
        inclination_error = abs(
            self._camera_line_vertical_angle_rad(positions) - target_inclination_rad
        )
        joint_motion = sum(
            abs(positions[name] - self.positions[name]) for name in ARM_JOINTS
        )
        return (
            position_error
            + pitch_error
            + inclination_error
            + joint_motion * 1e-8
        )


class CameraVectorV2App(CameraVectorTerminalApp):
    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self.root.title("JetArm camera-vector terminal V2")

    def _vertical_panel(self, parent: Any) -> Any:
        frame = super()._vertical_panel(parent)
        ttk.Button(
            frame,
            text="初始化",
            style="Action.TButton",
            command=self._initialize_home_pose,
        ).grid(row=4, column=0, sticky="ew", pady=(8, 0), ipady=10)
        return frame

    def _initialize_home_pose(self) -> None:
        self._safe_call(self.runtime.initialize_home_pose)
        self._update_grip_color()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="JetArm camera-line-relative terminal V2 for Ubuntu 22.04"
    )
    parser.add_argument("--config", default=str(DEFAULT_CONFIG_PATH), help="terminal JSON config")
    parser.add_argument("--port", default=None, help="serial device, for example /dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=None, help="override configured baudrate")
    parser.add_argument("--timeout", type=float, default=None, help="override serial timeout in seconds")
    parser.add_argument("--dry-run", action="store_true", help="open the UI without a real serial device")
    parser.add_argument("--list-ports", action="store_true", help="list detected USB serial devices and exit")
    parser.add_argument(
        "--diagnose-ports",
        action="store_true",
        help="diagnose USB-visible devices that have no Linux tty node",
    )
    return parser


def main(argv: Optional[list[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    if not sys.platform.startswith("linux"):
        print("WARNING: this standalone version is intended for Ubuntu 22.04", file=sys.stderr)

    if args.list_ports:
        ports = discover_linux_serial_ports()
        if not ports:
            print(serial_discovery_diagnostic())
            return 1
        print("\n".join(ports))
        return 0

    if args.diagnose_ports:
        print(serial_discovery_diagnostic())
        return 0

    try:
        settings = TerminalSettings.from_file(args.config)
        camera_config = CameraLineConfig.from_file(args.config)
        if tk is None:
            raise RuntimeError("Tkinter is not installed")
        root = tk.Tk()
        app_holder: dict[str, CameraVectorV2App] = {}

        def logger(message: str) -> None:
            app = app_holder.get("app")
            if app is None:
                print(message)
            else:
                app.append_log(message)

        selected_port: Optional[str] = None
        if args.dry_run:
            controller: Any = DryRunServoController(settings, logger=logger)
        else:
            if args.port:
                selected_port = select_linux_serial_port(args.port)
            else:
                root.title("JetArm camera-vector terminal V2")
                root.geometry("720x480")
                selected_port = choose_serial_port_dialog(root)
                if selected_port is None:
                    root.destroy()
                    return 0
                root.geometry("1080x680")
            controller = BusServoController(
                selected_port,
                args.baudrate or settings.baudrate,
                args.timeout if args.timeout is not None else settings.timeout_s,
            )

        runtime = CameraVectorV2Runtime(
            controller,
            settings,
            camera_config=camera_config,
            logger=logger,
        )
        runtime.initialize(use_home_positions=args.dry_run)
        app_holder["app"] = CameraVectorV2App(
            root,
            runtime,
            dry_run=args.dry_run,
            serial_port=selected_port,
        )
        root.mainloop()
        return 0
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
