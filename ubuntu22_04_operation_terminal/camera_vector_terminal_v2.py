"""Camera-line-relative JetArm terminal with pose-constrained analytic IK.

V2 defines the control frame from the current camera/grasp-point line:

- up: grasp point -> camera
- down: camera -> grasp point
- forward: the camera -> grasp line projected onto the base XY plane
- backward: the grasp -> camera line projected onto the base XY plane
- left/right: horizontal axes perpendicular to forward

The camera-line inclination is captured once for the runtime session (and reset
only by Home/initialization).  Each action first creates one absolute grasp-point
target, then selects the nearest valid whole-arm pose for that target.  Horizontal
motion keeps the action-start grasp-point height.  The horizontal azimuth of the
line may change because the four-axis arm needs J1 motion for arbitrary XY
translation, but its angle to vertical/horizontal remains fixed.
"""

from __future__ import annotations

import argparse
import asyncio
import itertools
import json
import math
import sys
import time
from pathlib import Path
from typing import Any, Awaitable, Callable, Optional

import numpy as np

from camera_vector_terminal import (
    CameraLineConfig,
    CameraRelativeFrame,
    CameraRelativeManualServoRuntime,
    CameraVectorTerminalApp,
    build_camera_relative_frame,
    settings_kinematics,
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
MOTION_SETTLE_S = 0.05
MAX_FEEDBACK_CORRECTIONS = 2
MIN_FEEDBACK_CORRECTION_S = 0.20
MAX_FEEDBACK_CORRECTION_S = 0.60
INITIAL_J6_POSITION = 400
TERMINAL_MOTION_DIRECTIONS = (
    "forward",
    "backward",
    "left",
    "right",
    "up",
    "down",
)
SleepFunction = Callable[[float], Awaitable[None]]


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
    *,
    pitch_rad: float | None = None,
) -> CameraRelativeFrame:
    """Build V2 directions from the camera/grasp line in base coordinates."""

    if pitch_rad is None:
        source = build_camera_relative_frame(settings, positions, camera_config)
        up = source.up
    else:
        kinematics = settings_kinematics(settings)
        yaw = kinematics.position_to_model_angle("J1", positions["J1"])
        yaw_cos = math.cos(yaw)
        yaw_sin = math.sin(yaw)
        tool_axis = np.array(
            (
                math.sin(pitch_rad) * yaw_cos,
                math.sin(pitch_rad) * yaw_sin,
                math.cos(pitch_rad),
            ),
            dtype=float,
        )
        normal_axis = np.array(
            (
                math.cos(pitch_rad) * yaw_cos,
                math.cos(pitch_rad) * yaw_sin,
                -math.sin(pitch_rad),
            ),
            dtype=float,
        )
        lateral_axis = np.array((-yaw_sin, yaw_cos, 0.0), dtype=float)
        up = _unit(
            camera_config.grasp_to_camera_along_tool_m * tool_axis
            + camera_config.grasp_to_camera_normal_m * normal_axis
            + camera_config.grasp_to_camera_lateral_m * lateral_axis
        )
    down = -up
    horizontal_grasp_to_camera = np.array((up[0], up[1], 0.0), dtype=float)
    # The real-arm front/back convention is the reverse of the original V2
    # projection.  Swap only these two axes; left/right were already correct
    # on hardware and must not be derived from the swapped forward vector.
    backward = _unit(horizontal_grasp_to_camera)
    forward = -backward
    left = _unit(np.cross(WORLD_UP, backward))
    return CameraRelativeFrame(
        up=up,
        down=down,
        forward=forward,
        backward=backward,
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
        # The desired camera/grasp inclination belongs to the runtime session,
        # not to one joystick hold.  Keeping this reference across actions is
        # what prevents a small error at the end of one action from becoming
        # the target pose of the next action.
        self._pose_reference_pitch_rad: float | None = None
        self._pose_reference_line_angle_rad: float | None = None
        self._locked_pitch_rad: float | None = None
        self._locked_height_m: float | None = None
        self._motion_target_tcp: np.ndarray | None = None
        self._last_interactive_target_tcp: np.ndarray | None = None
        self._interactive_finish_attempts = 0
        self._interactive_correction_busy = False
        self._last_motion_plan: dict[str, object] | None = None
        self._pose_relaxation_used = False
        self._pose_relaxation_executed = False
        self._pose_relaxation_reason: str | None = None
        self._pose_relaxation_reason_code: str | None = None
        self._pose_relaxation_step_count = 0

    def initialize(self, *, use_home_positions: bool = False) -> None:
        super().initialize(use_home_positions=use_home_positions)
        self.capture_camera_pose_reference()

    def capture_camera_pose_reference(self) -> dict[str, float]:
        """Capture the one inclination that every later V2 action must keep."""

        pitch = self._tool_pitch_rad(self.positions)
        self._pose_reference_pitch_rad = pitch
        self._pose_reference_line_angle_rad = (
            self._camera_line_vertical_angle_from_pitch(pitch)
        )
        return {
            "tool_pitch_deg": math.degrees(pitch),
            "camera_line_angle_from_vertical_deg": math.degrees(
                self._pose_reference_line_angle_rad
            ),
        }

    def camera_pose_reference_status(self) -> dict[str, object]:
        if (
            self._pose_reference_pitch_rad is None
            or self._pose_reference_line_angle_rad is None
        ):
            return {"captured": False}
        return {
            "captured": True,
            "scope": "runtime_session_until_home_or_initialize",
            "tool_pitch_deg": math.degrees(self._pose_reference_pitch_rad),
            "camera_line_angle_from_vertical_deg": math.degrees(
                self._pose_reference_line_angle_rad
            ),
        }

    def refresh_joint_positions_from_controller(self) -> dict[str, int]:
        """Replace commanded positions with current J1-J4 feedback."""

        refreshed: dict[str, int] = {}
        for joint_name in ARM_JOINTS:
            value = int(
                self.controller.read_position(self.settings.servo_id(joint_name))
            )
            low, high = self.settings.position_limits(joint_name)
            if not low <= value <= high:
                raise RuntimeError(
                    f"{joint_name} feedback {value} outside safe range {low}..{high}"
                )
            refreshed[joint_name] = value
        self.positions.update(refreshed)
        return dict(refreshed)

    def camera_relative_frame(self) -> CameraRelativeFrame:
        return build_camera_vector_v2_frame(
            self.settings,
            self.positions,
            self.camera_config,
        )

    def _reference_camera_relative_frame(self) -> CameraRelativeFrame:
        if self._pose_reference_pitch_rad is None:
            self.capture_camera_pose_reference()
        return build_camera_vector_v2_frame(
            self.settings,
            self.positions,
            self.camera_config,
            pitch_rad=self._pose_reference_pitch_rad,
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
        if self._interactive_correction_busy:
            self.vertical_direction = 0.0
            self.joystick_x = 0.0
            self.joystick_y = 0.0
            raise RuntimeError(
                "V2 GUI release correction is still running; wait before a new action"
            )

        horizontal_only = (
            abs(self.vertical_direction) <= HORIZONTAL_PROJECTION_EPSILON
            and math.hypot(self.joystick_x, self.joystick_y)
            > HORIZONTAL_PROJECTION_EPSILON
        )
        if self._locked_frame is None:
            # A new action always starts from joint feedback.  The feedback is
            # used for the grasp-point coordinate and IK continuity only; it
            # must never replace the session pose reference.
            self.refresh_joint_positions_from_controller()
            if (
                self._pose_reference_pitch_rad is None
                or self._pose_reference_line_angle_rad is None
            ):
                self.capture_camera_pose_reference()
            self._locked_frame = self._reference_camera_relative_frame()
            self._locked_line_angle_rad = self._pose_reference_line_angle_rad
            self._locked_pitch_rad = self._pose_reference_pitch_rad
            current_tcp = self.model.tcp(self.positions)
            self._locked_height_m = float(current_tcp[2])
            self._motion_target_tcp = current_tcp.copy()
            self._last_interactive_target_tcp = current_tcp.copy()
            self._interactive_finish_attempts = 0
            self._interactive_correction_busy = False
        self._height_hold_active = horizontal_only
        self.z_lock = horizontal_only

    def _clear_motion_lock(self, *, clear_forward: bool = False) -> None:
        super()._clear_motion_lock(clear_forward=clear_forward)
        self._locked_pitch_rad = None
        self._locked_height_m = None
        self._motion_target_tcp = None
        self.z_lock = False

    def go_home(self) -> None:
        super().go_home()
        self._last_interactive_target_tcp = None
        self._interactive_finish_attempts = 0
        self._interactive_correction_busy = False
        self.capture_camera_pose_reference()

    def stop_all(self) -> None:
        super().stop_all()
        self._last_interactive_target_tcp = None
        self._interactive_finish_attempts = 0
        self._interactive_correction_busy = False

    def discard_interactive_motion_target(self) -> None:
        self._last_interactive_target_tcp = None
        self._interactive_finish_attempts = 0
        self._interactive_correction_busy = False

    def begin_interactive_motion_finish(self) -> bool:
        if self._last_interactive_target_tcp is None:
            return False
        self._interactive_correction_busy = True
        return True

    def reset_pose_relaxation_status(self) -> None:
        self._pose_relaxation_used = False
        self._pose_relaxation_executed = False
        self._pose_relaxation_reason = None
        self._pose_relaxation_reason_code = None
        self._pose_relaxation_step_count = 0

    def pose_relaxation_status(self) -> dict[str, object]:
        return {
            "constraint": "camera_grasp_line_inclination",
            "mode": (
                "relaxed_due_to_downward_limit"
                if self._pose_relaxation_executed
                else "strict"
            ),
            "relaxed": self._pose_relaxation_executed,
            "relaxed_candidate_planned": self._pose_relaxation_used,
            "reason": self._pose_relaxation_reason,
            "reason_code": self._pose_relaxation_reason_code,
            "relaxed_step_count": self._pose_relaxation_step_count,
            "allowed_scope": "down_only",
        }

    def mark_pose_relaxation_executed(self) -> None:
        self._pose_relaxation_executed = True

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

    def plan_grasp_target(self, target_tcp: np.ndarray) -> dict[str, object]:
        """Solve one absolute grasp-point target against the session pose.

        This absolute-target planner never promotes the previous command's
        theoretical pose error into the reference for the next command.
        """

        target = np.asarray(target_tcp, dtype=float).reshape(3).copy()
        if not np.all(np.isfinite(target)):
            raise ValueError("target grasp point must contain finite XYZ values")
        if self._height_hold_active and self._locked_height_m is not None:
            target[2] = self._locked_height_m

        if self._locked_pitch_rad is not None:
            target_pitch = self._locked_pitch_rad
        elif self._pose_reference_pitch_rad is not None:
            target_pitch = self._pose_reference_pitch_rad
        else:
            target_pitch = self._tool_pitch_rad(self.positions)

        if self._pose_reference_line_angle_rad is not None:
            target_inclination = self._pose_reference_line_angle_rad
        elif self._locked_line_angle_rad is not None:
            target_inclination = self._locked_line_angle_rad
        else:
            target_inclination = self._camera_line_vertical_angle_from_pitch(
                target_pitch
            )

        current_tcp = self.model.tcp(self.positions)
        target_delta = target - current_tcp
        candidates = self._analytic_pose_candidates(target, target_pitch)
        reason_code: str | None = None
        message: str | None = None
        strict = True

        if candidates:
            valid_candidates = [
                item
                for item in candidates
                if float(np.linalg.norm(self.model.tcp(item) - target))
                <= IK_POSITION_TOLERANCE_M
                and abs(
                    self._camera_line_vertical_angle_rad(item)
                    - target_inclination
                )
                <= IK_INCLINATION_TOLERANCE_RAD
            ]
            if valid_candidates:
                # Once accuracy is inside the hard tolerance, continuity and
                # joint-limit margin first select the local IK branch.  Pose
                # accuracy then selects the best discrete-servo neighbor
                # inside that branch.
                ranked_candidates = [
                    (self._joint_continuity_rank(item), item)
                    for item in valid_candidates
                ]
                minimum_max_motion = min(
                    rank[0] for rank, _item in ranked_candidates
                )
                local_branch = [
                    item
                    for rank, item in ranked_candidates
                    if rank[0] <= minimum_max_motion + 0.01
                ]
                best = min(
                    local_branch,
                    key=lambda item: self._pose_candidate_score(
                        item,
                        target,
                        target_pitch,
                        target_inclination,
                    ),
                )
            else:
                best = min(
                    candidates,
                    key=lambda item: self._pose_candidate_score(
                        item,
                        target,
                        target_pitch,
                        target_inclination,
                    ),
                )
            position_error = float(np.linalg.norm(self.model.tcp(best) - target))
            inclination_error = abs(
                self._camera_line_vertical_angle_rad(best) - target_inclination
            )
            if (
                position_error > IK_POSITION_TOLERANCE_M
                or inclination_error > IK_INCLINATION_TOLERANCE_RAD
            ):
                reason_code = "strict_pose_error_exceeded"
                message = (
                    "V2 strict absolute pose error exceeded: "
                    f"position={position_error * 1000.0:.1f}mm, "
                    f"inclination={math.degrees(inclination_error):.2f}deg"
                )
                best = self._relax_downward_pose_or_reject(
                    target_delta, reason_code, message
                )
                strict = False
        else:
            reason_code = "strict_pose_unreachable_or_joint_limit"
            message = "V2 strict absolute pose target is unreachable or at a joint limit"
            best = self._relax_downward_pose_or_reject(
                target_delta, reason_code, message
            )
            strict = False

        solved_tcp = self.model.tcp(best)
        solved_position_error = float(np.linalg.norm(solved_tcp - target))
        solved_inclination = self._camera_line_vertical_angle_rad(best)
        solved_inclination_error = abs(solved_inclination - target_inclination)
        relaxed = bool(self._pose_relaxation_used and not strict)
        accepted = best != self.positions or (
            solved_position_error <= IK_POSITION_TOLERANCE_M
            and (relaxed or solved_inclination_error <= IK_INCLINATION_TOLERANCE_RAD)
        )
        plan: dict[str, object] = {
            "status": "planned" if accepted else "rejected",
            "accepted": accepted,
            "strict_pose": strict,
            "relaxed": relaxed,
            "reason_code": reason_code,
            "message": message,
            "start_grasp_point_m": [float(value) for value in current_tcp],
            "target_grasp_point_m": [float(value) for value in target],
            "solved_grasp_point_m": [float(value) for value in solved_tcp],
            "target_joint_positions": dict(best),
            "target_tool_pitch_deg": math.degrees(target_pitch),
            "target_camera_line_angle_deg": math.degrees(target_inclination),
            "solved_camera_line_angle_deg": math.degrees(solved_inclination),
            "position_error_m": solved_position_error,
            "inclination_error_deg": math.degrees(solved_inclination_error),
            "candidate_count": len(candidates),
        }
        self._last_motion_plan = plan
        return plan

    def _solve_next_positions(self, target_delta: np.ndarray) -> dict[str, int]:
        target_tcp = self.model.tcp(self.positions) + target_delta
        plan = self.plan_grasp_target(target_tcp)
        return dict(plan["target_joint_positions"])

    def command_joint_pose(
        self,
        target_positions: dict[str, int],
        *,
        run_time_s: float,
    ) -> bool:
        run_time_ms = max(1, int(round(float(run_time_s) * 1000.0)))
        changed = False
        for joint_name in ARM_JOINTS:
            target = int(target_positions[joint_name])
            if target == self.positions[joint_name]:
                continue
            self.controller.move_servo(
                self.settings.servo_id(joint_name), target, run_time_ms
            )
            self.positions[joint_name] = target
            changed = True
        return changed

    def joint_pose_within_motion_budget(
        self,
        target_positions: dict[str, int],
        *,
        run_time_s: float,
    ) -> bool:
        tick_s = max(1e-6, float(self.settings.tick_s))
        base_allowed_rad = math.radians(
            float(self.settings.max_joint_step_deg)
        ) * max(
            1.0, float(run_time_s) / tick_s
        )
        for joint_name in ARM_JOINTS:
            current_angle = self.model.position_to_model_angle(
                joint_name, self.positions[joint_name]
            )
            target_angle = self.model.position_to_model_angle(
                joint_name, int(target_positions[joint_name])
            )
            low, high = self.settings.position_limits(joint_name)
            one_unit_angle = abs(
                self.model.position_to_model_angle(joint_name, min(high, low + 1))
                - self.model.position_to_model_angle(joint_name, low)
            )
            refinement_allowance = one_unit_angle * max(
                1, int(self.settings.local_search_step_units) + 1
            )
            if (
                abs(target_angle - current_angle)
                > base_allowed_rad + refinement_allowance + 1e-9
            ):
                return False
        return True

    def step_cartesian(
        self, dt: float, *, run_time_s: Optional[float] = None
    ) -> bool:
        """Advance an interactive hold through absolute grasp-point targets."""

        velocity = self.cartesian_velocity()
        if float(np.linalg.norm(velocity)) < 1e-9:
            return False
        if self._motion_target_tcp is None:
            self._motion_target_tcp = self.model.tcp(self.positions).copy()
        next_target = self._motion_target_tcp + velocity * float(dt)
        if self._height_hold_active and self._locked_height_m is not None:
            next_target[2] = self._locked_height_m

        plan = self.plan_grasp_target(next_target)
        if not bool(plan["accepted"]):
            return True
        if not bool(plan["strict_pose"]):
            self.logger(
                "V2 GUI continuous control rejected relaxed pose; "
                "use a timed down action so feedback can be verified"
            )
            return True
        target_positions = dict(plan["target_joint_positions"])
        execution_time_s = dt if run_time_s is None else run_time_s
        if not self.joint_pose_within_motion_budget(
            target_positions, run_time_s=execution_time_s
        ):
            self.logger("V2 GUI pose rejected: joint motion exceeds the time budget")
            return True
        self.command_joint_pose(target_positions, run_time_s=execution_time_s)
        if bool(plan["strict_pose"]):
            self._motion_target_tcp = next_target
        else:
            self._motion_target_tcp = self.model.tcp(self.positions).copy()
        self._last_interactive_target_tcp = self._motion_target_tcp.copy()
        angle_deg = self.camera_line_vertical_angle_deg()
        self.logger(f"camera-grasp line angle from vertical: {angle_deg:.1f}deg")
        return True

    def finish_interactive_motion(self) -> dict[str, object]:
        """Verify/correct a released GUI hold against its absolute target."""

        if self._last_interactive_target_tcp is None:
            return {"status": "idle", "corrected": False}
        if (
            abs(self.vertical_direction) > HORIZONTAL_PROJECTION_EPSILON
            or math.hypot(self.joystick_x, self.joystick_y)
            > HORIZONTAL_PROJECTION_EPSILON
        ):
            return {"status": "active", "corrected": False}
        target_tcp = self._last_interactive_target_tcp.copy()
        self.refresh_joint_positions_from_controller()
        actual_tcp = self.model.tcp(self.positions).copy()
        actual_angle = self._camera_line_vertical_angle_rad(self.positions)
        target_angle = (
            self._pose_reference_line_angle_rad
            if self._pose_reference_line_angle_rad is not None
            else actual_angle
        )
        position_error = float(np.linalg.norm(actual_tcp - target_tcp))
        inclination_error = abs(actual_angle - target_angle)
        if (
            position_error <= IK_POSITION_TOLERANCE_M
            and inclination_error <= IK_INCLINATION_TOLERANCE_RAD
        ):
            attempts = self._interactive_finish_attempts
            self.discard_interactive_motion_target()
            self.logger(
                "GUI release verified: "
                f"position error={position_error * 1000.0:.1f}mm, "
                f"angle error={math.degrees(inclination_error):.2f}deg"
            )
            return {
                "status": "ok",
                "verified": True,
                "corrected": attempts > 0,
                "actual_camera_line_angle_deg": math.degrees(actual_angle),
                "position_error_m": position_error,
                "inclination_error_deg": math.degrees(inclination_error),
            }
        if self._interactive_finish_attempts >= MAX_FEEDBACK_CORRECTIONS:
            attempts = self._interactive_finish_attempts
            self.discard_interactive_motion_target()
            message = (
                "GUI release correction did not reach the absolute pose: "
                f"position={position_error * 1000.0:.1f}mm, "
                f"inclination={math.degrees(inclination_error):.2f}deg"
            )
            self.logger(message)
            return {
                "status": "error",
                "verified": False,
                "corrected": attempts > 0,
                "error": message,
            }
        plan = self.plan_grasp_target(target_tcp)
        if not bool(plan["accepted"]) or not bool(plan["strict_pose"]):
            message = str(
                plan.get("message")
                or "released GUI target cannot keep the camera-grasp pose"
            )
            self.logger(message)
            self.discard_interactive_motion_target()
            return {
                "status": "error",
                "corrected": False,
                "error": message,
                "motion_plan": plan,
            }
        if not self.joint_pose_within_motion_budget(
            dict(plan["target_joint_positions"]),
            run_time_s=MIN_FEEDBACK_CORRECTION_S,
        ):
            self.discard_interactive_motion_target()
            message = "GUI release correction exceeds the joint-motion time budget"
            self.logger(message)
            return {
                "status": "error",
                "verified": False,
                "corrected": False,
                "error": message,
                "motion_plan": plan,
            }
        corrected = self.command_joint_pose(
            dict(plan["target_joint_positions"]),
            run_time_s=MIN_FEEDBACK_CORRECTION_S,
        )
        if not corrected:
            self.discard_interactive_motion_target()
            message = "GUI release correction made no joint progress"
            self.logger(message)
            return {
                "status": "error",
                "verified": False,
                "corrected": False,
                "error": message,
                "motion_plan": plan,
            }
        self._interactive_finish_attempts += 1
        self._interactive_correction_busy = True
        self.logger(
            "GUI release correction: "
            f"target angle={float(plan['target_camera_line_angle_deg']):.2f}deg, "
            f"attempt={self._interactive_finish_attempts}"
        )
        return {
            "status": "correcting",
            "verified": False,
            "corrected": True,
            "retry_after_s": MIN_FEEDBACK_CORRECTION_S + MOTION_SETTLE_S,
            "motion_plan": plan,
        }

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

    def _joint_continuity_rank(
        self, positions: dict[str, int]
    ) -> tuple[float, float, float]:
        normalized_motion: list[float] = []
        normalized_margins: list[float] = []
        for joint_name in ARM_JOINTS:
            low, high = self.settings.position_limits(joint_name)
            span = max(1, high - low)
            normalized_motion.append(
                abs(positions[joint_name] - self.positions[joint_name]) / span
            )
            normalized_margins.append(
                min(positions[joint_name] - low, high - positions[joint_name])
                / span
            )
        return (
            max(normalized_motion),
            sum(value * value for value in normalized_motion),
            -min(normalized_margins),
        )


def apply_terminal_motion_input(
    runtime: CameraVectorV2Runtime,
    direction: str,
    speed_cm_s: float,
) -> dict[str, object]:
    """Apply the same normalized input used by the V2 terminal controls."""

    if direction not in TERMINAL_MOTION_DIRECTIONS:
        raise ValueError(f"unsupported V2 terminal direction: {direction}")
    speed = float(speed_cm_s)
    if not math.isfinite(speed) or speed <= 0.0:
        raise ValueError("motion speed must be a positive finite number")

    horizontal = direction in {"forward", "backward", "left", "right"}
    maximum_speed_cm_s = (
        runtime.settings.max_horizontal_speed_m_s
        if horizontal
        else runtime.settings.vertical_speed_m_s
    ) * 100.0
    if speed > maximum_speed_cm_s:
        raise ValueError(
            f"motion speed cannot exceed {maximum_speed_cm_s:g} cm/s"
        )
    input_ratio = speed / maximum_speed_cm_s

    runtime.set_vertical_direction(0.0)
    runtime.center_joystick()
    if direction == "forward":
        runtime.set_joystick(0.0, -input_ratio)
    elif direction == "backward":
        runtime.set_joystick(0.0, input_ratio)
    elif direction == "left":
        runtime.set_joystick(-input_ratio, 0.0)
    elif direction == "right":
        runtime.set_joystick(input_ratio, 0.0)
    elif direction == "up":
        runtime.set_vertical_direction(input_ratio)
    else:
        runtime.set_vertical_direction(-input_ratio)

    frame = runtime.active_camera_relative_frame()
    vector = getattr(frame, direction)
    return {
        "direction": direction,
        "speed_cm_s": speed,
        "maximum_speed_cm_s": maximum_speed_cm_s,
        "input_ratio": input_ratio,
        "joystick_x": float(runtime.joystick_x),
        "joystick_y": float(runtime.joystick_y),
        "vertical_direction": float(runtime.vertical_direction),
        "direction_unit_base": [float(value) for value in vector],
    }


def release_terminal_motion_input(runtime: CameraVectorV2Runtime) -> None:
    """Release V2 terminal controls exactly like mouse/button release."""

    try:
        runtime.set_vertical_direction(0.0)
        runtime.center_joystick()
    except Exception as exc:
        runtime.logger(f"V2 input release recovered from feedback error: {exc}")
    finally:
        # A failed feedback read can happen while an input is being activated.
        # Clear the raw state without invoking another motion-lock refresh so
        # an exception can never leave a direction command latched.
        runtime.vertical_direction = 0.0
        runtime.joystick_x = 0.0
        runtime.joystick_y = 0.0
        runtime._clear_motion_lock()


async def execute_terminal_motion(
    runtime: CameraVectorV2Runtime,
    *,
    direction: str,
    speed_cm_s: float,
    duration_s: float,
    real_time: bool,
    sleep: SleepFunction = asyncio.sleep,
    monotonic: Callable[[], float] = time.monotonic,
    collect_tcp_samples: bool = False,
) -> dict[str, object]:
    """Plan and execute one terminal action as one absolute TCP target.

    Both the shell entry point and the manual-pixel V2 adapter use this exact
    path: input action -> new grasp coordinate -> best pose -> synchronized
    joint command -> feedback verification.
    """

    del monotonic  # Retained in the public signature for compatibility.
    duration = float(duration_s)
    if not math.isfinite(duration) or duration <= 0.0:
        raise ValueError("motion duration must be a positive finite number")
    runtime.reset_pose_relaxation_status()
    try:
        terminal_input = apply_terminal_motion_input(runtime, direction, speed_cm_s)
    except BaseException:
        release_terminal_motion_input(runtime)
        runtime.discard_interactive_motion_target()
        raise
    samples_m: list[list[float]] = []
    feedback_corrections = 0
    relaxed_iterations = 0
    steps = 0
    error: str | None = None

    start_tcp = runtime.model.tcp(runtime.positions).copy()
    direction_unit = np.asarray(
        terminal_input["direction_unit_base"], dtype=float
    )
    target_tcp = start_tcp + direction_unit * (
        float(speed_cm_s) * duration / 100.0
    )
    if direction in {"forward", "backward", "left", "right"}:
        target_tcp[2] = start_tcp[2]

    if collect_tcp_samples:
        samples_m.append([float(value) for value in start_tcp])

    try:
        plan = runtime.plan_grasp_target(target_tcp)
    except BaseException:
        release_terminal_motion_input(runtime)
        runtime.discard_interactive_motion_target()
        raise
    final_plan = plan
    status = "ok" if bool(plan["accepted"]) else "error"
    if status == "error":
        error = str(plan.get("message") or "V2 absolute grasp target was rejected")
    initial_run_time = (
        duration if bool(plan["strict_pose"]) else float(runtime.settings.tick_s)
    )
    if status == "ok" and not runtime.joint_pose_within_motion_budget(
        dict(plan["target_joint_positions"]),
        run_time_s=initial_run_time,
    ):
        status = "error"
        error = "V2 target joint motion exceeds the requested time budget"

    try:
        if status == "ok":
            changed = runtime.command_joint_pose(
                dict(plan["target_joint_positions"]),
                run_time_s=initial_run_time,
            )
            if changed and not bool(plan["strict_pose"]):
                runtime.mark_pose_relaxation_executed()
            steps = 1 if changed else 0
            if real_time:
                await sleep(initial_run_time + MOTION_SETTLE_S)
            else:
                await sleep(0.0)
            runtime.refresh_joint_positions_from_controller()

            if not bool(plan["strict_pose"]):
                max_relaxed_steps = max(
                    1,
                    int(math.ceil(duration / float(runtime.settings.tick_s))),
                )
                previous_tcp = start_tcp.copy()
                while steps < max_relaxed_steps:
                    actual_tcp = runtime.model.tcp(runtime.positions).copy()
                    position_error = float(np.linalg.norm(actual_tcp - target_tcp))
                    if position_error <= IK_POSITION_TOLERANCE_M:
                        break
                    step_progress = float(
                        np.dot(actual_tcp - previous_tcp, direction_unit)
                    )
                    if steps > 0 and step_progress <= 1e-6:
                        status = "error"
                        error = "V2 relaxed downward motion made no forward progress"
                        break
                    if (
                        direction == "down"
                        and steps > 0
                        and actual_tcp[2] >= previous_tcp[2] - 1e-6
                    ):
                        status = "error"
                        error = "V2 relaxed downward motion did not reduce grasp height"
                        break
                    previous_tcp = actual_tcp
                    next_plan = runtime.plan_grasp_target(target_tcp)
                    final_plan = next_plan
                    if not bool(next_plan["accepted"]):
                        status = "error"
                        error = str(
                            next_plan.get("message")
                            or "V2 relaxed downward target was rejected"
                        )
                        break
                    step_run_time = (
                        max(
                            float(runtime.settings.tick_s),
                            duration
                            - steps * float(runtime.settings.tick_s),
                        )
                        if bool(next_plan["strict_pose"])
                        else float(runtime.settings.tick_s)
                    )
                    if not runtime.joint_pose_within_motion_budget(
                        dict(next_plan["target_joint_positions"]),
                        run_time_s=step_run_time,
                    ):
                        status = "error"
                        error = (
                            "V2 relaxed/strict transition exceeds the remaining "
                            "joint-motion time budget"
                        )
                        break
                    if not runtime.command_joint_pose(
                        dict(next_plan["target_joint_positions"]),
                        run_time_s=step_run_time,
                    ):
                        status = "error"
                        error = "V2 relaxed downward solver made no joint progress"
                        break
                    relaxed_iterations += 1
                    steps += 1
                    if real_time:
                        await sleep(step_run_time + MOTION_SETTLE_S)
                    else:
                        await sleep(0.0)
                    runtime.refresh_joint_positions_from_controller()

            if real_time and bool(plan["strict_pose"]):
                correction_duration = min(
                    MAX_FEEDBACK_CORRECTION_S,
                    max(MIN_FEEDBACK_CORRECTION_S, duration * 0.25),
                )
                while feedback_corrections < MAX_FEEDBACK_CORRECTIONS:
                    actual_tcp = runtime.model.tcp(runtime.positions)
                    actual_angle = runtime._camera_line_vertical_angle_rad(
                        runtime.positions
                    )
                    target_angle = math.radians(
                        float(plan["target_camera_line_angle_deg"])
                    )
                    if (
                        float(np.linalg.norm(actual_tcp - target_tcp))
                        <= IK_POSITION_TOLERANCE_M
                        and abs(actual_angle - target_angle)
                        <= IK_INCLINATION_TOLERANCE_RAD
                    ):
                        break
                    correction_plan = runtime.plan_grasp_target(target_tcp)
                    final_plan = correction_plan
                    if (
                        not bool(correction_plan["accepted"])
                        or not bool(correction_plan["strict_pose"])
                    ):
                        break
                    if not runtime.joint_pose_within_motion_budget(
                        dict(correction_plan["target_joint_positions"]),
                        run_time_s=correction_duration,
                    ):
                        break
                    if not runtime.command_joint_pose(
                        dict(correction_plan["target_joint_positions"]),
                        run_time_s=correction_duration,
                    ):
                        break
                    feedback_corrections += 1
                    steps += 1
                    await sleep(correction_duration + MOTION_SETTLE_S)
                    runtime.refresh_joint_positions_from_controller()

            actual_tcp = runtime.model.tcp(runtime.positions).copy()
            actual_angle = runtime._camera_line_vertical_angle_rad(runtime.positions)
            target_angle = math.radians(float(plan["target_camera_line_angle_deg"]))
            position_error = float(np.linalg.norm(actual_tcp - target_tcp))
            inclination_error = abs(actual_angle - target_angle)
            if status == "ok" and not bool(plan["strict_pose"]):
                direction_progress = float(
                    np.dot(actual_tcp - start_tcp, direction_unit)
                )
                if (
                    position_error > IK_POSITION_TOLERANCE_M
                    or direction_progress <= 1e-6
                ):
                    status = "error"
                    error = (
                        "V2 relaxed downward motion did not reach the absolute "
                        f"grasp target: position={position_error * 1000.0:.1f}mm, "
                        f"direction_progress={direction_progress * 1000.0:.1f}mm"
                    )
            elif (
                bool(plan["strict_pose"])
                and (
                    position_error > IK_POSITION_TOLERANCE_M
                    or inclination_error > IK_INCLINATION_TOLERANCE_RAD
                )
            ):
                status = "error"
                error = (
                    "V2 feedback did not reach the absolute pose target: "
                    f"position={position_error * 1000.0:.1f}mm, "
                    f"inclination={math.degrees(inclination_error):.2f}deg"
                )
        else:
            actual_tcp = runtime.model.tcp(runtime.positions).copy()
            actual_angle = runtime._camera_line_vertical_angle_rad(runtime.positions)
            position_error = float(np.linalg.norm(actual_tcp - target_tcp))
            inclination_error = abs(
                actual_angle
                - math.radians(float(plan["target_camera_line_angle_deg"]))
            )

        direction_progress = float(
            np.dot(actual_tcp - start_tcp, direction_unit)
        )
        if (
            status == "ok"
            and float(speed_cm_s) * duration > 0.0
            and direction_progress <= 1e-6
        ):
            status = "error"
            error = "V2 non-zero motion request made no directional progress"

        if collect_tcp_samples:
            samples_m.append([float(value) for value in actual_tcp])
    finally:
        release_terminal_motion_input(runtime)
        runtime.discard_interactive_motion_target()

    result: dict[str, object] = {
        "status": status,
        "execution_path": "camera_vector_terminal_v2_absolute_grasp_target",
        "workflow": [
            "terminal_input",
            "absolute_grasp_target",
            "best_pose_ik",
            "synchronized_joint_adjustment",
            "joint_feedback_verification",
        ],
        "terminal_input": terminal_input,
        "duration_s": duration,
        "requested_distance_cm": float(speed_cm_s) * duration,
        "start_grasp_point_m": [float(value) for value in start_tcp],
        "target_grasp_point_m": [float(value) for value in target_tcp],
        "actual_grasp_point_m": [float(value) for value in actual_tcp],
        "motion_plan": plan,
        "final_motion_plan": final_plan,
        "target_joint_positions": dict(final_plan["target_joint_positions"]),
        "joint_positions": dict(runtime.positions),
        "target_camera_line_angle_deg": float(
            plan["target_camera_line_angle_deg"]
        ),
        "actual_camera_line_angle_deg": math.degrees(actual_angle),
        "camera_line_angle_error_deg": math.degrees(inclination_error),
        "position_error_m": position_error,
        "direction_progress_m": direction_progress,
        "feedback_corrections": feedback_corrections,
        "relaxed_iterations": relaxed_iterations,
        "steps": steps,
        "tcp_samples_m": samples_m,
        "camera_pose_reference": runtime.camera_pose_reference_status(),
        "camera_pose_constraint": runtime.pose_relaxation_status(),
    }
    if error is not None:
        result["error"] = error
    return result


class CameraVectorV2App(CameraVectorTerminalApp):
    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._interactive_finish_scheduled = False
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

    def _release_hold(self, release: Callable[[], Any]) -> None:
        was_active = release in self.active_releases
        super()._release_hold(release)
        if was_active:
            self._schedule_interactive_pose_finish()

    def _release_joystick(self) -> None:
        was_active = self.joystick_active
        super()._release_joystick()
        if was_active:
            self._schedule_interactive_pose_finish()

    def _schedule_interactive_pose_finish(self) -> None:
        if self._interactive_finish_scheduled:
            return
        if not self.runtime.begin_interactive_motion_finish():
            return
        delay_ms = max(
            10,
            int(round((self.runtime.settings.tick_s + MOTION_SETTLE_S) * 1000.0)),
        )
        self._interactive_finish_scheduled = True
        self.root.after(
            delay_ms,
            self._finish_interactive_pose_round,
        )

    def _finish_interactive_pose_round(self) -> None:
        self._interactive_finish_scheduled = False
        try:
            result = self.runtime.finish_interactive_motion()
            self._refresh_velocity()
        except Exception as exc:
            self.append_log(f"ERROR: GUI release verification failed: {exc}")
            return
        if result.get("status") == "correcting":
            retry_after_s = float(
                result.get(
                    "retry_after_s",
                    MIN_FEEDBACK_CORRECTION_S + MOTION_SETTLE_S,
                )
            )
            self.root.after(
                max(10, int(round(retry_after_s * 1000.0))),
                self._finish_interactive_pose_round,
            )
            self._interactive_finish_scheduled = True


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="JetArm camera-line-relative terminal V2 for Ubuntu 22.04"
    )
    parser.add_argument("--config", default=str(DEFAULT_CONFIG_PATH), help="terminal JSON config")
    parser.add_argument("--port", default=None, help="serial device, for example /dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=None, help="override configured baudrate")
    parser.add_argument("--timeout", type=float, default=None, help="override serial timeout in seconds")
    parser.add_argument("--dry-run", action="store_true", help="run without a real serial device")
    parser.add_argument("--list-ports", action="store_true", help="list detected USB serial devices and exit")
    parser.add_argument(
        "--diagnose-ports",
        action="store_true",
        help="diagnose USB-visible devices that have no Linux tty node",
    )
    parser.add_argument(
        "--motion-direction",
        choices=TERMINAL_MOTION_DIRECTIONS,
        default=None,
        help="headless terminal input direction",
    )
    parser.add_argument(
        "--motion-speed-cm-s",
        type=float,
        default=None,
        help="headless terminal input speed in cm/s",
    )
    parser.add_argument(
        "--motion-duration-s",
        type=float,
        default=None,
        help="headless terminal input hold duration in seconds",
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
        motion_values = (
            args.motion_direction,
            args.motion_speed_cm_s,
            args.motion_duration_s,
        )
        motion_requested = any(value is not None for value in motion_values)
        if motion_requested and not all(value is not None for value in motion_values):
            raise ValueError(
                "--motion-direction, --motion-speed-cm-s and "
                "--motion-duration-s must be provided together"
            )

        if motion_requested:
            logger = print
            selected_port: Optional[str] = None
            if args.dry_run:
                controller: Any = DryRunServoController(settings, logger=logger)
            else:
                selected_port = select_linux_serial_port(args.port)
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
            try:
                runtime.initialize(use_home_positions=args.dry_run)
                result = asyncio.run(
                    execute_terminal_motion(
                        runtime,
                        direction=args.motion_direction,
                        speed_cm_s=args.motion_speed_cm_s,
                        duration_s=args.motion_duration_s,
                        real_time=not args.dry_run,
                        collect_tcp_samples=True,
                    )
                )
            finally:
                runtime.close()
            print(json.dumps(result, ensure_ascii=False))
            return 0

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
