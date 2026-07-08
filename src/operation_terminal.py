"""Visual manual operation terminal for directly controlling JetArm servos.

The UI is intentionally thin: mouse/touch gestures are translated into calls on
``BusServoController``.  J5/J6 use motor-speed commands.  Cartesian TCP motion
uses short, repeated position commands for J1-J4 based on the current
kinematic model.
"""

from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Optional

import numpy as np

try:
    import tkinter as tk
    from tkinter import messagebox, ttk
except ImportError:  # pragma: no cover - exercised only on minimal Python builds
    tk = None
    ttk = None
    messagebox = None

try:
    from .arm_model import JetArmKinematicModel
    from .bus_servo import DEFAULT_BAUDRATE, BusServoController
    from .joint_controller import DEFAULT_CONFIG_PATH, load_joint_config
except ImportError:
    from arm_model import JetArmKinematicModel  # type: ignore
    from bus_servo import DEFAULT_BAUDRATE, BusServoController  # type: ignore
    from joint_controller import DEFAULT_CONFIG_PATH, load_joint_config  # type: ignore


DEFAULT_HOME_POSITIONS = {
    "joint1_base_yaw": 485,
    "joint2_shoulder_pitch": 478,
    "joint3_elbow_pitch": 641,
    "joint4_wrist_pitch": 890,
    "joint5_wrist_roll": 500,
}


@dataclass
class OperationTerminalConfig:
    config_path: Path = DEFAULT_CONFIG_PATH
    com_port: str = "COM15"
    baudrate: int = DEFAULT_BAUDRATE
    timeout_s: float = 0.2
    tick_s: float = 0.08
    vertical_speed_m_s: float = 0.05
    max_horizontal_speed_m_s: float = 0.05
    j5_speed: int = 100
    j6_speed: int = 100
    j6_grip_speed: int = 300
    home_run_time_ms: int = 1200
    max_joint_step_deg: float = 4.0
    damping: float = 0.02
    local_search_step_units: int = 8
    home_positions: dict[str, int] = field(default_factory=lambda: dict(DEFAULT_HOME_POSITIONS))


class DryRunServoController:
    """Servo-controller stand-in used by ``--dry-run`` and tests."""

    def __init__(self, positions: Optional[dict[int, int]] = None, logger: Optional[Callable[[str], None]] = None) -> None:
        self.positions = dict(positions or {})
        self.logger = logger or (lambda _message: None)
        self.move_calls: list[tuple[int, int, int]] = []
        self.motor_calls: list[tuple[int, int]] = []
        self.closed = False

    def read_position(self, servo_id: int) -> int:
        value = int(self.positions.get(servo_id, 500))
        self.logger(f"DRY read_position servo={servo_id} -> {value}")
        return value

    def move_servo(self, servo_id: int, target_position: int, run_time_ms: int) -> None:
        self.positions[int(servo_id)] = int(target_position)
        self.move_calls.append((int(servo_id), int(target_position), int(run_time_ms)))
        self.logger(f"DRY move_servo servo={servo_id} target={target_position} run_time_ms={run_time_ms}")

    def set_motor_speed(self, servo_id: int, speed: int) -> None:
        self.motor_calls.append((int(servo_id), int(speed)))
        self.logger(f"DRY set_motor_speed servo={servo_id} speed={speed}")

    def close(self) -> None:
        self.closed = True


class ManualServoRuntime:
    """Runtime state machine for the visual operation terminal."""

    def __init__(
        self,
        controller: Any,
        model: JetArmKinematicModel,
        config: OperationTerminalConfig,
        *,
        logger: Optional[Callable[[str], None]] = None,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        self.controller = controller
        self.model = model
        self.config = config
        self.logger = logger or (lambda _message: None)
        self.monotonic = monotonic

        self.arm_position_joints = tuple(model.arm_joint_names[:4])
        self.j5_joint = model.resolve_joint_name("J5")
        self.j6_joint = model.resolve_joint_name("J6")
        self.servo_ids = {name: int(model.joints[name]["servo_id"]) for name in model.joints}
        self.positions: dict[str, int] = {}
        self.vertical_direction = 0
        self.joystick_x = 0.0
        self.joystick_y = 0.0
        self.j6_grip_locked = False
        self.last_step_at: Optional[float] = None

    def initialize(self, *, use_home_positions: bool = False) -> None:
        """Load the initial arm positions before the control loop starts."""

        self.positions.clear()
        for joint_name in self.arm_position_joints:
            if use_home_positions:
                position = int(self.config.home_positions[joint_name])
            else:
                position = int(self.controller.read_position(self.servo_ids[joint_name]))
            self._validate_position(joint_name, position)
            self.positions[joint_name] = position
        self.last_step_at = self.monotonic()
        self.logger("操作终端已就绪，J1-J4 当前位姿已载入")

    def set_vertical_direction(self, direction: int) -> None:
        self.vertical_direction = max(-1, min(1, int(direction)))
        if self.vertical_direction > 0:
            self.logger("TCP 上升速度: 5 cm/s")
        elif self.vertical_direction < 0:
            self.logger("TCP 下降速度: 5 cm/s")
        else:
            self.logger("TCP 上下运动停止")

    def set_joystick(self, x: float, y: float) -> tuple[float, float]:
        self.joystick_x, self.joystick_y = clamp_unit_circle(x, y)
        return self.joystick_x, self.joystick_y

    def center_joystick(self) -> None:
        self.set_joystick(0.0, 0.0)
        self.logger("水平摇杆回中")

    def set_j5_speed(self, speed: int) -> None:
        self.controller.set_motor_speed(self.servo_ids[self.j5_joint], int(speed))
        self.logger(f"J5 速度: {speed}")

    def rotate_j5_counterclockwise(self) -> None:
        self.set_j5_speed(-abs(self.config.j5_speed))

    def rotate_j5_clockwise(self) -> None:
        self.set_j5_speed(abs(self.config.j5_speed))

    def stop_j5(self) -> None:
        self.set_j5_speed(0)

    def set_j6_speed(self, speed: int) -> bool:
        if self.j6_grip_locked:
            self.logger("J6 抓紧锁定中，忽略松/闭误触")
            return False
        self.controller.set_motor_speed(self.servo_ids[self.j6_joint], int(speed))
        self.logger(f"J6 速度: {speed}")
        return True

    def open_j6(self) -> bool:
        return self.set_j6_speed(-abs(self.config.j6_speed))

    def close_j6(self) -> bool:
        return self.set_j6_speed(abs(self.config.j6_speed))

    def stop_j6(self) -> bool:
        return self.set_j6_speed(0)

    def toggle_grip_lock(self) -> bool:
        self.j6_grip_locked = not self.j6_grip_locked
        if self.j6_grip_locked:
            self.controller.set_motor_speed(self.servo_ids[self.j6_joint], abs(self.config.j6_grip_speed))
            self.logger("J6 抓紧锁定: ON，速度 300")
        else:
            self.controller.set_motor_speed(self.servo_ids[self.j6_joint], 0)
            self.logger("J6 抓紧锁定: OFF，J6 停止")
        return self.j6_grip_locked

    def stop_all(self) -> None:
        self.vertical_direction = 0
        self.set_joystick(0.0, 0.0)
        self.controller.set_motor_speed(self.servo_ids[self.j5_joint], 0)
        self.controller.set_motor_speed(self.servo_ids[self.j6_joint], 0)
        self.j6_grip_locked = False
        self.logger("全部停止")

    def go_home(self) -> None:
        self.vertical_direction = 0
        self.set_joystick(0.0, 0.0)
        self.controller.set_motor_speed(self.servo_ids[self.j5_joint], 0)
        for joint_name, target in self.config.home_positions.items():
            if joint_name == self.j6_joint:
                continue
            servo_id = self.servo_ids[joint_name]
            self.controller.move_servo(servo_id, int(target), self.config.home_run_time_ms)
            if joint_name in self.arm_position_joints:
                self.positions[joint_name] = int(target)
        self.logger("已发送 home 位姿")

    def close(self) -> None:
        try:
            self.stop_all()
        finally:
            close = getattr(self.controller, "close", None)
            if callable(close):
                close()

    def tick(self) -> None:
        now = self.monotonic()
        if self.last_step_at is None:
            self.last_step_at = now
            return
        dt = max(0.001, min(0.25, now - self.last_step_at))
        self.last_step_at = now
        self.step_cartesian(dt)

    def step_cartesian(self, dt: float) -> bool:
        velocity = self.cartesian_velocity()
        if np.linalg.norm(velocity) < 1e-9:
            return False

        target_delta = velocity * float(dt)
        target_positions = self._solve_next_positions(target_delta)
        if target_positions == self.positions:
            self.logger("TCP 目标已到达关节限位或当前姿态无法继续")
            return False

        run_time_ms = max(1, int(round(dt * 1000)))
        for joint_name, target in target_positions.items():
            if target == self.positions.get(joint_name):
                continue
            self.controller.move_servo(self.servo_ids[joint_name], target, run_time_ms)
            self.positions[joint_name] = target
        return True

    def cartesian_velocity(self) -> np.ndarray:
        # Canvas coordinates: +x is screen right, +y is screen down.
        # Robot coordinates: +x forward, +y left, +z up.
        vx_forward = -self.joystick_y * self.config.max_horizontal_speed_m_s
        vy_left = -self.joystick_x * self.config.max_horizontal_speed_m_s
        vz_up = self.vertical_direction * self.config.vertical_speed_m_s
        return np.array([vx_forward, vy_left, vz_up], dtype=float)

    def _solve_next_positions(self, target_delta: np.ndarray) -> dict[str, int]:
        current_positions = dict(self.positions)
        jacobian = np.array(self.model.tcp_jacobian_from_positions(current_positions), dtype=float)[:, :4]
        damping_matrix = (self.config.damping ** 2) * np.eye(3)

        try:
            dq = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + damping_matrix, target_delta)
        except np.linalg.LinAlgError:
            dq = np.zeros(len(self.arm_position_joints), dtype=float)

        max_step_rad = math.radians(self.config.max_joint_step_deg)
        dq = np.clip(dq, -max_step_rad, max_step_rad)

        candidate = self._positions_from_delta_q(dq)
        return self._local_refine_positions(candidate, target_delta)

    def _positions_from_delta_q(self, dq: np.ndarray) -> dict[str, int]:
        positions = dict(self.positions)
        for index, joint_name in enumerate(self.arm_position_joints):
            current_q = self.model.position_to_model_angle_rad(joint_name, self.positions[joint_name])
            next_q = self._clamp_model_angle(joint_name, current_q + float(dq[index]))
            positions[joint_name] = self.model.model_angle_rad_to_servo_position(joint_name, next_q)
            positions[joint_name] = self._clamp_position(joint_name, positions[joint_name])
        return positions

    def _local_refine_positions(self, seed: dict[str, int], target_delta: np.ndarray) -> dict[str, int]:
        current_tcp = np.array(self.model.forward_kinematics_from_positions(self.positions).tcp_xyz, dtype=float)
        target_tcp = current_tcp + target_delta
        best = dict(seed)
        best_error = self._tcp_error(best, target_tcp)
        step = max(1, int(self.config.local_search_step_units))

        candidates: list[dict[str, int]] = [dict(self.positions), dict(seed)]
        for joint_name in self.arm_position_joints:
            for direction in (-1, 1):
                candidate = dict(seed)
                candidate[joint_name] = self._clamp_position(joint_name, candidate[joint_name] + direction * step)
                candidates.append(candidate)

        for j2_direction in (-1, 1):
            pitch_candidate = dict(seed)
            for joint_name in self.arm_position_joints[1:]:
                pitch_candidate[joint_name] = self._clamp_position(
                    joint_name,
                    pitch_candidate[joint_name] + j2_direction * step,
                )
            candidates.append(pitch_candidate)

        for candidate in candidates:
            error = self._tcp_error(candidate, target_tcp)
            if error + 1e-12 < best_error:
                best = candidate
                best_error = error

        current_error = self._tcp_error(self.positions, target_tcp)
        return best if best_error + 1e-12 < current_error else dict(self.positions)

    def _tcp_error(self, positions: dict[str, int], target_tcp: np.ndarray) -> float:
        tcp = np.array(self.model.forward_kinematics_from_positions(positions).tcp_xyz, dtype=float)
        return float(np.linalg.norm(tcp - target_tcp))

    def _validate_position(self, joint_name: str, position: int) -> None:
        minimum, maximum = self._position_limits(joint_name)
        if not minimum <= position <= maximum:
            raise ValueError(f"{joint_name} position {position} outside {minimum}..{maximum}")

    def _clamp_position(self, joint_name: str, position: int) -> int:
        minimum, maximum = self._position_limits(joint_name)
        return max(minimum, min(maximum, int(round(position))))

    def _position_limits(self, joint_name: str) -> tuple[int, int]:
        position = self.model.joints[joint_name]["position"]
        return int(position["min"]), int(position["max"])

    def _clamp_model_angle(self, joint_name: str, value: float) -> float:
        joint = self.model.joints[joint_name]
        angle = joint["angle"]
        sign = float(joint.get("direction_sign") or 1.0)
        low = math.radians(sign * float(angle["min_deg"]))
        high = math.radians(sign * float(angle["max_deg"]))
        if low > high:
            low, high = high, low
        return max(low, min(high, value))


def clamp_unit_circle(x: float, y: float) -> tuple[float, float]:
    length = math.hypot(float(x), float(y))
    if length <= 1.0:
        return float(x), float(y)
    if length == 0:
        return 0.0, 0.0
    return float(x) / length, float(y) / length


class OperationTerminalApp:
    def __init__(self, root: Any, runtime: ManualServoRuntime, *, dry_run: bool = False) -> None:
        if tk is None or ttk is None:
            raise RuntimeError("Tkinter is not available in this Python installation")
        self.root = root
        self.runtime = runtime
        self.dry_run = dry_run
        self.joystick_radius = 118
        self.joystick_center = 135
        self.knob_radius = 26
        self.active_releases: list[Callable[[], Any]] = []
        self.joystick_active = False

        self.root.title("JetArm 操作终端")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.bind_all("<ButtonRelease-1>", self._on_global_mouse_release, add="+")
        self.root.configure(bg="#f6f8fb")
        self._build_styles()
        self._build_layout()
        self._schedule_tick()

    def _build_styles(self) -> None:
        style = ttk.Style()
        style.configure("Terminal.TFrame", background="#f6f8fb")
        style.configure("Panel.TFrame", background="#ffffff", relief="solid", borderwidth=1)
        style.configure("Title.TLabel", background="#f6f8fb", foreground="#142033", font=("Microsoft YaHei UI", 14, "bold"))
        style.configure("Value.TLabel", background="#ffffff", foreground="#142033", font=("Microsoft YaHei UI", 10))
        style.configure("Command.TButton", font=("Microsoft YaHei UI", 11), padding=(10, 8))
        style.configure("Stop.TButton", font=("Microsoft YaHei UI", 12, "bold"), padding=(10, 10))

    def _build_layout(self) -> None:
        outer = ttk.Frame(self.root, style="Terminal.TFrame", padding=16)
        outer.grid(row=0, column=0, sticky="nsew")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        outer.columnconfigure(1, weight=1)

        title = "JetArm 操作终端"
        if self.dry_run:
            title += " (dry-run)"
        ttk.Label(outer, text=title, style="Title.TLabel").grid(row=0, column=0, columnspan=4, sticky="w", pady=(0, 12))

        self._build_vertical_panel(outer).grid(row=1, column=0, sticky="nsew", padx=(0, 12))
        self._build_joystick_panel(outer).grid(row=1, column=1, sticky="nsew", padx=(0, 12))
        self._build_j5_panel(outer).grid(row=1, column=2, sticky="nsew", padx=(0, 12))
        self._build_j6_panel(outer).grid(row=1, column=3, sticky="nsew")
        self._build_status_panel(outer).grid(row=2, column=0, columnspan=4, sticky="nsew", pady=(12, 0))

    def _build_vertical_panel(self, parent: Any) -> Any:
        frame = ttk.Frame(parent, style="Panel.TFrame", padding=12)
        frame.columnconfigure(0, weight=1)
        ttk.Label(frame, text="上下", style="Value.TLabel").grid(row=0, column=0, sticky="w")

        up = ttk.Button(frame, text="▲ 上", style="Command.TButton")
        up.grid(row=1, column=0, sticky="ew", pady=(12, 8), ipady=20)
        self._bind_hold_button(up, lambda: self.runtime.set_vertical_direction(1), lambda: self.runtime.set_vertical_direction(0))

        down = ttk.Button(frame, text="▼ 下", style="Command.TButton")
        down.grid(row=2, column=0, sticky="ew", pady=(0, 16), ipady=20)
        self._bind_hold_button(down, lambda: self.runtime.set_vertical_direction(-1), lambda: self.runtime.set_vertical_direction(0))

        home = ttk.Button(frame, text="home", style="Command.TButton", command=self._safe_home)
        home.grid(row=3, column=0, sticky="ew", pady=(18, 0), ipady=14)
        return frame

    def _build_joystick_panel(self, parent: Any) -> Any:
        frame = ttk.Frame(parent, style="Panel.TFrame", padding=12)
        frame.columnconfigure(0, weight=1)
        ttk.Label(frame, text="前后左右", style="Value.TLabel").grid(row=0, column=0, sticky="w")

        size = self.joystick_center * 2
        self.joystick = tk.Canvas(frame, width=size, height=size, bg="#ffffff", highlightthickness=0)
        self.joystick.grid(row=1, column=0, pady=(8, 0))
        self._draw_joystick()
        self.joystick.bind("<ButtonPress-1>", self._on_joystick_move)
        self.joystick.bind("<B1-Motion>", self._on_joystick_move)
        self.joystick.bind("<ButtonRelease-1>", self._on_joystick_release)
        return frame

    def _build_j5_panel(self, parent: Any) -> Any:
        frame = ttk.Frame(parent, style="Panel.TFrame", padding=12)
        frame.columnconfigure(0, weight=1)
        ttk.Label(frame, text="J5", style="Value.TLabel").grid(row=0, column=0, sticky="w")

        ccw = ttk.Button(frame, text="↶ 逆时针", style="Command.TButton")
        ccw.grid(row=1, column=0, sticky="ew", pady=(28, 10), ipady=18)
        self._bind_hold_button(ccw, self.runtime.rotate_j5_counterclockwise, self.runtime.stop_j5)

        cw = ttk.Button(frame, text="↷ 顺时针", style="Command.TButton")
        cw.grid(row=2, column=0, sticky="ew", pady=(0, 10), ipady=18)
        self._bind_hold_button(cw, self.runtime.rotate_j5_clockwise, self.runtime.stop_j5)
        return frame

    def _build_j6_panel(self, parent: Any) -> Any:
        frame = ttk.Frame(parent, style="Panel.TFrame", padding=12)
        frame.columnconfigure(0, weight=1)
        ttk.Label(frame, text="J6", style="Value.TLabel").grid(row=0, column=0, sticky="w")

        open_button = ttk.Button(frame, text="松", style="Command.TButton")
        open_button.grid(row=1, column=0, sticky="ew", pady=(18, 10), ipady=18)
        self._bind_hold_button(open_button, self.runtime.open_j6, self.runtime.stop_j6)

        close_button = ttk.Button(frame, text="闭", style="Command.TButton")
        close_button.grid(row=2, column=0, sticky="ew", pady=(0, 18), ipady=18)
        self._bind_hold_button(close_button, self.runtime.close_j6, self.runtime.stop_j6)

        self.grip_button = tk.Button(
            frame,
            text="抓紧",
            command=self._toggle_grip,
            bg="#1fa463",
            fg="#ffffff",
            activebackground="#178a51",
            relief="flat",
            font=("Microsoft YaHei UI", 12, "bold"),
            padx=12,
            pady=12,
        )
        self.grip_button.grid(row=3, column=0, sticky="ew", pady=(8, 0))
        return frame

    def _build_status_panel(self, parent: Any) -> Any:
        frame = ttk.Frame(parent, style="Panel.TFrame", padding=12)
        frame.columnconfigure(0, weight=1)

        controls = ttk.Frame(frame, style="Panel.TFrame")
        controls.grid(row=0, column=0, sticky="ew")
        stop = ttk.Button(controls, text="全部停止", style="Stop.TButton", command=self._safe_stop_all)
        stop.pack(side="left")

        self.velocity_label = ttk.Label(frame, text="", style="Value.TLabel")
        self.velocity_label.grid(row=1, column=0, sticky="w", pady=(10, 6))
        self.log_text = tk.Text(frame, height=8, bg="#0e1726", fg="#dce6f6", insertbackground="#ffffff", relief="flat")
        self.log_text.grid(row=2, column=0, sticky="nsew")
        frame.rowconfigure(2, weight=1)
        return frame

    def append_log(self, message: str) -> None:
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.insert("end", f"[{timestamp}] {message}\n")
        self.log_text.see("end")
        self._refresh_velocity_label()

    def _refresh_velocity_label(self) -> None:
        velocity = self.runtime.cartesian_velocity()
        self.velocity_label.configure(
            text=(
                f"TCP 速度: 前 {velocity[0] * 100:+.1f} cm/s, "
                f"左 {velocity[1] * 100:+.1f} cm/s, 上 {velocity[2] * 100:+.1f} cm/s"
            )
        )

    def _bind_hold_button(self, button: Any, on_press: Callable[[], Any], on_release: Callable[[], Any]) -> None:
        def press(_event: Any) -> None:
            if on_release not in self.active_releases:
                self.active_releases.append(on_release)
            self._safe_call(on_press)

        def release(_event: Any) -> None:
            self._release_hold(on_release)

        button.bind("<ButtonPress-1>", press)
        button.bind("<ButtonRelease-1>", release)
        button.bind("<Leave>", lambda _event: None)

    def _release_hold(self, on_release: Callable[[], Any]) -> None:
        if on_release not in self.active_releases:
            return
        self.active_releases.remove(on_release)
        self._safe_call(on_release)

    def _release_all_holds(self) -> None:
        for on_release in list(self.active_releases):
            self._release_hold(on_release)

    def _on_global_mouse_release(self, _event: Any) -> None:
        self._release_all_holds()
        if self.joystick_active:
            self._release_joystick()

    def _safe_call(self, action: Callable[[], Any]) -> None:
        try:
            action()
            self._refresh_velocity_label()
        except Exception as exc:
            self._show_error(exc)

    def _safe_stop_all(self) -> None:
        self._safe_call(self.runtime.stop_all)
        self._update_grip_button()
        self._draw_joystick()

    def _safe_home(self) -> None:
        self._safe_call(self.runtime.go_home)
        self._update_grip_button()
        self._draw_joystick()

    def _toggle_grip(self) -> None:
        self._safe_call(self.runtime.toggle_grip_lock)
        self._update_grip_button()

    def _update_grip_button(self) -> None:
        if self.runtime.j6_grip_locked:
            self.grip_button.configure(bg="#d52b2b", activebackground="#b72121")
        else:
            self.grip_button.configure(bg="#1fa463", activebackground="#178a51")

    def _draw_joystick(self) -> None:
        c = self.joystick_center
        r = self.joystick_radius
        self.joystick.delete("all")
        self.joystick.create_oval(c - r, c - r, c + r, c + r, fill="#edf2f7", outline="#9ca9bc", width=2)
        self.joystick.create_line(c, c - r, c, c + r, fill="#cad3df", width=2)
        self.joystick.create_line(c - r, c, c + r, c, fill="#cad3df", width=2)
        self.joystick.create_text(c, c - r + 22, text="前", fill="#142033", font=("Microsoft YaHei UI", 13, "bold"))
        self.joystick.create_text(c, c + r - 22, text="后", fill="#142033", font=("Microsoft YaHei UI", 13, "bold"))
        self.joystick.create_text(c - r + 24, c, text="左", fill="#142033", font=("Microsoft YaHei UI", 13, "bold"))
        self.joystick.create_text(c + r - 24, c, text="右", fill="#142033", font=("Microsoft YaHei UI", 13, "bold"))

        knob_x = c + self.runtime.joystick_x * r
        knob_y = c + self.runtime.joystick_y * r
        kr = self.knob_radius
        self.joystick.create_oval(knob_x - kr, knob_y - kr, knob_x + kr, knob_y + kr, fill="#2f6fed", outline="#143d91", width=2)

    def _on_joystick_move(self, event: Any) -> None:
        self.joystick_active = True
        c = self.joystick_center
        r = self.joystick_radius
        x, y = self.runtime.set_joystick((event.x - c) / r, (event.y - c) / r)
        self._draw_joystick()
        self._refresh_velocity_label()

    def _on_joystick_release(self, _event: Any) -> None:
        self._release_joystick()

    def _release_joystick(self) -> None:
        self.joystick_active = False
        self.runtime.center_joystick()
        self._draw_joystick()
        self._refresh_velocity_label()

    def _schedule_tick(self) -> None:
        try:
            self.runtime.tick()
            self._refresh_velocity_label()
        except Exception as exc:
            self._show_error(exc)
        self.root.after(max(10, int(self.runtime.config.tick_s * 1000)), self._schedule_tick)

    def _show_error(self, exc: Exception) -> None:
        self.append_log(f"ERROR: {exc}")
        if messagebox is not None:
            messagebox.showerror("JetArm 操作终端", str(exc))

    def on_close(self) -> None:
        try:
            self.runtime.close()
        finally:
            self.root.destroy()


def _build_runtime(args: argparse.Namespace, logger: Optional[Callable[[str], None]] = None) -> ManualServoRuntime:
    config_path = Path(args.config)
    loaded = load_joint_config(config_path)
    metadata = loaded.get("metadata") if isinstance(loaded.get("metadata"), dict) else {}
    terminal_config = OperationTerminalConfig(
        config_path=config_path,
        com_port=args.com_port or metadata.get("default_com_port") or "COM15",
        baudrate=args.baudrate or int(metadata.get("baudrate", DEFAULT_BAUDRATE)),
        timeout_s=args.timeout,
        tick_s=args.tick_ms / 1000.0,
        vertical_speed_m_s=args.vertical_speed_cm_s / 100.0,
        max_horizontal_speed_m_s=args.max_horizontal_speed_cm_s / 100.0,
    )
    model = JetArmKinematicModel(loaded)
    if args.dry_run:
        positions = {
            int(model.joints[joint_name]["servo_id"]): position
            for joint_name, position in terminal_config.home_positions.items()
        }
        controller = DryRunServoController(positions, logger=logger)
    else:
        controller = BusServoController(
            terminal_config.com_port,
            baudrate=terminal_config.baudrate,
            timeout=terminal_config.timeout_s,
        )
    runtime = ManualServoRuntime(controller, model, terminal_config, logger=logger)
    runtime.initialize(use_home_positions=args.dry_run)
    return runtime


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="JetArm visual operation terminal for direct servo control")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG_PATH), help="joint_servo_map.yaml path")
    parser.add_argument("--com-port", default=None, help="serial port; defaults to YAML metadata")
    parser.add_argument("--baudrate", type=int, default=None, help="baudrate; defaults to YAML metadata")
    parser.add_argument("--timeout", type=float, default=0.2, help="serial read timeout in seconds")
    parser.add_argument("--tick-ms", type=int, default=80, help="cartesian control update interval")
    parser.add_argument("--vertical-speed-cm-s", type=float, default=5.0, help="up/down TCP speed")
    parser.add_argument("--max-horizontal-speed-cm-s", type=float, default=5.0, help="joystick edge TCP speed")
    parser.add_argument("--dry-run", action="store_true", help="show the terminal without opening a serial port")
    return parser


def main(argv: Optional[list[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if tk is None:
        print("ERROR: Tkinter is not available in this Python installation")
        return 1

    root = tk.Tk()
    app_holder: dict[str, OperationTerminalApp] = {}

    def logger(message: str) -> None:
        app = app_holder.get("app")
        if app is not None:
            app.append_log(message)
        else:
            print(message)

    try:
        runtime = _build_runtime(args, logger=logger)
        app_holder["app"] = OperationTerminalApp(root, runtime, dry_run=args.dry_run)
        root.mainloop()
        return 0
    except Exception as exc:
        print(f"ERROR: {exc}")
        if messagebox is not None:
            messagebox.showerror("JetArm 操作终端", str(exc))
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
