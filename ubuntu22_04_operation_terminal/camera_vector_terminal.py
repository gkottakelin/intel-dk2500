"""Camera-to-grasp-point relative JetArm terminal.

This terminal reuses the standalone Ubuntu servo terminal, but maps the six
TCP directions into a frame built from the measured camera-to-grasp-point line:

- up: grasp point -> camera
- down: camera -> grasp point
- forward/backward/left/right: the plane perpendicular to that line
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

import numpy as np

from jetarm_terminal import (
    ARM_JOINTS,
    DEFAULT_CONFIG_PATH,
    BusServoController,
    DryRunServoController,
    ManualServoRuntime,
    OperationTerminalApp,
    TerminalSettings,
    choose_serial_port_dialog,
    discover_linux_serial_ports,
    select_linux_serial_port,
    serial_discovery_diagnostic,
    tk,
)


DEFAULT_GRASP_TO_CAMERA_ALONG_TOOL_M = -0.04
DEFAULT_GRASP_TO_CAMERA_NORMAL_M = 0.055
DEFAULT_GRASP_TO_CAMERA_LATERAL_M = 0.0
EPSILON = 1e-9


@dataclass(frozen=True)
class CameraLineConfig:
    """Signed camera position relative to the grasp point."""

    grasp_to_camera_along_tool_m: float = DEFAULT_GRASP_TO_CAMERA_ALONG_TOOL_M
    grasp_to_camera_normal_m: float = DEFAULT_GRASP_TO_CAMERA_NORMAL_M
    grasp_to_camera_lateral_m: float = DEFAULT_GRASP_TO_CAMERA_LATERAL_M

    @classmethod
    def from_file(cls, path: str | Path = DEFAULT_CONFIG_PATH) -> "CameraLineConfig":
        with Path(path).open("r", encoding="utf-8") as file:
            data = json.load(file)
        values = data.get("camera_control", {})
        if not isinstance(values, dict):
            values = {}
        return cls(
            grasp_to_camera_along_tool_m=float(
                values.get(
                    "grasp_to_camera_along_tool_m",
                    DEFAULT_GRASP_TO_CAMERA_ALONG_TOOL_M,
                )
            ),
            grasp_to_camera_normal_m=float(
                values.get(
                    "grasp_to_camera_normal_m",
                    DEFAULT_GRASP_TO_CAMERA_NORMAL_M,
                )
            ),
            grasp_to_camera_lateral_m=float(
                values.get(
                    "grasp_to_camera_lateral_m",
                    DEFAULT_GRASP_TO_CAMERA_LATERAL_M,
                )
            ),
        )

    def vector_length_m(self) -> float:
        return math.sqrt(
            self.grasp_to_camera_along_tool_m**2
            + self.grasp_to_camera_normal_m**2
            + self.grasp_to_camera_lateral_m**2
        )


@dataclass(frozen=True)
class CameraRelativeFrame:
    up: np.ndarray
    down: np.ndarray
    forward: np.ndarray
    backward: np.ndarray
    left: np.ndarray
    right: np.ndarray


def _unit(vector: np.ndarray) -> np.ndarray:
    length = float(np.linalg.norm(vector))
    if length < EPSILON:
        raise ValueError("cannot normalize a near-zero vector")
    return vector / length


def _project_to_plane(vector: np.ndarray, plane_normal: np.ndarray) -> np.ndarray:
    return vector - plane_normal * float(np.dot(vector, plane_normal))


def _first_valid_plane_axis(plane_normal: np.ndarray, candidates: tuple[np.ndarray, ...]) -> np.ndarray:
    for candidate in candidates:
        projected = _project_to_plane(candidate, plane_normal)
        if float(np.linalg.norm(projected)) >= EPSILON:
            return _unit(projected)
    raise ValueError("cannot build camera-relative control plane")


def build_camera_relative_frame(
    settings: TerminalSettings,
    positions: dict[str, int],
    camera_config: CameraLineConfig,
) -> CameraRelativeFrame:
    """Build the current camera-relative control frame in base coordinates."""

    if camera_config.vector_length_m() < EPSILON:
        raise ValueError("camera_control grasp-to-camera vector must be non-zero")

    kinematics = settings_kinematics(settings)
    q1, q2, q3, q4 = [
        kinematics.position_to_model_angle(name, positions[name])
        for name in ARM_JOINTS
    ]
    pitch = q2 + q3 + q4
    yaw_cos = math.cos(q1)
    yaw_sin = math.sin(q1)

    tool_axis = np.array(
        (math.sin(pitch) * yaw_cos, math.sin(pitch) * yaw_sin, math.cos(pitch)),
        dtype=float,
    )
    normal_axis = np.array(
        (math.cos(pitch) * yaw_cos, math.cos(pitch) * yaw_sin, -math.sin(pitch)),
        dtype=float,
    )
    lateral_axis = np.array((-yaw_sin, yaw_cos, 0.0), dtype=float)

    grasp_to_camera = (
        camera_config.grasp_to_camera_along_tool_m * tool_axis
        + camera_config.grasp_to_camera_normal_m * normal_axis
        + camera_config.grasp_to_camera_lateral_m * lateral_axis
    )
    up = _unit(grasp_to_camera)
    forward = _first_valid_plane_axis(
        up,
        (
            np.array((0.0, 0.0, 1.0), dtype=float),
            normal_axis,
            np.array((1.0, 0.0, 0.0), dtype=float),
            np.array((0.0, 1.0, 0.0), dtype=float),
        ),
    )
    left = _unit(np.cross(forward, up))
    return CameraRelativeFrame(
        up=up,
        down=-up,
        forward=forward,
        backward=-forward,
        left=left,
        right=-left,
    )


def settings_kinematics(settings: TerminalSettings) -> Any:
    # Import lazily through jetarm_terminal to keep tests focused and avoid
    # duplicating the standalone terminal's kinematic implementation.
    from jetarm_terminal import JetArmKinematics

    return JetArmKinematics(settings)


class CameraRelativeManualServoRuntime(ManualServoRuntime):
    """Manual runtime whose Cartesian velocity follows the camera-grasp frame."""

    def __init__(
        self,
        controller: Any,
        settings: TerminalSettings,
        *,
        camera_config: CameraLineConfig | None = None,
        logger: Optional[Any] = None,
        monotonic: Any = None,
    ) -> None:
        kwargs: dict[str, Any] = {"logger": logger}
        if monotonic is not None:
            kwargs["monotonic"] = monotonic
        super().__init__(controller, settings, **kwargs)
        self.camera_config = camera_config or CameraLineConfig()

    def camera_relative_frame(self) -> CameraRelativeFrame:
        return build_camera_relative_frame(
            self.settings,
            self.positions,
            self.camera_config,
        )

    def cartesian_velocity(self) -> np.ndarray:
        frame = self.camera_relative_frame()
        plane_forward = -self.joystick_y * self.settings.max_horizontal_speed_m_s
        plane_left = -self.joystick_x * self.settings.max_horizontal_speed_m_s
        line_up = self.vertical_direction * self.settings.vertical_speed_m_s
        return (
            frame.forward * plane_forward
            + frame.left * plane_left
            + frame.up * line_up
        )


class CameraVectorTerminalApp(OperationTerminalApp):
    """Small title wrapper for the existing terminal UI."""

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self.root.title("JetArm camera-vector terminal")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="JetArm camera-to-grasp-point relative terminal for Ubuntu 22.04"
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
        app_holder: dict[str, CameraVectorTerminalApp] = {}

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
                root.title("JetArm camera-vector terminal")
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

        runtime = CameraRelativeManualServoRuntime(
            controller,
            settings,
            camera_config=camera_config,
            logger=logger,
        )
        runtime.initialize(use_home_positions=args.dry_run)
        app_holder["app"] = CameraVectorTerminalApp(
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
