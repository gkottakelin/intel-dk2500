"""Configure JetArm interfaces and the fixed Agent grasp-point pixel."""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Callable, Optional

from .arm_control import DEFAULT_TERMINAL_CONFIG, _load_terminal_module

try:
    from ubuntu22_04_gemini_camera.orbbec_native import (
        CameraDeviceInfo,
        enumerate_devices as enumerate_orbbec_devices,
    )
except ModuleNotFoundError:  # Imported as project.src.jetarm_agent in tests.
    from project.ubuntu22_04_gemini_camera.orbbec_native import (
        CameraDeviceInfo,
        enumerate_devices as enumerate_orbbec_devices,
    )


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_DEVICE_CONFIG_PATH = PROJECT_ROOT / "config" / "devices.json"


@dataclass(frozen=True)
class RuntimeDeviceConfig:
    arm_mode: str = "off"
    arm_port: str = ""
    arm_terminal_config: str = str(DEFAULT_TERMINAL_CONFIG)
    rgb_camera: str = ""
    rgb_camera_name: str = ""
    grasp_point_x: float | None = None
    grasp_point_y: float | None = None

    def validate(self) -> None:
        if self.arm_mode not in {"off", "dry-run", "hardware"}:
            raise ValueError("arm_mode必须是off、dry-run或hardware")
        if self.arm_mode == "hardware" and not self.arm_port.strip():
            raise ValueError("hardware模式必须配置机械臂串口")
        if not self.arm_terminal_config.strip():
            raise ValueError("arm_terminal_config不能为空")
        if (self.grasp_point_x is None) != (self.grasp_point_y is None):
            raise ValueError("抓取点像素X和Y必须同时配置或同时留空")
        for label, value in (
            ("抓取点像素X", self.grasp_point_x),
            ("抓取点像素Y", self.grasp_point_y),
        ):
            if value is not None and (
                not math.isfinite(float(value)) or float(value) < 0.0
            ):
                raise ValueError(f"{label}必须是大于等于0的有限数字")

    @classmethod
    def load(
        cls, path: str | Path = DEFAULT_DEVICE_CONFIG_PATH, *, required: bool = True
    ) -> "RuntimeDeviceConfig":
        config_path = Path(path)
        if not config_path.is_file():
            if required:
                raise FileNotFoundError(f"设备配置不存在: {config_path}")
            return cls()
        with config_path.open("r", encoding="utf-8") as file:
            payload = json.load(file)
        raw_grasp_x = payload.get("grasp_point_x")
        raw_grasp_y = payload.get("grasp_point_y")
        config = cls(
            arm_mode=str(payload.get("arm_mode", "off")),
            arm_port=str(payload.get("arm_port", "")),
            arm_terminal_config=str(
                payload.get("arm_terminal_config", DEFAULT_TERMINAL_CONFIG)
            ),
            rgb_camera=str(payload.get("rgb_camera", "")),
            rgb_camera_name=str(payload.get("rgb_camera_name", "")),
            grasp_point_x=(
                None if raw_grasp_x is None or raw_grasp_x == "" else float(raw_grasp_x)
            ),
            grasp_point_y=(
                None if raw_grasp_y is None or raw_grasp_y == "" else float(raw_grasp_y)
            ),
        )
        config.validate()
        return config

    def save(self, path: str | Path = DEFAULT_DEVICE_CONFIG_PATH) -> Path:
        self.validate()
        config_path = Path(path)
        config_path.parent.mkdir(parents=True, exist_ok=True)
        config_path.write_text(
            json.dumps(asdict(self), ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        return config_path


def discover_rgb_cameras(
    enumerator: Callable[[], list[CameraDeviceInfo]] = enumerate_orbbec_devices,
) -> list[CameraDeviceInfo]:
    """Enumerate physical Gemini/Orbbec devices through the bundled SDK."""

    return list(enumerator())


def validate_device_interfaces(
    config: RuntimeDeviceConfig,
    *,
    camera_discover: Callable[[], list[CameraDeviceInfo]] = discover_rgb_cameras,
) -> list[str]:
    """Return preflight errors without opening either interface permanently."""

    errors: list[str] = []
    if config.arm_mode == "hardware":
        arm_path = Path(config.arm_port)
        if not arm_path.exists():
            errors.append(f"机械臂串口不存在: {config.arm_port}")
        elif not os.access(arm_path, os.R_OK | os.W_OK):
            errors.append(f"机械臂串口不可读写: {config.arm_port}")
    if config.rgb_camera:
        if config.rgb_camera.startswith("/dev/video"):
            errors.append("检测到旧V4L2相机配置，请重新配置并选择Orbbec USB设备/序列号")
        else:
            try:
                cameras = camera_discover()
            except Exception as exc:
                errors.append(f"Orbbec SDK枚举失败: {exc}")
            else:
                match = next(
                    (
                        camera
                        for camera in cameras
                        if config.rgb_camera
                        in {camera.serial_number, camera.uid, camera.selection_key}
                    ),
                    None,
                )
                if match is None:
                    errors.append(f"未发现配置的Orbbec相机: {config.rgb_camera}")
    return errors


def configure_devices_dialog(
    initial: RuntimeDeviceConfig,
) -> RuntimeDeviceConfig | None:
    terminal = _load_terminal_module()
    if terminal.tk is None or terminal.ttk is None:
        raise RuntimeError("缺少Tkinter，请安装python3-tk或使用--no-gui")
    tk = terminal.tk
    ttk = terminal.ttk
    root = tk.Tk()
    root.title("JetArm Agent 接口与抓取点配置")
    root.resizable(False, False)
    result: dict[str, RuntimeDeviceConfig | None] = {"config": None}

    body = ttk.Frame(root, padding=18)
    body.grid(row=0, column=0, sticky="nsew")
    body.columnconfigure(1, weight=1)
    ttk.Label(body, text="机械臂、Gemini RGB相机与抓取点", font=("Noto Sans CJK SC", 13, "bold")).grid(
        row=0, column=0, columnspan=3, sticky="w", pady=(0, 14)
    )

    mode_value = tk.StringVar(value=initial.arm_mode)
    port_value = tk.StringVar(value=initial.arm_port)
    camera_value = tk.StringVar(value=initial.rgb_camera)
    grasp_x_value = tk.StringVar(
        value="" if initial.grasp_point_x is None else f"{initial.grasp_point_x:g}"
    )
    grasp_y_value = tk.StringVar(
        value="" if initial.grasp_point_y is None else f"{initial.grasp_point_y:g}"
    )
    status_value = tk.StringVar(value="")

    ttk.Label(body, text="机械臂模式").grid(row=1, column=0, sticky="w", padx=(0, 10))
    ttk.Combobox(
        body,
        textvariable=mode_value,
        values=("hardware", "dry-run", "off"),
        state="readonly",
        width=16,
    ).grid(row=1, column=1, sticky="ew", pady=4)

    ttk.Label(body, text="机械臂串口").grid(row=2, column=0, sticky="w", padx=(0, 10))
    port_box = ttk.Combobox(body, textvariable=port_value, width=62, state="normal")
    port_box.grid(row=2, column=1, sticky="ew", pady=4)

    ttk.Label(body, text="Gemini设备/序列号").grid(row=3, column=0, sticky="w", padx=(0, 10))
    camera_box = ttk.Combobox(body, textvariable=camera_value, width=62, state="readonly")
    camera_box.grid(row=3, column=1, sticky="ew", pady=4)
    camera_by_label: dict[str, CameraDeviceInfo] = {}

    def refresh() -> None:
        nonlocal camera_by_label
        ports = terminal.discover_linux_serial_ports()
        port_box.configure(values=ports)
        if not port_value.get() and ports:
            port_value.set(ports[0])
        enumeration_error = ""
        try:
            cameras = discover_rgb_cameras()
        except Exception as exc:
            cameras = []
            enumeration_error = str(exc)
        camera_by_label = {camera.label: camera for camera in cameras}
        camera_box.configure(values=list(camera_by_label))
        selected = next(
            (
                camera.label
                for camera in cameras
                if camera_value.get()
                in {camera.serial_number, camera.uid, camera.selection_key}
            ),
            "",
        )
        if selected:
            camera_value.set(selected)
        elif cameras:
            camera_value.set(cameras[0].label)
        else:
            camera_value.set("")
        if enumeration_error:
            status_value.set(f"Orbbec SDK枚举失败: {enumeration_error}")
        elif cameras:
            status_value.set(
                f"发现 {len(ports)} 个机械臂候选串口、{len(cameras)} 台Orbbec相机"
            )
        else:
            status_value.set(f"发现 {len(ports)} 个机械臂候选串口，未发现Orbbec相机")

    ttk.Button(body, text="刷新", command=refresh).grid(
        row=2, column=2, rowspan=2, sticky="nsew", padx=(10, 0), pady=4
    )
    ttk.Label(body, text="抓取点像素").grid(row=4, column=0, sticky="w", padx=(0, 10))
    grasp_frame = ttk.Frame(body)
    grasp_frame.grid(row=4, column=1, sticky="w", pady=4)
    ttk.Label(grasp_frame, text="X").pack(side="left")
    ttk.Entry(grasp_frame, textvariable=grasp_x_value, width=12).pack(
        side="left", padx=(5, 16)
    )
    ttk.Label(grasp_frame, text="Y").pack(side="left")
    ttk.Entry(grasp_frame, textvariable=grasp_y_value, width=12).pack(
        side="left", padx=(5, 0)
    )
    ttk.Label(
        body, textvariable=status_value, foreground="#526172", wraplength=650
    ).grid(row=5, column=0, columnspan=3, sticky="w", pady=(8, 12))

    def save() -> None:
        camera_text = camera_value.get().strip()
        camera = camera_by_label.get(camera_text)
        try:
            raw_grasp_x = grasp_x_value.get().strip()
            raw_grasp_y = grasp_y_value.get().strip()
            grasp_x = float(raw_grasp_x) if raw_grasp_x else None
            grasp_y = float(raw_grasp_y) if raw_grasp_y else None
            config = RuntimeDeviceConfig(
                arm_mode=mode_value.get().strip(),
                arm_port=port_value.get().strip(),
                arm_terminal_config=initial.arm_terminal_config,
                rgb_camera=camera.selection_key if camera else "",
                rgb_camera_name=camera.name if camera else "",
                grasp_point_x=grasp_x,
                grasp_point_y=grasp_y,
            )
            config.validate()
        except ValueError as exc:
            if terminal.messagebox is not None:
                terminal.messagebox.showerror("设备配置", str(exc), parent=root)
            return
        result["config"] = config
        root.destroy()

    buttons = ttk.Frame(body)
    buttons.grid(row=6, column=0, columnspan=3, sticky="e")
    ttk.Button(buttons, text="取消", command=root.destroy).pack(side="left", padx=(0, 8))
    ttk.Button(buttons, text="保存配置", command=save).pack(side="left")
    root.protocol("WM_DELETE_WINDOW", root.destroy)
    refresh()
    root.mainloop()
    return result["config"]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="配置JetArm Agent机械臂、RGB相机接口与抓取点像素"
    )
    parser.add_argument("--config", default=str(DEFAULT_DEVICE_CONFIG_PATH))
    parser.add_argument("--no-gui", action="store_true")
    parser.add_argument("--arm-mode", choices=("off", "dry-run", "hardware"))
    parser.add_argument("--arm-port")
    parser.add_argument("--camera", help="Orbbec相机序列号或UID")
    parser.add_argument("--camera-name", default="")
    parser.add_argument("--grasp-point-x", type=float)
    parser.add_argument("--grasp-point-y", type=float)
    parser.add_argument("--arm-config", default=str(DEFAULT_TERMINAL_CONFIG))
    return parser


def main(argv: Optional[list[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    path = Path(args.config)
    initial = RuntimeDeviceConfig.load(path, required=False)
    if args.no_gui:
        config = RuntimeDeviceConfig(
            arm_mode=args.arm_mode or initial.arm_mode,
            arm_port=args.arm_port if args.arm_port is not None else initial.arm_port,
            arm_terminal_config=args.arm_config or initial.arm_terminal_config,
            rgb_camera=args.camera if args.camera is not None else initial.rgb_camera,
            rgb_camera_name=args.camera_name or initial.rgb_camera_name,
            grasp_point_x=(
                args.grasp_point_x
                if args.grasp_point_x is not None
                else initial.grasp_point_x
            ),
            grasp_point_y=(
                args.grasp_point_y
                if args.grasp_point_y is not None
                else initial.grasp_point_y
            ),
        )
    else:
        config = configure_devices_dialog(initial)
        if config is None:
            print("已取消设备配置。")
            return 0
    saved = config.save(path)
    errors = validate_device_interfaces(config)
    print(f"设备配置已保存: {saved}")
    print(json.dumps(asdict(config), ensure_ascii=False, indent=2))
    if errors:
        print("接口检查未通过:", file=sys.stderr)
        for error in errors:
            print(f"- {error}", file=sys.stderr)
        return 1
    print("接口检查: ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
