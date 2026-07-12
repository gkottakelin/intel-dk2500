"""Tkinter user interface for the lightweight JetArm launcher."""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path
from typing import Any

try:
    import tkinter as tk
    from tkinter import filedialog, messagebox, ttk
except ImportError:  # pragma: no cover - depends on the host desktop install.
    tk = None  # type: ignore[assignment]
    filedialog = messagebox = ttk = None  # type: ignore[assignment]

from .config_store import (
    env_file_declares,
    flatten_json,
    load_json,
    save_json,
    validate_agent_values,
    validate_device_values,
)
from .emergency_stop import active_targets, request_emergency_stop
from .terminal_launcher import (
    LaunchSpec,
    default_launch_specs,
    launch_in_terminal,
    open_project_folder,
    open_usage_guide,
)


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEVICE_CONFIG = PROJECT_ROOT / "config" / "devices.json"
DEVICE_EXAMPLE_CONFIG = PROJECT_ROOT / "config" / "devices.example.json"
AI_CONFIG = PROJECT_ROOT / "config" / "ai_agent.json"
DEFAULT_TERMINAL_CONFIG = (
    PROJECT_ROOT / "ubuntu22_04_operation_terminal" / "config" / "terminal.json"
)
USAGE_GUIDE = PROJECT_ROOT / "使用教程.txt"


class ControlCenterApp:
    def __init__(self, root: "tk.Tk") -> None:
        self.root = root
        self.root.title("JetArm 总控终端")
        self.root.geometry("940x690")
        self.root.minsize(820, 590)
        self.status = tk.StringVar(value="就绪。按下功能键将在独立终端中运行原有程序。")
        self._configure_style()
        self._build_main_view()

    def _configure_style(self) -> None:
        style = ttk.Style(self.root)
        available = style.theme_names()
        if "clam" in available:
            style.theme_use("clam")
        style.configure("Title.TLabel", font=("Noto Sans CJK SC", 20, "bold"))
        style.configure("Subtitle.TLabel", font=("Noto Sans CJK SC", 10))
        style.configure("CardTitle.TLabel", font=("Noto Sans CJK SC", 12, "bold"))
        style.configure("Launch.TButton", font=("Noto Sans CJK SC", 11, "bold"), padding=10)
        style.configure("Status.TLabel", padding=(10, 8))

    def _build_main_view(self) -> None:
        outer = ttk.Frame(self.root, padding=22)
        outer.pack(fill="both", expand=True)
        outer.columnconfigure(0, weight=1)

        ttk.Label(outer, text="JetArm 总控终端", style="Title.TLabel").grid(
            row=0, column=0, sticky="w"
        )
        ttk.Label(
            outer,
            text="负责启动现有模块、查看配置和软件急停，不改动机械臂、相机或抓取工作流。",
            style="Subtitle.TLabel",
            foreground="#526172",
        ).grid(row=1, column=0, sticky="w", pady=(3, 18))

        cards = ttk.Frame(outer)
        cards.grid(row=2, column=0, sticky="nsew")
        cards.columnconfigure(0, weight=1)
        cards.columnconfigure(1, weight=1)
        specs = default_launch_specs()
        for index, spec in enumerate(specs):
            self._add_launch_card(cards, spec, index // 2, index % 2)

        utility = ttk.LabelFrame(outer, text="管理与配置", padding=14)
        utility.grid(row=3, column=0, sticky="ew", pady=(18, 0))
        utility.columnconfigure(0, weight=1)
        utility.columnconfigure(1, weight=1)
        utility.columnconfigure(2, weight=1)
        ttk.Button(
            utility,
            text="配置中心",
            command=self.open_config_center,
            style="Launch.TButton",
        ).grid(row=0, column=0, sticky="ew", padx=(0, 7))
        ttk.Button(
            utility,
            text="打开使用说明",
            command=self._open_usage_guide,
            style="Launch.TButton",
        ).grid(row=0, column=1, sticky="ew", padx=7)
        ttk.Button(
            utility,
            text="打开项目目录",
            command=self._open_project_folder,
            style="Launch.TButton",
        ).grid(row=0, column=2, sticky="ew", padx=(7, 0))
        tk.Button(
            utility,
            text="紧急停止机械臂",
            command=self._emergency_stop,
            bg="#c62828",
            fg="#ffffff",
            activebackground="#8e0000",
            activeforeground="#ffffff",
            relief="flat",
            font=("Noto Sans CJK SC", 13, "bold"),
            padx=12,
            pady=12,
        ).grid(row=1, column=0, columnspan=3, sticky="ew", pady=(14, 0))

        ttk.Label(
            outer,
            text=(
                "资源提示：机械臂控制、人工测试V2和Agent可能争用串口；"
                "相机显示和Agent可能争用相机。总控不会额外打开这些设备。"
            ),
            foreground="#8a5a00",
            wraplength=880,
        ).grid(row=4, column=0, sticky="w", pady=(16, 8))
        ttk.Separator(outer).grid(row=5, column=0, sticky="ew")
        ttk.Label(
            outer, textvariable=self.status, style="Status.TLabel", wraplength=870
        ).grid(row=6, column=0, sticky="ew")

    def _add_launch_card(
        self, parent: "ttk.Frame", spec: LaunchSpec, row: int, column: int
    ) -> None:
        frame = ttk.LabelFrame(parent, padding=14)
        frame.grid(
            row=row,
            column=column,
            sticky="nsew",
            padx=(0, 8) if column == 0 else (8, 0),
            pady=7,
        )
        frame.columnconfigure(0, weight=1)
        ttk.Label(frame, text=spec.title.replace("JetArm · ", ""), style="CardTitle.TLabel").grid(
            row=0, column=0, sticky="w"
        )
        ttk.Label(
            frame,
            text=spec.description,
            foreground="#526172",
            wraplength=360,
        ).grid(row=1, column=0, sticky="w", pady=(5, 8))
        if spec.resource_note:
            ttk.Label(frame, text=spec.resource_note, foreground="#8a5a00").grid(
                row=2, column=0, sticky="w", pady=(0, 8)
            )
        ttk.Button(
            frame,
            text="打开终端",
            command=lambda selected=spec: self._launch(selected),
            style="Launch.TButton",
        ).grid(row=3, column=0, sticky="ew")

    def _launch(self, spec: LaunchSpec) -> None:
        try:
            launch_in_terminal(PROJECT_ROOT, spec)
        except Exception as exc:
            messagebox.showerror("启动失败", str(exc), parent=self.root)
            self.status.set(f"{spec.title}启动失败：{exc}")
            return
        self.status.set(f"已打开独立终端：{spec.title}")

    def _open_project_folder(self) -> None:
        try:
            open_project_folder(PROJECT_ROOT)
        except Exception as exc:
            messagebox.showerror("打开失败", str(exc), parent=self.root)
            self.status.set(f"打开项目目录失败：{exc}")
            return
        self.status.set(f"已打开项目目录：{PROJECT_ROOT}")

    def _open_usage_guide(self) -> None:
        try:
            open_usage_guide(USAGE_GUIDE)
        except Exception as exc:
            messagebox.showerror("打开失败", str(exc), parent=self.root)
            self.status.set(f"打开使用说明失败：{exc}")
            return
        self.status.set(f"已打开使用说明：{USAGE_GUIDE}")

    def _emergency_stop(self) -> None:
        result = request_emergency_stop(PROJECT_ROOT)
        if not result.active:
            if result.failures:
                messagebox.showerror(
                    "软件急停失败",
                    "\n".join(result.failures),
                    parent=self.root,
                )
                self.status.set("软件急停失败：" + "；".join(result.failures))
            else:
                self.status.set("未发现由总控启动的机械臂程序，未发送急停。")
            return

        names = "、".join(target.key for target in result.signaled)
        if result.failures:
            messagebox.showwarning(
                "软件急停部分失败",
                "\n".join(result.failures),
                parent=self.root,
            )
        if result.signaled:
            self.status.set(f"已向 {names} 发送急停，正在等待程序退出确认…")
            expected = tuple(target.key for target in result.signaled)
            self.root.after(1500, lambda: self._confirm_emergency_stop(expected))
        else:
            self.status.set("软件急停未能向任何机械臂程序发送信号。")

    def _confirm_emergency_stop(self, expected: tuple[str, ...]) -> None:
        remaining = {
            target.key for target in active_targets(PROJECT_ROOT)
        }.intersection(expected)
        if remaining:
            names = "、".join(sorted(remaining))
            self.status.set(f"急停未确认：{names} 仍在运行，请立即使用物理断电急停。")
            messagebox.showerror(
                "急停未确认",
                f"{names} 在1.5秒内未退出。\n请立即使用物理断电急停。",
                parent=self.root,
            )
            return
        self.status.set("软件急停已确认：机械臂程序已停止并退出。")

    def open_config_center(self) -> None:
        window = tk.Toplevel(self.root)
        window.title("JetArm 配置中心")
        window.geometry("930x680")
        window.minsize(780, 560)
        notebook = ttk.Notebook(window)
        notebook.pack(fill="both", expand=True, padx=14, pady=14)
        self._build_device_tab(notebook)
        self._build_agent_tab(notebook)
        self._build_arm_parameters_tab(notebook)

    def _initial_device_payload(self) -> dict[str, Any]:
        if DEVICE_CONFIG.is_file():
            return load_json(DEVICE_CONFIG)
        return load_json(DEVICE_EXAMPLE_CONFIG)

    def _build_device_tab(self, notebook: "ttk.Notebook") -> None:
        frame = ttk.Frame(notebook, padding=18)
        notebook.add(frame, text="接口与抓取点")
        frame.columnconfigure(1, weight=1)
        payload = self._initial_device_payload()

        mode = tk.StringVar(value=str(payload.get("arm_mode", "off")))
        port = tk.StringVar(value=str(payload.get("arm_port", "")))
        terminal_config = tk.StringVar(
            value=str(
                payload.get(
                    "arm_terminal_config",
                    "ubuntu22_04_operation_terminal/config/terminal.json",
                )
            )
        )
        camera = tk.StringVar(value=str(payload.get("rgb_camera", "")))
        camera_name = tk.StringVar(value=str(payload.get("rgb_camera_name", "")))
        grasp_x = tk.StringVar(value=self._display_number(payload.get("grasp_point_x")))
        grasp_y = tk.StringVar(value=self._display_number(payload.get("grasp_point_y")))
        device_status = tk.StringVar(
            value=f"保存位置：{DEVICE_CONFIG}（本机配置，不提交Git）"
        )

        ttk.Label(
            frame,
            text="机械臂、相机接口和抓取点像素",
            style="CardTitle.TLabel",
        ).grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 14))

        self._form_label(frame, "机械臂模式", 1)
        ttk.Combobox(
            frame,
            textvariable=mode,
            values=("hardware", "dry-run", "off"),
            state="readonly",
        ).grid(row=1, column=1, sticky="ew", pady=5)

        self._form_label(frame, "机械臂串口", 2)
        port_box = ttk.Combobox(frame, textvariable=port, state="normal")
        port_box.grid(row=2, column=1, sticky="ew", pady=5)

        def refresh_ports() -> None:
            choices: list[str] = []
            for pattern in (
                "/dev/serial/by-id/*",
                "/dev/ttyUSB*",
                "/dev/ttyACM*",
            ):
                choices.extend(str(path) for path in Path("/").glob(pattern.lstrip("/")))
            choices = list(dict.fromkeys(choices))
            port_box.configure(values=choices)
            if choices and not port.get().strip():
                port.set(choices[0])
            device_status.set(f"发现 {len(choices)} 个候选串口；未打开任何串口。")

        ttk.Button(frame, text="扫描串口", command=refresh_ports).grid(
            row=2, column=2, padx=(10, 0), pady=5
        )

        self._form_label(frame, "Gemini序列号/UID", 3)
        ttk.Entry(frame, textvariable=camera).grid(row=3, column=1, sticky="ew", pady=5)
        self._form_label(frame, "相机名称", 4)
        ttk.Entry(frame, textvariable=camera_name).grid(
            row=4, column=1, sticky="ew", pady=5
        )
        ttk.Label(
            frame,
            text="填写Orbbec设备序列号或UID；配置中心不会打开相机。",
            foreground="#526172",
        ).grid(row=5, column=1, sticky="w")

        self._form_label(frame, "抓取点像素", 6)
        point_frame = ttk.Frame(frame)
        point_frame.grid(row=6, column=1, sticky="w", pady=5)
        ttk.Label(point_frame, text="X").pack(side="left")
        ttk.Entry(point_frame, textvariable=grasp_x, width=12).pack(
            side="left", padx=(5, 18)
        )
        ttk.Label(point_frame, text="Y").pack(side="left")
        ttk.Entry(point_frame, textvariable=grasp_y, width=12).pack(
            side="left", padx=(5, 0)
        )

        self._form_label(frame, "机械臂参数文件", 7)
        ttk.Entry(frame, textvariable=terminal_config, state="readonly").grid(
            row=7, column=1, sticky="ew", pady=5
        )
        ttk.Label(
            frame,
            text="参数文件只允许在“机械臂参数（只读）”页查看。",
            foreground="#8a5a00",
        ).grid(row=8, column=1, sticky="w")

        ttk.Separator(frame).grid(row=9, column=0, columnspan=3, sticky="ew", pady=16)
        ttk.Label(frame, textvariable=device_status, wraplength=760).grid(
            row=10, column=0, columnspan=3, sticky="w"
        )

        def save_devices() -> None:
            try:
                x, y = validate_device_values(
                    arm_mode=mode.get(),
                    arm_port=port.get(),
                    arm_terminal_config=terminal_config.get(),
                    grasp_x=grasp_x.get(),
                    grasp_y=grasp_y.get(),
                )
                updated = dict(payload)
                updated.update(
                    {
                        "arm_mode": mode.get().strip(),
                        "arm_port": port.get().strip(),
                        "arm_terminal_config": terminal_config.get().strip(),
                        "rgb_camera": camera.get().strip(),
                        "rgb_camera_name": camera_name.get().strip(),
                        "grasp_point_x": x,
                        "grasp_point_y": y,
                    }
                )
                save_json(DEVICE_CONFIG, updated)
            except Exception as exc:
                messagebox.showerror("保存失败", str(exc), parent=frame)
                return
            payload.clear()
            payload.update(updated)
            device_status.set(f"已保存：{DEVICE_CONFIG}")
            self.status.set("接口与抓取点配置已保存。")

        ttk.Button(frame, text="保存接口与抓取点配置", command=save_devices).grid(
            row=11, column=0, columnspan=3, sticky="e", pady=(18, 0)
        )

    def _build_agent_tab(self, notebook: "ttk.Notebook") -> None:
        frame = ttk.Frame(notebook, padding=18)
        notebook.add(frame, text="Agent接口")
        frame.columnconfigure(1, weight=1)
        payload = load_json(AI_CONFIG)
        api = payload.setdefault("api", {})
        if not isinstance(api, dict):
            api = {}
            payload["api"] = api

        provider = tk.StringVar(value=str(api.get("provider", "openai_compatible")))
        base_url = tk.StringVar(value=str(api.get("base_url", "")))
        model = tk.StringVar(value=str(api.get("model", "")))
        api_key_env = tk.StringVar(value=str(api.get("api_key_env", "JETARM_API_KEY")))
        timeout = tk.StringVar(value=str(api.get("timeout_s", 60)))
        agent_status = tk.StringVar(value=f"配置文件：{AI_CONFIG}")

        ttk.Label(frame, text="Agent API与模型", style="CardTitle.TLabel").grid(
            row=0, column=0, columnspan=2, sticky="w", pady=(0, 14)
        )
        self._form_label(frame, "接口类型", 1)
        ttk.Entry(frame, textvariable=provider, state="readonly").grid(
            row=1, column=1, sticky="ew", pady=5
        )
        self._form_label(frame, "API Base URL", 2)
        ttk.Entry(frame, textvariable=base_url).grid(row=2, column=1, sticky="ew", pady=5)
        self._form_label(frame, "模型名称", 3)
        ttk.Entry(frame, textvariable=model).grid(row=3, column=1, sticky="ew", pady=5)
        self._form_label(frame, "API Key环境变量", 4)
        ttk.Entry(frame, textvariable=api_key_env).grid(row=4, column=1, sticky="ew", pady=5)
        self._form_label(frame, "请求超时（秒）", 5)
        ttk.Entry(frame, textvariable=timeout).grid(row=5, column=1, sticky="ew", pady=5)

        key_state = tk.StringVar()

        def refresh_key_state(*_args: object) -> None:
            name = api_key_env.get().strip()
            configured = bool(os.environ.get(name)) or env_file_declares(
                PROJECT_ROOT / ".env", name
            )
            key_state.set(
                f"{name or '未指定'}：{'已配置' if configured else '未检测到'}"
                "（只检查是否存在，不读取或显示密钥）"
            )

        api_key_env.trace_add("write", refresh_key_state)
        refresh_key_state()
        ttk.Label(frame, textvariable=key_state, foreground="#526172").grid(
            row=6, column=1, sticky="w", pady=(0, 8)
        )
        ttk.Label(
            frame,
            text="API Key仍只保存在环境变量或项目.env中，不会写入JSON配置。",
            foreground="#8a5a00",
        ).grid(row=7, column=0, columnspan=2, sticky="w", pady=(8, 16))
        ttk.Separator(frame).grid(row=8, column=0, columnspan=2, sticky="ew")
        ttk.Label(frame, textvariable=agent_status, wraplength=760).grid(
            row=9, column=0, columnspan=2, sticky="w", pady=(14, 0)
        )

        def save_agent() -> None:
            try:
                timeout_value = validate_agent_values(
                    base_url=base_url.get(),
                    model=model.get(),
                    api_key_env=api_key_env.get(),
                    timeout_s=timeout.get(),
                )
                api.update(
                    {
                        "provider": "openai_compatible",
                        "base_url": base_url.get().strip().rstrip("/"),
                        "model": model.get().strip(),
                        "api_key_env": api_key_env.get().strip(),
                        "timeout_s": timeout_value,
                    }
                )
                save_json(AI_CONFIG, payload)
            except Exception as exc:
                messagebox.showerror("保存失败", str(exc), parent=frame)
                return
            agent_status.set(f"已保存：{AI_CONFIG}")
            self.status.set("Agent接口配置已保存。")

        ttk.Button(frame, text="保存Agent接口配置", command=save_agent).grid(
            row=10, column=0, columnspan=2, sticky="e", pady=(18, 0)
        )

    def _build_arm_parameters_tab(self, notebook: "ttk.Notebook") -> None:
        frame = ttk.Frame(notebook, padding=14)
        notebook.add(frame, text="机械臂参数（只读）")
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(2, weight=1)
        source = tk.StringVar()
        ttk.Label(
            frame,
            text="机械臂参数只读视图",
            style="CardTitle.TLabel",
        ).grid(row=0, column=0, sticky="w")
        ttk.Label(frame, textvariable=source, foreground="#526172", wraplength=820).grid(
            row=1, column=0, sticky="w", pady=(5, 10)
        )

        tree = ttk.Treeview(frame, columns=("parameter", "value"), show="headings")
        tree.heading("parameter", text="参数")
        tree.heading("value", text="值")
        tree.column("parameter", width=520, minwidth=300, stretch=True)
        tree.column("value", width=230, minwidth=120, stretch=True)
        scroll = ttk.Scrollbar(frame, orient="vertical", command=tree.yview)
        tree.configure(yscrollcommand=scroll.set)
        tree.grid(row=2, column=0, sticky="nsew")
        scroll.grid(row=2, column=1, sticky="ns")
        current: dict[str, Any] = {}

        def parameter_path() -> Path:
            try:
                devices = self._initial_device_payload()
                raw = str(devices.get("arm_terminal_config", "")).strip()
            except Exception:
                raw = ""
            if not raw:
                return DEFAULT_TERMINAL_CONFIG
            path = Path(raw).expanduser()
            return path if path.is_absolute() else PROJECT_ROOT / path

        def refresh() -> None:
            path = parameter_path()
            try:
                payload = load_json(path)
            except Exception as exc:
                source.set(f"读取失败：{path}：{exc}")
                messagebox.showerror("读取机械臂参数失败", str(exc), parent=frame)
                return
            current.clear()
            current.update(payload)
            tree.delete(*tree.get_children())
            for key, value in flatten_json(payload):
                tree.insert("", "end", values=(key, value))
            source.set(
                f"来源：配置文件 {path} ｜ 共 {len(tuple(flatten_json(payload)))} 项 ｜ "
                "此页面没有编辑或保存机械臂参数的功能"
            )

        def copy_json() -> None:
            self.root.clipboard_clear()
            self.root.clipboard_append(
                json.dumps(current, ensure_ascii=False, indent=2)
            )
            self.status.set("机械臂参数JSON已复制到剪贴板。")

        def export_json() -> None:
            destination = filedialog.asksaveasfilename(
                parent=frame,
                title="导出机械臂参数副本",
                defaultextension=".json",
                filetypes=(("JSON文件", "*.json"), ("所有文件", "*")),
                initialfile="jetarm_parameters_export.json",
            )
            if not destination:
                return
            save_json(Path(destination), current)
            self.status.set(f"机械臂参数副本已导出：{destination}")

        buttons = ttk.Frame(frame)
        buttons.grid(row=3, column=0, columnspan=2, sticky="e", pady=(12, 0))
        ttk.Button(buttons, text="刷新", command=refresh).pack(side="left", padx=(0, 8))
        ttk.Button(buttons, text="复制JSON", command=copy_json).pack(
            side="left", padx=(0, 8)
        )
        ttk.Button(buttons, text="导出副本", command=export_json).pack(side="left")
        refresh()

    @staticmethod
    def _display_number(value: Any) -> str:
        if value is None or value == "":
            return ""
        try:
            return f"{float(value):g}"
        except (TypeError, ValueError):
            return str(value)

    @staticmethod
    def _form_label(parent: "ttk.Frame", text: str, row: int) -> None:
        ttk.Label(parent, text=text).grid(
            row=row, column=0, sticky="w", padx=(0, 12), pady=5
        )


def main() -> int:
    if tk is None or ttk is None or messagebox is None:
        print("缺少Tkinter，请在Ubuntu安装 python3-tk。", file=sys.stderr)
        return 1
    root = tk.Tk()
    ControlCenterApp(root)
    root.mainloop()
    return 0
