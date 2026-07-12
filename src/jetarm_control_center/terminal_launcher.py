"""Open existing JetArm workflows in independent desktop terminals."""

from __future__ import annotations

import shlex
import shutil
import subprocess
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Sequence


@dataclass(frozen=True)
class LaunchSpec:
    key: str
    title: str
    description: str
    command: str
    resource_note: str = ""
    emergency_stop: bool = False


def default_launch_specs() -> tuple[LaunchSpec, ...]:
    ai_python = "${JETARM_AI_PYTHON:-.venv-ai/bin/python}"

    def ai_command(arguments: str = "") -> str:
        invocation = f'"$AI_PYTHON" -m src.jetarm_agent{arguments}'
        return (
            f'AI_PYTHON="{ai_python}"; '
            'if [[ -x "$AI_PYTHON" ]]; then '
            f"{invocation}; "
            "else "
            'echo "未找到AI虚拟环境：$AI_PYTHON" >&2; '
            'echo "请先按项目说明创建 .venv-ai，或设置 JETARM_AI_PYTHON。" >&2; '
            "false; fi"
        )
    return (
        LaunchSpec(
            key="git_pull",
            title="JetArm · Git Pull",
            description="查看工作区状态并以 fast-forward 方式拉取 GitHub 更新",
            command="git status --short; git pull --ff-only",
        ),
        LaunchSpec(
            key="arm_terminal",
            title="JetArm · 机械臂控制",
            description="运行机械臂操作终端原 run.sh",
            command="bash ubuntu22_04_operation_terminal/run.sh",
            resource_note="占用机械臂串口",
            emergency_stop=True,
        ),
        LaunchSpec(
            key="camera",
            title="JetArm · 相机显示",
            description="运行 Gemini 相机显示原 run.sh",
            command="bash ubuntu22_04_gemini_camera/run.sh",
            resource_note="占用 Gemini 相机",
        ),
        LaunchSpec(
            key="manual_v2",
            title="JetArm · 基于摄像头的机械臂操控",
            description="启动现有基于摄像头的人工像素闭环操控，不改动其工作流",
            command=ai_command(" --manual-pixel-test-v2"),
            resource_note="硬件模式会占用机械臂串口",
            emergency_stop=True,
        ),
        LaunchSpec(
            key="agent",
            title="JetArm · Agent",
            description="启动现有自然语言与抓取 Agent",
            command=ai_command(),
            resource_note="按配置占用机械臂串口和 Gemini 相机",
            emergency_stop=True,
        ),
    )


def build_shell_command(
    project_root: Path,
    command: str,
    *,
    emergency_stop_key: str | None = None,
    emergency_stop_token: str | None = None,
) -> str:
    quoted_root = shlex.quote(str(project_root))
    prefix = ""
    cleanup = ""
    if emergency_stop_key is not None:
        if not emergency_stop_key.replace("_", "").isalnum():
            raise ValueError("急停注册键只能包含字母、数字和下划线")
        token = emergency_stop_token or uuid.uuid4().hex
        runtime_dir = project_root / ".jetarm_runtime"
        registry = runtime_dir / f"{emergency_stop_key}.estop.json"
        prefix = (
            f"mkdir -p -- {shlex.quote(str(runtime_dir))}; "
            f"JETARM_ESTOP_KEY={shlex.quote(emergency_stop_key)}; "
            f"JETARM_ESTOP_TOKEN={shlex.quote(token)}; "
            "export JETARM_ESTOP_TOKEN; "
            'JETARM_ESTOP_PGID="$(ps -o pgid= -p "$$" | tr -d \' \')"; '
            f"JETARM_ESTOP_FILE={shlex.quote(str(registry))}; "
            "printf "
            "'{\"key\":\"%s\",\"pid\":%s,\"pgid\":%s,\"token\":\"%s\"}\\n' "
            '"$JETARM_ESTOP_KEY" "$$" "$JETARM_ESTOP_PGID" "$JETARM_ESTOP_TOKEN" '
            '> "$JETARM_ESTOP_FILE"; '
            'trap \'rm -f -- "$JETARM_ESTOP_FILE"\' EXIT; '
        )
        cleanup = 'rm -f -- "$JETARM_ESTOP_FILE"; trap - EXIT; '
    return (
        f"cd -- {quoted_root}; "
        f"{prefix}"
        f"{command}; "
        "JETARM_EXIT_CODE=$?; "
        f"{cleanup}"
        'printf "\\n[JetArm总控] 程序已结束，退出码: %s\\n" "$JETARM_EXIT_CODE"; '
        'printf "按 Enter 关闭此终端..."; read -r; '
        "exit \"$JETARM_EXIT_CODE\""
    )


def find_terminal(which: Callable[[str], str | None] = shutil.which) -> str | None:
    for executable in (
        "gnome-terminal",
        "kgx",
        "konsole",
        "xfce4-terminal",
        "x-terminal-emulator",
        "xterm",
    ):
        path = which(executable)
        if path:
            return path
    return None


def terminal_argv(executable: str, *, title: str, shell_command: str) -> list[str]:
    name = Path(executable).name
    if name in {"gnome-terminal", "kgx"}:
        title_args = [f"--title={title}"] if name == "gnome-terminal" else []
        return [executable, *title_args, "--", "bash", "-lc", shell_command]
    if name == "konsole":
        return [
            executable,
            "--new-tab",
            "-p",
            f"tabtitle={title}",
            "-e",
            "bash",
            "-lc",
            shell_command,
        ]
    if name == "xfce4-terminal":
        return [
            executable,
            f"--title={title}",
            "--disable-server",
            "-x",
            "bash",
            "-lc",
            shell_command,
        ]
    return [executable, "-T", title, "-e", "bash", "-lc", shell_command]


def launch_in_terminal(
    project_root: Path,
    spec: LaunchSpec,
    *,
    popen: Callable[..., subprocess.Popen[bytes]] = subprocess.Popen,
) -> subprocess.Popen[bytes]:
    executable = find_terminal()
    if executable is None:
        raise RuntimeError(
            "未找到可用桌面终端，请安装 gnome-terminal 或 x-terminal-emulator"
        )
    command = build_shell_command(
        project_root,
        spec.command,
        emergency_stop_key=spec.key if spec.emergency_stop else None,
    )
    argv: Sequence[str] = terminal_argv(
        executable, title=spec.title, shell_command=command
    )
    return popen(list(argv), cwd=str(project_root), start_new_session=True)


def open_project_folder(project_root: Path) -> subprocess.Popen[bytes]:
    executable = shutil.which("xdg-open")
    if executable is None:
        raise RuntimeError("未找到xdg-open，无法打开项目目录")
    return subprocess.Popen(
        [executable, str(project_root)],
        cwd=str(project_root),
        start_new_session=True,
    )


def open_usage_guide(guide_path: Path) -> subprocess.Popen[bytes]:
    if not guide_path.is_file():
        raise RuntimeError(f"未找到使用说明：{guide_path}")
    executable = shutil.which("xdg-open")
    if executable is None:
        raise RuntimeError("未找到xdg-open，无法打开使用说明")
    return subprocess.Popen(
        [executable, str(guide_path)],
        cwd=str(guide_path.parent),
        start_new_session=True,
    )
