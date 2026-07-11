"""Software emergency-stop registry for workflows launched by the control center."""

from __future__ import annotations

import json
import os
import signal
from dataclasses import dataclass
from pathlib import Path
from typing import Callable


RUNTIME_DIRECTORY_NAME = ".jetarm_runtime"
REGISTRY_SUFFIX = ".estop.json"


@dataclass(frozen=True)
class EmergencyStopTarget:
    key: str
    pid: int
    pgid: int
    token: str
    path: Path


@dataclass(frozen=True)
class EmergencyStopResult:
    active: tuple[EmergencyStopTarget, ...]
    signaled: tuple[EmergencyStopTarget, ...]
    failures: tuple[str, ...]


def runtime_directory(project_root: Path) -> Path:
    return Path(project_root) / RUNTIME_DIRECTORY_NAME


def registry_path(project_root: Path, key: str) -> Path:
    return runtime_directory(project_root) / f"{key}{REGISTRY_SUFFIX}"


def _process_has_token(pid: int, token: str) -> bool:
    """Reject stale/recycled PIDs by matching the launch token in /proc."""

    if pid <= 1 or not token:
        return False
    expected = f"JETARM_ESTOP_TOKEN={token}".encode("utf-8")
    try:
        environment = Path(f"/proc/{pid}/environ").read_bytes()
        if expected in environment.split(b"\0"):
            return True
    except OSError:
        pass
    # The launcher exports the token after bash starts. Some Linux kernels only
    # expose the initial environment through /proc, while bash's -lc command
    # line still contains the unique token assignment.
    try:
        command_line = Path(f"/proc/{pid}/cmdline").read_bytes()
    except OSError:
        return False
    return token.encode("utf-8") in command_line


def active_targets(
    project_root: Path,
    *,
    process_matches: Callable[[int, str], bool] = _process_has_token,
    remove_stale: bool = True,
) -> tuple[EmergencyStopTarget, ...]:
    directory = runtime_directory(project_root)
    if not directory.is_dir():
        return ()
    targets: list[EmergencyStopTarget] = []
    for path in sorted(directory.glob(f"*{REGISTRY_SUFFIX}")):
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
            target = EmergencyStopTarget(
                key=str(payload["key"]),
                pid=int(payload["pid"]),
                pgid=int(payload["pgid"]),
                token=str(payload["token"]),
                path=path,
            )
            valid = (
                target.path == registry_path(project_root, target.key)
                and target.pgid > 1
                and process_matches(target.pid, target.token)
            )
        except (OSError, ValueError, TypeError, KeyError, json.JSONDecodeError):
            valid = False
            target = None
        if valid and target is not None:
            targets.append(target)
        elif remove_stale:
            try:
                path.unlink()
            except OSError:
                pass
    return tuple(targets)


def request_emergency_stop(
    project_root: Path,
    *,
    signal_group: Callable[[int, int], None] | None = None,
    current_process_group: Callable[[], int] | None = None,
    process_matches: Callable[[int, str], bool] = _process_has_token,
) -> EmergencyStopResult:
    """Send SIGINT to every active arm workflow launched by this control center."""

    if signal_group is None:
        signal_group = getattr(os, "killpg", None)
    if current_process_group is None:
        current_process_group = getattr(os, "getpgrp", None)
    if signal_group is None or current_process_group is None:
        return EmergencyStopResult((), (), ("软件急停仅支持Ubuntu/Linux进程组",))
    active = active_targets(project_root, process_matches=process_matches)
    signaled: list[EmergencyStopTarget] = []
    failures: list[str] = []
    own_group = current_process_group()
    sent_groups: set[int] = set()
    for target in active:
        if target.pgid == own_group:
            failures.append(f"{target.key}: 拒绝中断总控自身进程组{target.pgid}")
            continue
        if target.pgid in sent_groups:
            signaled.append(target)
            continue
        try:
            signal_group(target.pgid, signal.SIGINT)
        except OSError as exc:
            failures.append(f"{target.key}: {exc}")
            continue
        sent_groups.add(target.pgid)
        signaled.append(target)
    return EmergencyStopResult(active, tuple(signaled), tuple(failures))
