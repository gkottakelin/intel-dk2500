"""Small JSON configuration helpers used by the control center."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Iterator, Mapping


def load_json(path: Path, *, default: Mapping[str, Any] | None = None) -> dict[str, Any]:
    if not path.is_file():
        return dict(default or {})
    with path.open("r", encoding="utf-8") as file:
        payload = json.load(file)
    if not isinstance(payload, dict):
        raise ValueError(f"配置文件根节点必须是JSON对象: {path}")
    return payload


def save_json(path: Path, payload: Mapping[str, Any]) -> None:
    """Write JSON atomically so a failed save does not truncate the config."""

    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def validate_device_values(
    *,
    arm_mode: str,
    arm_port: str,
    arm_terminal_config: str,
    grasp_x: str,
    grasp_y: str,
) -> tuple[float | None, float | None]:
    if arm_mode not in {"off", "dry-run", "hardware"}:
        raise ValueError("机械臂模式必须是 hardware、dry-run 或 off")
    if arm_mode == "hardware" and not arm_port.strip():
        raise ValueError("hardware模式必须填写机械臂串口")
    if not arm_terminal_config.strip():
        raise ValueError("操作终端配置文件路径不能为空")
    if bool(grasp_x.strip()) != bool(grasp_y.strip()):
        raise ValueError("抓取点像素X和Y必须同时填写或同时留空")
    if not grasp_x.strip():
        return None, None
    try:
        x = float(grasp_x)
        y = float(grasp_y)
    except ValueError as exc:
        raise ValueError("抓取点像素必须是数字") from exc
    if not math.isfinite(x) or not math.isfinite(y) or x < 0 or y < 0:
        raise ValueError("抓取点像素必须是大于等于0的有限数字")
    return x, y


def validate_agent_values(
    *,
    base_url: str,
    model: str,
    api_key_env: str,
    timeout_s: str,
) -> float:
    if not base_url.strip():
        raise ValueError("API Base URL不能为空")
    if not model.strip():
        raise ValueError("模型名称不能为空")
    if not api_key_env.strip():
        raise ValueError("API Key环境变量名不能为空")
    try:
        timeout = float(timeout_s)
    except ValueError as exc:
        raise ValueError("API超时时间必须是数字") from exc
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError("API超时时间必须大于0")
    return timeout


def flatten_json(
    payload: Mapping[str, Any], prefix: str = ""
) -> Iterator[tuple[str, str]]:
    """Yield every leaf as a dotted path and a display value."""

    for key, value in payload.items():
        path = f"{prefix}.{key}" if prefix else str(key)
        if isinstance(value, Mapping):
            yield from flatten_json(value, path)
        elif isinstance(value, list):
            yield path, json.dumps(value, ensure_ascii=False)
        elif isinstance(value, bool):
            yield path, "true" if value else "false"
        elif value is None:
            yield path, "null"
        else:
            yield path, str(value)


def env_file_declares(path: Path, variable_name: str) -> bool:
    """Report whether an env file declares a non-empty value without exposing it."""

    if not variable_name or not path.is_file():
        return False
    for raw_line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        name, value = line.split("=", 1)
        if name.strip() == variable_name and value.strip().strip("'\""):
            return True
    return False
