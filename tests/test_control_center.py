from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path, PurePosixPath

from src.jetarm_control_center.config_store import (
    env_file_declares,
    flatten_json,
    load_json,
    save_json,
    validate_agent_values,
    validate_device_values,
)
from src.jetarm_control_center.terminal_launcher import (
    build_shell_command,
    default_launch_specs,
    terminal_argv,
)


class ControlCenterConfigTests(unittest.TestCase):
    def test_json_roundtrip_and_flatten(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.json"
            payload = {"home": {"J1": 500}, "enabled": True}
            save_json(path, payload)
            self.assertEqual(load_json(path), payload)
            self.assertEqual(
                list(flatten_json(payload)),
                [("home.J1", "500"), ("enabled", "true")],
            )

    def test_device_values_require_both_grasp_coordinates(self) -> None:
        with self.assertRaisesRegex(ValueError, "同时填写"):
            validate_device_values(
                arm_mode="dry-run",
                arm_port="",
                arm_terminal_config="terminal.json",
                grasp_x="320",
                grasp_y="",
            )
        self.assertEqual(
            validate_device_values(
                arm_mode="hardware",
                arm_port="/dev/ttyUSB0",
                arm_terminal_config="terminal.json",
                grasp_x="320",
                grasp_y="147",
            ),
            (320.0, 147.0),
        )

    def test_agent_values(self) -> None:
        self.assertEqual(
            validate_agent_values(
                base_url="https://example.test/v1",
                model="model",
                api_key_env="API_KEY",
                timeout_s="60",
            ),
            60.0,
        )

    def test_env_key_is_not_returned(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / ".env"
            path.write_text("SECRET_KEY=very-secret\n", encoding="utf-8")
            self.assertTrue(env_file_declares(path, "SECRET_KEY"))
            self.assertFalse(env_file_declares(path, "OTHER_KEY"))


class ControlCenterLauncherTests(unittest.TestCase):
    def test_every_required_workflow_has_a_launch_spec(self) -> None:
        keys = {spec.key for spec in default_launch_specs()}
        self.assertEqual(
            keys, {"git_pull", "arm_terminal", "camera", "manual_v2", "agent"}
        )

    def test_shell_command_changes_to_project_and_keeps_terminal_open(self) -> None:
        command = build_shell_command(PurePosixPath("/tmp/jet arm"), "example --flag")
        self.assertIn("cd -- '/tmp/jet arm'", command)
        self.assertIn("example --flag", command)
        self.assertIn("read -r", command)

    def test_missing_ai_environment_does_not_bypass_terminal_pause(self) -> None:
        commands = {
            spec.key: spec.command for spec in default_launch_specs()
        }
        self.assertNotIn("exit 1", commands["agent"])
        self.assertIn("false; fi", commands["agent"])

    def test_gnome_terminal_arguments_do_not_wrap_command_in_extra_quotes(self) -> None:
        argv = terminal_argv(
            "/usr/bin/gnome-terminal",
            title="JetArm测试",
            shell_command="echo ok",
        )
        self.assertEqual(argv[-3:], ["bash", "-lc", "echo ok"])


if __name__ == "__main__":
    unittest.main()
