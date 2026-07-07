import json
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

try:
    from project.src.jetarm_agent.arm_control import (
        ArmControlConfig,
        ArmControlError,
        JetArmToolController,
        build_arm_tool_registry,
        choose_arm_serial_port,
        looks_like_arm_command,
    )
    from project.src.jetarm_agent.config import AgentSettings
    from project.src.jetarm_agent.openai_compatible import (
        FunctionToolCall,
        ToolModelResponse,
    )
    from project.src.jetarm_agent.tool_agent import ToolCallingSession
except ModuleNotFoundError:
    from src.jetarm_agent.arm_control import (
        ArmControlConfig,
        ArmControlError,
        JetArmToolController,
        build_arm_tool_registry,
        choose_arm_serial_port,
        looks_like_arm_command,
    )
    from src.jetarm_agent.config import AgentSettings
    from src.jetarm_agent.openai_compatible import FunctionToolCall, ToolModelResponse
    from src.jetarm_agent.tool_agent import ToolCallingSession


PROJECT_ROOT = Path(__file__).resolve().parents[1]


class FakeToolModelClient:
    def __init__(self, responses):
        self.responses = list(responses)
        self.requests = []

    async def complete_with_tools(self, messages, tools, *, tool_choice="auto"):
        self.requests.append(
            {"messages": list(messages), "tools": list(tools), "tool_choice": tool_choice}
        )
        return self.responses.pop(0)


class ArmControlDryRunTest(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.controller = JetArmToolController(ArmControlConfig(mode="dry-run"))

    async def asyncTearDown(self):
        self.controller.close()

    async def test_moves_forward_five_centimeters_in_small_steps(self):
        result = await self.controller.move_tcp("forward", 5)

        self.assertEqual(result["status"], "ok")
        self.assertEqual(result["requested_distance_cm"], 5)
        self.assertGreater(result["estimated_distance_cm"], 4)
        self.assertEqual(result["steps"], 13)
        self.assertTrue(self.controller.controller.move_calls)
        self.assertTrue(
            all(
                servo_id in {1, 2, 3, 4}
                for servo_id, _target, _run_time in self.controller.controller.move_calls
            )
        )

    async def test_rejects_distance_above_safety_limit(self):
        with self.assertRaisesRegex(ArmControlError, "不能超过10"):
            await self.controller.move_tcp("forward", 10.1)
        self.assertEqual(self.controller.controller.move_calls, [])

    async def test_wrist_and_gripper_motor_actions_always_stop(self):
        await self.controller.rotate_wrist("clockwise", 0.5)
        await self.controller.control_gripper("open", 0.5)
        await self.controller.control_gripper("grip_lock")
        await self.controller.control_gripper("release_lock")

        calls = self.controller.controller.motor_calls
        self.assertEqual(calls[0:2], [(5, 100), (5, 0)])
        self.assertEqual(calls[2:4], [(10, -100), (10, 0)])
        self.assertEqual(calls[4:6], [(10, 300), (10, 0)])

    async def test_home_stop_and_state_cover_original_terminal_actions(self):
        home = await self.controller.go_home()
        stopped = await self.controller.stop_all()
        state = await self.controller.state()

        self.assertEqual(home["joint_positions"], {"J1": 500, "J2": 550, "J3": 550, "J4": 900})
        self.assertEqual(stopped["action"], "stop_all")
        self.assertIn("tcp_cm", state)
        home_servo_ids = {
            servo_id
            for servo_id, _target, _run_time in self.controller.controller.move_calls
        }
        self.assertEqual(home_servo_ids, {1, 2, 3, 4, 5, 10})

    async def test_tool_registry_exposes_only_bounded_arm_functions(self):
        schemas = build_arm_tool_registry(self.controller).schemas()
        names = {schema["function"]["name"] for schema in schemas}

        self.assertEqual(
            names,
            {
                "move_jetarm_tcp",
                "rotate_jetarm_wrist",
                "control_jetarm_gripper",
                "move_jetarm_home",
                "stop_jetarm",
                "get_jetarm_state",
            },
        )
        move_schema = next(
            schema for schema in schemas if schema["function"]["name"] == "move_jetarm_tcp"
        )
        self.assertEqual(
            move_schema["function"]["parameters"]["properties"]["distance_cm"]["maximum"],
            10,
        )

    async def test_arm_command_detection_does_not_require_model_guessing(self):
        self.assertTrue(looks_like_arm_command("向前移动5厘米"))
        self.assertTrue(looks_like_arm_command("夹紧夹爪"))
        self.assertFalse(looks_like_arm_command("请介绍一下机械臂的结构"))

    async def test_serial_chooser_reuses_ubuntu_terminal_dialog(self):
        class FakeRoot:
            def __init__(self):
                self.withdrawn = False
                self.destroyed = False

            def withdraw(self):
                self.withdrawn = True

            def destroy(self):
                self.destroyed = True

        root = FakeRoot()
        calls = []

        def choose(fake_root, initial_port):
            calls.append((fake_root, initial_port))
            return "/dev/ttyUSB0"

        terminal = SimpleNamespace(
            tk=SimpleNamespace(Tk=lambda: root),
            choose_serial_port_dialog=choose,
        )
        patch_target = f"{choose_arm_serial_port.__module__}._load_terminal_module"
        with patch(patch_target, return_value=terminal):
            selected = choose_arm_serial_port()

        self.assertEqual(selected, "/dev/ttyUSB0")
        self.assertEqual(calls, [(root, None)])
        self.assertTrue(root.withdrawn)
        self.assertTrue(root.destroyed)

    async def test_ai_tool_call_executes_distance_planner_and_returns_result(self):
        fake = FakeToolModelClient(
            [
                ToolModelResponse(content="好的。", tool_calls=()),
                ToolModelResponse(
                    content="",
                    tool_calls=(
                        FunctionToolCall(
                            call_id="arm-call-1",
                            name="move_jetarm_tcp",
                            arguments=json.dumps(
                                {"direction": "forward", "distance_cm": 1}
                            ),
                        ),
                    ),
                ),
                ToolModelResponse(content="已向前移动1厘米。", tool_calls=()),
            ]
        )
        settings = AgentSettings.from_sources(
            PROJECT_ROOT / "config" / "ai_agent.json", environ={}
        )
        session = ToolCallingSession(
            settings,
            fake,
            build_arm_tool_registry(self.controller),
        )

        command = "向前移动1厘米"
        result = await session.ask(
            command,
            require_any_tool=looks_like_arm_command(command),
            required_tool_retries=1,
        )

        self.assertEqual(result.text, "已向前移动1厘米。")
        self.assertEqual(result.tool_calls[0].name, "move_jetarm_tcp")
        self.assertEqual(result.tool_calls[0].result["status"], "ok")
        self.assertTrue(self.controller.controller.move_calls)
        self.assertEqual(len(fake.requests), 3)
        self.assertIn("必须调用", fake.requests[1]["messages"][-1]["content"])
        tool_result_message = fake.requests[2]["messages"][-1]
        self.assertEqual(tool_result_message["role"], "tool")
        self.assertEqual(tool_result_message["tool_call_id"], "arm-call-1")


if __name__ == "__main__":
    unittest.main()
