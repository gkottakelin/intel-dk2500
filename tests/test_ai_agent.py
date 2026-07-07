import json
import os
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

try:
    from project.src.jetarm_agent.config import AgentSettings, ConfigurationError
    from project.src.jetarm_agent.openai_compatible import APIClientError, OpenAICompatibleClient
    from project.src.jetarm_agent.roundtrip_test import run_counter_roundtrip_test
    from project.src.jetarm_agent.session import ChatSession
    from project.src.jetarm_agent.tooling import TestCounter, ToolExecutionError, ToolRegistry
except ModuleNotFoundError:
    from src.jetarm_agent.config import AgentSettings, ConfigurationError
    from src.jetarm_agent.openai_compatible import APIClientError, OpenAICompatibleClient
    from src.jetarm_agent.roundtrip_test import run_counter_roundtrip_test
    from src.jetarm_agent.session import ChatSession
    from src.jetarm_agent.tooling import TestCounter, ToolExecutionError, ToolRegistry


def write_config(directory: str, **api_overrides) -> Path:
    api = {
        "provider": "openai_compatible",
        "base_url": "https://config.example/v1",
        "model": "config-model",
        "api_key_env": "TEST_API_KEY",
        "timeout_s": 30,
    }
    api.update(api_overrides)
    path = Path(directory) / "ai_agent.json"
    path.write_text(
        json.dumps(
            {
                "api": api,
                "conversation": {
                    "temperature": 0.2,
                    "max_tokens": 100,
                    "max_history_messages": 4,
                    "system_prompt": "test system prompt",
                },
            }
        ),
        encoding="utf-8",
    )
    return path


class FakeStream:
    def __init__(self, tokens):
        self.tokens = list(tokens)

    def __aiter__(self):
        self.index = 0
        return self

    async def __anext__(self):
        if self.index >= len(self.tokens):
            raise StopAsyncIteration
        token = self.tokens[self.index]
        self.index += 1
        return SimpleNamespace(
            choices=[SimpleNamespace(delta=SimpleNamespace(content=token))]
        )


class FakeCompletions:
    def __init__(self, responses):
        self.responses = list(responses)
        self.requests = []

    async def create(self, **kwargs):
        self.requests.append(kwargs)
        response = self.responses.pop(0)
        if isinstance(response, Exception):
            raise response
        if isinstance(response, list):
            return FakeStream(response)
        return response


class FakeOpenAIClient:
    def __init__(self, responses):
        self.completions = FakeCompletions(responses)
        self.chat = SimpleNamespace(completions=self.completions)


def tool_call_response(
    name: str = TestCounter.TOOL_NAME,
    arguments: str = '{"amount": 1}',
    call_id: str = "call-test-1",
):
    return SimpleNamespace(
        choices=[
            SimpleNamespace(
                message=SimpleNamespace(
                    content=None,
                    tool_calls=[
                        SimpleNamespace(
                            id=call_id,
                            type="function",
                            function=SimpleNamespace(name=name, arguments=arguments),
                        )
                    ],
                )
            )
        ]
    )


def text_response(text: str):
    return SimpleNamespace(
        choices=[
            SimpleNamespace(
                message=SimpleNamespace(content=text, tool_calls=None)
            )
        ]
    )


class AgentSettingsTest(unittest.TestCase):
    def test_environment_and_cli_override_json(self):
        with tempfile.TemporaryDirectory() as directory:
            path = write_config(directory)
            settings = AgentSettings.from_sources(
                path,
                environ={
                    "JETARM_API_BASE_URL": "https://env.example/v1/",
                    "JETARM_API_MODEL": "env-model",
                },
                model="cli-model",
            )
        self.assertEqual(settings.base_url, "https://env.example/v1")
        self.assertEqual(settings.model, "cli-model")

    def test_api_key_is_read_only_from_environment(self):
        with tempfile.TemporaryDirectory() as directory:
            settings = AgentSettings.from_sources(write_config(directory), environ={})
        with self.assertRaises(ConfigurationError):
            settings.resolve_api_key({})
        self.assertEqual(settings.resolve_api_key({"TEST_API_KEY": "secret"}), "secret")

    def test_kimi_can_omit_temperature_and_send_extra_body(self):
        with tempfile.TemporaryDirectory() as directory:
            path = write_config(directory, extra_body={"thinking": {"type": "disabled"}})
            payload = json.loads(path.read_text(encoding="utf-8"))
            payload["conversation"]["temperature"] = None
            path.write_text(json.dumps(payload), encoding="utf-8")
            settings = AgentSettings.from_sources(path, environ={})

        self.assertIsNone(settings.temperature)
        self.assertEqual(settings.extra_body, {"thinking": {"type": "disabled"}})


class OpenAICompatibleClientTest(unittest.TestCase):
    def test_socks_proxy_initialization_error_has_actionable_message(self):
        class BrokenAsyncOpenAI:
            def __init__(self, **kwargs):
                raise ValueError(
                    "Unknown scheme for proxy URL URL('socks://127.0.0.1:7897/')"
                )

        with tempfile.TemporaryDirectory() as directory:
            settings = AgentSettings.from_sources(
                write_config(directory),
                environ={"TEST_API_KEY": "secret"},
            )
            fake_openai = SimpleNamespace(AsyncOpenAI=BrokenAsyncOpenAI)
            with patch.dict(os.environ, {"TEST_API_KEY": "secret"}), patch.dict(
                sys.modules, {"openai": fake_openai}
            ):
                with self.assertRaisesRegex(APIClientError, "SOCKS.*requirements-ai.txt"):
                    OpenAICompatibleClient(settings)


class ChatSessionTest(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.settings = AgentSettings.from_sources(write_config(temporary.name), environ={})

    async def test_streaming_reply_and_multi_turn_history(self):
        fake = FakeOpenAIClient([["你", "好"], ["第二", "次"]])
        client = OpenAICompatibleClient(self.settings, client=fake)
        session = ChatSession(self.settings, client)
        seen = []

        first = await session.ask("你好", on_token=seen.append)
        second = await session.ask("继续")

        self.assertEqual(first, "你好")
        self.assertEqual(second, "第二次")
        self.assertEqual(seen, ["你", "好"])
        self.assertEqual(len(session.history), 4)
        second_request = fake.completions.requests[1]["messages"]
        self.assertEqual(second_request[0]["role"], "system")
        self.assertEqual(second_request[1]["content"], "你好")
        self.assertEqual(second_request[2]["content"], "你好")
        self.assertEqual(second_request[3]["content"], "继续")

    async def test_failed_request_does_not_pollute_history(self):
        fake = FakeOpenAIClient([RuntimeError("offline")])
        client = OpenAICompatibleClient(self.settings, client=fake)
        session = ChatSession(self.settings, client)

        with self.assertRaises(APIClientError):
            await session.ask("失败测试")
        self.assertEqual(session.history, [])

    async def test_clear_removes_context(self):
        fake = FakeOpenAIClient([["ok"]])
        session = ChatSession(
            self.settings,
            OpenAICompatibleClient(self.settings, client=fake),
        )
        await session.ask("test")
        session.clear()
        self.assertEqual(session.history, [])


class ToolCallingRoundTripTest(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        path = write_config(
            temporary.name,
            base_url="https://api.moonshot.cn/v1",
            model="kimi-k2.6",
            extra_body={"thinking": {"type": "disabled"}},
        )
        payload = json.loads(path.read_text(encoding="utf-8"))
        payload["conversation"]["temperature"] = None
        path.write_text(json.dumps(payload), encoding="utf-8")
        self.settings = AgentSettings.from_sources(path, environ={})

    async def test_program_ai_tool_ai_roundtrip(self):
        fake = FakeOpenAIClient(
            [
                tool_call_response(),
                text_response("测试完成，计数器为1。"),
            ]
        )
        client = OpenAICompatibleClient(self.settings, client=fake)
        slept = []
        statuses = []

        async def fake_sleep(seconds):
            slept.append(seconds)

        result = await run_counter_roundtrip_test(
            self.settings,
            client,
            delay_s=3,
            sleep=fake_sleep,
            on_status=statuses.append,
        )

        self.assertEqual(slept, [3])
        self.assertEqual(result.counter, 1)
        self.assertEqual(result.tool_call_count, 1)
        self.assertEqual(result.answer, "测试完成，计数器为1。")
        self.assertEqual(len(fake.completions.requests), 2)

        first_request, second_request = fake.completions.requests
        self.assertEqual(first_request["messages"][-1], {"role": "user", "content": "ok"})
        self.assertEqual(first_request["tool_choice"], "auto")
        self.assertNotIn("temperature", first_request)
        self.assertEqual(
            first_request["extra_body"], {"thinking": {"type": "disabled"}}
        )
        self.assertEqual(second_request["tool_choice"], "none")
        tool_message = second_request["messages"][-1]
        self.assertEqual(tool_message["role"], "tool")
        self.assertEqual(tool_message["tool_call_id"], "call-test-1")
        self.assertEqual(json.loads(tool_message["content"]), {"status": "ok", "count": 1})
        self.assertTrue(any("程序 -> AI: ok" in status for status in statuses))

    async def test_roundtrip_reprompts_when_kimi_first_returns_text(self):
        fake = FakeOpenAIClient(
            [
                text_response("ok"),
                tool_call_response(),
                text_response("计数器为1。"),
            ]
        )
        client = OpenAICompatibleClient(self.settings, client=fake)

        async def no_sleep(_seconds):
            return None

        result = await run_counter_roundtrip_test(
            self.settings,
            client,
            delay_s=0,
            sleep=no_sleep,
        )

        self.assertEqual(result.counter, 1)
        self.assertEqual(len(fake.completions.requests), 3)
        retry_message = fake.completions.requests[1]["messages"][-1]
        self.assertEqual(retry_message["role"], "user")
        self.assertIn(TestCounter.TOOL_NAME, retry_message["content"])

    async def test_registry_rejects_unregistered_code(self):
        registry = ToolRegistry([TestCounter().definition()])
        with self.assertRaisesRegex(ToolExecutionError, "未注册工具"):
            await registry.execute("run_arbitrary_python", {"code": "print('unsafe')"})


if __name__ == "__main__":
    unittest.main()
