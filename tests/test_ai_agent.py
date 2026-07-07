import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from project.src.jetarm_agent.config import AgentSettings, ConfigurationError
from project.src.jetarm_agent.openai_compatible import APIClientError, OpenAICompatibleClient
from project.src.jetarm_agent.session import ChatSession


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
        return FakeStream(response)


class FakeOpenAIClient:
    def __init__(self, responses):
        self.completions = FakeCompletions(responses)
        self.chat = SimpleNamespace(completions=self.completions)


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


if __name__ == "__main__":
    unittest.main()
