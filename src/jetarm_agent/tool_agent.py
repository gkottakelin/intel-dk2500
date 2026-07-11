"""Model -> local tool -> model execution loop."""

from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Any, Callable

from .config import AgentSettings
from .openai_compatible import OpenAICompatibleClient, ToolModelResponse
from .tooling import (
    ToolExecutionError,
    ToolExecutionPayload,
    ToolImage,
    ToolRegistry,
)
from .visual_tiles import (
    TARGET_PIXEL_CONTROL_TOOL,
    VISUAL_TILE_TOOL,
    VisualTileLocator,
)

SEQUENTIAL_MOTION_TOOLS = frozenset(
    {
        "move_jetarm",
        "move_jetarm_tcp",
        "move_jetarm_by_pixel_error",
        "control_jetarm_to_target_pixel",
    }
)
RGB_CAMERA_TOOL = "get_rgb_camera_frame"
MAX_VISUAL_CLOSED_LOOP_ROUNDS = 200


@dataclass(frozen=True)
class ExecutedToolCall:
    call_id: str
    name: str
    arguments: dict[str, Any]
    result: object
    images: tuple[ToolImage, ...] = ()


@dataclass(frozen=True)
class ToolAgentResult:
    text: str
    tool_calls: tuple[ExecutedToolCall, ...]


class ToolCallingSession:
    """Run bounded tool calls while preserving valid conversation messages."""

    def __init__(
        self,
        settings: AgentSettings,
        client: OpenAICompatibleClient,
        registry: ToolRegistry,
        *,
        system_prompt: str | None = None,
        max_rounds: int = MAX_VISUAL_CLOSED_LOOP_ROUNDS,
    ) -> None:
        if max_rounds < 1:
            raise ValueError("max_rounds必须大于0")
        self.settings = settings
        self.client = client
        self.registry = registry
        self.visual_tile_locator: VisualTileLocator | None = None
        if RGB_CAMERA_TOOL in self.registry.names():
            self.visual_tile_locator = VisualTileLocator()
            if VISUAL_TILE_TOOL not in self.registry.names():
                self.registry.register(self.visual_tile_locator.definition())
        self.system_prompt = system_prompt or settings.system_prompt
        self.max_rounds = max_rounds
        self.history: list[dict[str, Any]] = []

    def clear(self) -> None:
        self.history.clear()
        if self.visual_tile_locator is not None:
            self.visual_tile_locator.clear()

    async def ask(
        self,
        text: str,
        *,
        first_tool_choice: object = "auto",
        allow_additional_tools: bool = True,
        require_any_tool: bool = False,
        required_tool_name: str | None = None,
        required_tool_retries: int = 1,
        preselected_tool_name: str | None = None,
        preselected_tool_arguments: dict[str, Any] | None = None,
        on_tool_call: Callable[[ExecutedToolCall], None] | None = None,
    ) -> ToolAgentResult:
        user_text = text.strip()
        if not user_text:
            raise ValueError("对话内容不能为空")

        turn: list[dict[str, Any]] = [{"role": "user", "content": user_text}]
        executed: list[ExecutedToolCall] = []
        tool_choice = first_tool_choice
        retry_count = 0
        camera_tool_available = RGB_CAMERA_TOOL in self.registry.names()
        fresh_rgb_observation = False

        if preselected_tool_arguments is not None:
            selected_tool_name = preselected_tool_name or required_tool_name
            if selected_tool_name is None:
                raise ValueError(
                    "预选工具必须提供preselected_tool_name或required_tool_name"
                )
            call_id = f"local-{selected_tool_name}-{len(self.history)}"
            raw_arguments = json.dumps(preselected_tool_arguments, ensure_ascii=False)
            turn.append(
                {
                    "role": "assistant",
                    "content": None,
                    "tool_calls": [
                        {
                            "id": call_id,
                            "type": "function",
                            "function": {
                                "name": selected_tool_name,
                                "arguments": raw_arguments,
                            },
                        }
                    ],
                }
            )
            arguments, result, images = await self._execute(
                selected_tool_name, raw_arguments
            )
            executed_call = ExecutedToolCall(
                call_id=call_id,
                name=selected_tool_name,
                arguments=arguments,
                result=result,
                images=images,
            )
            executed.append(executed_call)
            if on_tool_call is not None:
                on_tool_call(executed_call)
            turn.append(
                {
                    "role": "tool",
                    "tool_call_id": call_id,
                    "name": selected_tool_name,
                    "content": json.dumps(result, ensure_ascii=False),
                }
            )
            if images:
                self._append_latest_images(turn, images, observation=result)
            if selected_tool_name == RGB_CAMERA_TOOL:
                fresh_rgb_observation = self._successful_rgb_result(result, images)
                self._remember_rgb_frame(result, images)

        for _ in range(self.max_rounds):
            messages = [
                {"role": "system", "content": self.system_prompt},
                *self.history,
                *turn,
            ]
            response = await self.client.complete_with_tools(
                messages,
                self.registry.schemas(),
                tool_choice=tool_choice,
            )
            turn.append(response.assistant_message())

            if not response.tool_calls:
                required_name_executed = required_tool_name is None or any(
                    call.name == required_tool_name for call in executed
                )
                any_tool_executed = not require_any_tool or bool(executed)
                if not required_name_executed or not any_tool_executed:
                    if retry_count >= required_tool_retries:
                        required = required_tool_name or "任一已注册工具"
                        raise RuntimeError(f"AI没有调用必需工具: {required}")
                    retry_count += 1
                    required = required_tool_name or "适合当前指令的已注册工具"
                    turn.append(
                        {
                            "role": "user",
                            "content": (
                                f"本次指令必须调用{required}，"
                                "请现在返回tool_calls，不要只回复文字。"
                            ),
                        }
                    )
                    tool_choice = "auto"
                    continue
                answer = response.content.strip()
                if not answer:
                    raise RuntimeError("API返回了空回复且没有工具调用")
                self.history.extend(turn)
                self._trim_history()
                return ToolAgentResult(answer, tuple(executed))

            latest_images: tuple[ToolImage, ...] = ()
            latest_image_observation: object | None = None
            motion_call_seen = False
            tile_call_seen = False
            successful_motion = False
            rgb_was_visible_to_model = fresh_rgb_observation
            for tool_call in response.tool_calls:
                if tool_call.name == VISUAL_TILE_TOOL and not rgb_was_visible_to_model:
                    try:
                        parsed_arguments = json.loads(tool_call.arguments or "{}")
                    except json.JSONDecodeError:
                        parsed_arguments = {}
                    arguments = (
                        parsed_arguments if isinstance(parsed_arguments, dict) else {}
                    )
                    result = {
                        "status": "error",
                        "error": (
                            "本回合开始时模型尚未看到最新RGB图像或上一层裁剪图。"
                            "请先查看已返回的图像，下一回合再选择目标分块。"
                        ),
                    }
                    images = ()
                elif tool_call.name == VISUAL_TILE_TOOL and tile_call_seen:
                    try:
                        parsed_arguments = json.loads(tool_call.arguments or "{}")
                    except json.JSONDecodeError:
                        parsed_arguments = {}
                    arguments = (
                        parsed_arguments if isinstance(parsed_arguments, dict) else {}
                    )
                    result = {
                        "status": "error",
                        "error": (
                            "每个模型回合只允许选择一层目标分块。"
                            "必须先查看上一层返回的新裁剪图，再选择下一层。"
                        ),
                    }
                    images = ()
                elif (
                    tool_call.name == TARGET_PIXEL_CONTROL_TOOL
                    and self.visual_tile_locator is not None
                    and not self.visual_tile_locator.ready
                ):
                    try:
                        parsed_arguments = json.loads(tool_call.arguments or "{}")
                    except json.JSONDecodeError:
                        parsed_arguments = {}
                    arguments = (
                        parsed_arguments if isinstance(parsed_arguments, dict) else {}
                    )
                    result = {
                        "status": "error",
                        "error": (
                            "目标像素控制前必须完成数据层分块定位："
                            f"当前{self.visual_tile_locator.depth}/"
                            f"{self.visual_tile_locator.required_depth}层。"
                            f"请调用{VISUAL_TILE_TOOL}，每次查看返回的新图后再选择下一层。"
                        ),
                        "visual_tile_localization": self.visual_tile_locator.summary(),
                    }
                    images = ()
                    motion_call_seen = True
                elif tool_call.name in SEQUENTIAL_MOTION_TOOLS and motion_call_seen:
                    try:
                        parsed_arguments = json.loads(tool_call.arguments or "{}")
                    except json.JSONDecodeError:
                        parsed_arguments = {}
                    arguments = (
                        parsed_arguments if isinstance(parsed_arguments, dict) else {}
                    )
                    result = {
                        "status": "error",
                        "error": (
                            "同一轮只允许下发一条机械臂移动命令。"
                            "请等待上一条返回status=ok后，再单独下发下一条。"
                        ),
                    }
                    images = ()
                elif (
                    tool_call.name in SEQUENTIAL_MOTION_TOOLS
                    and camera_tool_available
                    and not rgb_was_visible_to_model
                ):
                    try:
                        parsed_arguments = json.loads(tool_call.arguments or "{}")
                    except json.JSONDecodeError:
                        parsed_arguments = {}
                    arguments = (
                        parsed_arguments if isinstance(parsed_arguments, dict) else {}
                    )
                    result = {
                        "status": "error",
                        "error": (
                            "视觉闭环禁止在没有最新RGB图像时移动。"
                            "必须先调用get_rgb_camera_frame并把图像传给Agent，"
                            "再根据该图像单独下发一条移动命令。"
                        ),
                    }
                    images = ()
                    motion_call_seen = True
                else:
                    if tool_call.name in SEQUENTIAL_MOTION_TOOLS:
                        motion_call_seen = True
                    raw_arguments = tool_call.arguments
                    if (
                        tool_call.name == TARGET_PIXEL_CONTROL_TOOL
                        and self.visual_tile_locator is not None
                    ):
                        try:
                            parsed_arguments = json.loads(raw_arguments or "{}")
                        except json.JSONDecodeError:
                            parsed_arguments = {}
                        if not isinstance(parsed_arguments, dict):
                            parsed_arguments = {}
                        target_x, target_y = self.visual_tile_locator.target_pixel()
                        parsed_arguments["target_x"] = target_x
                        parsed_arguments["target_y"] = target_y
                        raw_arguments = json.dumps(
                            parsed_arguments, ensure_ascii=False
                        )
                    arguments, result, images = await self._execute(
                        tool_call.name, raw_arguments
                    )
                    if tool_call.name == VISUAL_TILE_TOOL:
                        tile_call_seen = True
                    if tool_call.name == RGB_CAMERA_TOOL:
                        fresh_rgb_observation = self._successful_rgb_result(
                            result, images
                        )
                        self._remember_rgb_frame(result, images)
                    elif tool_call.name in SEQUENTIAL_MOTION_TOOLS:
                        fresh_rgb_observation = False
                        successful_motion = self._successful_result(result)
                        if (
                            tool_call.name == TARGET_PIXEL_CONTROL_TOOL
                            and isinstance(result, dict)
                            and self.visual_tile_locator is not None
                        ):
                            result["target_pixel_localization"] = (
                                self.visual_tile_locator.summary()
                            )
                executed_call = ExecutedToolCall(
                    call_id=tool_call.call_id,
                    name=tool_call.name,
                    arguments=arguments,
                    result=result,
                    images=images,
                )
                executed.append(executed_call)
                if on_tool_call is not None:
                    on_tool_call(executed_call)
                turn.append(
                    {
                        "role": "tool",
                        "tool_call_id": tool_call.call_id,
                        "name": tool_call.name,
                        "content": json.dumps(result, ensure_ascii=False),
                    }
                )
                if images:
                    latest_images = images
                    latest_image_observation = result

            if latest_images:
                self._append_latest_images(
                    turn,
                    latest_images,
                    observation=latest_image_observation,
                )

            if successful_motion and camera_tool_available:
                camera_call_id = f"auto-rgb-after-motion-{len(executed)}"
                raw_arguments = "{}"
                turn.append(
                    {
                        "role": "assistant",
                        "content": None,
                        "tool_calls": [
                            {
                                "id": camera_call_id,
                                "type": "function",
                                "function": {
                                    "name": RGB_CAMERA_TOOL,
                                    "arguments": raw_arguments,
                                },
                            }
                        ],
                    }
                )
                arguments, result, images = await self._execute(
                    RGB_CAMERA_TOOL, raw_arguments
                )
                executed_call = ExecutedToolCall(
                    call_id=camera_call_id,
                    name=RGB_CAMERA_TOOL,
                    arguments=arguments,
                    result=result,
                    images=images,
                )
                executed.append(executed_call)
                if on_tool_call is not None:
                    on_tool_call(executed_call)
                turn.append(
                    {
                        "role": "tool",
                        "tool_call_id": camera_call_id,
                        "name": RGB_CAMERA_TOOL,
                        "content": json.dumps(result, ensure_ascii=False),
                    }
                )
                fresh_rgb_observation = self._successful_rgb_result(result, images)
                self._remember_rgb_frame(result, images)
                if images:
                    self._append_latest_images(turn, images, observation=result)

            tool_choice = "auto" if allow_additional_tools else "none"

        raise RuntimeError(f"工具调用超过最大轮数: {self.max_rounds}")

    @staticmethod
    def _successful_result(result: object) -> bool:
        return isinstance(result, dict) and result.get("status") == "ok"

    @classmethod
    def _successful_rgb_result(
        cls, result: object, images: tuple[ToolImage, ...]
    ) -> bool:
        return cls._successful_result(result) and bool(images)

    def _remember_rgb_frame(
        self, result: object, images: tuple[ToolImage, ...]
    ) -> None:
        if (
            self.visual_tile_locator is None
            or not self._successful_rgb_result(result, images)
        ):
            return
        try:
            self.visual_tile_locator.set_frame(images[0])
        except ToolExecutionError:
            self.visual_tile_locator.clear()

    async def _execute(
        self, name: str, raw_arguments: str
    ) -> tuple[dict[str, Any], object, tuple[ToolImage, ...]]:
        try:
            parsed = json.loads(raw_arguments or "{}")
            if not isinstance(parsed, dict):
                raise ToolExecutionError("工具参数必须是JSON对象")
            raw_result = await self.registry.execute(name, parsed)
            if isinstance(raw_result, ToolExecutionPayload):
                return parsed, raw_result.value, raw_result.images
            return parsed, raw_result, ()
        except (json.JSONDecodeError, ToolExecutionError, ValueError) as exc:
            arguments = parsed if "parsed" in locals() and isinstance(parsed, dict) else {}
            return arguments, {"status": "error", "error": str(exc)}, ()
        except Exception as exc:
            return {}, {"status": "error", "error": f"工具执行异常: {exc}"}, ()

    @staticmethod
    def _remove_images(messages: list[dict[str, Any]]) -> None:
        """Keep text/tool history while removing older base64 camera frames."""

        retained: list[dict[str, Any]] = []
        for message in messages:
            content = message.get("content")
            if not isinstance(content, list):
                retained.append(message)
                continue
            filtered = [
                part
                for part in content
                if not (
                    isinstance(part, dict)
                    and part.get("type") in {"image", "image_url"}
                )
            ]
            if filtered:
                retained.append({**message, "content": filtered})
        messages[:] = retained

    def _append_latest_images(
        self,
        turn: list[dict[str, Any]],
        images: tuple[ToolImage, ...],
        *,
        observation: object | None = None,
    ) -> None:
        self._remove_images(self.history)
        self._remove_images(turn)
        turn.append(
            {
                "role": "user",
                "content": [
                    {
                        "type": "text",
                        "text": (
                            "这是JetArm单路RGB相机刚刚返回的最新画面；"
                            "与它对应的抓取点坐标和相机姿态位于紧邻的工具结果arm_pose中。"
                            + self._rgb_coordinate_instruction(observation)
                        ),
                    },
                    *(image.openai_content_part() for image in images),
                ],
            }
        )

    @staticmethod
    def _rgb_coordinate_instruction(observation: object | None) -> str:
        localization = (
            observation.get("visual_tile_localization")
            if isinstance(observation, dict)
            else None
        )
        if isinstance(localization, dict):
            bounds = localization.get("original_bounds_inclusive")
            depth = localization.get("depth")
            required_depth = localization.get("required_depth")
            ready = bool(localization.get("ready"))
            target = localization.get("estimated_target_pixel")
            return (
                "这是数据层分块后的原始RGB局部图，没有在像素上绘制坐标标签；"
                f"它对应原图边界={bounds}，当前定位层数={depth}/{required_depth}。"
                + (
                    f"分块定位已完成，最终目标像素={target}；"
                    "调用control_jetarm_to_target_pixel时程序会强制使用该坐标，禁止自行改写。"
                    if ready
                    else (
                        f"请把当前局部图在数据层看作{localization.get('grid')}，"
                        f"只调用一次{VISUAL_TILE_TOOL}选择目标物品中心所在行列。"
                    )
                )
            )
        camera = (
            observation.get("camera")
            if isinstance(observation, dict)
            else None
        )
        if not isinstance(camera, dict):
            return (
                "像素坐标必须使用原图左上角为(0,0)、X向右、Y向下的标准图像坐标。"
            )
        width = camera.get("width")
        height = camera.get("height")
        grasp = camera.get("grasp_point_pixel")
        right = int(width) - 1 if isinstance(width, (int, float)) else "width-1"
        bottom = int(height) - 1 if isinstance(height, (int, float)) else "height-1"
        return (
            f"原始图像尺寸={width}x{height}；"
            "坐标必须直接读取原始RGB图像像素；"
            "左上角(0,0)，X向右增大，Y向下增大；"
            f"右上角=({right},0)，左下角=(0,{bottom})，"
            f"右下角=({right},{bottom})；"
            f"用户抓取点像素={grasp}。"
            "只提交目标物品中心的target_x/target_y；"
            "禁止使用左下角原点、Y向上坐标、百分比坐标、"
            "缩放后坐标或上下翻转坐标。"
        )

    def _trim_history(self) -> None:
        limit = self.settings.max_history_messages
        while len(self.history) > limit:
            next_user = next(
                (
                    index
                    for index, message in enumerate(self.history[1:], 1)
                    if message.get("role") == "user"
                ),
                len(self.history),
            )
            del self.history[:next_user]
