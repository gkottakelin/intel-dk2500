"""Data-layer hierarchical image tiles for model-guided pixel localization."""

from __future__ import annotations

import base64
from collections.abc import Mapping
from typing import Any

from .tooling import ToolDefinition, ToolExecutionError, ToolExecutionPayload, ToolImage


VISUAL_TILE_TOOL = "zoom_rgb_target_tile"
TARGET_PIXEL_CONTROL_TOOL = "control_jetarm_to_target_pixel"
VISUAL_TILE_GRID_SIZE = 3
VISUAL_TILE_REQUIRED_DEPTH = 4


class VisualTileLocator:
    """Narrow the latest RGB frame without drawing coordinate labels on it."""

    def __init__(
        self,
        *,
        grid_size: int = VISUAL_TILE_GRID_SIZE,
        required_depth: int = VISUAL_TILE_REQUIRED_DEPTH,
        jpeg_quality: int = 95,
    ) -> None:
        if grid_size < 2:
            raise ValueError("分块网格边长必须至少为2")
        if required_depth < 1:
            raise ValueError("分块定位深度必须至少为1")
        self.grid_size = int(grid_size)
        self.required_depth = int(required_depth)
        self.jpeg_quality = int(jpeg_quality)
        self.clear()

    def clear(self) -> None:
        self._frame: Any = None
        self._width = 0
        self._height = 0
        self._bounds = (0, 0, 0, 0)
        self._depth = 0

    @property
    def has_frame(self) -> bool:
        return self._frame is not None

    @property
    def depth(self) -> int:
        return self._depth

    @property
    def ready(self) -> bool:
        return self.has_frame and self._depth >= self.required_depth

    def definition(self) -> ToolDefinition:
        last = self.grid_size - 1
        return ToolDefinition(
            name=VISUAL_TILE_TOOL,
            description=(
                "在不向图像绘制标签的情况下，对最新RGB画面的目标区域进行数据层分块放大。"
                f"当前画面在数据层等分为{self.grid_size}行x{self.grid_size}列；"
                f"row=0是最上方、row={last}是最下方，column=0是最左侧、"
                f"column={last}是最右侧。只选择用户指定物品中心所在的一块。"
                "每次调用返回该块JPEG及其在原始图像中的精确边界；看到返回的新图后再调用下一次，"
                f"共完成{self.required_depth}层。每个模型回合只允许调用一次本工具。"
            ),
            parameters={
                "type": "object",
                "properties": {
                    "row": {
                        "type": "integer",
                        "minimum": 0,
                        "maximum": last,
                        "description": "目标物品中心所在行；0从最上方开始。",
                    },
                    "column": {
                        "type": "integer",
                        "minimum": 0,
                        "maximum": last,
                        "description": "目标物品中心所在列；0从最左侧开始。",
                    },
                },
                "required": ["row", "column"],
                "additionalProperties": False,
            },
            handler=self.zoom,
        )

    def set_frame(self, image: ToolImage) -> None:
        try:
            import cv2
            import numpy as np
        except ImportError as exc:
            raise ToolExecutionError("分块定位需要OpenCV和NumPy") from exc

        try:
            encoded = base64.b64decode(image.data, validate=True)
        except (ValueError, TypeError) as exc:
            raise ToolExecutionError("最新RGB图像不是有效Base64数据") from exc
        array = np.frombuffer(encoded, dtype=np.uint8)
        frame = cv2.imdecode(array, cv2.IMREAD_COLOR)
        if frame is None or frame.ndim < 2:
            raise ToolExecutionError("无法解码最新RGB图像用于分块定位")

        height, width = frame.shape[:2]
        if width < self.grid_size or height < self.grid_size:
            raise ToolExecutionError(
                f"RGB图像尺寸{width}x{height}不足以执行{self.grid_size}x{self.grid_size}分块"
            )
        self._frame = frame
        self._width = int(width)
        self._height = int(height)
        self._bounds = (0, 0, self._width, self._height)
        self._depth = 0

    async def zoom(self, arguments: Mapping[str, Any]) -> ToolExecutionPayload:
        if not self.has_frame:
            raise ToolExecutionError("没有可分块的最新RGB图像，请先调用get_rgb_camera_frame")
        if self.ready:
            raise ToolExecutionError(
                "分块定位已完成，请使用返回的最终原图坐标调用目标像素控制工具"
            )

        row = self._validate_index(arguments.get("row"), "row")
        column = self._validate_index(arguments.get("column"), "column")
        x0, y0, x1, y1 = self._bounds
        x_edges = self._partition_edges(x0, x1)
        y_edges = self._partition_edges(y0, y1)
        selected = (
            x_edges[column],
            y_edges[row],
            x_edges[column + 1],
            y_edges[row + 1],
        )
        selected_x0, selected_y0, selected_x1, selected_y1 = selected
        crop = self._frame[selected_y0:selected_y1, selected_x0:selected_x1]
        if crop.size == 0:
            raise ToolExecutionError("选中的分块为空，无法继续定位")

        try:
            import cv2
        except ImportError as exc:
            raise ToolExecutionError("分块定位需要OpenCV") from exc
        encoded_ok, encoded = cv2.imencode(
            ".jpg",
            crop,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
        )
        if not encoded_ok:
            raise ToolExecutionError("目标分块JPEG编码失败")

        self._bounds = selected
        self._depth += 1
        localization = self.summary()
        localization.update(
            {
                "selected_row": row,
                "selected_column": column,
                "next_action": (
                    "call_control_jetarm_to_target_pixel"
                    if self.ready
                    else "inspect_returned_crop_then_zoom_again"
                ),
            }
        )
        value = {
            "status": "ok",
            "mcp": VISUAL_TILE_TOOL,
            "visual_tile_localization": localization,
        }
        image = ToolImage(
            data=base64.b64encode(encoded.tobytes()).decode("ascii"),
            mime_type="image/jpeg",
        )
        return ToolExecutionPayload(value=value, images=(image,))

    def summary(self) -> dict[str, Any]:
        x0, y0, x1, y1 = self._bounds
        center_x = (x0 + x1 - 1) / 2.0
        center_y = (y0 + y1 - 1) / 2.0
        return {
            "method": "hierarchical_data_layer_tiles",
            "grid": {"rows": self.grid_size, "columns": self.grid_size},
            "depth": self._depth,
            "required_depth": self.required_depth,
            "ready": self.ready,
            "original_image_size": {
                "width": self._width,
                "height": self._height,
            },
            "original_bounds_inclusive": {
                "x_min": x0,
                "y_min": y0,
                "x_max": x1 - 1,
                "y_max": y1 - 1,
            },
            "estimated_target_pixel": {
                "x": round(center_x, 3),
                "y": round(center_y, 3),
            },
            "maximum_quantization_error_px": {
                "x": round(max(0.0, (x1 - x0 - 1) / 2.0), 3),
                "y": round(max(0.0, (y1 - y0 - 1) / 2.0), 3),
            },
        }

    def target_pixel(self) -> tuple[float, float]:
        if not self.ready:
            raise ToolExecutionError(
                f"分块定位尚未完成：当前{self._depth}/{self.required_depth}层"
            )
        pixel = self.summary()["estimated_target_pixel"]
        return float(pixel["x"]), float(pixel["y"])

    def _validate_index(self, value: object, name: str) -> int:
        if isinstance(value, bool) or not isinstance(value, int):
            raise ToolExecutionError(f"{name}必须是整数")
        if not 0 <= value < self.grid_size:
            raise ToolExecutionError(
                f"{name}必须在0到{self.grid_size - 1}之间"
            )
        return int(value)

    def _partition_edges(self, start: int, end: int) -> list[int]:
        length = end - start
        return [
            start + (length * index) // self.grid_size
            for index in range(self.grid_size)
        ] + [end]
