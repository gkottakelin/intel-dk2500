"""Gemini Pro Plus Windows RGB-D stream test.

Controls:
    q / ESC: quit
    s: save current color/depth preview images to project/data/rgbd_samples

Run:
    python project/src/gemini_windows/camera_stream_test.py
"""

from __future__ import annotations

import argparse
from pathlib import Path
from time import strftime

import cv2
import numpy as np

from gemini_common import (
    choose_rgbd_config,
    depth_frame_to_mm,
    frame_to_bgr_image,
    get_camera_intrinsics,
    import_orbbec_sdk,
    median_depth_at,
    pixel_to_camera_point_mm,
    render_depth,
)


ESC_KEY = 27
WINDOW_NAME = "Gemini Pro Plus RGB-D"


class ClickState:
    def __init__(self) -> None:
        self.u: int | None = None
        self.v: int | None = None

    def update(self, u: int, v: int) -> None:
        self.u = u
        self.v = v


def _on_mouse(event: int, x: int, y: int, _flags: int, userdata: ClickState) -> None:
    if event == cv2.EVENT_LBUTTONDOWN:
        userdata.update(x, y)


def _draw_click_info(color: np.ndarray, depth_mm: np.ndarray, click: ClickState, intrinsics) -> None:
    if click.u is None or click.v is None:
        return
    if not (0 <= click.u < color.shape[1] and 0 <= click.v < color.shape[0]):
        return

    depth_for_click = depth_mm
    u = click.u
    v = click.v
    if depth_for_click.shape[:2] != color.shape[:2]:
        scale_x = depth_for_click.shape[1] / color.shape[1]
        scale_y = depth_for_click.shape[0] / color.shape[0]
        depth_u = int(round(u * scale_x))
        depth_v = int(round(v * scale_y))
    else:
        depth_u = u
        depth_v = v

    z_mm = median_depth_at(depth_for_click, depth_u, depth_v)
    cv2.circle(color, (u, v), 6, (0, 255, 255), 2)
    if z_mm is None:
        text = f"({u},{v}) depth: invalid"
    else:
        text = f"({u},{v}) depth: {z_mm:.1f} mm"
        if intrinsics is not None:
            used_intrinsics = intrinsics
            calc_u, calc_v = (u, v)
            if depth_for_click.shape[:2] != color.shape[:2]:
                calc_u, calc_v = (depth_u, depth_v)
            x_mm, y_mm, z_mm = pixel_to_camera_point_mm(calc_u, calc_v, z_mm, used_intrinsics)
            text += f" | camera: x={x_mm:.1f}, y={y_mm:.1f}, z={z_mm:.1f} mm"
    cv2.putText(color, text, (20, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)


def _save_sample(color: np.ndarray, depth_vis: np.ndarray, depth_mm: np.ndarray, output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    stamp = strftime("%Y%m%d_%H%M%S")
    cv2.imwrite(str(output_dir / f"{stamp}_color.png"), color)
    cv2.imwrite(str(output_dir / f"{stamp}_depth_preview.png"), depth_vis)
    np.save(str(output_dir / f"{stamp}_depth_mm.npy"), depth_mm)
    print(f"已保存样本到：{output_dir}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Gemini Pro Plus Windows RGB-D stream test")
    parser.add_argument("--save-dir", default="project/data/rgbd_samples", help="按 s 保存样本的目录")
    parser.add_argument("--prefer-default-color", action="store_true", help="不强制选择 RGB 彩色流，使用 SDK 默认彩色流")
    args = parser.parse_args()

    sdk = import_orbbec_sdk()
    pipeline = sdk.Pipeline()
    stream_config = choose_rgbd_config(pipeline, prefer_rgb=not args.prefer_default_color)
    click = ClickState()
    output_dir = Path(args.save_dir)

    try:
        pipeline.start(stream_config.config)
    except Exception as exc:
        print(f"启动相机失败：{exc}")
        print("请确认 Gemini Pro Plus 已连接、Orbbec Viewer 已关闭、SDK 已正确安装。")
        return

    intrinsics = get_camera_intrinsics(pipeline, stream_config)
    if intrinsics:
        print("相机内参：")
        for name, value in intrinsics.items():
            print(f"  {name}: {value}")
    else:
        print("未能读取内参；仍继续显示 RGB-D 数据。")

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(WINDOW_NAME, _on_mouse, click)
    print("运行中：左键点击彩色图读取深度，按 s 保存样本，按 q 或 ESC 退出。")

    last_color = None
    last_depth_vis = None
    last_depth_mm = None

    try:
        while True:
            frames = pipeline.wait_for_frames(1000)
            if frames is None:
                continue

            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if color_frame is None or depth_frame is None:
                continue

            color_image = frame_to_bgr_image(color_frame)
            if color_image is None:
                print(f"不支持的彩色格式：{color_frame.get_format()}，可尝试 --prefer-default-color")
                continue

            depth_mm = depth_frame_to_mm(depth_frame)
            depth_vis = render_depth(depth_mm)

            # Prefer color intrinsics when depth and color have matching display size; otherwise use depth intrinsics.
            active_intrinsics = None
            if intrinsics:
                active_intrinsics = intrinsics.get("color") if depth_mm.shape[:2] == color_image.shape[:2] else intrinsics.get("depth")

            display_color = color_image.copy()
            _draw_click_info(display_color, depth_mm, click, active_intrinsics)
            if depth_vis.shape[:2] != display_color.shape[:2]:
                depth_vis_show = cv2.resize(depth_vis, (display_color.shape[1], display_color.shape[0]), interpolation=cv2.INTER_NEAREST)
            else:
                depth_vis_show = depth_vis

            combined = np.hstack((display_color, depth_vis_show))
            cv2.imshow(WINDOW_NAME, combined)

            last_color = color_image
            last_depth_vis = depth_vis
            last_depth_mm = depth_mm

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q"), ESC_KEY):
                break
            if key in (ord("s"), ord("S")) and last_color is not None and last_depth_vis is not None and last_depth_mm is not None:
                _save_sample(last_color, last_depth_vis, last_depth_mm, output_dir)
    finally:
        cv2.destroyAllWindows()
        try:
            pipeline.stop()
        except Exception:
            pass


if __name__ == "__main__":
    main()
