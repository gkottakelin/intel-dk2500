"""OpenCV UVC color stream test for Gemini Pro Plus on Windows.

This is useful because Gemini Pro Plus exposes its color camera as a UVC
"USB Camera" device on Windows, while depth is handled by OpenNI.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from time import strftime


WINDOW_NAME = "Gemini Pro Plus UVC Color"


def open_camera(index: int, width: int, height: int, fps: int):
    import cv2

    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    return cap


def probe_indices(max_index: int, width: int, height: int, fps: int) -> list[int]:
    available: list[int] = []
    for index in range(max_index + 1):
        cap = open_camera(index, width, height, fps)
        ok, frame = cap.read()
        cap.release()
        if ok and frame is not None:
            available.append(index)
            print(f"[OK] camera index {index}: {frame.shape[1]}x{frame.shape[0]}")
        else:
            print(f"[--] camera index {index}: unavailable")
    return available


def main() -> None:
    parser = argparse.ArgumentParser(description="OpenCV UVC color preview for Gemini Pro Plus")
    parser.add_argument("--index", type=int, default=None, help="OpenCV camera index. If omitted, probe and use the first available index.")
    parser.add_argument("--max-index", type=int, default=6, help="Maximum camera index to probe when --index is omitted.")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--save-dir", default="project/data/rgbd_samples", help="Directory for saved color images.")
    args = parser.parse_args()

    import cv2

    if args.index is None:
        print("未指定 --index，开始探测 OpenCV 摄像头索引...")
        available = probe_indices(args.max_index, args.width, args.height, args.fps)
        if not available:
            print("未找到可用 UVC 摄像头。请确认 Orbbec Viewer 已关闭，并检查 Windows 相机权限。")
            return
        index = available[0]
        print(f"使用第一个可用摄像头 index={index}。若不是 Gemini 彩色画面，请用 --index 指定。")
    else:
        index = args.index

    cap = open_camera(index, args.width, args.height, args.fps)
    if not cap.isOpened():
        print(f"无法打开摄像头 index={index}")
        return

    save_dir = Path(args.save_dir)
    save_dir.mkdir(parents=True, exist_ok=True)

    print("运行中：按 s 保存彩色图，按 q 或 ESC 退出。")
    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            print("读取彩色帧失败")
            break

        cv2.imshow(WINDOW_NAME, frame)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), ord("Q"), 27):
            break
        if key in (ord("s"), ord("S")):
            path = save_dir / f"{strftime('%Y%m%d_%H%M%S')}_uvc_color.png"
            cv2.imwrite(str(path), frame)
            print(f"已保存：{path}")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
