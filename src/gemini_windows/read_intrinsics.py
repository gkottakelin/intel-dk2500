"""Read Gemini Pro Plus camera intrinsics through pyorbbecsdk."""

from __future__ import annotations

import json

from gemini_common import choose_rgbd_config, get_camera_intrinsics, import_orbbec_sdk


def main() -> None:
    sdk = import_orbbec_sdk()
    pipeline = sdk.Pipeline()
    stream_config = choose_rgbd_config(pipeline)

    try:
        pipeline.start(stream_config.config)
        intrinsics = get_camera_intrinsics(pipeline, stream_config)
        if not intrinsics:
            print("未能读取相机内参。请确认 SDK 版本，并先运行 camera_stream_test.py 验证出流。")
            return
        payload = {name: value.__dict__ for name, value in intrinsics.items()}
        print(json.dumps(payload, ensure_ascii=False, indent=2))
    finally:
        try:
            pipeline.stop()
        except Exception:
            pass


if __name__ == "__main__":
    main()
