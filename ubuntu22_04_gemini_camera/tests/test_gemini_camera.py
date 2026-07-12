import json
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np


APP_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP_ROOT))

from gemini_camera import (  # noqa: E402
    CameraSettings,
    build_parser,
    map_display_to_depth,
    map_window_click_to_image_pixel,
    median_depth_at,
    pixel_to_camera_point_mm,
    select_camera_device,
)
from orbbec_native import (  # noqa: E402
    CameraDeviceInfo,
    Intrinsics,
    NativeFrame,
    OrbbecSession,
    depth_frame_to_mm,
)


class GeminiCameraTest(unittest.TestCase):
    def setUp(self):
        self.first = CameraDeviceInfo(0, "Gemini", "SN001", "UID001", 0x2BC5, 0x0614)
        self.second = CameraDeviceInfo(1, "Gemini", "SN002", "UID002", 0x2BC5, 0x0614)

    def test_loads_default_config(self):
        settings = CameraSettings.from_file(APP_ROOT / "config" / "camera.json")
        self.assertEqual(settings.frame_timeout_ms, 1000)
        self.assertEqual(settings.click_window, 7)
        self.assertEqual(settings.sdk_library.name, "libOrbbecSDK.so.1.5.7")
        self.assertEqual(settings.sdk_library.parent.name, "x64")

    def test_color_only_cli_flag(self):
        self.assertFalse(build_parser().parse_args([]).color_only)
        self.assertTrue(build_parser().parse_args(["--color-only"]).color_only)

    def test_run_script_enables_color_only_mode(self):
        script = (APP_ROOT / "run.sh").read_text(encoding="utf-8")
        self.assertIn('gemini_camera.py --color-only "$@"', script)

    def test_rejects_even_click_window(self):
        with tempfile.TemporaryDirectory() as directory:
            data = json.loads((APP_ROOT / "config" / "camera.json").read_text(encoding="utf-8"))
            data["depth"]["click_window"] = 4
            path = Path(directory) / "camera.json"
            path.write_text(json.dumps(data), encoding="utf-8")
            with self.assertRaises(ValueError):
                CameraSettings.from_file(path)

    def test_selects_by_serial_or_uid(self):
        devices = [self.first, self.second]
        self.assertEqual(select_camera_device(devices, "SN002"), self.second)
        self.assertEqual(select_camera_device(devices, "UID001"), self.first)

    def test_requires_explicit_choice_for_multiple_devices(self):
        with self.assertRaises(RuntimeError):
            select_camera_device([self.first, self.second], None)

    def test_depth_frame_converts_scale_to_millimeters(self):
        raw = np.array([[1000, 2000], [0, 500]], dtype=np.uint16)
        frame = NativeFrame(2, 2, 11, raw.tobytes(), 0.1)
        converted = depth_frame_to_mm(frame)
        np.testing.assert_allclose(converted, [[100.0, 200.0], [0.0, 50.0]])

    def test_median_depth_ignores_invalid_values(self):
        depth = np.array([[0, 0, 0], [0, 800, 900], [0, 10000, 12000]], dtype=np.float32)
        self.assertEqual(median_depth_at(depth, 1, 1, 3), 900.0)

    def test_maps_color_pixel_to_depth_resolution(self):
        self.assertEqual(map_display_to_depth(320, 240, (640, 480), (640, 400)), (320, 200))

    def test_maps_resized_window_click_to_original_rgb_pixel(self):
        self.assertEqual(
            map_window_click_to_image_pixel(
                640, 480, (1280, 960), (640, 480)
            ),
            (320, 240),
        )
        self.assertEqual(
            map_window_click_to_image_pixel(
                1279, 959, (1280, 960), (640, 480)
            ),
            (639, 479),
        )
        self.assertIsNone(
            map_window_click_to_image_pixel(
                1280, 960, (1280, 960), (640, 480)
            )
        )

    def test_color_only_viewer_registers_pixel_click_callback(self):
        source = (APP_ROOT / "gemini_camera.py").read_text(encoding="utf-8")
        self.assertIn("cv2.setMouseCallback(RGB_WINDOW_NAME, on_mouse)", source)
        self.assertIn("draw_color_pixel_info(display_color, click)", source)

    def test_pixel_to_camera_coordinates(self):
        intrinsics = Intrinsics(640, 400, 500.0, 500.0, 320.0, 200.0)
        self.assertEqual(pixel_to_camera_point_mm(320, 200, 1000.0, intrinsics), (0.0, 0.0, 1000.0))

    def test_color_only_wait_never_requests_depth_frame(self):
        class FakeApi:
            def __init__(self):
                self.calls = []

            def call(self, name, *_args):
                self.calls.append(name)
                return {
                    "ob_pipeline_wait_for_frameset": 11,
                    "ob_frameset_color_frame": 22,
                }[name]

            def safe_delete(self, name, _value):
                self.calls.append(name)

        session = OrbbecSession.__new__(OrbbecSession)
        session.api = FakeApi()
        session.pipeline = 7
        expected = NativeFrame(1, 1, 23, b"\x00\x00\x00")
        session._copy_frame = lambda _frame, is_depth: expected

        frame = session.wait_for_color_frame(1000)

        self.assertEqual(frame, expected)
        self.assertNotIn("ob_frameset_depth_frame", session.api.calls)


if __name__ == "__main__":
    unittest.main()
