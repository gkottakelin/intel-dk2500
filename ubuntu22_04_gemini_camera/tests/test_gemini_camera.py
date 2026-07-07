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
    map_display_to_depth,
    median_depth_at,
    pixel_to_camera_point_mm,
    select_camera_device,
)
from orbbec_native import CameraDeviceInfo, Intrinsics, NativeFrame, depth_frame_to_mm  # noqa: E402


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

    def test_pixel_to_camera_coordinates(self):
        intrinsics = Intrinsics(640, 400, 500.0, 500.0, 320.0, 200.0)
        self.assertEqual(pixel_to_camera_point_mm(320, 200, 1000.0, intrinsics), (0.0, 0.0, 1000.0))


if __name__ == "__main__":
    unittest.main()
