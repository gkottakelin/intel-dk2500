"""Unit tests for src/jetarm_agent/red_block_detector.py."""

from __future__ import annotations

import unittest

import numpy as np

try:
    from project.src.jetarm_agent.red_block_detector import (
        RedBlockResult,
        detect_red_block_from_bgr,
        detect_red_block_mask,
        detect_red_block_from_camera,
    )
except ModuleNotFoundError:
    from src.jetarm_agent.red_block_detector import (
        RedBlockResult,
        detect_red_block_from_bgr,
        detect_red_block_mask,
        detect_red_block_from_camera,
    )


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_bgr_solid(width: int, height: int, bgr: tuple[int, int, int]) -> np.ndarray:
    """Create a solid-colour BGR image."""
    img = np.zeros((height, width, 3), dtype=np.uint8)
    img[:, :] = bgr
    return img


def _draw_red_rect(
    img: np.ndarray, x: int, y: int, w: int, h: int
) -> np.ndarray:
    """Draw a pure-red (BGR=(0,0,255)) filled rectangle on a BGR image."""
    img = img.copy()
    img[y : y + h, x : x + w] = (0, 0, 255)
    return img


# ---------------------------------------------------------------------------
# Test cases
# ---------------------------------------------------------------------------


class RedBlockDetectionFromBgrTest(unittest.TestCase):
    """Tests for the pure-vision detect_red_block_from_bgr function."""

    def test_single_red_block_returns_center(self):
        """A single red rectangle should be detected at its geometric centre."""
        img = _make_bgr_solid(640, 480, (0, 0, 0))
        img = _draw_red_rect(img, x=200, y=150, w=100, h=80)

        result = detect_red_block_from_bgr(img)

        self.assertIsNotNone(result)
        center_x, center_y = result.center
        # Centre of the 100×80 rectangle = (200 + 50, 150 + 40)
        self.assertAlmostEqual(center_x, 250, delta=3)
        self.assertAlmostEqual(center_y, 190, delta=3)
        self.assertGreater(result.area, 100)

    def test_no_red_pixels_returns_none(self):
        """An image with no red should produce no detection."""
        img = _make_bgr_solid(640, 480, (128, 128, 128))
        result = detect_red_block_from_bgr(img)
        self.assertIsNone(result)

    def test_mostly_blue_image_returns_none(self):
        """Blue pixels should not trigger the red detector."""
        img = _make_bgr_solid(320, 240, (255, 0, 0))  # BGR = pure blue
        result = detect_red_block_from_bgr(img)
        self.assertIsNone(result)

    def test_largest_among_multiple_red_regions(self):
        """When there are several red regions, pick the one with the largest area."""
        img = _make_bgr_solid(640, 480, (0, 0, 0))
        img = _draw_red_rect(img, x=50, y=50, w=30, h=30)  # small
        img = _draw_red_rect(img, x=300, y=200, w=120, h=100)  # largest
        img = _draw_red_rect(img, x=500, y=100, w=40, h=50)  # medium

        result = detect_red_block_from_bgr(img)

        self.assertIsNotNone(result)
        # The largest is the 120x100 at (300, 200), centre ~(360, 250)
        self.assertAlmostEqual(result.center[0], 360, delta=3)
        self.assertAlmostEqual(result.center[1], 250, delta=3)
        self.assertGreater(result.area, 5000)

    def test_min_area_filters_noise(self):
        """Small red specks below min_area are ignored."""
        img = _make_bgr_solid(640, 480, (0, 0, 0))
        img = _draw_red_rect(img, x=100, y=100, w=5, h=5)  # 25 px²

        result = detect_red_block_from_bgr(img, min_area=500)
        self.assertIsNone(result)

    def test_min_area_zero_accepts_small_region(self):
        """With min_area=0 the small red region is detected."""
        img = _make_bgr_solid(640, 480, (0, 0, 0))
        img = _draw_red_rect(img, x=100, y=100, w=5, h=5)

        result = detect_red_block_from_bgr(img, min_area=0)
        self.assertIsNotNone(result)

    def test_result_is_red_block_result_dataclass(self):
        """The return value should be a frozen RedBlockResult."""
        img = _make_bgr_solid(640, 480, (0, 0, 0))
        img = _draw_red_rect(img, x=200, y=150, w=100, h=80)

        result = detect_red_block_from_bgr(img)

        self.assertIsInstance(result, RedBlockResult)
        self.assertIsInstance(result.center, tuple)
        self.assertIsInstance(result.bbox, tuple)
        self.assertIsInstance(result.area, float)
        self.assertEqual(len(result.center), 2)
        self.assertEqual(len(result.bbox), 4)
        # Frozen dataclass — assignment should raise
        with self.assertRaises(Exception):
            result.center = (0, 0)  # type: ignore[misc]

    def test_result_bbox_encloses_red_region(self):
        """The bounding box should fully contain the red rectangle."""
        img = _make_bgr_solid(640, 480, (0, 0, 0))
        img = _draw_red_rect(img, x=200, y=150, w=100, h=80)

        result = detect_red_block_from_bgr(img)

        self.assertIsNotNone(result)
        bx, by, bw, bh = result.bbox
        self.assertLessEqual(bx, 200)
        self.assertLessEqual(by, 150)
        self.assertGreaterEqual(bx + bw, 300)
        self.assertGreaterEqual(by + bh, 230)


class RedBlockDetectionMaskTest(unittest.TestCase):
    """Tests for detect_red_block_mask."""

    def test_returns_mask_array(self):
        img = _make_bgr_solid(640, 480, (0, 0, 0))
        img = _draw_red_rect(img, x=200, y=150, w=100, h=80)

        result, mask = detect_red_block_mask(img)

        self.assertIsNotNone(result)
        self.assertIsInstance(mask, np.ndarray)
        self.assertEqual(mask.shape[:2], (480, 640))
        self.assertEqual(mask.dtype, np.uint8)

    def test_none_result_still_returns_mask(self):
        img = _make_bgr_solid(640, 480, (128, 128, 128))

        result, mask = detect_red_block_mask(img)

        self.assertIsNone(result)
        self.assertIsInstance(mask, np.ndarray)
        self.assertEqual(mask.shape[:2], (480, 640))


class RedBlockDetectionFromCameraTest(unittest.TestCase):
    """Tests for the camera convenience wrapper."""

    def test_decodes_jpeg_and_detects_red(self):
        """Simulate a camera JPEG payload and verify the full pipeline."""
        import cv2

        # Build a synthetic BGR image with a red rectangle, then encode as JPEG
        img = _make_bgr_solid(640, 480, (0, 0, 0))
        img = _draw_red_rect(img, x=200, y=150, w=100, h=80)
        ok, encoded = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
        self.assertTrue(ok)

        jpeg_bytes = encoded.tobytes()

        class FakeRGBJpegFrame:
            data = jpeg_bytes
            width = 640
            height = 480

        def fake_capture(*args, **kwargs):
            return FakeRGBJpegFrame()

        result = detect_red_block_from_camera(
            "fake-device",
            capture_rgb=fake_capture,
        )

        self.assertIsNotNone(result)
        # JPEG lossiness adds some tolerance
        self.assertAlmostEqual(result.center[0], 250, delta=8)
        self.assertAlmostEqual(result.center[1], 190, delta=8)

    def test_camera_returns_none_when_no_red(self):
        """Full pipeline returns None when the scene has no red."""
        import cv2

        img = _make_bgr_solid(640, 480, (128, 128, 128))
        ok, encoded = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
        self.assertTrue(ok)

        class FakeRGBJpegFrame:
            data = encoded.tobytes()
            width = 640
            height = 480

        result = detect_red_block_from_camera(
            "fake-device",
            capture_rgb=lambda *a, **kw: FakeRGBJpegFrame(),
        )

        self.assertIsNone(result)


if __name__ == "__main__":
    unittest.main()
