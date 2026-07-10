"""Manual pixel closed-loop V2 profile.

The interaction workflow, pixel-scale model, dynamic tolerance bands, staged
descent, final grasp, and Home behavior are intentionally reused from the V1
manual pixel test.  Only the grasp-point default and Cartesian motion runtime
change in V2.
"""

from __future__ import annotations

import argparse


DEFAULT_MANUAL_GRASP_X = 320.0
DEFAULT_MANUAL_GRASP_Y = 147.0
CAMERA_VECTOR_VERSION = "v2"


async def run_manual_pixel_test_v2(args: argparse.Namespace) -> int:
    """Run the shared manual workflow with the V2 camera-vector runtime."""

    # Import lazily so cli.py can expose this profile as a mutually exclusive
    # mode without introducing a module-import cycle.
    from .cli import _run_manual_pixel_test

    return await _run_manual_pixel_test(
        args,
        default_grasp_x=DEFAULT_MANUAL_GRASP_X,
        default_grasp_y=DEFAULT_MANUAL_GRASP_Y,
        camera_vector_version=CAMERA_VECTOR_VERSION,
        display_name="手动像素闭环测试V2",
    )
