"""JetArm desktop control center.

The control center is intentionally only a launcher and configuration viewer.
It does not own the arm serial port, camera, or any motion workflow.
"""

from .app import main

__all__ = ["main"]
