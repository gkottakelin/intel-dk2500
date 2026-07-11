"""JetArm desktop control center.

The control center launches existing workflows, shows configuration, and can
interrupt arm workflows that it launched. It never opens the arm serial port
or camera itself.
"""

from .app import main

__all__ = ["main"]
