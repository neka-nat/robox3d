"""Bundled robot models.

Each model ships with its upstream license and attribution in its directory.
Load with :func:`robox3d.load_urdf`::

    import robox3d
    robot = robox3d.load_urdf(world, robox3d.assets.so101())
"""

from __future__ import annotations

from pathlib import Path

_ROOT = Path(__file__).parent


def so101() -> Path:
    """Path to the bundled SO-ARM101 URDF (TheRobotStudio SO-101, Apache-2.0)."""
    return _ROOT / "so101" / "so101_new_calib.urdf"
