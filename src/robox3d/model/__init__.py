"""Layer 2: robot model layer (URDF -> box3d conversion)."""

from .urdf import Robot, load_urdf

__all__ = ["Robot", "load_urdf"]
