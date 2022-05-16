"""LiDAR: a range sensor via batched raycasting."""

from __future__ import annotations

import numpy as np

from .._ffi import ffi, shim
from ..core.body import Body
from ..core.world import World
from ..math import quat_rotate, quat_to_matrix


class Lidar:
    """2D scanning LiDAR. Emits rays at equal angular spacing in the sensor frame's
    XY plane.

    If body is given, the sensor follows that body (offset is body-local). The scan
    hits every shape in the world (including the robot itself), so mount it outward
    via offset to avoid self-occlusion.
    """

    def __init__(
        self,
        world: World,
        body: Body | None = None,
        offset=(0.0, 0.0, 0.0),
        num_rays: int = 360,
        fov: float = 2 * np.pi,
        max_range: float = 10.0,
        no_hit_value: float = np.inf,
    ):
        if shim is None:
            raise RuntimeError("Lidar requires librobox3d_shim")
        self.world = world
        self.body = body
        self.offset = np.asarray(offset, dtype=float)
        self.num_rays = num_rays
        self.max_range = max_range
        self.no_hit_value = no_hit_value

        endpoint = fov < 2 * np.pi
        self.angles = np.linspace(
            -fov / 2, fov / 2, num_rays, endpoint=endpoint, dtype=np.float64
        )
        # ray directions in the sensor frame (XY plane, +X is forward)
        self._dirs_local = np.column_stack(
            [np.cos(self.angles), np.sin(self.angles), np.zeros(num_rays)]
        )
        self._origins = np.empty((num_rays, 3), dtype=np.float32)
        self._dirs = np.empty((num_rays, 3), dtype=np.float32)
        self._fractions = np.empty(num_rays, dtype=np.float32)
        self._hits = np.empty(num_rays, dtype=np.uint8)

    def scan(self) -> np.ndarray:
        """Return the range array (num_rays,) [m]. Misses are no_hit_value."""
        if self.body is not None:
            q = self.body.rotation
            origin = self.body.position + quat_rotate(q, self.offset)
            rot = quat_to_matrix(q)
            dirs_world = self._dirs_local @ rot.T
        else:
            origin = self.offset
            dirs_world = self._dirs_local

        self._origins[:] = origin
        self._dirs[:] = dirs_world * self.max_range

        shim.rxCastRaysClosest(
            self.world.id,
            ffi.cast("float*", ffi.from_buffer(self._origins)),
            ffi.cast("float*", ffi.from_buffer(self._dirs)),
            self.num_rays,
            ffi.cast("float*", ffi.from_buffer(self._fractions)),
            ffi.cast("uint8_t*", ffi.from_buffer(self._hits)),
        )
        ranges = self._fractions.astype(np.float64) * self.max_range
        ranges[self._hits == 0] = self.no_hit_value
        return ranges
