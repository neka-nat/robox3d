"""IMU: synthesize an accelerometer (specific force) + gyroscope from body state."""

from __future__ import annotations

import numpy as np

from .._ffi import ffi, lib
from ..core.body import Body, _vec3
from ..math import quat_conj, quat_rotate


class IMU:
    """An IMU mounted on a body.

    The accelerometer returns specific force: (a_world - g) rotated into the sensor
    frame. At rest it reads the reaction to gravity (0, 0, +9.81) (Z-up).

    Usage: call update(dt) after world.step(dt) every step, then read() to get the
    values. Acceleration is a backward difference of velocity, so it returns zero
    until the first update.
    """

    def __init__(self, body: Body, offset=(0.0, 0.0, 0.0)):
        self.body = body
        self.offset = np.asarray(offset, dtype=float)
        self._prev_velocity: np.ndarray | None = None
        self._accel_world = np.zeros(3)

    def _point_state(self) -> tuple[np.ndarray, np.ndarray]:
        q = self.body.rotation
        point = self.body.position + quat_rotate(q, self.offset)
        v = lib.b3Body_GetWorldPointVelocity(self.body.id, _vec3(point))
        return np.array([v.x, v.y, v.z]), q

    def update(self, dt: float) -> None:
        """Call right after world.step(dt)."""
        v, _ = self._point_state()
        if self._prev_velocity is not None:
            self._accel_world = (v - self._prev_velocity) / dt
        self._prev_velocity = v

    def read(self) -> tuple[np.ndarray, np.ndarray]:
        """(accel [m/s², specific force, sensor frame], gyro [rad/s, sensor frame])"""
        q = self.body.rotation
        qi = quat_conj(q)
        g = self.body._world.gravity
        accel = quat_rotate(qi, self._accel_world - g)
        gyro = quat_rotate(qi, self.body.angular_velocity)
        return accel, gyro
