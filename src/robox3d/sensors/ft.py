"""Force-torque (F/T) sensor: reads a joint's constraint force and torque."""

from __future__ import annotations

import numpy as np

from ..core.joints import _JointBase
from ..math import quat_conj, quat_rotate


class FTSensor:
    """Read the constraint force and torque acting on a joint as an F/T sensor.

    frame:
    - "joint": reading in body B's (the child link's) joint frame (equivalent to
      a real F/T sensor; the sensor's Z axis is the joint axis)
    - "world": reading in the world frame

    Note: contributions from the motor, spring, and limits are not included in the
    constraint force (per b3Joint_GetConstraintForce/Torque). Using a weld joint at
    the sensor mount puts the entire load on the constraint, giving the cleanest reading.
    """

    def __init__(self, joint: _JointBase, frame: str = "joint"):
        if frame not in ("joint", "world"):
            raise ValueError(f"frame must be 'joint' or 'world': {frame}")
        self.joint = joint
        self.frame = frame

    def read(self) -> tuple[np.ndarray, np.ndarray]:
        """Return (force [N], torque [N·m])."""
        f = self.joint.constraint_force
        t = self.joint.constraint_torque
        if self.frame == "world":
            return f, t
        _, q = self.joint._frame_on("b")
        qi = quat_conj(q)
        return quat_rotate(qi, f), quat_rotate(qi, t)
