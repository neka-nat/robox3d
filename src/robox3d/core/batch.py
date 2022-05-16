"""Batch state access (via the C shim).

Reads/writes the state of many bodies and joints at once as numpy arrays. This
layer keeps the number of FFI calls in the hot loop (RL, control) at O(1).
"""

from __future__ import annotations

import numpy as np

from .._ffi import ffi, shim
from .body import Body
from .joints import RevoluteJoint


def _require_shim():
    if shim is None:
        raise RuntimeError(
            "librobox3d_shim not found. Check your CMake build."
        )
    return shim


def _float_ptr(arr: np.ndarray):
    return ffi.cast("float*", ffi.from_buffer(arr))


class BodyGroup:
    """Bulk state access for multiple bodies."""

    def __init__(self, bodies: list[Body]):
        self._shim = _require_shim()
        self._n = len(bodies)
        self._ids = ffi.new("b3BodyId[]", [b.id for b in bodies])

    def __len__(self) -> int:
        return self._n

    def poses(self, out: np.ndarray | None = None) -> np.ndarray:
        """Poses of all bodies (n, 7): x, y, z, qx, qy, qz, qw."""
        if out is None:
            out = np.empty((self._n, 7), dtype=np.float32)
        self._shim.rxBodyGetPoses(self._ids, self._n, _float_ptr(out))
        return out

    def velocities(self, out: np.ndarray | None = None) -> np.ndarray:
        """Velocities of all bodies (n, 6): vx, vy, vz, wx, wy, wz."""
        if out is None:
            out = np.empty((self._n, 6), dtype=np.float32)
        self._shim.rxBodyGetVelocities(self._ids, self._n, _float_ptr(out))
        return out


class RevoluteGroup:
    """Bulk state access and commands for multiple revolute joints."""

    def __init__(self, joints: list[RevoluteJoint]):
        self._shim = _require_shim()
        self._n = len(joints)
        self._ids = ffi.new("b3JointId[]", [j.id for j in joints])

    def __len__(self) -> int:
        return self._n

    def angles(self, out: np.ndarray | None = None) -> np.ndarray:
        if out is None:
            out = np.empty(self._n, dtype=np.float32)
        self._shim.rxRevoluteGetAngles(self._ids, self._n, _float_ptr(out))
        return out

    def speeds(self, out: np.ndarray | None = None) -> np.ndarray:
        if out is None:
            out = np.empty(self._n, dtype=np.float32)
        self._shim.rxRevoluteGetSpeeds(self._ids, self._n, _float_ptr(out))
        return out

    def set_targets(self, targets) -> None:
        """Set spring target angles (rad) for all joints at once."""
        arr = np.ascontiguousarray(targets, dtype=np.float32)
        assert arr.size == self._n
        self._shim.rxRevoluteSetTargets(self._ids, self._n, _float_ptr(arr))

    def set_motor_speeds(self, speeds) -> None:
        arr = np.ascontiguousarray(speeds, dtype=np.float32)
        assert arr.size == self._n
        self._shim.rxRevoluteSetMotorSpeeds(self._ids, self._n, _float_ptr(arr))

    def set_max_motor_torques(self, torques) -> None:
        arr = np.ascontiguousarray(torques, dtype=np.float32)
        assert arr.size == self._n
        self._shim.rxRevoluteSetMaxMotorTorques(self._ids, self._n, _float_ptr(arr))

    def constraint_loads(self, out: np.ndarray | None = None) -> np.ndarray:
        """Constraint forces/torques of all joints (n, 6): fx, fy, fz, tx, ty, tz."""
        if out is None:
            out = np.empty((self._n, 6), dtype=np.float32)
        self._shim.rxJointGetConstraintLoads(self._ids, self._n, _float_ptr(out))
        return out
