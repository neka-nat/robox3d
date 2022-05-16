"""Quaternion and vector utilities.

box3d's inline math functions are not exported from the shared library, so the
ones we need are implemented on the Python side.

Quaternions are represented as numpy arrays [x, y, z, w] (matching the memory
layout of b3Quat = {v(xyz), s(w)}, and compatible with scipy's xyzw order).
"""

from __future__ import annotations

import numpy as np


def quat_identity() -> np.ndarray:
    return np.array([0.0, 0.0, 0.0, 1.0])


def quat_from_axis_angle(axis, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    half = 0.5 * angle
    return np.array([*(axis * np.sin(half)), np.cos(half)])


def quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array(
        [
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        ]
    )


def quat_conj(q: np.ndarray) -> np.ndarray:
    return np.array([-q[0], -q[1], -q[2], q[3]])


def quat_rotate(q: np.ndarray, v) -> np.ndarray:
    """Rotate vector v by quaternion q."""
    qv = np.array([*v, 0.0])
    return quat_mul(quat_mul(q, qv), quat_conj(q))[:3]


def quat_z_to(axis) -> np.ndarray:
    """Minimal-rotation quaternion mapping the Z axis onto the given direction (used to set joint axes)."""
    z = np.array([0.0, 0.0, 1.0])
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    d = float(np.dot(z, axis))
    if d > 1.0 - 1e-12:
        return quat_identity()
    if d < -1.0 + 1e-12:
        return quat_from_axis_angle([1.0, 0.0, 0.0], np.pi)
    c = np.cross(z, axis)
    q = np.array([*c, 1.0 + d])
    return q / np.linalg.norm(q)


def matrix_to_quat(m: np.ndarray) -> np.ndarray:
    """Convert a rotation matrix (3x3) to a quaternion [x, y, z, w]."""
    m = np.asarray(m, dtype=float)
    t = np.trace(m)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        return np.array(
            [(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s]
        )
    i = int(np.argmax(np.diag(m)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = np.sqrt(m[i, i] - m[j, j] - m[k, k] + 1.0) * 2
    q = np.empty(4)
    q[i] = 0.25 * s
    q[j] = (m[j, i] + m[i, j]) / s
    q[k] = (m[k, i] + m[i, k]) / s
    q[3] = (m[k, j] - m[j, k]) / s
    return q


def quat_to_matrix(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )
