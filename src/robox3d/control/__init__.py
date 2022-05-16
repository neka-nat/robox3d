"""Layer 3: control utilities.

box3d's joint springs are specified as "hertz / dampingRatio normalized by the
effective inertia" (τ = -k·Δθ - c·θ̇, k = I_eff·(2πf)², c = 2ζ·√(k·I_eff)).
Converting from the kp [N·m/rad] / kd [N·m·s/rad] gains standard in robotics
requires the joint's effective inertia I_eff.

Important (validation-report experiments 2 and 3): I_eff is determined by the
"two adjacent bodies" only, and does not include downstream chain inertia. When
specifying kp on a multi-link chain, convert using the value returned by
joint_effective_inertia().
"""

from __future__ import annotations

import numpy as np

from .._ffi import lib
from ..core.joints import PrismaticJoint, _JointBase
from ..math import quat_to_matrix

# large speed used to saturate the motor in pseudo torque control (rad/s, m/s)
TORQUE_CONTROL_SPEED = 1.0e5

# Effective DC stiffness of box3d's soft-constraint spring: k_eff = I_eff·(2πf)²·SPRING_DC_GAIN.
# Calibrated empirically (constant, independent of f, ζ, and substep count; regression-tested in tests/test_control.py).
# If box3d's solver implementation changes, the test will fail, so recalibrate.
SPRING_DC_GAIN = 1.0 / 4.180


def _body_inertia_about_axis(body_id, anchor: np.ndarray, axis: np.ndarray) -> float:
    """Body's rotational inertia about the axis through the anchor point (world frame)."""
    mass = lib.b3Body_GetMass(body_id)
    if mass == 0.0:
        return np.inf  # static body

    q = lib.b3Body_GetRotation(body_id)
    rot = quat_to_matrix(np.array([q.v.x, q.v.y, q.v.z, q.s]))
    i_local = lib.b3Body_GetLocalRotationalInertia(body_id)
    i_mat = np.array(
        [
            [i_local.cx.x, i_local.cy.x, i_local.cz.x],
            [i_local.cx.y, i_local.cy.y, i_local.cz.y],
            [i_local.cx.z, i_local.cy.z, i_local.cz.z],
        ]
    )
    i_world = rot @ i_mat @ rot.T

    com = lib.b3Body_GetWorldCenterOfMass(body_id)
    r = np.array([com.x, com.y, com.z]) - anchor
    r_perp = r - axis * float(r @ axis)  # perpendicular distance component from the axis
    return float(axis @ i_world @ axis) + mass * float(r_perp @ r_perp)


def joint_effective_inertia(joint: _JointBase) -> float:
    """Return the effective inertia (revolute: kg·m²) / effective mass (prismatic:
    kg) that box3d's joint spring is normalized against.

    I_eff = 1 / (1/I_A + 1/I_B). Static bodies are treated as infinite.
    """
    if isinstance(joint, PrismaticJoint):
        inv = 0.0
        for body_id in (joint.body_a, joint.body_b):
            m = lib.b3Body_GetMass(body_id)
            inv += 1.0 / m if m > 0 else 0.0
        if inv == 0.0:
            raise ValueError("Both bodies are static")
        return 1.0 / inv

    anchor = joint.world_anchor
    axis = joint.world_axis
    inv = 0.0
    for body_id in (joint.body_a, joint.body_b):
        i_axis = _body_inertia_about_axis(body_id, anchor, axis)
        inv += 1.0 / i_axis if np.isfinite(i_axis) and i_axis > 0 else 0.0
    if inv == 0.0:
        raise ValueError("Both bodies are static")
    return 1.0 / inv


def pd_to_spring(kp: float, kd: float | None, inertia: float) -> tuple[float, float]:
    """Convert kp [N·m/rad] / kd [N·m·s/rad] to (hertz, damping_ratio).

    kp is exactly calibrated as the steady-state stiffness (τ = kp·Δθ) via
    SPRING_DC_GAIN. kd=None gives critical damping (damping_ratio=1). When kd is
    given, the damping is approximate (a soft constraint's damping is frequency
    dependent).
    """
    if kp <= 0:
        raise ValueError("kp must be positive")
    k_native = kp / SPRING_DC_GAIN  # box3d's internal nominal stiffness I·ω²
    hertz = np.sqrt(k_native / inertia) / (2 * np.pi)
    if kd is None:
        return float(hertz), 1.0
    damping_ratio = kd / (2 * np.sqrt(kp * inertia))
    return float(hertz), float(damping_ratio)


def spring_to_pd(hertz: float, damping_ratio: float, inertia: float) -> tuple[float, float]:
    """Convert (hertz, damping_ratio) to (kp, kd) (the inverse of pd_to_spring)."""
    omega = 2 * np.pi * hertz
    kp = inertia * omega**2 * SPRING_DC_GAIN
    kd = 2 * damping_ratio * np.sqrt(kp * inertia)
    return float(kp), float(kd)


__all__ = [
    "joint_effective_inertia",
    "pd_to_spring",
    "spring_to_pd",
    "TORQUE_CONTROL_SPEED",
    "SPRING_DC_GAIN",
]
