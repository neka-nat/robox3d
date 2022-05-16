"""Energy-conservation regression test (a reduced version of validation-report.md experiment 1).

Monitors whether the solver's dissipation characteristics degrade when box3d is updated.
"""

import numpy as np
import pytest

from robox3d._ffi import lib
from robox3d.math import quat_from_axis_angle, quat_to_matrix


def total_energy(body, g=9.81):
    m = body.mass
    com = body.center_of_mass
    v = body.linear_velocity  # origin velocity, but accurate enough for validating a simple pendulum
    w = body.angular_velocity
    i_local = lib.b3Body_GetLocalRotationalInertia(body.id)
    i_mat = np.array(
        [
            [i_local.cx.x, i_local.cy.x, i_local.cz.x],
            [i_local.cx.y, i_local.cy.y, i_local.cz.y],
            [i_local.cx.z, i_local.cy.z, i_local.cz.z],
        ]
    )
    rot = quat_to_matrix(body.rotation)
    i_world = rot @ i_mat @ rot.T
    ke = 0.5 * m * float(v @ v) + 0.5 * float(w @ i_world @ w)
    return ke + m * g * com[2]


def test_pendulum_energy_drift(world):
    anchor = world.create_body(kind="static")
    rot = quat_from_axis_angle([0, 1, 0], -np.pi / 2)
    link = world.create_body(position=(0, 0, 0), rotation=rot)
    link.add_capsule((0, 0, -0.05), (0, 0, -0.95), radius=0.05, density=340.0)
    world.create_revolute_joint(anchor, link, anchor=(0, 0, 0), axis=(0, 1, 0))

    e_char = link.mass * 9.81 * 0.5  # PE difference from horizontal to the lowest point
    e0 = total_energy(link)
    for _ in range(5 * 240):  # 5 s
        world.step(1 / 240)
    drift = abs(total_energy(link) - e0) / e_char
    # experiment 1: 0.33%/10s at 240Hz/substep4. With regression margin, cap at 2%/5s
    assert drift < 0.02
