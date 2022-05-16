"""Regression tests for the parallel-hinge spring tracking artifact.

box3d's revolute joint shares one soft-constraint stiffness between the pivot
and the axis-alignment (perpendicularity) rows, and the alignment correction
leaks a spurious rotation into the hinge DOF that grows ~linearly with the
constraint hertz. On a chain where a run of parallel hinges is bracketed by
perpendicular hinges and carries a distal payload, the leaks add coherently and
freeze the springs away from their targets. Root cause and measurements:
docs/spring-chain-investigation.md.
"""

import numpy as np
import pytest

import robox3d

Y, X, Z = (0.0, 1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0)


def _chain_error(axes, masses, constraint_hertz: float, length: float = 0.12) -> float:
    """Max |angle - target| after settling a spring-driven serial chain (zero-g)."""
    with robox3d.World(gravity=(0, 0, 0)) as world:
        parent = world.create_body(kind="static")
        joints = []
        for i, (axis, mass) in enumerate(zip(axes, masses)):
            body = world.create_body(position=((i + 0.5) * length, 0, 0))
            body.add_box(half_extents=(length / 2, 0.015, 0.015), density=1.0)
            body.set_mass_data(
                mass,
                (0, 0, 0),
                np.diag(
                    [0.5 * mass * 0.015**2, mass * length**2 / 12, mass * length**2 / 12]
                ),
            )
            joints.append(
                world.create_revolute_joint(
                    parent,
                    body,
                    anchor=(i * length, 0, 0),
                    axis=axis,
                    spring=(240.0, 1.0),
                    target_angle=0.5,
                    constraint_hertz=constraint_hertz,
                )
            )
            parent = body
        for _ in range(6 * 240):
            world.step(1 / 240)
        return float(np.abs(np.array([j.angle for j in joints]) - 0.5).max())


def test_parallel_run_with_payload_reproduces_leak():
    """Perpendicular → parallel run → perpendicular + heavy tip: large frozen error
    at high constraint hertz. Documents the artifact; if a box3d update fixes it,
    this will fail and the guidance in docs/ should be revisited."""
    err = _chain_error([Z, Y, Y, Y, X], [0.1, 0.1, 0.1, 0.1, 1.0], constraint_hertz=240.0)
    assert err > 0.05, f"leak artifact unexpectedly small: {err}"


def test_low_constraint_hertz_restores_tracking():
    """Lowering the constraint hertz to ~20 Hz drives the same chain below 0.01 rad."""
    err = _chain_error([Z, Y, Y, Y, X], [0.1, 0.1, 0.1, 0.1, 1.0], constraint_hertz=20.0)
    assert err < 0.01, f"tracking error too large at constraint_hertz=20: {err}"


def test_pure_parallel_chain_is_benign():
    """Without the perpendicular bracket + payload the chain converges."""
    err = _chain_error([Y, Y, Y], [0.1, 0.1, 0.1], constraint_hertz=240.0)
    assert err < 0.01, f"pure parallel chain should track: {err}"


def test_so101_tracks_with_default_position_control():
    """The bundled SO-ARM101 tracks within ~3° using enable_position_control's
    default constraint_hertz=60 (was 0.39 rad before the fix)."""
    with robox3d.World(gravity=(0, 0, 0)) as world:
        robot = robox3d.load_urdf(world, robox3d.assets.so101())
        robot.enable_position_control(hertz=240.0)
        robot.set_targets(np.full(6, 0.5))
        for _ in range(4 * 240):
            world.step(1 / 240)
        err = np.abs(robot.positions() - 0.5).max()
    assert err < 0.06, f"SO101 tracking error too large: {err}"
