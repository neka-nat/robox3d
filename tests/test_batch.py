import numpy as np
import pytest

import robox3d
from conftest import build_chain


def test_body_group_matches_scalar(world):
    bodies, joints = build_chain(world, n_links=4)
    for _ in range(120):
        world.step(1 / 240)

    group = robox3d.BodyGroup(bodies)
    poses = group.poses()
    vels = group.velocities()
    assert poses.shape == (4, 7)
    assert vels.shape == (4, 6)
    for i, b in enumerate(bodies):
        np.testing.assert_allclose(poses[i, :3], b.position, rtol=1e-6)
        np.testing.assert_allclose(poses[i, 3:], b.rotation, rtol=1e-5, atol=1e-6)
        np.testing.assert_allclose(vels[i, :3], b.linear_velocity, rtol=1e-4, atol=1e-6)
        np.testing.assert_allclose(vels[i, 3:], b.angular_velocity, rtol=1e-4, atol=1e-6)


def test_revolute_group_matches_scalar(world):
    bodies, joints = build_chain(world, n_links=4)
    for _ in range(120):
        world.step(1 / 240)

    group = robox3d.RevoluteGroup(joints)
    angles = group.angles()
    speeds = group.speeds()
    loads = group.constraint_loads()
    assert angles.shape == (4,)
    assert loads.shape == (4, 6)
    for i, j in enumerate(joints):
        assert angles[i] == pytest.approx(j.angle, abs=1e-6)
        assert speeds[i] == pytest.approx(j.speed, abs=1e-5)
        np.testing.assert_allclose(loads[i, :3], j.constraint_force, rtol=1e-5, atol=1e-6)


def test_revolute_group_set_targets(world):
    bodies, joints = build_chain(world, n_links=4)
    group = robox3d.RevoluteGroup(joints)
    targets = np.radians([10.0, -10.0, 20.0, -20.0]).astype(np.float32)
    group.set_targets(targets)
    for _ in range(720):  # 3 s
        world.step(1 / 240)
    angles = np.degrees(group.angles())
    np.testing.assert_allclose(angles, [10, -10, 20, -20], atol=1.0)


def test_step_n_equivalent(world):
    bodies, joints = build_chain(world, n_links=3)
    with robox3d.World() as world2:
        bodies2, joints2 = build_chain(world2, n_links=3)
        for _ in range(100):
            world.step(1 / 240)
        world2.step_n(100, 1 / 240)
        np.testing.assert_array_equal(
            robox3d.BodyGroup(bodies).poses(), robox3d.BodyGroup(bodies2).poses()
        )
