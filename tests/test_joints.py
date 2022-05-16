import numpy as np
import pytest

from robox3d.math import quat_from_axis_angle


def make_pendulum(world):
    anchor = world.create_body(kind="static")
    rot = quat_from_axis_angle([0, 1, 0], -np.pi / 2)
    link = world.create_body(position=(0, 0, 0), rotation=rot)
    link.add_capsule((0, 0, -0.05), (0, 0, -0.95), radius=0.05, density=340.0)
    joint = world.create_revolute_joint(anchor, link, anchor=(0, 0, 0), axis=(0, 1, 0))
    return link, joint


def test_pendulum_swings(world):
    link, joint = make_pendulum(world)
    assert joint.angle == pytest.approx(0.0, abs=1e-5)
    for _ in range(120):  # 0.5 s
        world.step(1 / 240)
    # released from horizontal, it falls downward (the angle increases)
    assert abs(joint.angle) > np.radians(30)


def test_spring_position_control(world):
    link, joint = make_pendulum(world)
    joint.enable_spring(hertz=60.0, damping_ratio=1.0)
    joint.target_angle = np.radians(45)
    for _ in range(480):  # 2 s
        world.step(1 / 240)
    assert np.degrees(joint.angle) == pytest.approx(45.0, abs=0.5)
    assert joint.speed == pytest.approx(0.0, abs=1e-2)


def test_motor_velocity_control(world):
    link, joint = make_pendulum(world)
    joint.enable_motor(max_torque=50.0, speed=1.0)
    for _ in range(480):  # 2 s
        world.step(1 / 240)
    # after 2 s: tracking error within 1% (experiment 4B)
    assert joint.speed == pytest.approx(1.0, rel=0.01)


def test_limits(world):
    link, joint = make_pendulum(world)
    joint.enable_limit(lower=np.radians(-20), upper=np.radians(20))
    for _ in range(480):
        world.step(1 / 240)
    assert abs(np.degrees(joint.angle)) <= 20.5


def test_constraint_force_reads_gravity(world):
    link, joint = make_pendulum(world)
    joint.enable_spring(hertz=120.0, damping_ratio=1.0)
    for _ in range(480):
        world.step(1 / 240)
    # once statically balanced, the joint supports the link's weight
    f = joint.constraint_force
    weight = link.mass * 9.81
    assert np.linalg.norm(f) == pytest.approx(weight, rel=0.1)
