"""Layer 3 (control) tests: kp calibration, effective inertia, torque control, gravity compensation."""

from pathlib import Path

import numpy as np
import pytest

import robox3d
from robox3d.control import (
    TORQUE_CONTROL_SPEED,
    joint_effective_inertia,
    pd_to_spring,
)
from robox3d.math import quat_from_axis_angle

ARM_URDF = Path(__file__).parent.parent / "examples" / "assets" / "simple_arm.urdf"


def make_pendulum(world, axis=(0, 1, 0)):
    anchor = world.create_body(kind="static")
    rot = quat_from_axis_angle([0, 1, 0], -np.pi / 2)
    link = world.create_body(position=(0, 0, 0), rotation=rot)
    link.add_capsule((0, 0, -0.05), (0, 0, -0.95), radius=0.05, density=340.0)
    joint = world.create_revolute_joint(anchor, link, anchor=(0, 0, 0), axis=axis)
    return link, joint


def test_effective_inertia_pendulum(world):
    """Against a static body, effective inertia = the link's inertia about the axis (parallel-axis included)."""
    link, joint = make_pendulum(world)
    inertia = joint_effective_inertia(joint)
    # thin-rod approximation: I = mL²/3 (off by a few % since it's a capsule)
    rod = link.mass * 1.0**2 / 3
    assert inertia == pytest.approx(rod, rel=0.10)


def test_kp_dc_stiffness_calibration(world):
    """Regression test for SPRING_DC_GAIN: kp's steady-state stiffness matches measurement.

    If box3d's solver implementation changes and this fails, recalibrate control.SPRING_DC_GAIN.
    """
    link, joint = make_pendulum(world)
    kp = 200.0
    hertz, damping = pd_to_spring(kp, None, joint_effective_inertia(joint))
    joint.enable_spring(hertz, damping)
    joint.target_angle = 0.0
    for _ in range(720):
        world.step(1 / 240)
    tau_g = link.mass * 9.81 * 0.5 * np.cos(joint.angle)
    k_measured = tau_g / abs(joint.angle)
    assert k_measured == pytest.approx(kp, rel=0.03)


def test_torque_control_acceleration(world):
    """Pseudo torque control: a constant torque produces angular acceleration α = τ/I.

    Verified with rotation about the vertical (Z) axis to avoid interference from gravity.
    """
    link, joint = make_pendulum(world, axis=(0, 0, 1))
    inertia = joint_effective_inertia(joint)
    tau = 2.0
    joint.enable_motor(max_torque=tau, speed=TORQUE_CONTROL_SPEED)
    t_span = 0.25
    for _ in range(int(240 * t_span)):
        world.step(1 / 240)
    omega_expected = tau * t_span / inertia
    assert joint.speed == pytest.approx(omega_expected, rel=0.03)


def test_gravity_compensation_holds(world):
    robot = robox3d.load_urdf(world, ARM_URDF)
    robot.enable_torque_control()
    q0 = robot.positions()
    for _ in range(240):  # 1 s
        robot.set_torques(robot.gravity_compensation())
        world.step(1 / 240)
    drift = np.degrees(np.abs(robot.positions() - q0)).max()
    assert drift < 6.0, f"gravity compensation failed to hold: {drift:.1f}°"


def test_no_compensation_collapses(world):
    robot = robox3d.load_urdf(world, ARM_URDF)
    robot.enable_torque_control()
    for _ in range(240):
        robot.set_torques(np.zeros(6))
        world.step(1 / 240)
    assert np.degrees(np.abs(robot.positions())).max() > 30.0


def test_torque_effort_clamp(world):
    """set_torques is clamped by the URDF effort limits."""
    robot = robox3d.load_urdf(world, ARM_URDF)
    robot.enable_torque_control()
    robot.set_torques(np.full(6, 1e4))  # wrist joints' effort is 28
    j = robot.joint("wrist_3")
    assert j.max_motor_torque == pytest.approx(28.0)


def test_ff_plus_fb(world):
    """Combined gravity-compensation FF + weak spring FB holds with high accuracy."""
    robot = robox3d.load_urdf(world, ARM_URDF)
    robot.enable_position_control(kp=50.0)  # weak gain that sags a lot on its own
    robot.enable_torque_control(disable_springs=False)
    for _ in range(480):
        robot.set_torques(robot.gravity_compensation())
        world.step(1 / 240)
    err = np.degrees(np.abs(robot.positions())).max()
    assert err < 0.5, f"FF+FB hold error: {err:.2f}°"


def test_gravity_compensation_extra_payload(world):
    """Including the EE payload via extra_bodies restores hold accuracy."""
    robot = robox3d.load_urdf(world, ARM_URDF)
    payload = world.create_body(position=(1.05, 0, 0.22))
    payload.add_box((0.05, 0.05, 0.05), density=2000.0)  # 2 kg
    world.create_weld_joint(robot.link_body("wrist_3_link"), payload, anchor=(1.0, 0, 0.22))
    # Heavy EE payload: favor pivot rigidity over parallel-hinge tracking
    # (docs/spring-chain-investigation.md) — keep the stiff 240 Hz constraints.
    robot.enable_position_control(kp=100.0, constraint_hertz=240.0)
    robot.enable_torque_control(disable_springs=False)
    for _ in range(720):
        robot.set_torques(robot.gravity_compensation(extra_bodies=[payload]))
        world.step(1 / 240)
    err = np.degrees(np.abs(robot.positions())).max()
    assert err < 0.5, f"hold error with payload compensation: {err:.2f}°"
