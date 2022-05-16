"""Layer 3 (sensors) tests: F/T, IMU, LiDAR, contact.

Milestone: the F/T sensor reading matches statics (quasi-static inverse dynamics).
"""

from pathlib import Path

import numpy as np
import pytest

import robox3d

ARM_URDF = Path(__file__).parent.parent / "examples" / "assets" / "simple_arm.urdf"


# ------------------------------------------------------------------ F/T


@pytest.fixture
def arm_with_payload(world):
    """Attach a 2 kg payload to the arm tip with a weld (= an F/T sensor mount)."""
    robot = robox3d.load_urdf(world, ARM_URDF)
    robot.enable_position_control(hertz=120.0)
    payload = world.create_body(position=(1.05, 0, 0.22))
    payload.add_box((0.05, 0.05, 0.05), density=2000.0)  # 2 kg
    mount = world.create_weld_joint(
        robot.link_body("wrist_3_link"), payload, anchor=(1.0, 0, 0.22)
    )
    for _ in range(720):  # settle
        world.step(1 / 240)
    return robot, payload, mount


def test_ft_static_wrench_matches_statics(arm_with_payload, world):
    """Milestone: the F/T reading matches quasi-static inverse dynamics (statics)."""
    robot, payload, mount = arm_with_payload
    sensor = robox3d.FTSensor(mount, frame="world")
    f, t = sensor.read()
    # force = payload support force
    assert np.linalg.norm(f) == pytest.approx(payload.mass * 9.81, rel=0.05)
    assert f[2] > 0
    # torque = (COM - anchor) × F (the constraint torque is reported as a pure couple)
    t_expected = np.cross(payload.center_of_mass - mount.world_anchor, f)
    np.testing.assert_allclose(t, t_expected, atol=0.05)


def test_ft_joint_frame_preserves_magnitude(arm_with_payload):
    robot, payload, mount = arm_with_payload
    f_w, t_w = robox3d.FTSensor(mount, frame="world").read()
    f_j, t_j = robox3d.FTSensor(mount, frame="joint").read()
    assert np.linalg.norm(f_j) == pytest.approx(np.linalg.norm(f_w), rel=1e-5)
    assert np.linalg.norm(t_j) == pytest.approx(np.linalg.norm(t_w), rel=1e-5)


def test_ft_on_revolute_reads_subtree_weight(world):
    robot = robox3d.load_urdf(world, ARM_URDF)
    robot.enable_position_control(hertz=120.0)
    for _ in range(720):
        world.step(1 / 240)
    f, _ = robox3d.FTSensor(robot.joint("wrist_1"), frame="world").read()
    subtree_weight = (1.2 + 1.2 + 0.25) * 9.81  # mass from wrist_1 downstream
    assert np.linalg.norm(f) == pytest.approx(subtree_weight, rel=0.05)


# ------------------------------------------------------------------ IMU


def test_imu_static_reads_gravity_reaction(world):
    ground = world.create_body(kind="static")
    ground.add_box((5, 5, 0.1), offset=(0, 0, -0.1))
    box = world.create_body(position=(0, 0, 0.25))
    box.add_box((0.25, 0.25, 0.25))
    imu = robox3d.IMU(box)
    for _ in range(240):
        world.step(1 / 240)
        imu.update(1 / 240)
    accel, gyro = imu.read()
    np.testing.assert_allclose(accel, [0, 0, 9.81], atol=0.02)
    np.testing.assert_allclose(gyro, [0, 0, 0], atol=1e-4)


def test_imu_gyro_reads_angular_velocity(world):
    body = world.create_body(position=(0, 0, 5))
    body.add_sphere(0.2)
    body.angular_velocity = (0, 0, 2.0)
    imu = robox3d.IMU(body)
    world.step(1 / 240)
    imu.update(1 / 240)
    _, gyro = imu.read()
    assert gyro[2] == pytest.approx(2.0, rel=1e-3)


def test_imu_free_fall_reads_zero(world):
    body = world.create_body(position=(0, 0, 50))
    body.add_sphere(0.2)
    imu = robox3d.IMU(body)
    for _ in range(24):
        world.step(1 / 240)
        imu.update(1 / 240)
    accel, _ = imu.read()
    # an accelerometer in free fall is weightless (zero specific force)
    np.testing.assert_allclose(accel, [0, 0, 0], atol=0.05)


# ------------------------------------------------------------------ LiDAR


def test_lidar_wall_distance(world):
    wall = world.create_body(kind="static", position=(2.0, 0, 0))
    wall.add_box((0.1, 5, 5))
    lidar = robox3d.Lidar(world, offset=(0, 0, 0), num_rays=9, fov=np.pi / 2, max_range=10)
    ranges = lidar.scan()
    assert ranges[4] == pytest.approx(1.9, abs=0.02)  # front
    assert ranges[0] == pytest.approx(1.9 / np.cos(np.pi / 4), abs=0.03)  # ±45°


def test_lidar_no_hit_is_inf(world):
    lidar = robox3d.Lidar(world, num_rays=4, max_range=5.0)
    assert np.all(np.isinf(lidar.scan()))


def test_lidar_follows_body(world):
    wall = world.create_body(kind="static", position=(2.0, 0, 0))
    wall.add_box((0.1, 5, 5))
    robot_body = world.create_body(position=(1.0, 0, 0), kind="kinematic")
    lidar = robox3d.Lidar(world, body=robot_body, num_rays=1, fov=0.01, max_range=10)
    assert lidar.scan()[0] == pytest.approx(0.9, abs=0.02)


# ------------------------------------------------------------------ Contact


def test_contact_resting_force_equals_weight(world):
    ground = world.create_body(kind="static")
    ground.add_box((5, 5, 0.1), offset=(0, 0, -0.1))
    box = world.create_body(position=(0, 0, 0.3))
    box.add_box((0.25, 0.25, 0.25), density=500.0)
    sensor = robox3d.ContactSensor(box)
    for _ in range(480):
        world.step(1 / 240)
    reading = sensor.read()
    assert reading.touching
    assert reading.normal_force_magnitude == pytest.approx(box.mass * 9.81, rel=0.02)
    assert reading.normal_force[2] > 0  # upward support force


def test_contact_airborne_not_touching(world):
    box = world.create_body(position=(0, 0, 10))
    box.add_box((0.25, 0.25, 0.25))
    world.step(1 / 240)
    reading = robox3d.ContactSensor(box).read()
    assert not reading.touching
    assert reading.normal_force_magnitude == 0.0
