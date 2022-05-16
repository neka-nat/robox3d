"""Layer 2 (URDF -> box3d conversion) tests. Milestone: position control of a 6-axis arm."""

from pathlib import Path

import numpy as np
import pytest

import robox3d

ARM_URDF = Path(__file__).parent.parent / "examples" / "assets" / "simple_arm.urdf"


@pytest.fixture
def arm(world):
    return robox3d.load_urdf(world, ARM_URDF)


def test_structure(arm):
    assert arm.dof == 6
    assert arm.actuated == [
        "shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3",
    ]
    # a fixed link without inertia is merged into the parent body
    assert arm.link_body("flange") is arm.link_body("wrist_3_link")
    # joint wrapper types
    assert isinstance(arm.joint("elbow"), robox3d.RevoluteJoint)
    assert "flange_fixed" not in arm.joints  # already merged


def test_inertia_override(arm):
    assert arm.link_body("upper_arm_link").mass == pytest.approx(8.4, rel=1e-5)
    assert arm.link_body("forearm_link").mass == pytest.approx(2.3, rel=1e-5)
    # COM location (+X 0.2125 m from the link origin -> nearly the same in world coordinates)
    com = arm.link_body("upper_arm_link").center_of_mass
    np.testing.assert_allclose(com, [0.2125, 0.0, 0.22], atol=1e-4)


def test_collision_shapes_present(arm, world):
    # cast a ray down onto the upper arm (a cylinder at z=0.22, radius 0.05)
    hit = world.cast_ray_closest(origin=(0.2, 0, 1.0), translation=(0, 0, -1.0))
    assert hit is not None
    point, normal, _ = hit
    assert point[2] == pytest.approx(0.27, abs=0.01)


def test_position_hold(arm, world):
    """Milestone: hold the pose under gravity (max gravity-torque configuration)."""
    arm.enable_position_control(hertz=120.0)
    for _ in range(480):  # 2 s
        world.step(1 / 240)
    err_deg = np.degrees(np.abs(arm.positions()))
    assert err_deg.max() < 1.5, f"hold error too large: {err_deg}"


def test_trajectory_tracking(arm, world):
    """Milestone: track a sinusoidal trajectory on all joints simultaneously."""
    arm.enable_position_control(hertz=120.0)
    amp = np.array([0.5, 0.4, 0.5, 0.6, 0.6, 0.8])
    freq = 0.25  # Hz
    dt = 1 / 240
    max_err = 0.0
    for i in range(4 * 240):  # 4 s (one period)
        t = i * dt
        q_des = amp * np.sin(2 * np.pi * freq * t)
        arm.set_targets(q_des)
        world.step(dt)
        if t > 0.5:  # skip the initial transient
            max_err = max(max_err, np.abs(arm.positions() - q_des).max())
    assert np.degrees(max_err) < 3.0, f"tracking error: {np.degrees(max_err):.2f}°"


def test_joint_limits(arm, world):
    arm.enable_position_control(hertz=120.0)
    targets = np.zeros(6)
    targets[2] = 3.5  # elbow's limit is ±2.9
    arm.set_targets(targets)
    for _ in range(720):
        world.step(1 / 240)
    assert arm.positions()[2] <= 2.95


def test_fixed_base_static(arm):
    assert arm.base_body.mass == 0.0  # static body


def test_limit_beyond_pi_warns(world, tmp_path):
    urdf = tmp_path / "wide_limit.urdf"
    urdf.write_text("""<?xml version="1.0"?>
<robot name="wide">
  <link name="base"><inertial><mass value="1"/>
    <inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/></inertial></link>
  <joint name="j1" type="revolute">
    <parent link="base"/><child link="l1"/><axis xyz="0 0 1"/>
    <limit lower="-6.28" upper="6.28" effort="10" velocity="1"/>
  </joint>
  <link name="l1"><inertial><mass value="1"/>
    <inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/></inertial></link>
</robot>""")
    with pytest.warns(UserWarning, match="0.99"):
        robox3d.load_urdf(world, urdf)


def test_mesh_collision_hull(world, tmp_path):
    """Convex-hull conversion of mesh collision (verified by generating an STL on the fly)."""
    trimesh = pytest.importorskip("trimesh")
    box = trimesh.creation.box(extents=(0.4, 0.4, 0.4))
    stl = tmp_path / "box.stl"
    box.export(stl)
    urdf = tmp_path / "mesh_bot.urdf"
    urdf.write_text(f"""<?xml version="1.0"?>
<robot name="mesh_bot">
  <link name="base">
    <inertial><mass value="2"/>
      <inertia ixx="0.05" iyy="0.05" izz="0.05" ixy="0" ixz="0" iyz="0"/></inertial>
    <collision><geometry><mesh filename="box.stl"/></geometry></collision>
  </link>
</robot>""")

    ground = world.create_body(kind="static")
    ground.add_box((5, 5, 0.1), offset=(0, 0, -0.1))
    robot = robox3d.load_urdf(world, urdf, position=(0, 0, 1.0), fixed_base=False)
    for _ in range(480):
        world.step(1 / 240)
    # convex hull of a 0.4 m box mesh -> rests at half-height, 0.2 m
    assert robot.base_body.position[2] == pytest.approx(0.2, abs=0.01)


def test_elbow_rotation_direction(arm, world):
    """A positive rotation about +Y lowers the tip (right-handed sign consistency)."""
    arm.enable_position_control(hertz=120.0)
    tip0 = arm.link_body("wrist_3_link").position.copy()
    targets = np.zeros(6)
    targets[1] = 0.5  # shoulder_lift +0.5 rad
    arm.set_targets(targets)
    for _ in range(480):
        world.step(1 / 240)
    tip = arm.link_body("wrist_3_link").position
    assert tip[2] < tip0[2] - 0.1  # the tip lowers
