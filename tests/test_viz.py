"""Layer 4 (visualization) tests: scene description, protocol, streaming, recording."""

import json
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

import numpy as np
import pytest

import robox3d
from robox3d.viz import (
    PoseRecorder,
    VizServer,
    decode_pose_frame,
    encode_pose_frame,
    load_recording,
    scene_message,
)

ARM_URDF = Path(__file__).parent.parent / "examples" / "assets" / "simple_arm.urdf"


def test_scene_message_shapes(world):
    ground = world.create_body(kind="static", name="ground")
    ground.add_box((5, 5, 0.1), offset=(0, 0, -0.1))
    ball = world.create_body(position=(0, 0, 1), name="ball")
    ball.add_sphere(0.2, center=(0.1, 0, 0))
    ball.color = "#ff0000"
    rod = world.create_body(position=(1, 0, 1), name="rod")
    rod.add_capsule((0, 0, 0), (0, 0, -0.5), radius=0.05)

    scene = json.loads(scene_message(world))
    assert scene["type"] == "scene"
    by_name = {b["name"]: b for b in scene["bodies"]}

    assert by_name["ground"]["kind"] == "static"
    assert by_name["ground"]["shapes"][0]["kind"] == "hull"
    assert len(by_name["ground"]["shapes"][0]["points"]) == 8  # box convex hull

    ball_shape = by_name["ball"]["shapes"][0]
    assert ball_shape["kind"] == "sphere"
    assert ball_shape["radius"] == pytest.approx(0.2)
    assert ball_shape["center"] == pytest.approx([0.1, 0, 0])
    assert by_name["ball"]["color"] == "#ff0000"

    rod_shape = by_name["rod"]["shapes"][0]
    assert rod_shape["kind"] == "capsule"
    assert rod_shape["p2"] == pytest.approx([0, 0, -0.5])


def test_scene_message_urdf_arm(world):
    robot = robox3d.load_urdf(world, ARM_URDF)
    scene = json.loads(scene_message(world))
    names = [b["name"] for b in scene["bodies"]]
    assert "simple_arm/base_link" in names
    assert "simple_arm/wrist_3_link" in names
    # The merged flange box shows up as a shape on the wrist_3 body (cylinder hull + box hull)
    wrist3 = next(b for b in scene["bodies"] if b["name"] == "simple_arm/wrist_3_link")
    assert len(wrist3["shapes"]) == 2


def test_pose_frame_roundtrip(world):
    body = world.create_body(position=(1, 2, 3))
    body.add_sphere(0.1)
    group = robox3d.BodyGroup(world.bodies)
    frame = encode_pose_frame(1.25, group.poses())
    t, poses = decode_pose_frame(frame)
    assert t == 1.25
    assert poses.shape == (1, 7)
    np.testing.assert_allclose(poses[0, :3], [1, 2, 3], atol=1e-6)
    np.testing.assert_allclose(poses[0, 3:], [0, 0, 0, 1], atol=1e-6)


def test_viz_server_streams(world):
    """A client connection receives the scene JSON followed by pose frames."""
    from websockets.sync.client import connect

    body = world.create_body(position=(0, 0, 5), name="ball")
    body.add_sphere(0.1)
    server = VizServer(world, port=18765, fps=120.0)
    server.start()
    try:
        stop = threading.Event()

        def sim_loop():
            while not stop.is_set():
                world.step(1 / 240)
                server.update()
                time.sleep(0.002)

        t = threading.Thread(target=sim_loop, daemon=True)
        t.start()
        with connect("ws://127.0.0.1:18765", open_timeout=5) as ws:
            scene = json.loads(ws.recv(timeout=5))
            assert scene["type"] == "scene"
            assert scene["bodies"][0]["name"] == "ball"
            frame1 = ws.recv(timeout=5)
            assert isinstance(frame1, bytes)
            t1, poses1 = decode_pose_frame(frame1)
            frame2 = ws.recv(timeout=5)
            t2, poses2 = decode_pose_frame(frame2)
            assert t2 > t1
            assert poses2[0, 2] < poses1[0, 2]  # falling
        stop.set()
        t.join(timeout=2)
    finally:
        server.stop()


def test_viz_server_serves_viewer_http(world, tmp_path, monkeypatch):
    """Non-WebSocket GETs on the same port serve the bundled viewer (or a fallback page)."""
    from robox3d.viz import server as server_module

    body = world.create_body(position=(0, 0, 1), name="ball")
    body.add_sphere(0.1)
    server = VizServer(world, port=18767)

    # Fake bundled viewer build
    static = tmp_path / "static"
    (static / "assets").mkdir(parents=True)
    (static / "index.html").write_text("<html>fake viewer</html>")
    (static / "assets" / "app.js").write_text("console.log('hi')")
    monkeypatch.setattr(server_module, "STATIC_DIR", static)

    server.start()
    try:
        with urllib.request.urlopen("http://127.0.0.1:18767/", timeout=5) as resp:
            assert resp.status == 200
            assert resp.headers["Content-Type"].startswith("text/html")
            assert b"fake viewer" in resp.read()
        with urllib.request.urlopen("http://127.0.0.1:18767/assets/app.js", timeout=5) as resp:
            assert resp.status == 200
            assert resp.headers["Content-Type"] == "text/javascript"
        with pytest.raises(urllib.error.HTTPError) as exc_info:
            urllib.request.urlopen("http://127.0.0.1:18767/nope.js", timeout=5)
        assert exc_info.value.code == 404

        # Without a viewer build, the root path serves a self-explanatory fallback page
        monkeypatch.setattr(server_module, "STATIC_DIR", tmp_path / "missing")
        with urllib.request.urlopen("http://127.0.0.1:18767/", timeout=5) as resp:
            assert resp.status == 200
            assert b"robox3d" in resp.read()
    finally:
        server.stop()


def test_scene_visuals_and_robot_info(world, tmp_path):
    """Visual extraction (primitives + GLB meshes, merging, colors) and robot description."""
    trimesh = pytest.importorskip("trimesh")
    import base64

    ball = trimesh.creation.icosphere(radius=0.1)
    stl = tmp_path / "ball.stl"
    ball.export(stl)
    urdf = tmp_path / "vis_bot.urdf"
    urdf.write_text("""<?xml version="1.0"?>
<robot name="vis_bot">
  <material name="red"><color rgba="1 0 0 1"/></material>
  <link name="base">
    <inertial><mass value="1"/>
      <inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/></inertial>
    <visual>
      <origin xyz="0 0 0.1"/>
      <geometry><cylinder radius="0.05" length="0.2"/></geometry>
      <material name="red"/>
    </visual>
  </link>
  <joint name="j1" type="revolute">
    <parent link="base"/><child link="l1"/><origin xyz="0 0 0.2"/><axis xyz="0 1 0"/>
    <limit lower="-1.5" upper="1.5" effort="10" velocity="1"/>
  </joint>
  <link name="l1">
    <inertial><mass value="0.5"/>
      <inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/></inertial>
    <visual><geometry><mesh filename="ball.stl"/></geometry></visual>
  </link>
  <joint name="tip_fixed" type="fixed">
    <parent link="l1"/><child link="tip"/><origin xyz="0 0 0.1"/>
  </joint>
  <link name="tip">
    <visual><geometry><box size="0.02 0.02 0.02"/></geometry></visual>
  </link>
</robot>""")

    robot = robox3d.load_urdf(world, urdf)
    scene = json.loads(scene_message(world, robots=[robot]))
    by_name = {b["name"]: b for b in scene["bodies"]}

    base_vis = by_name["vis_bot/base"]["visuals"]
    assert base_vis[0]["kind"] == "cylinder"
    assert base_vis[0]["color"] == "#ff0000"  # named material reference resolved
    assert base_vis[0]["xf"]["p"] == pytest.approx([0, 0, 0.1])

    l1_vis = by_name["vis_bot/l1"]["visuals"]
    # Mesh (GLB) + the merged tip box
    kinds = {v["kind"] for v in l1_vis}
    assert kinds == {"mesh", "box"}
    glb = next(v for v in l1_vis if v["kind"] == "mesh")
    assert base64.b64decode(glb["glb"])[:4] == b"glTF"
    box = next(v for v in l1_vis if v["kind"] == "box")
    assert box["xf"]["p"] == pytest.approx([0, 0, 0.1])  # includes merge offset

    # Robot description (for sliders)
    robot_msg = scene["robots"][0]
    assert robot_msg["name"] == "vis_bot"
    assert robot_msg["joints"][0]["name"] == "j1"
    assert robot_msg["joints"][0]["lower"] == pytest.approx(-1.5)


def test_viz_server_set_target_command(world):
    """set_target commands from the viewer are applied on the sim thread."""
    from websockets.sync.client import connect

    robot = robox3d.load_urdf(world, ARM_URDF)
    robot.enable_position_control(hertz=120.0)
    server = VizServer(world, robot=robot, port=18766)
    server.start()
    try:
        with connect("ws://127.0.0.1:18766", open_timeout=5) as ws:
            scene = json.loads(ws.recv(timeout=5))
            assert scene["robots"][0]["name"] == "simple_arm"
            ws.send(json.dumps({"type": "set_target", "joint": "elbow", "value": 0.8}))
            ws.send(json.dumps({"type": "set_target", "joint": "wrist_1", "value": 99.0}))
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                world.step(1 / 240)
                server.update()  # commands are applied here (sim thread)
                if (
                    robot.joint("elbow").target_angle != 0.0
                    and robot.joint("wrist_1").target_angle != 0.0
                ):
                    break
                time.sleep(0.005)
        assert robot.joint("elbow").target_angle == pytest.approx(0.8)
        # Clamped at the ±2.9 limit
        assert robot.joint("wrist_1").target_angle == pytest.approx(2.9)

        # Step past the snapshot throttle so the target lands in the scene message.
        for _ in range(30):
            world.step(1 / 240)
            server.update()
        # A late-joining client sees the current targets, not the boot values.
        with connect("ws://127.0.0.1:18766", open_timeout=5) as ws:
            scene = json.loads(ws.recv(timeout=5))
            joints = {j["name"]: j for j in scene["robots"][0]["joints"]}
            assert joints["elbow"]["value"] == pytest.approx(0.8)
            assert joints["wrist_1"]["value"] == pytest.approx(2.9)
    finally:
        server.stop()


def test_recorder_roundtrip(world, tmp_path):
    body = world.create_body(position=(0, 0, 5), name="ball")
    body.add_sphere(0.1)
    path = tmp_path / "test.rbx"
    with PoseRecorder(world, path) as rec:
        for _ in range(10):
            world.step(1 / 240)
            rec.update()

    scene_json, frames = load_recording(path)
    assert json.loads(scene_json)["bodies"][0]["name"] == "ball"
    assert len(frames) == 10
    t_first, poses_first = decode_pose_frame(frames[0])
    t_last, poses_last = decode_pose_frame(frames[-1])
    assert t_last > t_first
    assert poses_last[0, 2] < poses_first[0, 2]
