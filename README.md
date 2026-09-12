# robox3d

**Lightweight robot simulation in Python, with a viewer that runs in your browser.**

robox3d wraps [Box3D](https://github.com/erincatto/box3d) — the new 3D physics
engine by Erin Catto (author of Box2D) — into a batteries-included robotics
toolkit: URDF loading, position/torque control, F/T·IMU·LiDAR·contact sensors,
and WebSocket pose streaming to a React Three Fiber viewer. Physics runs
headless; the viewer is just a browser tab.

[![CI](https://github.com/neka-nat/robox3d/actions/workflows/ci.yml/badge.svg)](https://github.com/neka-nat/robox3d/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Python](https://img.shields.io/badge/python-3.10%2B-blue)](pyproject.toml)

![SO-ARM101 driven by joint sliders in the browser viewer](docs/media/so101_viewer.gif)

*A bundled SO-ARM101 simulated by robox3d and teleoperated with joint sliders — everything above runs from `pip install` + one command, visualized in a plain browser tab.*

```bash
pip install "robox3d[viz]"
python -m robox3d.demo so101   # SO-ARM101 with joint sliders — opens at http://localhost:8765
```

*日本語のREADMEは [README.ja.md](README.ja.md) にあります。*

## Why robox3d?

- **Browser-native visualization** — no OpenGL window, no X forwarding. The sim
  serves its own web viewer (HTTP + WebSocket on one port). Run headless on a
  server, watch from your laptop. Joint sliders in the viewer drive the robot
  live.
- **Zero-friction install** — a small C17 engine with no dependencies, shipped
  as prebuilt wheels.
- **URDF in, physics out** — links, joints, inertia, collision meshes (convex
  hull or [CoACD](https://github.com/SarahWeiii/CoACD) decomposition), visual
  meshes and colors all handled.
- **Robotics-grade control & sensing** — spring position control with
  physical-unit gains (kp in N·m/rad, DC-calibrated), pseudo torque control,
  gravity-compensation feedforward, and F/T, IMU, LiDAR (batched raycasts),
  and contact sensors.
- **Native stepping & batch access** — the physics loop runs in C, with batch
  APIs for reading body poses and updating joint targets from Python.
- **Record & replay** — pose recordings (`.rbx`) use the same wire format as
  live streaming; replay them into the same viewer.

## Quickstart

```python
import numpy as np
import robox3d

with robox3d.World() as world:   # Z-up, gravity (0, 0, -9.81)
    ground = world.create_body(kind="static")
    ground.add_box(half_extents=(10, 10, 0.1), offset=(0, 0, -0.1))

    link = world.create_body(position=(0, 0, 1))
    link.add_capsule((0, 0, 0), (0, 0, -0.5), radius=0.05)

    joint = world.create_revolute_joint(
        ground, link, anchor=(0, 0, 1), axis=(0, 1, 0),
        spring=(60.0, 1.0),      # spring position control (hertz, damping_ratio)
    )
    joint.target_angle = np.radians(45)

    for _ in range(240):
        world.step(1 / 240)      # substeps=4 (default)
    print(np.degrees(joint.angle))
```

### Load a robot from URDF

```python
robot = robox3d.load_urdf(world, "robot.urdf")   # fixed base, self-collision off
robot.enable_position_control(hertz=120.0)
robot.set_targets(q_des)                          # auto-clamped to URDF limits
q, qd = robot.positions(), robot.velocities()
```

An SO-ARM101 model ([TheRobotStudio SO-101](https://github.com/TheRobotStudio/SO-ARM100),
Apache-2.0) is bundled:

```python
robot = robox3d.load_urdf(world, robox3d.assets.so101())
```

Mesh collision defaults to a single convex hull per link. For finer shapes use
`mesh_mode="coacd"` (`pip install "robox3d[coacd]"`; results are cached in
`~/.cache/robox3d/`).

### Visualize in the browser

```python
from robox3d.viz import VizServer

server = VizServer(world, robot=robot)  # needs: pip install "robox3d[viz]"
server.start()                          # call after all bodies are created
while running:
    world.step(dt)
    server.update()                     # stream poses + apply viewer commands
```

Open `server.url` (default `http://127.0.0.1:8765`) — the same port serves the
viewer over HTTP and streams poses over WebSocket. Passing `robot=` adds joint
sliders to the viewer. URDF `<visual>` elements (colors and meshes included)
are displayed automatically, with a HUD toggle for collision shapes.

Record and replay (same format as the live stream):

```python
from robox3d.viz import PoseRecorder
with PoseRecorder(world, "run.rbx") as rec:
    for _ in range(1000):
        world.step(dt)
        rec.update()
```

```bash
python -m robox3d.viz.record run.rbx   # replays to the viewer
```

### Control and sensors

```python
# Gains in physical units (kp: N·m/rad); steady-state stiffness is calibrated
robot.enable_position_control(kp=200.0)

# Torque control (gravity-compensation feedforward + spring feedback)
robot.enable_torque_control(disable_springs=False)
robot.set_torques(robot.gravity_compensation())   # call every step

# Sensors
ft = robox3d.FTSensor(mount_joint, frame="joint") # constraint force/torque
imu = robox3d.IMU(body)                           # specific force + gyro
lidar = robox3d.Lidar(world, body=base, num_rays=360, max_range=10)
touch = robox3d.ContactSensor(foot_body)          # contact + net normal force
```

For many joints/bodies in an RL or control loop, the batch API
(`RevoluteGroup` / `BodyGroup`) turns per-step FFI traffic into O(1) calls:

```python
group = robox3d.RevoluteGroup(joints)
group.set_targets(q_des)          # np.ndarray (n,)
q, qd = group.angles(), group.speeds()
```

## Examples

| Script | What it shows |
|---|---|
| `python -m robox3d.demo so101` | SO-ARM101 teleop in the browser |
| `examples/viz_arm.py` | 6-DoF arm + falling boxes, sliders or auto trajectory, recording |
| `examples/arm_trajectory.py` | Sinusoidal joint-space tracking, accuracy report |
| `examples/arm_sensors.py` | F/T sensor vs. static analysis, IMU, LiDAR, gravity compensation |
| `examples/pendulum.py` | Free swing and spring position-control step response |
| `examples/falling_box.py` | Hello-world rigid body |

Every example serves the browser viewer by default and keeps the scene moving
until you Ctrl+C — open the printed URL to watch it live (real-time paced).
Use `--headless` for a fast numeric run without the viewer, and `--port` to
run several examples at once.

## Scope and limitations

Box3D is a maximal-coordinate rigid-body engine. See the
[limitations and control tuning guide](docs/limitations.md) for practical
details:

- Tracking accuracy and stability depend on the model, payload, time step,
  and constraint tuning. Validate these for your application.
- Revolute **joint limits are capped at ±0.99π** by the engine; wider URDF
  limits fall back to command clamping.
- Use at least four substeps. Constructing `World(substeps=...)` with a value
  below four emits `UnstableSimulationWarning`.
- On chains where **parallel hinges are bracketed by perpendicular ones** (most
  arms), the engine's axis-alignment constraint leaks a small torque into the
  hinge. `enable_position_control(constraint_hertz=...)` trades pivot rigidity
  for tracking accuracy; the default is 60 Hz.
- Spring position control does not enforce URDF effort limits.
- Determinism regression tests cover repeated runs and selected thread counts;
  they do not establish identical results across all platforms and scenes.

## Development

```bash
git clone --recursive https://github.com/neka-nat/robox3d
cd robox3d
uv sync                        # builds box3d + shim via scikit-build-core
uv run pytest
uv run python tools/build_viewer.py   # bundle the web viewer (needs pnpm)
uv run python examples/viz_arm.py
```

See the [development guide](docs/development.md) for prerequisites, viewer
development, source layout, and binding generation. The
[documentation index](docs/README.md) links to the available guides.

## Credits

- [Box3D](https://github.com/erincatto/box3d) by Erin Catto (MIT)
- Bundled [SO-ARM101](https://github.com/TheRobotStudio/SO-ARM100) model by
  TheRobotStudio (Apache-2.0)

## License

MIT
