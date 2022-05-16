"""Wire protocol shared with the web viewer (layer 4).

- Scene description: JSON, sent once on connect. Body names, kinds, colors,
  shapes, and visuals.
- Pose frame: binary (little-endian)
    [u8 msgType=1][u8 x3 reserved][f64 simTime][f32 x 7 x n]  (x,y,z,qx,qy,qz,qw)

Shapes are read back from the engine (single source of truth). box/cylinder/mesh
all live in the engine as convex hulls, so the viewer reconstructs them with
ConvexGeometry from the hull vertices.

The scene is frozen when streaming starts (adding bodies requires a server
restart).
"""

from __future__ import annotations

import json
import struct

import numpy as np

from .._ffi import ffi, lib
from ..core.body import Body
from ..core.world import World

PROTOCOL_VERSION = 1
MSG_POSES = 1

_FRAME_HEADER = struct.Struct("<B3xd")  # msgType, reserved, simTime


def _shape_dict(shape_id) -> dict | None:
    stype = lib.b3Shape_GetType(shape_id)
    if stype == lib.b3_sphereShape:
        s = lib.b3Shape_GetSphere(shape_id)
        return {
            "kind": "sphere",
            "center": [s.center.x, s.center.y, s.center.z],
            "radius": s.radius,
        }
    if stype == lib.b3_capsuleShape:
        c = lib.b3Shape_GetCapsule(shape_id)
        return {
            "kind": "capsule",
            "p1": [c.center1.x, c.center1.y, c.center1.z],
            "p2": [c.center2.x, c.center2.y, c.center2.z],
            "radius": c.radius,
        }
    if stype == lib.b3_hullShape:
        hull = lib.b3Shape_GetHull(shape_id)
        base = ffi.cast("char*", hull)
        points = ffi.cast("b3Vec3*", base + hull.pointOffset)
        pts = [
            [points[i].x, points[i].y, points[i].z] for i in range(hull.vertexCount)
        ]
        return {"kind": "hull", "points": pts}
    return None  # mesh / heightfield / compound not supported (v1)


def body_shapes(body: Body) -> list[dict]:
    count = lib.b3Body_GetShapeCount(body.id)
    if count == 0:
        return []
    shape_ids = ffi.new("b3ShapeId[]", count)
    n = lib.b3Body_GetShapes(body.id, shape_ids, count)
    shapes = []
    for i in range(n):
        d = _shape_dict(shape_ids[i])
        if d is not None:
            shapes.append(d)
    return shapes


def robot_target_values(robot) -> list[float]:
    """Current control targets of the actuated joints, in ``robot.actuated`` order."""
    from ..core.joints import PrismaticJoint

    values = []
    for name in robot.actuated:
        joint = robot.joints[name]
        prismatic = isinstance(joint, PrismaticJoint)
        values.append(joint.target_translation if prismatic else joint.target_angle)
    return values


def _robot_message(robot) -> dict:
    """Robot description for the joint-slider UI."""
    import numpy as np

    from ..core.joints import PrismaticJoint

    values = robot_target_values(robot)
    joints = []
    for k, name in enumerate(robot.actuated):
        joint = robot.joints[name]
        limits = robot.position_limits.get(name)
        prismatic = isinstance(joint, PrismaticJoint)
        lower, upper = limits if limits else ((-1.0, 1.0) if prismatic else (-np.pi, np.pi))
        joints.append(
            {
                "name": name,
                "index": k,
                "kind": "prismatic" if prismatic else "revolute",
                "lower": lower,
                "upper": upper,
                "value": values[k],
            }
        )
    return {"name": robot.name, "joints": joints}


def scene_dict(world: World, robots: list | None = None) -> dict:
    """Build the scene description from all bodies in the world."""
    bodies = []
    for i, body in enumerate(world.bodies):
        mass = body.mass
        bodies.append(
            {
                "i": i,
                "name": body.name or f"body_{i}",
                "kind": "static" if mass == 0.0 else "dynamic",
                "color": body.color,
                "shapes": body_shapes(body),
                "visuals": body.visuals,
            }
        )
    return {
        "type": "scene",
        "version": PROTOCOL_VERSION,
        "bodies": bodies,
        "robots": [_robot_message(r) for r in (robots or [])],
    }


def scene_message(world: World, robots: list | None = None) -> str:
    """Build the scene-description JSON from all bodies in the world."""
    return json.dumps(scene_dict(world, robots=robots))


def encode_pose_frame(sim_time: float, poses: np.ndarray) -> bytes:
    """Pack an (n,7) float32 pose array into a binary frame."""
    return _FRAME_HEADER.pack(MSG_POSES, sim_time) + poses.tobytes()


def decode_pose_frame(data: bytes) -> tuple[float, np.ndarray]:
    """Unpack a binary frame into (simTime, poses(n,7)). For tests and replay."""
    msg_type, sim_time = _FRAME_HEADER.unpack_from(data)
    if msg_type != MSG_POSES:
        raise ValueError(f"Unknown message type: {msg_type}")
    poses = np.frombuffer(data, dtype=np.float32, offset=_FRAME_HEADER.size)
    return sim_time, poses.reshape(-1, 7)
