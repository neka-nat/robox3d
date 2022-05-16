import numpy as np
import pytest

import robox3d
from robox3d.math import quat_from_axis_angle


@pytest.fixture
def world():
    with robox3d.World() as w:
        yield w


def build_chain(world, n_links: int = 4, link_length: float = 0.3, spring=(120.0, 1.0)):
    """A serial chain extending horizontally along +X (for tests)."""
    ground = world.create_body(kind="static")
    rot = quat_from_axis_angle([0, 1, 0], -np.pi / 2)
    bodies, joints = [], []
    parent = ground
    for i in range(n_links):
        top = (i * link_length, 0.0, 0.0)
        body = world.create_body(position=top, rotation=rot)
        body.add_capsule(
            (0, 0, -0.05), (0, 0, -(link_length - 0.05)), radius=0.05, density=1200.0
        )
        joint = world.create_revolute_joint(
            parent, body, anchor=top, axis=(0, 1, 0), spring=spring
        )
        bodies.append(body)
        joints.append(joint)
        parent = body
    return bodies, joints
