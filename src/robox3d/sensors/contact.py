"""Contact sensor: a body's contact state and net normal force."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .._ffi import ffi, lib
from ..core.body import Body


@dataclass
class ContactReading:
    touching: bool
    normal_force: np.ndarray  # net normal force on the body (N, world frame)
    num_points: int

    @property
    def normal_force_magnitude(self) -> float:
        return float(np.linalg.norm(self.normal_force))


def _same_body(body_id_a, body_id_b) -> bool:
    return (
        body_id_a.index1 == body_id_b.index1
        and body_id_a.world0 == body_id_b.world0
        and body_id_a.generation == body_id_b.generation
    )


class ContactSensor:
    """Read a body's contacts (pull-based: aggregates the contact manifolds at the
    moment read() is called).

    The normal force is obtained by dividing the impulse by the most recent step size.
    """

    def __init__(self, body: Body):
        self.body = body

    def read(self) -> ContactReading:
        body_id = self.body.id
        capacity = lib.b3Body_GetContactCapacity(body_id)
        force = np.zeros(3)
        n_points = 0
        if capacity > 0:
            buf = ffi.new("b3ContactData[]", capacity)
            n = lib.b3Body_GetContactData(body_id, buf, capacity)
            # normalImpulse is a per-substep impulse
            dt = self.body._world.last_substep_dt
            for k in range(n):
                cd = buf[k]
                # the normal points shapeA -> shapeB; if this body is A, the force it receives is along -normal
                shape_body_a = lib.b3Shape_GetBody(cd.shapeIdA)
                sign = -1.0 if _same_body(shape_body_a, body_id) else 1.0
                for mi in range(cd.manifoldCount):
                    m = cd.manifolds[mi]
                    normal = np.array([m.normal.x, m.normal.y, m.normal.z])
                    for pi in range(m.pointCount):
                        impulse = m.points[pi].normalImpulse
                        if impulse > 0.0:
                            force += sign * normal * (impulse / dt)
                            n_points += 1
        return ContactReading(
            touching=n_points > 0, normal_force=force, num_points=n_points
        )
