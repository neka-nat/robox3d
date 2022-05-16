"""Thin wrapper around a rigid body."""

from __future__ import annotations

import numpy as np

from .. import math as rxmath
from .._ffi import ffi, lib


def _vec3(v):
    return ffi.new("b3Vec3*", {"x": float(v[0]), "y": float(v[1]), "z": float(v[2])})[0]


def _get_vec3(cv) -> np.ndarray:
    return np.array([cv.x, cv.y, cv.z])


class Body:
    """Wrapper around a box3d body. Created via World.create_body()."""

    __slots__ = ("id", "_world", "name", "color", "visuals")

    def __init__(self, body_id, world, name: str | None = None):
        self.id = body_id
        self._world = world
        self.name = name
        self.color: str | None = None  # visualization color "#rrggbb" (None uses the viewer's default color)
        self.visuals: list[dict] = []  # visualization-only geometry (see viz/protocol.py); if empty, collision shapes are shown

    # ------------------------------------------------------------ state

    @property
    def position(self) -> np.ndarray:
        return _get_vec3(lib.b3Body_GetPosition(self.id))

    @property
    def rotation(self) -> np.ndarray:
        """Orientation quaternion [x, y, z, w]."""
        q = lib.b3Body_GetRotation(self.id)
        return np.array([q.v.x, q.v.y, q.v.z, q.s])

    @property
    def pose(self) -> tuple[np.ndarray, np.ndarray]:
        return self.position, self.rotation

    def set_pose(self, position, rotation=None) -> None:
        q = rxmath.quat_identity() if rotation is None else np.asarray(rotation, dtype=float)
        cq = ffi.new(
            "b3Quat*",
            {"v": {"x": q[0], "y": q[1], "z": q[2]}, "s": q[3]},
        )[0]
        lib.b3Body_SetTransform(self.id, _vec3(position), cq)

    @property
    def linear_velocity(self) -> np.ndarray:
        return _get_vec3(lib.b3Body_GetLinearVelocity(self.id))

    @linear_velocity.setter
    def linear_velocity(self, v) -> None:
        lib.b3Body_SetLinearVelocity(self.id, _vec3(v))

    @property
    def angular_velocity(self) -> np.ndarray:
        return _get_vec3(lib.b3Body_GetAngularVelocity(self.id))

    @angular_velocity.setter
    def angular_velocity(self, w) -> None:
        lib.b3Body_SetAngularVelocity(self.id, _vec3(w))

    @property
    def mass(self) -> float:
        return lib.b3Body_GetMass(self.id)

    def set_mass_data(self, mass: float, center, inertia: np.ndarray) -> None:
        """Set mass properties explicitly (e.g. to apply URDF inertia values).

        Call this after all shapes have been added (adding a shape recomputes the mass).

        center: center of mass in body-local coordinates
        inertia: inertia tensor (3x3) about the center of mass, in body-local axes
        """
        inertia = np.asarray(inertia, dtype=float)
        md = ffi.new("b3MassData*")
        md.mass = mass
        md.center = _vec3(center)
        for col, name in enumerate(("cx", "cy", "cz")):
            c = getattr(md.inertia, name)
            c.x, c.y, c.z = inertia[:, col]
        lib.b3Body_SetMassData(self.id, md[0])

    @property
    def center_of_mass(self) -> np.ndarray:
        """Center of mass in world coordinates."""
        return _get_vec3(lib.b3Body_GetWorldCenterOfMass(self.id))

    # ------------------------------------------------------------ forces

    def apply_force(self, force, wake: bool = True) -> None:
        """Apply a force at the center of mass (consumed on the next step)."""
        lib.b3Body_ApplyForceToCenter(self.id, _vec3(force), wake)

    def apply_torque(self, torque, wake: bool = True) -> None:
        lib.b3Body_ApplyTorque(self.id, _vec3(torque), wake)

    # ------------------------------------------------------------ shapes

    def _shape_def(self, density: float, friction: float, restitution: float):
        sd = lib.b3DefaultShapeDef()
        sd.density = density
        sd.baseMaterial.friction = friction
        sd.baseMaterial.restitution = restitution
        return sd

    def add_sphere(
        self,
        radius: float,
        center=(0.0, 0.0, 0.0),
        density: float = 1000.0,
        friction: float = 0.6,
        restitution: float = 0.0,
    ):
        sd = self._shape_def(density, friction, restitution)
        sphere = ffi.new("b3Sphere*")
        sphere.center = _vec3(center)
        sphere.radius = radius
        return lib.b3CreateSphereShape(self.id, ffi.addressof(sd), sphere)

    def add_capsule(
        self,
        point1,
        point2,
        radius: float,
        density: float = 1000.0,
        friction: float = 0.6,
        restitution: float = 0.0,
    ):
        sd = self._shape_def(density, friction, restitution)
        capsule = ffi.new("b3Capsule*")
        capsule.center1 = _vec3(point1)
        capsule.center2 = _vec3(point2)
        capsule.radius = radius
        return lib.b3CreateCapsuleShape(self.id, ffi.addressof(sd), capsule)

    def add_box(
        self,
        half_extents,
        offset=None,
        density: float = 1000.0,
        friction: float = 0.6,
        restitution: float = 0.0,
    ):
        hx, hy, hz = (float(x) for x in half_extents)
        if offset is None:
            hull = lib.b3MakeBoxHull(hx, hy, hz)
        else:
            hull = lib.b3MakeOffsetBoxHull(hx, hy, hz, _vec3(offset))
        sd = self._shape_def(density, friction, restitution)
        # b3BoxHull.base's offset is a relative reference, so a value copy is safe
        hull_ref = ffi.new("b3BoxHull*", hull)
        return lib.b3CreateHullShape(self.id, ffi.addressof(sd), ffi.addressof(hull_ref.base))

    def add_hull(
        self,
        points,
        density: float = 1000.0,
        friction: float = 0.6,
        restitution: float = 0.0,
        max_vertices: int = 32,
    ):
        """Add the convex hull of a point cloud as a collision shape (for meshes)."""
        pts = np.ascontiguousarray(points, dtype=np.float32).reshape(-1, 3)
        c_points = ffi.new("b3Vec3[]", len(pts))
        buf = ffi.buffer(c_points)
        np.frombuffer(buf, dtype=np.float32)[:] = pts.ravel()
        hull = lib.b3CreateHull(c_points, len(pts), max_vertices)
        if hull == ffi.NULL:
            raise ValueError(f"Failed to build convex hull ({len(pts)} points)")
        try:
            sd = self._shape_def(density, friction, restitution)
            return lib.b3CreateHullShape(self.id, ffi.addressof(sd), hull)
        finally:
            lib.b3DestroyHull(hull)
