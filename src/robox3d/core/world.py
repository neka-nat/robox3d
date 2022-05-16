"""Thin wrapper around the simulation world."""

from __future__ import annotations

import warnings

import numpy as np

from .. import math as rxmath
from .._ffi import ffi, lib
from . import _log
from .body import Body, _vec3
from .joints import PrismaticJoint, RevoluteJoint, WeldJoint

MIN_RECOMMENDED_SUBSTEPS = 4

# Recommended constraint stiffness for robotics use (validation-report.md experiment 2: anchor separation 5mm -> 0.3mm)
DEFAULT_CONSTRAINT_HERTZ = 240.0


class UnstableSimulationWarning(RuntimeWarning):
    """The solver detected a NaN body velocity (the simulation has diverged)."""


class World:
    """box3d world. Can be used as a context manager.

    >>> with World() as world:
    ...     body = world.create_body(position=(0, 0, 1))
    ...     body.add_sphere(0.1)
    ...     world.step(1 / 240)
    """

    def __init__(
        self,
        gravity=(0.0, 0.0, -9.81),
        substeps: int = 4,
        worker_count: int = 1,
        enable_sleep: bool = False,
    ):
        if substeps < MIN_RECOMMENDED_SUBSTEPS:
            warnings.warn(
                f"substeps={substeps} is not recommended. box3d's solver is designed "
                f"around substeps, and multi-link systems diverge when substeps<4 "
                f"(validation-report.md experiment 1).",
                UnstableSimulationWarning,
                stacklevel=2,
            )
        wd = lib.b3DefaultWorldDef()
        wd.gravity = _vec3(gravity)
        wd.workerCount = worker_count
        wd.enableSleep = enable_sleep
        self.id = lib.b3CreateWorld(ffi.addressof(wd))
        self.substeps = substeps
        self.gravity = np.asarray(gravity, dtype=float)
        self.last_dt = 1.0 / 240.0  # most recent step size (used e.g. for sensor conversions)
        self.last_substep_dt = self.last_dt / substeps  # contact impulses are per-substep
        self.time = 0.0  # accumulated simulation time
        self.bodies: list[Body] = []  # registry of bodies in creation order (used e.g. for visualization)
        self._destroyed = False

    # ------------------------------------------------------------ lifecycle

    def destroy(self) -> None:
        if not self._destroyed:
            lib.b3DestroyWorld(self.id)
            self._destroyed = True

    def __enter__(self) -> "World":
        return self

    def __exit__(self, *exc) -> None:
        self.destroy()

    # ------------------------------------------------------------ stepping

    def step(self, dt: float = 1.0 / 240.0, substeps: int | None = None) -> None:
        sub = substeps or self.substeps
        lib.b3World_Step(self.id, dt, sub)
        self.last_dt = dt
        self.last_substep_dt = dt / sub
        self.time += dt
        self._check_log()

    def step_n(self, n: int, dt: float = 1.0 / 240.0, substeps: int | None = None) -> None:
        """Step n times in one call (single FFI call; e.g. for RL frame skipping)."""
        from .._ffi import shim

        sub = substeps or self.substeps
        shim.rxWorldStepN(self.id, dt, sub, n)
        self.last_dt = dt
        self.last_substep_dt = dt / sub
        self.time += dt * n
        self._check_log()

    def _check_log(self) -> None:
        if _log.pending():
            for msg in _log.drain():
                if "unstable" in msg:
                    warnings.warn(
                        f"box3d solver: {msg} (body velocity became NaN; review dt/substeps/mass ratios)",
                        UnstableSimulationWarning,
                        stacklevel=3,
                    )

    # ------------------------------------------------------------ bodies

    def create_body(
        self,
        position=(0.0, 0.0, 0.0),
        rotation=None,
        kind: str = "dynamic",
        name: str | None = None,
    ) -> Body:
        """Create a body. kind: "dynamic" | "static" | "kinematic"."""
        types = {
            "static": lib.b3_staticBody,
            "kinematic": lib.b3_kinematicBody,
            "dynamic": lib.b3_dynamicBody,
        }
        bd = lib.b3DefaultBodyDef()
        bd.type = types[kind]
        bd.position = _vec3(position)
        if rotation is not None:
            q = np.asarray(rotation, dtype=float)
            bd.rotation.v.x, bd.rotation.v.y, bd.rotation.v.z, bd.rotation.s = q
        body = Body(lib.b3CreateBody(self.id, ffi.addressof(bd)), self, name=name)
        self.bodies.append(body)
        return body

    # ------------------------------------------------------------ joints

    @staticmethod
    def _set_joint_frames(jd_base, parent: Body, child: Body, anchor, axis) -> None:
        """Set the local frames from a world-space anchor and axis (= the joint frame's Z axis)."""
        q_joint = rxmath.quat_z_to(axis)
        anchor = np.asarray(anchor, dtype=float)
        for body, frame in ((parent, jd_base.localFrameA), (child, jd_base.localFrameB)):
            p_body = body.position
            q_inv = rxmath.quat_conj(body.rotation)
            frame.p = _vec3(rxmath.quat_rotate(q_inv, anchor - p_body))
            fq = rxmath.quat_mul(q_inv, q_joint)
            frame.q.v.x, frame.q.v.y, frame.q.v.z, frame.q.s = fq

    def create_revolute_joint(
        self,
        parent: Body,
        child: Body,
        anchor,
        axis,
        spring: tuple[float, float] | None = None,
        target_angle: float = 0.0,
        motor: tuple[float, float] | None = None,
        limits: tuple[float, float] | None = None,
        collide_connected: bool = False,
        constraint_hertz: float | None = DEFAULT_CONSTRAINT_HERTZ,
    ) -> RevoluteJoint:
        """Create a revolute joint from a world-space anchor point and rotation axis.

        spring: (hertz, damping_ratio) — enable spring position control
        motor:  (max_torque, speed) — enable motor velocity control
        limits: (lower, upper) rad — range-of-motion limit
        constraint_hertz: constraint stiffness. None leaves the box3d default
        """
        jd = lib.b3DefaultRevoluteJointDef()
        jd.base.bodyIdA = parent.id
        jd.base.bodyIdB = child.id
        jd.base.collideConnected = collide_connected
        self._set_joint_frames(jd.base, parent, child, anchor, axis)

        jd.targetAngle = target_angle
        if spring is not None:
            jd.enableSpring = True
            jd.hertz, jd.dampingRatio = spring
        if motor is not None:
            jd.enableMotor = True
            jd.maxMotorTorque, jd.motorSpeed = motor
        if limits is not None:
            jd.enableLimit = True
            jd.lowerAngle, jd.upperAngle = limits

        joint = RevoluteJoint(lib.b3CreateRevoluteJoint(self.id, ffi.addressof(jd)), self)
        if constraint_hertz is not None:
            joint.set_constraint_tuning(constraint_hertz)
        return joint

    def create_prismatic_joint(
        self,
        parent: Body,
        child: Body,
        anchor,
        axis,
        spring: tuple[float, float] | None = None,
        target_translation: float = 0.0,
        motor: tuple[float, float] | None = None,
        limits: tuple[float, float] | None = None,
        collide_connected: bool = False,
        constraint_hertz: float | None = DEFAULT_CONSTRAINT_HERTZ,
    ) -> PrismaticJoint:
        """Create a prismatic joint from a world-space anchor point and travel axis.

        spring: (hertz, damping_ratio) — enable spring position control
        motor:  (max_force, speed) — enable motor velocity control
        limits: (lower, upper) m — range-of-motion limit
        """
        jd = lib.b3DefaultPrismaticJointDef()
        jd.base.bodyIdA = parent.id
        jd.base.bodyIdB = child.id
        jd.base.collideConnected = collide_connected
        self._set_joint_frames(jd.base, parent, child, anchor, axis)

        jd.targetTranslation = target_translation
        if spring is not None:
            jd.enableSpring = True
            jd.hertz, jd.dampingRatio = spring
        if motor is not None:
            jd.enableMotor = True
            jd.maxMotorForce, jd.motorSpeed = motor
        if limits is not None:
            jd.enableLimit = True
            jd.lowerTranslation, jd.upperTranslation = limits

        joint = PrismaticJoint(lib.b3CreatePrismaticJoint(self.id, ffi.addressof(jd)), self)
        if constraint_hertz is not None:
            joint.set_constraint_tuning(constraint_hertz)
        return joint

    def create_weld_joint(
        self,
        parent: Body,
        child: Body,
        anchor=None,
        collide_connected: bool = False,
        constraint_hertz: float | None = DEFAULT_CONSTRAINT_HERTZ,
    ) -> WeldJoint:
        """Fully constrain two bodies together (equivalent to a URDF fixed joint).

        If anchor is omitted, the child body's origin is used as the shared frame.
        """
        if anchor is None:
            anchor = child.position
        jd = lib.b3DefaultWeldJointDef()
        jd.base.bodyIdA = parent.id
        jd.base.bodyIdB = child.id
        jd.base.collideConnected = collide_connected
        self._set_joint_frames(jd.base, parent, child, anchor, (0.0, 0.0, 1.0))

        joint = WeldJoint(lib.b3CreateWeldJoint(self.id, ffi.addressof(jd)), self)
        if constraint_hertz is not None:
            joint.set_constraint_tuning(constraint_hertz)
        return joint

    # ------------------------------------------------------------ queries

    def cast_ray_closest(self, origin, translation):
        """Cast a ray and return the closest hit, or None if nothing was hit.

        Returns: (point, normal, fraction)
        """
        filter_ = lib.b3DefaultQueryFilter()
        result = lib.b3World_CastRayClosest(self.id, _vec3(origin), _vec3(translation), filter_)
        if not result.hit:
            return None
        return (
            np.array([result.point.x, result.point.y, result.point.z]),
            np.array([result.normal.x, result.normal.y, result.normal.z]),
            result.fraction,
        )
