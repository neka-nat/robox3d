"""Thin wrappers around joints."""

from __future__ import annotations

import numpy as np

from .._ffi import ffi, lib


def _get_vec3(cv) -> np.ndarray:
    return np.array([cv.x, cv.y, cv.z])


class _JointBase:
    __slots__ = ("id", "_world")

    def __init__(self, joint_id, world):
        self.id = joint_id
        self._world = world

    def _frame_on(self, which: str):
        """World pose (origin, quat) of the joint frame on body A/B."""
        from ..math import quat_mul, quat_rotate

        if which == "a":
            body = lib.b3Joint_GetBodyA(self.id)
            frame = lib.b3Joint_GetLocalFrameA(self.id)
        else:
            body = lib.b3Joint_GetBodyB(self.id)
            frame = lib.b3Joint_GetLocalFrameB(self.id)
        xf = lib.b3Body_GetTransform(body)
        q_body = np.array([xf.q.v.x, xf.q.v.y, xf.q.v.z, xf.q.s])
        p_body = np.array([xf.p.x, xf.p.y, xf.p.z])
        origin = p_body + quat_rotate(q_body, _get_vec3(frame.p))
        q = quat_mul(q_body, np.array([frame.q.v.x, frame.q.v.y, frame.q.v.z, frame.q.s]))
        return origin, q

    @property
    def world_anchor(self) -> np.ndarray:
        """Joint anchor in world coordinates (from body A's frame)."""
        return self._frame_on("a")[0]

    @property
    def world_axis(self) -> np.ndarray:
        """Joint axis (frame Z axis) direction in world coordinates."""
        from ..math import quat_rotate

        _, q = self._frame_on("a")
        return quat_rotate(q, (0.0, 0.0, 1.0))

    @property
    def body_a(self):
        return lib.b3Joint_GetBodyA(self.id)

    @property
    def body_b(self):
        return lib.b3Joint_GetBodyB(self.id)

    @property
    def constraint_force(self) -> np.ndarray:
        """Constraint force (N). Equivalent to an F/T sensor reading."""
        return _get_vec3(lib.b3Joint_GetConstraintForce(self.id))

    @property
    def constraint_torque(self) -> np.ndarray:
        """Constraint torque (N·m). Equivalent to an F/T sensor reading."""
        return _get_vec3(lib.b3Joint_GetConstraintTorque(self.id))

    def set_constraint_tuning(self, hertz: float, damping_ratio: float = 0.0) -> None:
        """Set the constraint stiffness.

        High values (240 Hz) keep pivots tight under load (validation-report.md,
        experiment 2), but on chains with parallel hinges the axis-alignment
        constraint leaks into the hinge DOF proportionally to this value and
        degrades spring position tracking — use ~60 Hz for position-controlled
        joints (docs/spring-chain-investigation.md).
        """
        lib.b3Joint_SetConstraintTuning(self.id, hertz, damping_ratio)


class RevoluteJoint(_JointBase):
    """Revolute (hinge) joint. Created via World.create_revolute_joint().

    The hinge axis is the Z axis of the joint's local frame.
    """

    __slots__ = ()

    # ------------------------------------------------------------ state

    @property
    def angle(self) -> float:
        """Joint angle (rad)."""
        return lib.b3RevoluteJoint_GetAngle(self.id)

    @property
    def speed(self) -> float:
        """Relative angular velocity about the hinge axis (rad/s)."""
        from .._ffi import shim

        out = ffi.new("float[1]")
        ids = ffi.new("b3JointId[1]", [self.id])
        shim.rxRevoluteGetSpeeds(ids, 1, out)
        return out[0]

    # ------------------------------------------------------------ position control (spring)

    @property
    def target_angle(self) -> float:
        return lib.b3RevoluteJoint_GetTargetAngle(self.id)

    @target_angle.setter
    def target_angle(self, radians: float) -> None:
        lib.b3RevoluteJoint_SetTargetAngle(self.id, radians)

    def enable_spring(self, hertz: float, damping_ratio: float = 1.0) -> None:
        """Enable spring-based position control (tracks target_angle)."""
        lib.b3RevoluteJoint_SetSpringHertz(self.id, hertz)
        lib.b3RevoluteJoint_SetSpringDampingRatio(self.id, damping_ratio)
        lib.b3RevoluteJoint_EnableSpring(self.id, True)

    def disable_spring(self) -> None:
        lib.b3RevoluteJoint_EnableSpring(self.id, False)

    # ------------------------------------------------------------ velocity control (motor)

    @property
    def motor_speed(self) -> float:
        return lib.b3RevoluteJoint_GetMotorSpeed(self.id)

    @motor_speed.setter
    def motor_speed(self, radians_per_sec: float) -> None:
        lib.b3RevoluteJoint_SetMotorSpeed(self.id, radians_per_sec)

    @property
    def max_motor_torque(self) -> float:
        return lib.b3RevoluteJoint_GetMaxMotorTorque(self.id)

    @max_motor_torque.setter
    def max_motor_torque(self, torque: float) -> None:
        lib.b3RevoluteJoint_SetMaxMotorTorque(self.id, torque)

    @property
    def motor_torque(self) -> float:
        """Torque the motor is currently producing (N·m)."""
        return lib.b3RevoluteJoint_GetMotorTorque(self.id)

    def enable_motor(self, max_torque: float, speed: float = 0.0) -> None:
        lib.b3RevoluteJoint_SetMaxMotorTorque(self.id, max_torque)
        lib.b3RevoluteJoint_SetMotorSpeed(self.id, speed)
        lib.b3RevoluteJoint_EnableMotor(self.id, True)

    def disable_motor(self) -> None:
        lib.b3RevoluteJoint_EnableMotor(self.id, False)

    # ------------------------------------------------------------ limits

    def enable_limit(self, lower: float, upper: float) -> None:
        lib.b3RevoluteJoint_SetLimits(self.id, lower, upper)
        lib.b3RevoluteJoint_EnableLimit(self.id, True)

    def disable_limit(self) -> None:
        lib.b3RevoluteJoint_EnableLimit(self.id, False)


class PrismaticJoint(_JointBase):
    """Prismatic (linear) joint. The travel axis is the Z axis of the joint's local frame."""

    __slots__ = ()

    @property
    def translation(self) -> float:
        """Joint displacement (m)."""
        return lib.b3PrismaticJoint_GetTranslation(self.id)

    @property
    def speed(self) -> float:
        """Joint speed (m/s)."""
        return lib.b3PrismaticJoint_GetSpeed(self.id)

    @property
    def target_translation(self) -> float:
        return lib.b3PrismaticJoint_GetTargetTranslation(self.id)

    @target_translation.setter
    def target_translation(self, meters: float) -> None:
        lib.b3PrismaticJoint_SetTargetTranslation(self.id, meters)

    def enable_spring(self, hertz: float, damping_ratio: float = 1.0) -> None:
        lib.b3PrismaticJoint_SetSpringHertz(self.id, hertz)
        lib.b3PrismaticJoint_SetSpringDampingRatio(self.id, damping_ratio)
        lib.b3PrismaticJoint_EnableSpring(self.id, True)

    def disable_spring(self) -> None:
        lib.b3PrismaticJoint_EnableSpring(self.id, False)

    @property
    def motor_speed(self) -> float:
        return lib.b3PrismaticJoint_GetMotorSpeed(self.id)

    @motor_speed.setter
    def motor_speed(self, meters_per_sec: float) -> None:
        lib.b3PrismaticJoint_SetMotorSpeed(self.id, meters_per_sec)

    @property
    def max_motor_force(self) -> float:
        return lib.b3PrismaticJoint_GetMaxMotorForce(self.id)

    @max_motor_force.setter
    def max_motor_force(self, force: float) -> None:
        lib.b3PrismaticJoint_SetMaxMotorForce(self.id, force)

    def enable_motor(self, max_force: float, speed: float = 0.0) -> None:
        lib.b3PrismaticJoint_SetMaxMotorForce(self.id, max_force)
        lib.b3PrismaticJoint_SetMotorSpeed(self.id, speed)
        lib.b3PrismaticJoint_EnableMotor(self.id, True)

    def disable_motor(self) -> None:
        lib.b3PrismaticJoint_EnableMotor(self.id, False)

    def enable_limit(self, lower: float, upper: float) -> None:
        lib.b3PrismaticJoint_SetLimits(self.id, lower, upper)
        lib.b3PrismaticJoint_EnableLimit(self.id, True)

    def disable_limit(self) -> None:
        lib.b3PrismaticJoint_EnableLimit(self.id, False)


class WeldJoint(_JointBase):
    """Weld (fixed) joint. Corresponds to a URDF fixed joint."""

    __slots__ = ()
