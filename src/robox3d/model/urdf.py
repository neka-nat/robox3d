"""URDF -> box3d conversion (the core of layer 2).

Conversion rules:
- link -> Body (world transforms are built from the zero pose)
- joint: revolute/continuous -> RevoluteJoint, prismatic -> PrismaticJoint,
  fixed -> WeldJoint (but child links without inertia are merged into the parent
  body), floating -> unconstrained
- inertia: URDF <inertial> overrides b3's automatic computation (the tensor is
  rotated into the link frame)
- collision: see geometry.py (everything is converted to convex shapes)
- self-collision: disabled by default (a negative groupIndex is assigned per robot)

Limitations:
- box3d revolute limits go up to ±0.99π. URDF limits beyond that are warned about
  and disabled (e.g. UR arms' ±2π).
- planar joints are not supported
"""

from __future__ import annotations

import itertools
import warnings
from pathlib import Path

import numpy as np

from ..core import Body, PrismaticJoint, RevoluteJoint, World
from ..math import matrix_to_quat, quat_to_matrix
from .geometry import add_collision_geometry, visual_descriptor

_LIMIT_MAX = 0.99 * np.pi
_group_counter = itertools.count(1)


class Robot:
    """A robot built from a URDF. Created via load_urdf()."""

    def __init__(self, world: World, name: str):
        self.world = world
        self.name = name
        self.links: dict[str, Body] = {}  # link name -> Body (including merge targets)
        self.joints: dict[str, object] = {}  # joint name -> wrapper
        self.joint_types: dict[str, str] = {}  # joint name -> URDF type
        self.actuated: list[str] = []  # actuated joint names (in URDF order)
        self.effort_limits: dict[str, float | None] = {}
        self.position_limits: dict[str, tuple[float, float] | None] = {}
        self.base_body: Body | None = None
        self._revolute_group = None
        self._limits_lo: np.ndarray | None = None
        self._limits_hi: np.ndarray | None = None
        self._efforts: np.ndarray | None = None
        self._subtrees: dict[str, list[Body]] = {}  # actuated joint -> downstream bodies (for gravity compensation)

    # ------------------------------------------------------------ access

    @property
    def dof(self) -> int:
        return len(self.actuated)

    def joint(self, name: str):
        return self.joints[name]

    def link_body(self, name: str) -> Body:
        """The Body a link belongs to (a fixed-merged link maps to the parent's Body)."""
        return self.links[name]

    def _actuated_joints(self):
        return [self.joints[n] for n in self.actuated]

    def _all_revolute(self) -> bool:
        return all(isinstance(j, RevoluteJoint) for j in self._actuated_joints())

    def _group(self):
        if self._revolute_group is None and self._all_revolute():
            from ..core import RevoluteGroup

            self._revolute_group = RevoluteGroup(self._actuated_joints())
        return self._revolute_group

    # ------------------------------------------------------------ control

    def enable_position_control(
        self,
        hertz: float | None = None,
        damping_ratio: float = 1.0,
        *,
        kp: float | None = None,
        kd: float | None = None,
        constraint_hertz: float | None = 60.0,
    ) -> None:
        """Set spring position control on all actuated joints.

        Two ways to specify it:
        - hertz / damping_ratio: box3d-native normalized stiffness (same response
          for every joint)
        - kp / kd: gains in physical units (N·m/rad, N·m·s/rad), converted using
          each joint's effective inertia. If kd is omitted, critical damping is used.

        constraint_hertz: pivot/axis-alignment stiffness for the actuated joints.
        box3d shares one stiffness between the pivot and the axis-alignment
        constraint, and the alignment constraint leaks a torque into the hinge
        that grows ~linearly with this value; on serial chains with parallel
        hinges a high value (the world default 240) produces a large frozen
        tracking error (see docs/spring-chain-investigation.md). The default
        60 Hz balances tracking accuracy against pivot rigidity; use ~20 Hz when
        <0.01 rad tracking matters more than tight pivots, or None to keep the
        joints' current tuning.

        Note: springs do not enforce the effort limit (combine with set_torques for
        torque-limited control).
        """
        from ..control import joint_effective_inertia, pd_to_spring

        if kp is not None:
            for j in self._actuated_joints():
                h, d = pd_to_spring(kp, kd, joint_effective_inertia(j))
                j.enable_spring(h, d)
        else:
            hertz = 60.0 if hertz is None else hertz
            for j in self._actuated_joints():
                j.enable_spring(hertz, damping_ratio)
        if constraint_hertz is not None:
            for j in self._actuated_joints():
                # damping 2.0 matches box3d's native b3DefaultJointDef
                j.set_constraint_tuning(constraint_hertz, 2.0)

    def enable_torque_control(self, disable_springs: bool = True) -> None:
        """Enable torque command mode (command with set_torques every step).

        With disable_springs=False, this can be combined with spring position
        control (a feedforward + feedback setup).
        """
        for j in self._actuated_joints():
            if disable_springs:
                j.disable_spring()
            if isinstance(j, PrismaticJoint):
                j.enable_motor(max_force=0.0, speed=0.0)
            else:
                j.enable_motor(max_torque=0.0, speed=0.0)

    def set_torques(self, tau) -> None:
        """Torque/force commands for the actuated joints (N·m / N), in actuated order.

        Pseudo torque control that "saturates the velocity motor in the commanded
        direction and sets the max torque to |τ|" (development-plan §Phase 3). Call
        it every step. Commands are clamped by the URDF effort limits. Requires a
        prior enable_torque_control() (or a combined setup with disable_springs=False).
        """
        from ..control import TORQUE_CONTROL_SPEED

        tau = np.asarray(tau, dtype=float)
        if self._efforts is None:
            self._efforts = np.array(
                [self.effort_limits.get(n) or np.inf for n in self.actuated]
            )
        tau = np.clip(tau, -self._efforts, self._efforts)

        group = self._group()
        if group is not None:
            group.set_motor_speeds(np.sign(tau) * TORQUE_CONTROL_SPEED)
            group.set_max_motor_torques(np.abs(tau))
            return
        for joint, t in zip(self._actuated_joints(), tau, strict=True):
            joint.motor_speed = float(np.sign(t)) * TORQUE_CONTROL_SPEED
            if isinstance(joint, PrismaticJoint):
                joint.max_motor_force = abs(float(t))
            else:
                joint.max_motor_torque = abs(float(t))

    def gravity_compensation(self, extra_bodies: list[Body] | None = None) -> np.ndarray:
        """Compute gravity-compensation torques/forces (in actuated order) for the
        current pose, using statics.

        These cancel the torque that each joint's downstream bodies produce about
        its axis under gravity. Use with set_torques() as a feedforward term.

        extra_bodies: bodies outside the URDF, e.g. a payload attached to the end
        effector. Treated as downstream of every actuated joint (assumes an EE
        payload on a serial arm).
        """
        g = self.world.gravity
        extra = extra_bodies or []
        tau = np.zeros(self.dof)
        for k, name in enumerate(self.actuated):
            joint = self.joints[name]
            axis = joint.world_axis
            bodies = self._subtrees[name] + extra
            if isinstance(joint, PrismaticJoint):
                f = sum(b.mass for b in bodies) * g
                tau[k] = -float(axis @ f)
            else:
                anchor = joint.world_anchor
                t = np.zeros(3)
                for b in bodies:
                    t += np.cross(b.center_of_mass - anchor, b.mass * g)
                tau[k] = -float(axis @ t)
        return tau

    def _clamp_targets(self, q: np.ndarray) -> np.ndarray:
        """Clamp targets to the URDF joint limits.

        box3d limits are soft constraints, so a stiff spring with a target well
        beyond a limit will overshoot into it. As on a real controller, we clamp
        on the command side.
        """
        if self._limits_lo is None:
            self._limits_lo = np.array(
                [
                    (self.position_limits.get(n) or (-np.inf, np.inf))[0]
                    for n in self.actuated
                ]
            )
            self._limits_hi = np.array(
                [
                    (self.position_limits.get(n) or (-np.inf, np.inf))[1]
                    for n in self.actuated
                ]
            )
        return np.clip(q, self._limits_lo, self._limits_hi)

    def set_targets(self, q) -> None:
        """Set target positions (rad / m) for the actuated joints, in actuated order.

        Targets are clamped to the URDF joint limits.
        """
        q = self._clamp_targets(np.asarray(q, dtype=float))
        group = self._group()
        if group is not None:
            group.set_targets(q)
            return
        for joint, target in zip(self._actuated_joints(), q, strict=True):
            if isinstance(joint, PrismaticJoint):
                joint.target_translation = target
            else:
                joint.target_angle = target

    def positions(self) -> np.ndarray:
        """Current positions of the actuated joints (rad / m)."""
        group = self._group()
        if group is not None:
            return group.angles().astype(float)
        return np.array(
            [
                j.translation if isinstance(j, PrismaticJoint) else j.angle
                for j in self._actuated_joints()
            ]
        )

    def velocities(self) -> np.ndarray:
        group = self._group()
        if group is not None:
            return group.speeds().astype(float)
        return np.array([j.speed for j in self._actuated_joints()])


def _make_resolver(urdf_dir: Path, mesh_root):
    def resolve(filename: str) -> str:
        candidates: list[Path] = []
        if filename.startswith("package://"):
            pkg, _, rel = filename[len("package://") :].partition("/")
            if mesh_root is not None:
                candidates += [Path(mesh_root) / pkg / rel, Path(mesh_root) / rel]
            d = urdf_dir
            for _ in range(4):
                candidates += [d / pkg / rel, d / rel]
                d = d.parent
        elif filename.startswith("file://"):
            candidates = [Path(filename[len("file://") :])]
        else:
            candidates = [urdf_dir / filename, Path(filename)]
        for c in candidates:
            if c.exists():
                return str(c)
        raise FileNotFoundError(
            f"Mesh {filename} not found (searched: {[str(c) for c in candidates]}). "
            "Specify the package root with the mesh_root argument."
        )

    return resolve


def _origin(xf) -> np.ndarray:
    return np.eye(4) if xf is None else np.asarray(xf, dtype=float)


def _material_color(visual, model) -> str | None:
    """Return a visual element's color as "#rrggbb" (also resolves robot-level name references)."""
    mat = getattr(visual, "material", None)
    if mat is None:
        return None
    color = getattr(mat, "color", None)
    if color is None and getattr(mat, "name", None):
        for m in getattr(model, "materials", None) or []:
            if m.name == mat.name and getattr(m, "color", None) is not None:
                color = m.color
                break
    if color is None or getattr(color, "rgba", None) is None:
        return None
    r, g, b = (max(0, min(255, round(255 * float(c)))) for c in color.rgba[:3])
    return f"#{r:02x}{g:02x}{b:02x}"


def load_urdf(
    world: World,
    path,
    *,
    position=(0.0, 0.0, 0.0),
    rotation=None,
    fixed_base: bool = True,
    mesh_root=None,
    mesh_mode: str = "hull",
    cylinder_mode: str = "hull",
    self_collision: bool = False,
    density: float = 1000.0,
    friction: float = 0.8,
    coacd_threshold: float = 0.05,
    max_hull_vertices: int = 32,
    name: str | None = None,
) -> Robot:
    """Load a URDF and build the robot in the world.

    - fixed_base: make the base link a static body
    - mesh_root: root directory for package:// resolution
    - mesh_mode: "hull" (single convex hull) | "coacd" (convex decomposition, requires coacd)
    - cylinder_mode: "hull" (16-sided prism) | "capsule"
    - self_collision: False disables all collisions within the robot
    """
    import yourdfpy

    urdf_path = Path(path)
    urdf = yourdfpy.URDF.load(
        str(urdf_path),
        load_meshes=False,
        build_scene_graph=False,
        load_collision_meshes=False,
        build_collision_scene_graph=False,
    )
    model = urdf.robot
    resolve = _make_resolver(urdf_path.parent, mesh_root)
    link_map = {link.name: link for link in model.links}

    # --- link world transforms at the zero pose (BFS from the base)
    t_base = np.eye(4)
    t_base[:3, 3] = np.asarray(position, dtype=float)
    if rotation is not None:
        t_base[:3, :3] = quat_to_matrix(np.asarray(rotation, dtype=float))

    children: dict[str, list] = {}
    for j in model.joints:
        children.setdefault(j.parent, []).append(j)

    # root link (a link that is not the child of any joint)
    child_names = {j.child for j in model.joints}
    roots = [link.name for link in model.links if link.name not in child_names]
    if len(roots) != 1:
        raise ValueError(f"Root link is not unique: {roots}")
    base_link = roots[0]
    t_world: dict[str, np.ndarray] = {base_link: t_base}
    owner: dict[str, str] = {base_link: base_link}  # link -> link name of its owning body
    stack = [base_link]
    while stack:
        parent = stack.pop()
        for j in children.get(parent, []):
            t_world[j.child] = t_world[parent] @ _origin(j.origin)
            child_link = link_map[j.child]
            if j.type == "fixed" and child_link.inertial is None:
                owner[j.child] = owner[parent]  # fixed link without inertia merges into the parent
            else:
                owner[j.child] = j.child
            stack.append(j.child)

    missing = set(link_map) - set(t_world)
    if missing:
        raise ValueError(f"Links unreachable from base link {base_link}: {missing}")

    robot = Robot(world, name or model.name or urdf_path.stem)
    group_index = 0 if self_collision else -next(_group_counter)

    # --- create bodies (only for links that are not merged)
    for link in model.links:
        ln = link.name
        if owner[ln] != ln:
            continue
        kind = "static" if (ln == base_link and fixed_base) else "dynamic"
        t = t_world[ln]
        body = world.create_body(
            position=t[:3, 3],
            rotation=matrix_to_quat(t[:3, :3]),
            kind=kind,
            name=f"{robot.name}/{ln}",
        )
        robot.links[ln] = body
    robot.base_body = robot.links[base_link]
    for ln in link_map:
        if owner[ln] != ln:
            robot.links[ln] = robot.links[owner[ln]]

    # --- collision shapes (added to the owning body, with an offset)
    for link in model.links:
        body = robot.links[link.name]
        own_ln = owner[link.name]
        xf_body_link = np.linalg.inv(t_world[own_ln]) @ t_world[link.name]
        for col in link.collisions:
            xf = xf_body_link @ _origin(col.origin)
            add_collision_geometry(
                body,
                col.geometry,
                xf,
                resolve=resolve,
                density=density,
                friction=friction,
                cylinder_mode=cylinder_mode,
                mesh_mode=mesh_mode,
                coacd_threshold=coacd_threshold,
                max_hull_vertices=max_hull_vertices,
                group_index=group_index,
            )

    # --- visual (visualization-only geometry; attached to the owning body with an offset)
    for link in model.links:
        body = robot.links[link.name]
        own_ln = owner[link.name]
        xf_body_link = np.linalg.inv(t_world[own_ln]) @ t_world[link.name]
        for vis in link.visuals:
            desc = visual_descriptor(
                vis.geometry,
                xf_body_link @ _origin(vis.origin),
                resolve,
                _material_color(vis, model),
            )
            if desc is not None:
                body.visuals.append(desc)

    # --- inertia override (done after all shapes are added)
    for link in model.links:
        ln = link.name
        if owner[ln] != ln or (ln == base_link and fixed_base):
            continue
        inertial = link.inertial
        if inertial is not None and inertial.mass and inertial.mass > 0:
            io = _origin(inertial.origin)
            rot = io[:3, :3]
            inertia = rot @ np.asarray(inertial.inertia, dtype=float) @ rot.T
            robot.links[ln].set_mass_data(float(inertial.mass), io[:3, 3], inertia)
        elif robot.links[ln].mass == 0.0:
            warnings.warn(
                f"Link {ln} is a dynamic link with neither inertia nor collision. "
                "Setting a tiny mass (1g).",
                stacklevel=2,
            )
            robot.links[ln].set_mass_data(1e-3, (0, 0, 0), np.eye(3) * 1e-6)

    # --- create joints
    for j in model.joints:
        if owner[j.child] != j.child:
            continue  # already-merged fixed joint
        parent_body = robot.links[j.parent]
        child_body = robot.links[j.child]
        t_child = t_world[j.child]
        anchor = t_child[:3, 3]
        axis_local = np.asarray(j.axis if j.axis is not None else (1.0, 0.0, 0.0), dtype=float)
        axis_world = t_child[:3, :3] @ axis_local

        if j.type in ("revolute", "continuous"):
            limits = None
            if j.type == "revolute" and j.limit is not None and j.limit.lower is not None:
                lower, upper = float(j.limit.lower), float(j.limit.upper)
                if lower < -_LIMIT_MAX or upper > _LIMIT_MAX:
                    warnings.warn(
                        f"Joint {j.name}'s limit [{lower:.2f}, {upper:.2f}] exceeds "
                        f"box3d's ±0.99π maximum, so it is disabled.",
                        stacklevel=2,
                    )
                else:
                    limits = (lower, upper)
            joint = world.create_revolute_joint(
                parent_body, child_body, anchor, axis_world, limits=limits
            )
            robot.actuated.append(j.name)
        elif j.type == "prismatic":
            limits = None
            if j.limit is not None and j.limit.lower is not None:
                limits = (float(j.limit.lower), float(j.limit.upper))
            joint = world.create_prismatic_joint(
                parent_body, child_body, anchor, axis_world, limits=limits
            )
            robot.actuated.append(j.name)
        elif j.type == "fixed":
            joint = world.create_weld_joint(parent_body, child_body, anchor=anchor)
        elif j.type == "floating":
            continue  # unconstrained (free body)
        else:
            raise NotImplementedError(f"Unsupported joint type: {j.type} ({j.name})")

        robot.joints[j.name] = joint
        robot.joint_types[j.name] = j.type
        robot.effort_limits[j.name] = (
            float(j.limit.effort) if j.limit is not None and j.limit.effort else None
        )
        # URDF limits are used for command clamping (independent of box3d's ±0.99π constraint)
        if j.type != "continuous" and j.limit is not None and j.limit.lower is not None:
            robot.position_limits[j.name] = (float(j.limit.lower), float(j.limit.upper))
        else:
            robot.position_limits[j.name] = None

    # --- subtrees (for gravity compensation). Merged links have zero mass, so they can be excluded
    def _own_descendants(link_name: str) -> list[str]:
        acc = [link_name] if owner[link_name] == link_name else []
        for cj in children.get(link_name, []):
            acc += _own_descendants(cj.child)
        return acc

    for jn in robot.actuated:
        child_link = next(j.child for j in model.joints if j.name == jn)
        robot._subtrees[jn] = [robot.links[ln] for ln in _own_descendants(child_link)]

    return robot
