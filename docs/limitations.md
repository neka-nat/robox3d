# Limitations and control tuning

robox3d uses Box3D's maximal-coordinate rigid-body solver. Tracking accuracy,
constraint drift, and stability depend on geometry, mass ratios, time step,
and control settings. Check the behavior under the loads and motions your
application will use.

## Time step and substeps

`World` defaults to four substeps, and `world.step()` defaults to a time step
of `1 / 240` seconds. Use at least four substeps for articulated systems.
Constructing `World(substeps=...)` with a value below four emits
`UnstableSimulationWarning`; it does not reject the value. Smaller time steps
alone do not replace substepping for stiff chains.

## Joint limits

The engine's revolute limits are restricted to ±0.99π. If a URDF joint has a
wider range, the loader warns and disables its physical limit. Targets set
through `Robot.set_targets()` are still clamped to the URDF limits.

Physical limits are soft constraints, so loads or stiff springs can push a
joint beyond them. Command clamping does not prevent displacement caused by
external forces.

## Position control

`Robot.enable_position_control()` accepts spring frequency and damping, or
physical-unit `kp` / `kd` gains. Spring gains use the effective inertia of the
two adjacent bodies, which does not include the full downstream chain.
Payloads and large mass ratios therefore affect the response.

The `constraint_hertz` setting controls pivot and axis-alignment stiffness
separately from the position-control spring frequency:

```python
robot.enable_position_control(hertz=120.0, constraint_hertz=60.0)
```

The position-control default is 60 Hz with constraint damping ratio 2.0.
Passing `constraint_hertz=None` preserves the joints' existing tuning. The
low-level world joint factories default to 240 Hz before position control
applies its tuning.

On serial chains with parallel hinges bracketed by perpendicular hinges,
axis-alignment corrections can leave a persistent spring tracking error.
Lowering `constraint_hertz` can reduce that error but also lets pivots separate
more under load. A value around 20 Hz can be a starting point for tuning when
tracking matters more than pivot rigidity; it is not an accuracy guarantee.
Measure both joint error and pivot separation for your model and payload.

## Torque and effort limits

Torque control uses a velocity motor with a large target speed in the command
direction and a torque cap equal to the command magnitude. Call
`Robot.set_torques()` every simulation step after enabling torque control.
These motor commands are clamped to the URDF effort limits.

Position-control springs do not enforce effort limits. Combining spring
feedback with torque feedforward does not cap the total joint effort, because
the spring contribution remains unconstrained. Gravity compensation provides
static feedforward; it does not account for all dynamic effects.

## Collision geometry

Dynamic bodies use convex collision shapes. URDF meshes default to one convex
hull per link, which fills concavities. Use `mesh_mode="coacd"` with the
`coacd` extra for convex decomposition when needed. Visual meshes are separate
from collision geometry; use the viewer's collision toggle to inspect the
shapes used by the solver. Robot self-collision is disabled by default.

## Reproducibility and performance

The [determinism regression tests](../tests/test_determinism.py) compare body
pose bytes across repeated runs and selected thread counts on a small chain.
This does not establish reproducibility for every scene, operating system,
CPU, or engine version. Keep the model, inputs, simulation settings, and
software versions fixed when checking repeatability.

Throughput depends on the scene, contacts, substeps, and Python-side work.
Measure your own workload, including control and sensor updates. The
`BodyGroup` and `RevoluteGroup` APIs reduce per-body and per-joint FFI calls.

[Back to documentation](README.md)
