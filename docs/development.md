# Development guide

## Build from source

Use Python 3.10 or newer, uv, a C17 compiler, and CMake 3.22 or newer. Building
the browser viewer also requires Node.js and pnpm; the repository workflows
use Node.js 22 and pnpm 9.

```bash
git clone --recursive https://github.com/neka-nat/robox3d
cd robox3d
uv sync
uv run pytest tests/
uv run python tools/build_viewer.py
uv run python examples/viz_arm.py
```

For an existing checkout without the Box3D sources, run
`git submodule update --init --recursive` before building. The submodule commit
pins the engine version. scikit-build-core and CMake build Box3D and the C shim
and package their shared libraries with Python.

Examples serve the browser viewer by default. Use `--headless` for a finite
numeric run, or `--port` to select a different viewer port:

```bash
uv run python examples/falling_box.py --headless
uv run python examples/arm_trajectory.py --headless
```

## Viewer development

The viewer sources are in [viewer/](../viewer/). Build and type-check them with:

```bash
cd viewer
pnpm install --frozen-lockfile
pnpm build
```

From the repository root, `uv run python tools/build_viewer.py` installs the
viewer dependencies, builds the bundle, and copies it into
`src/robox3d/viz/static/` for serving through `VizServer`. Run this after viewer
changes when testing the bundled viewer. The generated files are not tracked
in Git.

## Source layout

| Location | Responsibility |
|---|---|
| [src/robox3d/_ffi/](../src/robox3d/_ffi/), [csrc/](../csrc/) | Generated cffi bindings and batch C shim |
| [src/robox3d/core/](../src/robox3d/core/) | Worlds, bodies, joints, and batch groups |
| [src/robox3d/model/](../src/robox3d/model/) | URDF loading, inertia, and geometry conversion |
| [src/robox3d/control/](../src/robox3d/control/) | Spring gain conversion, torque control, and gravity compensation |
| [src/robox3d/sensors/](../src/robox3d/sensors/) | F/T, IMU, LiDAR, and contact sensors |
| [src/robox3d/viz/](../src/robox3d/viz/), [viewer/](../viewer/) | Pose streaming, recording, and browser rendering |
| [tests/](../tests/), [examples/](../examples/) | Regression tests and runnable examples |

## Regenerate bindings

When changing the pinned Box3D version, regenerate the cffi declarations from
its C headers and review the generated diff:

```bash
uv run python tools/gen_ffi.py
uv run pytest tests/
```

The generator invokes the `gcc` C preprocessor. Its generated declarations
are tracked in Git. Review the shim and Python wrappers for affected APIs as
part of the same update.

[Back to documentation](README.md)
