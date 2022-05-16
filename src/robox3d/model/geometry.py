"""Convert URDF collision geometry to box3d shapes (layer 2).

Approach (see validation-report / development-plan):
- box: turned directly into a hull (b3MakeTransformedBoxHull)
- sphere: used as-is
- cylinder: a 16-sided prism convex hull by default ("hull"); "capsule" is also available
- mesh: a single convex hull by default ("hull"); "coacd" does convex decomposition
  (requires coacd; results are cached)

Dynamic bodies cannot use mesh collision (a box3d limitation), so everything is
converted to convex shapes.
"""

from __future__ import annotations

import hashlib
import os
import warnings
from pathlib import Path

import numpy as np

from .._ffi import ffi, lib
from ..core.body import Body, _vec3
from ..math import matrix_to_quat

CYLINDER_SEGMENTS = 16


def _transform_points(xf: np.ndarray, points: np.ndarray) -> np.ndarray:
    return points @ xf[:3, :3].T + xf[:3, 3]


def _set_group_index(shape_id, group_index: int) -> None:
    if group_index == 0:
        return
    f = lib.b3Shape_GetFilter(shape_id)
    f.groupIndex = group_index
    lib.b3Shape_SetFilter(shape_id, f, False)


def _cylinder_points(radius: float, length: float) -> np.ndarray:
    """Convex-hull vertices for a cylinder along the Z axis (URDF cylinders are centered at the origin and Z-aligned)."""
    theta = np.linspace(0.0, 2 * np.pi, CYLINDER_SEGMENTS, endpoint=False)
    ring = np.column_stack([radius * np.cos(theta), radius * np.sin(theta)])
    top = np.column_stack([ring, np.full(CYLINDER_SEGMENTS, length / 2)])
    bottom = np.column_stack([ring, np.full(CYLINDER_SEGMENTS, -length / 2)])
    return np.vstack([top, bottom])


def _cache_dir() -> Path:
    root = os.environ.get("XDG_CACHE_HOME", str(Path.home() / ".cache"))
    d = Path(root) / "robox3d" / "coacd"
    d.mkdir(parents=True, exist_ok=True)
    return d


def _coacd_hulls(mesh, cache_key: str, threshold: float) -> list[np.ndarray]:
    """Run CoACD convex decomposition and return the vertex array for each part (with a file cache)."""
    cache_file = _cache_dir() / f"{cache_key}.npz"
    if cache_file.exists():
        data = np.load(cache_file)
        return [data[k] for k in sorted(data.files)]

    try:
        import coacd
    except ImportError as e:
        raise ImportError(
            "mesh_mode='coacd' requires coacd: uv add coacd"
        ) from e

    cmesh = coacd.Mesh(np.asarray(mesh.vertices), np.asarray(mesh.faces))
    parts = coacd.run_coacd(cmesh, threshold=threshold)
    hulls = [np.asarray(verts, dtype=np.float32) for verts, _faces in parts]
    np.savez_compressed(cache_file, **{f"{i:04d}": h for i, h in enumerate(hulls)})
    return hulls


def load_mesh(filename: str, scale) -> "trimesh.Trimesh":
    import trimesh

    mesh = trimesh.load(filename, force="mesh")
    if scale is not None:
        mesh.apply_scale(scale)
    return mesh


# ---------------------------------------------------------------- visual (visualization-only geometry)


def _xf_dict(xf: np.ndarray) -> dict:
    q = matrix_to_quat(xf[:3, :3])
    return {"p": [float(v) for v in xf[:3, 3]], "q": [float(v) for v in q]}


def visual_descriptor(geometry, xf: np.ndarray, resolve, color: str | None) -> dict | None:
    """Convert a yourdfpy Geometry (visual element) into a descriptor dict for the viewer.

    Meshes are converted to GLB and embedded as base64 (scale is baked in, pose is
    sent via xf). If conversion fails, warn and return None (the viewer falls back
    to showing collision shapes).
    """
    import base64

    common = {"xf": _xf_dict(xf), "color": color}
    if geometry.box is not None:
        return {"kind": "box", "size": [float(v) for v in geometry.box.size], **common}
    if geometry.sphere is not None:
        return {"kind": "sphere", "radius": float(geometry.sphere.radius), **common}
    if geometry.cylinder is not None:
        return {
            "kind": "cylinder",
            "radius": float(geometry.cylinder.radius),
            "length": float(geometry.cylinder.length),
            **common,
        }
    if geometry.mesh is not None:
        try:
            mesh = load_mesh(resolve(geometry.mesh.filename), geometry.mesh.scale)
            glb = mesh.export(file_type="glb")
        except Exception as e:  # e.g. .dae when pycollada is not installed
            warnings.warn(
                f"Cannot load visual mesh {geometry.mesh.filename} ({e}). "
                "Falling back to collision shapes for display.",
                stacklevel=2,
            )
            return None
        return {"kind": "mesh", "glb": base64.b64encode(glb).decode(), **common}
    return None


def add_collision_geometry(
    body: Body,
    geometry,
    xf: np.ndarray,
    *,
    resolve,
    density: float = 1000.0,
    friction: float = 0.8,
    cylinder_mode: str = "hull",
    mesh_mode: str = "hull",
    coacd_threshold: float = 0.05,
    max_hull_vertices: int = 32,
    group_index: int = 0,
) -> list:
    """Add a yourdfpy Geometry to a body.

    xf: homogeneous transform (4x4), body frame <- geometry frame
    resolve: function resolving mesh file names (str -> str)
    Returns: list of created shape ids
    """
    shapes = []

    if geometry.box is not None:
        hx, hy, hz = np.asarray(geometry.box.size, dtype=float) / 2
        cxf = ffi.new("b3Transform*")
        cxf.p = _vec3(xf[:3, 3])
        q = matrix_to_quat(xf[:3, :3])
        cxf.q.v.x, cxf.q.v.y, cxf.q.v.z, cxf.q.s = q
        hull = lib.b3MakeTransformedBoxHull(hx, hy, hz, cxf[0])
        sd = body._shape_def(density, friction, 0.0)
        hull_ref = ffi.new("b3BoxHull*", hull)
        shapes.append(
            lib.b3CreateHullShape(body.id, ffi.addressof(sd), ffi.addressof(hull_ref.base))
        )

    elif geometry.sphere is not None:
        shapes.append(
            body.add_sphere(
                geometry.sphere.radius, center=xf[:3, 3], density=density, friction=friction
            )
        )

    elif geometry.cylinder is not None:
        r, length = geometry.cylinder.radius, geometry.cylinder.length
        if cylinder_mode == "capsule" and length > 2 * r:
            half = length / 2 - r
            ends = _transform_points(xf, np.array([[0, 0, half], [0, 0, -half]]))
            shapes.append(
                body.add_capsule(ends[0], ends[1], r, density=density, friction=friction)
            )
        else:
            pts = _transform_points(xf, _cylinder_points(r, length))
            shapes.append(
                body.add_hull(
                    pts, density=density, friction=friction, max_vertices=max_hull_vertices
                )
            )

    elif geometry.mesh is not None:
        filename = resolve(geometry.mesh.filename)
        mesh = load_mesh(filename, geometry.mesh.scale)
        if mesh_mode == "coacd":
            key_src = Path(filename).read_bytes() + repr(
                (geometry.mesh.scale, coacd_threshold)
            ).encode()
            cache_key = hashlib.sha256(key_src).hexdigest()[:32]
            for hull_pts in _coacd_hulls(mesh, cache_key, coacd_threshold):
                pts = _transform_points(xf, hull_pts)
                shapes.append(
                    body.add_hull(
                        pts, density=density, friction=friction, max_vertices=max_hull_vertices
                    )
                )
        else:
            pts = _transform_points(xf, np.asarray(mesh.convex_hull.vertices))
            shapes.append(
                body.add_hull(
                    pts, density=density, friction=friction, max_vertices=max_hull_vertices
                )
            )

    else:
        warnings.warn(f"Skipped unsupported geometry: {geometry}", stacklevel=2)

    for s in shapes:
        _set_group_index(s, group_index)
    return shapes
