"""Thin FFI layer over box3d (layer 1).

Exposes `ffi` / `lib` (the box3d C API) / `shim` (the batch API). No logic lives
here (to isolate the cost of tracking API versions).

Shared library search order:
1. the ROBOX3D_LIB_DIR environment variable (directory holding libbox3d and the shim)
2. the packaged robox3d/_lib/ (for wheel installs)
3. an in-repo build under build/native/ or build/skbuild/ (for development)
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

from cffi import FFI

from ._cdef import CDEF
from ._shim_cdef import SHIM_CDEF

ffi = FFI()
ffi.cdef(CDEF)
ffi.cdef(SHIM_CDEF)

_SUFFIX = ".dylib" if sys.platform == "darwin" else ".so"
_BOX3D_NAME = f"libbox3d{_SUFFIX}"
_SHIM_NAME = f"librobox3d_shim{_SUFFIX}"


def _candidate_dirs() -> list[Path]:
    dirs = []
    env = os.environ.get("ROBOX3D_LIB_DIR")
    if env:
        dirs.append(Path(env))

    pkg_dir = Path(__file__).resolve().parent.parent
    dirs.append(pkg_dir / "_lib")

    # editable install: the Python sources live in src/, but the CMake artifacts
    # go to site-packages/robox3d/_lib, so also search sys.path
    for entry in sys.path:
        candidate = Path(entry) / "robox3d" / "_lib"
        if candidate != pkg_dir / "_lib":
            dirs.append(candidate)

    # development: the CMake build tree at the repo root (a manual cmake build)
    repo = pkg_dir.parents[1]
    if (repo / "pyproject.toml").exists():
        for build_dir in (repo / "build").glob("*"):
            dirs.append(build_dir)
            dirs.append(build_dir / "bin")
            dirs.append(build_dir / "external" / "box3d" / "bin")
    return dirs


def _find_lib(name: str, dirs: list[Path], required: bool) -> str | None:
    for d in dirs:
        path = d / name
        if path.exists():
            return str(path)
    if required:
        raise OSError(
            f"{name} not found (searched: {[str(d) for d in dirs]}). "
            "Build it with `cmake -S . -B build/native && cmake --build build/native -j`, "
            "or point ROBOX3D_LIB_DIR at its location."
        )
    return None


_dirs = _candidate_dirs()
_box3d_path = _find_lib(_BOX3D_NAME, _dirs, required=True)
_shim_path = _find_lib(_SHIM_NAME, _dirs, required=False)

# the shim depends on libbox3d.so.0, so load that first with RTLD_GLOBAL to resolve it
lib = ffi.dlopen(_box3d_path, ffi.RTLD_NOW | ffi.RTLD_GLOBAL)
shim = ffi.dlopen(_shim_path) if _shim_path else None

__all__ = ["ffi", "lib", "shim"]
