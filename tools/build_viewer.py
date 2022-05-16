"""Build the web viewer and bundle it into the Python package.

Runs `pnpm install` + `pnpm build` in viewer/ and copies the output to
src/robox3d/viz/static/, where VizServer serves it over HTTP on the same
port as the WebSocket stream. Run this before building release wheels so
`pip install robox3d[viz]` ships the viewer.

Usage:
    python tools/build_viewer.py
"""

from __future__ import annotations

import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
VIEWER_DIR = ROOT / "viewer"
STATIC_DIR = ROOT / "src" / "robox3d" / "viz" / "static"


def main() -> int:
    pnpm = shutil.which("pnpm")
    if pnpm is None:
        print("error: pnpm not found — install it from https://pnpm.io", file=sys.stderr)
        return 1

    print(f"Building viewer in {VIEWER_DIR} ...")
    subprocess.run([pnpm, "install", "--frozen-lockfile"], cwd=VIEWER_DIR, check=True)
    subprocess.run([pnpm, "build"], cwd=VIEWER_DIR, check=True)

    dist = VIEWER_DIR / "dist"
    if not (dist / "index.html").is_file():
        print(f"error: {dist}/index.html not found after build", file=sys.stderr)
        return 1

    if STATIC_DIR.exists():
        shutil.rmtree(STATIC_DIR)
    shutil.copytree(dist, STATIC_DIR)
    n_files = sum(1 for p in STATIC_DIR.rglob("*") if p.is_file())
    print(f"Copied {n_files} files to {STATIC_DIR}")
    print("VizServer will now serve the viewer at its http:// URL.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
