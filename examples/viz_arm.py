"""Live visualization of the 6-axis arm (Phase 4).

Steps:
  1. Start this script:  uv run python examples/viz_arm.py
  2. Open the printed URL (http://localhost:8765) in a browser

The viewer is served directly by VizServer (build it once with
`python tools/build_viewer.py` if you cloned the repo). For viewer
development use `cd viewer && pnpm dev` and open http://localhost:5173.

Modes:
  default         : drive with the viewer's joint sliders (bidirectional commands)
  --trajectory    : automatic sinusoidal-trajectory demo
  --record out.rbx: also record poses to a file
                    (replay: uv run python -m robox3d.viz.record out.rbx)
"""

import argparse
import time
from pathlib import Path

import numpy as np

import robox3d
from robox3d.viz import PoseRecorder, VizServer

ARM_URDF = Path(__file__).parent / "assets" / "simple_arm.urdf"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trajectory", action="store_true", help="automatic sinusoidal-trajectory demo")
    parser.add_argument("--record", type=Path, default=None)
    parser.add_argument("--duration", type=float, default=float("inf"))
    parser.add_argument("--port", type=int, default=8765, help="viewer HTTP/WebSocket port")
    args = parser.parse_args()

    with robox3d.World() as world:
        ground = world.create_body(kind="static", name="ground")
        ground.add_box((2, 2, 0.05), offset=(0, 0, -0.05))

        robot = robox3d.load_urdf(world, ARM_URDF)
        robot.enable_position_control(hertz=120.0)

        # A few falling boxes (contact demo)
        for i in range(5):
            box = world.create_body(
                position=(0.5 + 0.15 * i, 0.6, 1.0 + 0.4 * i), name=f"box_{i}"
            )
            box.add_box((0.06, 0.06, 0.06), density=300.0)
            box.color = "#e8734c" if i % 2 else "#5fbf7a"

        server = VizServer(world, robot=robot, port=args.port)
        server.start()
        recorder = PoseRecorder(world, args.record) if args.record else None
        mode = "auto trajectory" if args.trajectory else "slider control"
        print(f"Serving viewer at {server.url} [{mode}] — open it in a browser")

        amp = np.array([0.8, 0.4, 0.5, 0.6, 0.6, 0.8])
        dt = 1 / 240
        start = time.monotonic()
        try:
            i = 0
            while world.time < args.duration:
                if args.trajectory:
                    robot.set_targets(amp * np.sin(2 * np.pi * 0.15 * world.time))
                world.step(dt)
                server.update()  # broadcast poses + apply slider commands
                if recorder:
                    recorder.update()
                i += 1
                # real-time pacing
                sleep = i * dt - (time.monotonic() - start)
                if sleep > 0:
                    time.sleep(sleep)
        except KeyboardInterrupt:
            pass
        finally:
            server.stop()
            if recorder:
                recorder.close()


if __name__ == "__main__":
    main()
