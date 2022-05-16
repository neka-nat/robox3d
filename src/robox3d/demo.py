"""Runnable demos for quick starts: ``python -m robox3d.demo <name>``.

Demos run the physics headless and serve the bundled web viewer over HTTP —
open the printed URL in a browser and drive the robot with the joint sliders.
"""

from __future__ import annotations

import argparse
import time

import robox3d
from robox3d import assets


def _serve(world: robox3d.World, robot, port: int, duration: float, label: str) -> None:
    from robox3d.viz import VizServer

    server = VizServer(world, robot=robot, port=port)
    server.start()
    print(f"{label} — open {server.url} in a browser (Ctrl+C to quit)")
    dt = 1 / 240
    steps = 0
    start = time.monotonic()
    try:
        while world.time < duration:
            world.step(dt)
            server.update()
            steps += 1
            # real-time pacing
            sleep = steps * dt - (time.monotonic() - start)
            if sleep > 0:
                time.sleep(sleep)
    except KeyboardInterrupt:
        pass
    finally:
        server.stop()


def so101(port: int = 8765, duration: float = float("inf")) -> None:
    """SO-ARM101 (TheRobotStudio SO-101) with interactive joint sliders."""
    with robox3d.World() as world:
        ground = world.create_body(kind="static", name="ground")
        ground.add_box((0.6, 0.6, 0.02), offset=(0, 0, -0.02))
        robot = robox3d.load_urdf(world, assets.so101())
        # Stiff springs so the small hobby-servo links track the sliders crisply.
        # enable_position_control's default constraint_hertz=60 avoids the
        # parallel-hinge tracking leak (docs/spring-chain-investigation.md).
        robot.enable_position_control(hertz=240.0)
        _serve(world, robot, port, duration, "SO-ARM101 teleop demo")


DEMOS = {"so101": so101}


def main() -> None:
    parser = argparse.ArgumentParser(
        prog="python -m robox3d.demo",
        description="Run a robox3d demo and serve the web viewer.",
    )
    parser.add_argument("name", choices=sorted(DEMOS), help="demo to run")
    parser.add_argument("--port", type=int, default=8765, help="HTTP/WebSocket port")
    parser.add_argument(
        "--duration", type=float, default=float("inf"), help="sim seconds to run (default: forever)"
    )
    args = parser.parse_args()
    DEMOS[args.name](port=args.port, duration=args.duration)


if __name__ == "__main__":
    main()
