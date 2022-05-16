"""Falling box: the minimal robox3d sample.

Serves the 3D viewer by default — open the printed URL in a browser. The box
re-drops every few seconds so there is always something to watch.
Run with --headless for a quick numeric run without the viewer.
"""

import argparse
import sys
import time

import robox3d


def main() -> None:
    sys.stdout.reconfigure(line_buffering=True)  # keep progress visible when piped
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--headless", action="store_true", help="run fast without the viewer and exit"
    )
    parser.add_argument("--port", type=int, default=8765, help="viewer HTTP/WebSocket port")
    args = parser.parse_args()

    with robox3d.World() as world:
        ground = world.create_body(kind="static")
        ground.add_box(half_extents=(10.0, 10.0, 0.1), offset=(0.0, 0.0, -0.1))

        box = world.create_body(position=(0.0, 0.0, 2.0))
        box.add_box(half_extents=(0.25, 0.25, 0.25), density=500.0)

        server = None
        if not args.headless:
            try:
                from robox3d.viz import VizServer

                server = VizServer(world, port=args.port)
                server.start()  # after all bodies are created
                print(f"Viewer: {server.url} — open in a browser (Ctrl+C to quit)")
            except ImportError:
                print('viz not installed (pip install "robox3d[viz]") — running headless')

        dt = 1 / 240
        start = time.monotonic()
        steps = 0

        def step() -> None:
            nonlocal steps
            world.step(dt)
            steps += 1
            if server:
                server.update()
                time.sleep(max(0.0, steps * dt - (time.monotonic() - start)))

        try:
            for i in range(240):
                step()
                if i % 60 == 0:
                    x, y, z = box.position
                    print(f"t={i / 240:.2f}s  z={z:6.3f}m  v={box.linear_velocity[2]:7.3f}m/s")

            z = box.position[2]
            print(f"final position z={z:.4f}m (expected: at rest at half the box height, 0.25m)")

            if server:
                print("Re-dropping the box every 2.5s — Ctrl+C to quit")
                next_drop = world.time + 1.5
                while True:
                    if world.time >= next_drop:
                        box.set_pose((0.0, 0.0, 2.0))
                        box.linear_velocity = (0.0, 0.0, 0.0)
                        box.angular_velocity = (0.0, 0.0, 0.0)
                        next_drop = world.time + 2.5
                    step()
        except KeyboardInterrupt:
            pass
        finally:
            if server:
                server.stop()


if __name__ == "__main__":
    main()
