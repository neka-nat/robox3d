"""Pendulum and spring position-control sample.

1. Free pendulum: released from horizontal and left to swing
2. Position control: spring step response to a 45° target angle
3. (viewer mode) the target then sweeps ±45° around the hanging pose forever

Serves the 3D viewer by default — open the printed URL in a browser.
Run with --headless for a quick numeric run without the viewer.
"""

import argparse
import sys
import time

import numpy as np

import robox3d
from robox3d.math import quat_from_axis_angle


def main() -> None:
    sys.stdout.reconfigure(line_buffering=True)  # keep progress visible when piped
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--headless", action="store_true", help="run fast without the viewer and exit"
    )
    parser.add_argument("--port", type=int, default=8765, help="viewer HTTP/WebSocket port")
    args = parser.parse_args()

    with robox3d.World() as world:
        anchor_body = world.create_body(kind="static")

        # Link: 1m from the origin along -Z. Rotate -90° about Y for a horizontal pose facing +X
        rot = quat_from_axis_angle([0, 1, 0], -np.pi / 2)
        link = world.create_body(position=(0, 0, 0), rotation=rot)
        link.add_capsule((0, 0, -0.05), (0, 0, -0.95), radius=0.05, density=340.0)

        joint = world.create_revolute_joint(
            anchor_body, link, anchor=(0, 0, 0), axis=(0, 1, 0)
        )

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
            print("--- Free pendulum (horizontal release) ---")
            for i in range(480):
                step()
                if i % 120 == 119:
                    print(f"t={(i + 1) / 240:.1f}s  angle={np.degrees(joint.angle):7.1f}°")

            print("--- Spring position control: target 45° ---")
            joint.enable_spring(hertz=60.0, damping_ratio=1.0)
            joint.target_angle = np.radians(45.0)
            for i in range(240):
                step()
                if i % 60 == 59:
                    print(
                        f"t={(i + 1) / 240:.2f}s  angle={np.degrees(joint.angle):6.2f}°"
                        f"  speed={joint.speed:7.3f} rad/s"
                    )

            err = np.degrees(joint.angle) - 45.0
            print(f"steady-state error: {err:+.3f}° "
                  "(from gravity torque; decreases as hertz increases)")

            if server:
                print("--- Driving the target ±45° around the hanging pose — Ctrl+C to quit ---")
                t0 = world.time
                while True:
                    # Joint angle 0 is the horizontal release pose, so straight
                    # down is +90°. Oscillate ±45° around it, starting smoothly
                    # from the 45° pose the step response ended at.
                    joint.target_angle = np.radians(90.0) - np.radians(45.0) * np.cos(
                        2 * np.pi * 0.15 * (world.time - t0)
                    )
                    step()
        except KeyboardInterrupt:
            pass
        finally:
            if server:
                server.stop()


if __name__ == "__main__":
    main()
