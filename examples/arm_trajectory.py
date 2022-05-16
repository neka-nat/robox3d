"""6-axis arm URDF loading and trajectory tracking (Phase 2 milestone).

Serves the 3D viewer by default — open the printed URL in a browser; the
sinusoidal trajectory keeps running after the accuracy report.
Run with --headless for a quick numeric run without the viewer.
"""

import argparse
import sys
import time
from pathlib import Path

import numpy as np

import robox3d

ARM_URDF = Path(__file__).parent / "assets" / "simple_arm.urdf"


def main() -> None:
    sys.stdout.reconfigure(line_buffering=True)  # keep progress visible when piped
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--headless", action="store_true", help="run fast without the viewer and exit"
    )
    parser.add_argument("--port", type=int, default=8765, help="viewer HTTP/WebSocket port")
    args = parser.parse_args()

    with robox3d.World() as world:
        robot = robox3d.load_urdf(world, ARM_URDF)
        print(f"robot: {robot.name} ({robot.dof} DOF)")
        print(f"actuated joints: {robot.actuated}")

        robot.enable_position_control(hertz=120.0, damping_ratio=1.0)

        server = None
        if not args.headless:
            try:
                from robox3d.viz import VizServer

                server = VizServer(world, robot=robot, port=args.port)
                server.start()  # after all bodies are created
                print(f"Viewer: {server.url} — open in a browser (Ctrl+C to quit)")
            except ImportError:
                print('viz not installed (pip install "robox3d[viz]") — running headless')

        amp = np.array([0.5, 0.4, 0.5, 0.6, 0.6, 0.8])
        freq = 0.25  # Hz
        dt = 1 / 240
        start = time.monotonic()
        steps = 0

        def step() -> None:
            nonlocal steps
            robot.set_targets(amp * np.sin(2 * np.pi * freq * world.time))
            world.step(dt)
            steps += 1
            if server:
                server.update()
                time.sleep(max(0.0, steps * dt - (time.monotonic() - start)))

        try:
            max_err = 0.0
            for i in range(4 * 240):
                step()
                t = world.time
                if t > 0.5:
                    q_des = amp * np.sin(2 * np.pi * freq * (t - dt))
                    max_err = max(max_err, np.abs(robot.positions() - q_des).max())
                if i % 240 == 239:
                    q = np.degrees(robot.positions())
                    print(f"t={t:.0f}s  q=[{', '.join(f'{v:6.1f}' for v in q)}] deg")

            print(f"max tracking error (t>0.5s): {np.degrees(max_err):.3f}°")
            ee = robot.link_body("wrist_3_link").position
            print(f"end-effector position: {np.round(ee, 3)}")

            if server:
                print("Trajectory keeps running — Ctrl+C to quit")
                while True:
                    step()
        except KeyboardInterrupt:
            pass
        finally:
            if server:
                server.stop()


if __name__ == "__main__":
    main()
