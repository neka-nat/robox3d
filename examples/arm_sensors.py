"""6-axis arm + sensors + gravity compensation demo (Phase 3).

- Weld-mount a 2kg payload at the end-effector (F/T sensor)
- Hold with gravity-compensation FF + spring FB while printing F/T and IMU readings
- Scan the surrounding walls with a base-mounted LiDAR

Serves the 3D viewer by default — open the printed URL in a browser; the hold
continues with live sensor readings (the joint sliders stay active, so drag
them to see the F/T readings change).
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

        # Payload + F/T sensor mount
        payload = world.create_body(position=(1.05, 0, 0.22))
        payload.add_box((0.05, 0.05, 0.05), density=2000.0)  # 2kg
        mount = world.create_weld_joint(
            robot.link_body("wrist_3_link"), payload, anchor=(1.0, 0, 0.22)
        )
        ft = robox3d.FTSensor(mount, frame="joint")
        imu = robox3d.IMU(payload)

        # Surrounding walls and base-mounted LiDAR (placed within the walls' height range)
        for x, y in ((3, 0), (-3, 0), (0, 3), (0, -3)):
            wall = world.create_body(kind="static", position=(x, y, 1.0))
            half = (0.1, 3, 1) if x else (3, 0.1, 1)
            wall.add_box(half)
        lidar = robox3d.Lidar(world, offset=(0, 0, 1.5), num_rays=8, max_range=10)

        # Control: gravity-compensation FF (payload included) + spring FB.
        # Holding a heavy end-effector payload favors pivot rigidity over
        # tracking, so raise constraint_hertz (docs/spring-chain-investigation.md).
        robot.enable_position_control(kp=100.0, constraint_hertz=240.0)
        robot.enable_torque_control(disable_springs=False)

        server = None
        if not args.headless:
            try:
                from robox3d.viz import VizServer

                server = VizServer(world, robot=robot, port=args.port)
                server.start()  # after all bodies are created
                print(f"Viewer: {server.url} — open in a browser (Ctrl+C to quit)")
            except ImportError:
                print('viz not installed (pip install "robox3d[viz]") — running headless')

        dt = 1 / 240
        start = time.monotonic()
        steps = 0

        def step() -> None:
            nonlocal steps
            robot.set_torques(robot.gravity_compensation(extra_bodies=[payload]))
            world.step(dt)
            imu.update(dt)
            steps += 1
            if server:
                server.update()
                time.sleep(max(0.0, steps * dt - (time.monotonic() - start)))

        def print_readings() -> None:
            f, t = ft.read()
            a, w = imu.read()
            print(f"t={world.time:.0f}s")
            print(f"  F/T: F={np.round(f, 2)} N  T={np.round(t, 3)} N·m "
                  f"(|F|={np.linalg.norm(f):.1f} vs payload weight {payload.mass * 9.81:.1f})")
            print(f"  IMU: accel={np.round(a, 2)} m/s²  gyro={np.round(w, 4)} rad/s")

        try:
            for i in range(3 * 240):
                step()
                if i % 240 == 239:
                    print_readings()

            err = np.degrees(np.abs(robot.positions())).max()
            print(f"holding error: {err:.3f}° (gravity-compensation FF + spring FB)")
            print(f"LiDAR (8 directions): {np.round(lidar.scan(), 2)} m")

            if server:
                print("Holding — sensor readings every 5s, sliders active, Ctrl+C to quit")
                while True:
                    step()
                    if steps % 1200 == 0:
                        print_readings()
        except KeyboardInterrupt:
            pass
        finally:
            if server:
                server.stop()


if __name__ == "__main__":
    main()
