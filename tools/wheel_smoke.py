"""Minimal smoke test executed inside cibuildwheel's test environment."""

import numpy as np

import robox3d


def main() -> None:
    with robox3d.World() as world:
        ground = world.create_body(kind="static")
        ground.add_box((5, 5, 0.1), offset=(0, 0, -0.1))
        ball = world.create_body(position=(0, 0, 1))
        ball.add_sphere(0.1)
        for _ in range(240):
            world.step(1 / 240)
        z = ball.position[2]
        assert np.isfinite(z) and z < 1.0, f"unexpected ball height: {z}"

    # Bundled SO-ARM101 model loads (exercises yourdfpy + mesh convex hulls)
    with robox3d.World() as world:
        robot = robox3d.load_urdf(world, robox3d.assets.so101())
        assert robot.dof == 6
    print("wheel smoke test OK")


if __name__ == "__main__":
    main()
