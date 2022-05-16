import numpy as np
import pytest

import robox3d


def test_ffi_loads():
    from robox3d._ffi import lib, shim

    v = lib.b3GetVersion()
    assert (v.major, v.minor) == (0, 1)
    assert shim is not None


def test_free_fall_matches_analytic(world):
    body = world.create_body(position=(0, 0, 10))
    body.add_sphere(0.1)
    dt = 1 / 240
    for _ in range(240):
        world.step(dt)
    # discrete analytic solution of semi-implicit Euler (integration is done per substep):
    # z = z0 - g*dt_sub^2*n(n+1)/2, dt_sub = dt/substeps, n = 240*substeps
    n = 240 * world.substeps
    dt_sub = dt / world.substeps
    expected = 10.0 - 9.81 * dt_sub * dt_sub * n * (n + 1) / 2
    assert body.position[2] == pytest.approx(expected, abs=1e-3)


def test_box_rests_on_ground(world):
    ground = world.create_body(kind="static")
    ground.add_box((5, 5, 0.1), offset=(0, 0, -0.1))
    box = world.create_body(position=(0, 0, 1.0))
    box.add_box((0.25, 0.25, 0.25))
    for _ in range(480):
        world.step(1 / 240)
    assert box.position[2] == pytest.approx(0.25, abs=5e-3)
    assert np.linalg.norm(box.linear_velocity) < 1e-3


def test_substeps_warning():
    with pytest.warns(robox3d.UnstableSimulationWarning):
        w = robox3d.World(substeps=1)
        w.destroy()


def test_context_manager_destroys():
    with robox3d.World() as w:
        pass
    assert w._destroyed


def test_cast_ray(world):
    ground = world.create_body(kind="static")
    ground.add_box((5, 5, 0.1), offset=(0, 0, -0.1))
    hit = world.cast_ray_closest(origin=(0, 0, 2), translation=(0, 0, -5))
    assert hit is not None
    point, normal, fraction = hit
    assert point[2] == pytest.approx(0.0, abs=1e-4)
    assert normal[2] == pytest.approx(1.0, abs=1e-4)
    miss = world.cast_ray_closest(origin=(0, 0, 2), translation=(0, 0, 5))
    assert miss is None
