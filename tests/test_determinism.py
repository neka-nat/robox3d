"""Determinism regression test (a reduced version of validation-report.md experiment 5).

Continuously monitors the byte-for-byte reproducibility that RL and CI regression rely on.
"""

import hashlib

import robox3d
from conftest import build_chain

N_STEPS = 500


def run_hash(worker_count: int) -> str:
    with robox3d.World(worker_count=worker_count) as world:
        bodies, joints = build_chain(world, n_links=6)
        group = robox3d.BodyGroup(bodies)
        h = hashlib.sha256()
        for _ in range(N_STEPS):
            world.step(1 / 240)
            h.update(group.poses().tobytes())
        return h.hexdigest()


def test_repeatable_single_thread():
    assert run_hash(1) == run_hash(1)


def test_repeatable_multi_thread():
    assert run_hash(2) == run_hash(2)


def test_thread_count_invariant():
    assert run_hash(1) == run_hash(2)
