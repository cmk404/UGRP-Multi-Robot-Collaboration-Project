"""Exact robot/obstacle contact-tick parity, including penetration boundary."""
from __future__ import annotations

from dataclasses import dataclass
from types import SimpleNamespace
import time
import unittest

import mujoco
import numpy as np

from sim.contact_audit_kernel import ContactAuditKernel
from sim.multi_masterpi_production import MultiMasterPiProductionV2


def reference_contact(data, robot_ids: set[int], obstacle_ids: set[int]) -> bool:
    return any(
        (int(c.geom1) in robot_ids and int(c.geom2) in obstacle_ids)
        or (int(c.geom2) in robot_ids and int(c.geom1) in obstacle_ids)
        for c in data.contact[:data.ncon] if c.dist < 0
    )


@dataclass
class FakeContacts:
    geom1: np.ndarray
    geom2: np.ndarray
    dist: np.ndarray

    def __getitem__(self, index):
        indices = range(len(self.dist))[index]
        if isinstance(indices, int):
            indices = (indices,)
        return [SimpleNamespace(geom1=int(self.geom1[i]), geom2=int(self.geom2[i]),
                                dist=float(self.dist[i])) for i in indices]


class ContactAuditKernelTests(unittest.TestCase):
    def test_both_directions_penetration_boundary_and_empty(self):
        robots, obstacles = {1, 3}, {7, 8}
        kernel = ContactAuditKernel(10, robots, obstacles)
        cases = [
            ([], [], [], False),
            ([1], [7], [-.001], True),
            ([8], [3], [-.001], True),
            ([1], [7], [0.0], False),
            ([1], [7], [.001], False),
            ([1], [2], [-.001], False),
            ([1, 2, 8], [2, 7, 3], [-.001, -.002, -.003], True),
        ]
        for g1, g2, dist, expected in cases:
            data = SimpleNamespace(ncon=len(dist), contact=FakeContacts(
                np.asarray(g1, dtype=np.int32), np.asarray(g2, dtype=np.int32),
                np.asarray(dist, dtype=float)))
            self.assertEqual(kernel.has_penetrating_contact(data), expected)
            self.assertEqual(kernel.has_penetrating_contact(data), reference_contact(data, robots, obstacles))

    def test_random_contacts_match_reference(self):
        rng = np.random.default_rng(23)
        robots, obstacles = {1, 3, 5}, {7, 8, 11}
        kernel = ContactAuditKernel(14, robots, obstacles)
        for count in (0, 1, 2, 20, 80):
            for _ in range(30):
                contacts = FakeContacts(
                    rng.integers(0, 14, count), rng.integers(0, 14, count),
                    rng.choice([-.02, 0., .02], count))
                data = SimpleNamespace(ncon=count, contact=contacts)
                self.assertEqual(kernel.has_penetrating_contact(data),
                                 reference_contact(data, robots, obstacles))

    def test_real_mujoco_contact_list_matches_reference(self):
        world = MultiMasterPiProductionV2(seed=11, render=False)
        try:
            robots = {i for i in range(world.model.ngeom)
                      if (mujoco.mj_id2name(world.model, mujoco.mjtObj.mjOBJ_GEOM, i) or '').startswith('r1__')}
            floor = mujoco.mj_name2id(world.model, mujoco.mjtObj.mjOBJ_GEOM, 'floor')
            obstacles = {floor}
            kernel = ContactAuditKernel(world.model.ngeom, robots, obstacles)
            for command in ([0., 0., 0., 0.], [.2, -.3, .4, -.1]):
                world.robot('r1').set_motor_commands(command)
                for _ in range(5):
                    world._physics_step_for(world.robot('r1'))
                    self.assertEqual(kernel.has_penetrating_contact(world.data),
                                     reference_contact(world.data, robots, obstacles))
        finally:
            world.close()


def benchmark_fixed_world(iterations: int = 20000) -> tuple[float, float, bool]:
    """Manual bounded benchmark on an actual, unchanged MuJoCo contact list."""
    world = MultiMasterPiProductionV2(seed=11, render=False)
    try:
        robots = {i for i in range(world.model.ngeom)
                  if (mujoco.mj_id2name(world.model, mujoco.mjtObj.mjOBJ_GEOM, i) or '').startswith('r1__')}
        # A non-colliding set exercises the full scan rather than short-circuiting.
        obstacles = {world.model.ngeom - 1}
        kernel = ContactAuditKernel(world.model.ngeom, robots, obstacles)
        expected = reference_contact(world.data, robots, obstacles)
        timings = []
        for fn in (lambda: reference_contact(world.data, robots, obstacles),
                   lambda: kernel.has_penetrating_contact(world.data)):
            start = time.perf_counter()
            for _ in range(iterations):
                result = fn()
            timings.append(time.perf_counter() - start)
            assert result == expected
        return timings[0], timings[1], expected
    finally:
        world.close()


if __name__ == '__main__':
    print('reference, kernel seconds, contact:', benchmark_fixed_world())
