"""Bit-exactness checks for sim.exact_speedups (the speedups must not change a single bit)."""
from __future__ import annotations

import hashlib
import unittest
from unittest import mock

import mujoco
import numpy as np

from sim import exact_speedups
from sim.exact_speedups import ContactPrefilter, ExactDriveKernel, install_drive_kernel, resolve
from sim.multi_masterpi_production import MultiMasterPiProductionV2


def state_digest(world) -> str:
    d = world.data
    parts = [d.qpos, d.qvel, d.act, d.ctrl, d.xfrc_applied, d.qacc_warmstart]
    parts += [c.motor_state for c in world.controllers.values()]
    return hashlib.sha256(b''.join(np.ascontiguousarray(p).tobytes() for p in parts)).hexdigest()


class ExactSpeedupTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.world = MultiMasterPiProductionV2(seed=11, render=False)
        cls.saved = mujoco.MjData(cls.world.model)
        mujoco.mj_copyData(cls.saved, cls.world.model, cls.world.data)

    @classmethod
    def tearDownClass(cls) -> None:
        cls.world.close()

    def setUp(self) -> None:
        self.world._fast_drive_kernel = None
        mujoco.mj_copyData(self.world.data, self.world.model, self.saved)
        for c in self.world.controllers.values():
            c.motor_state[:] = 0.0
            c.motor_command[:] = 0.0

    tearDown = setUp

    def test_resolve_sets(self) -> None:
        self.assertEqual(resolve(None), ('none', ()))
        self.assertEqual(resolve('exact-v1')[1], ('drive_kernel', 'contact_prefilter'))
        with self.assertRaises(ValueError):
            resolve('fast-but-different')

    def test_dot_order_self_check_on_this_build(self) -> None:
        # The kernel is only installed when this holds; on another BLAS the runner falls back (tested below).
        self.assertTrue(exact_speedups.dot_order_matches())

    def test_fallback_when_dot_order_differs(self) -> None:
        with mock.patch.object(exact_speedups, 'dot_order_matches', return_value=False):
            self.assertEqual(install_drive_kernel(self.world), 'fallback_original_dot_order_mismatch')
        self.assertIsNone(self.world._fast_drive_kernel)

    def _one_step(self, kernel: bool) -> tuple:
        """The world's real _physics_step_for with mj_step stubbed: pure drive arithmetic."""
        self.world._fast_drive_kernel = ExactDriveKernel(self.world) if kernel else None
        with mock.patch.object(mujoco, 'mj_step'):
            self.world._physics_step_for(self.world.controllers['r1'])
        d = self.world.data
        return (d.ctrl.tobytes(), d.xfrc_applied.tobytes(),
                tuple(c.motor_state.tobytes() for c in self.world.controllers.values()))

    def test_random_states_bitwise_equal_to_original_step(self) -> None:
        rng = np.random.default_rng(26)
        d = self.world.data
        for index in range(300):
            for c in self.world.controllers.values():
                c.motor_state[:] = rng.uniform(-1, 1, 4)
                c.motor_command[:] = (0.0 if index % 5 == 0 else 1e-7 if index % 5 == 1 else rng.uniform(-1, 1, 4))
                quat = rng.normal(size=4)
                d.qpos[c.base_qadr + 3:c.base_qadr + 7] = quat/np.linalg.norm(quat)
                d.qvel[c.base_dadr:c.base_dadr + 6] = rng.uniform(-2, 2, 6)
            states = {rid: c.motor_state.copy() for rid, c in self.world.controllers.items()}
            ctrl, xfrc = d.ctrl.copy(), d.xfrc_applied.copy()
            original = self._one_step(False)
            for rid, c in self.world.controllers.items():
                c.motor_state[:] = states[rid]
            d.ctrl[:], d.xfrc_applied[:] = ctrl, xfrc
            self.assertEqual(self._one_step(True), original, f'sample {index}')

    def test_real_steps_identical_trajectory(self) -> None:
        cmds = {'r1': [.6, .2, .6, .2], 'r2': [-.3, .4, .4, -.3], 'r3': [.05, -.05, .05, -.05]}
        digests = []
        for kernel in (False, True):
            self.setUp()
            if kernel:
                self.assertEqual(install_drive_kernel(self.world), 'exact_drive_kernel')
            trace = []
            for step in range(1600):
                if step == 800:
                    for c in self.world.controllers.values():
                        c.motor_command[:] = 0.0          # the stopped-damping branch
                elif step == 0:
                    for rid, c in self.world.controllers.items():
                        c.motor_command[:] = cmds[rid]
                self.world._physics_step_for(self.world.controllers['r1'])
                if step % 200 == 199:
                    trace.append(state_digest(self.world))
            digests.append(trace)
        self.assertEqual(digests[0], digests[1])

    def test_contact_prefilter_matches_python_loop(self) -> None:
        model, d = self.world.model, self.world.data
        names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g) or '' for g in range(model.ngeom)]
        geoms = {g for g, n in enumerate(names) if n.startswith('r2__')}
        prefilter = ContactPrefilter(model.ngeom, geoms)
        for c in self.world.controllers.values():
            c.motor_command[:] = [.5, .5, .5, .5]
        seen = 0
        for step in range(600):
            self.world._physics_step_for(self.world.controllers['r1'])
            want = [i for i in range(d.ncon) if {int(d.contact[i].geom1), int(d.contact[i].geom2)} & geoms]
            self.assertEqual(prefilter.indices(d), want)
            seen += len(want)
        self.assertGreater(seen, 0)
        self.assertEqual(ContactPrefilter(model.ngeom, set()).indices(d), [])


if __name__ == '__main__':
    unittest.main()
