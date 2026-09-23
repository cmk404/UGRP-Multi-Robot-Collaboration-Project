"""Numerical parity checks for the extracted shared-world drive arithmetic."""
from __future__ import annotations

import math
import time
import unittest

import numpy as np

from sim.masterpi_dynamics_v2 import (
    FORWARD_PATTERN, LEFT_PATTERN, MAX_WHEEL_RAD_S, YAW_LEFT_PATTERN,
)
from sim.multi_masterpi_production import MultiMasterPiProductionV2
from sim.physics_drive_kernel import PhysicsDriveKernel


def reference_apply(world, dt: float) -> None:
    """Unchanged arithmetic from MultiMasterPiProductionV2._physics_step_for."""
    world.data.xfrc_applied[:, :] = 0.0
    for c in world.controllers.values():
        alpha = 1.0 - math.exp(-dt / c.dynamics["motor_time_constant_s"])
        c.motor_state += alpha * (c.motor_command - c.motor_state)
        world.data.ctrl[c.wheel_act] = c.motor_state * MAX_WHEEL_RAD_S
        fwd = float(np.dot(c.motor_state, FORWARD_PATTERN) / 4.0)
        left = float(np.dot(c.motor_state, LEFT_PATTERN) / 4.0)
        yaw_cmd = float(np.dot(c.motor_state, YAW_LEFT_PATTERN) / 4.0)
        qvel = world.data.qvel[c.base_dadr:c.base_dadr + 6]
        vx_w, vy_w, wz = float(qvel[0]), float(qvel[1]), float(qvel[5])
        _, _, yaw = c.base_rpy(); cy, sy = math.cos(yaw), math.sin(yaw)
        vx_local = cy * vx_w + sy * vy_w; vy_local = -sy * vx_w + cy * vy_w
        stopped = float(np.max(np.abs(c.motor_command))) < 1e-6
        ld = c.dynamics["stop_linear_damping_n_per_mps" if stopped else "linear_damping_n_per_mps"]
        yd = c.dynamics["stop_yaw_damping_nm_per_radps" if stopped else "yaw_damping_nm_per_radps"]
        fx_l = c.dynamics["max_forward_force_n"] * fwd - ld * vx_local
        fy_l = c.dynamics["max_lateral_force_n"] * left - ld * vy_local
        tz = c.dynamics["max_yaw_torque_nm"] * yaw_cmd - yd * wz
        world.data.xfrc_applied[c.robot_bid, 0] = cy * fx_l - sy * fy_l
        world.data.xfrc_applied[c.robot_bid, 1] = sy * fx_l + cy * fy_l
        world.data.xfrc_applied[c.robot_bid, 5] = tz


class PhysicsDriveKernelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.world = MultiMasterPiProductionV2(seed=11, render=False)

    @classmethod
    def tearDownClass(cls) -> None:
        cls.world.close()

    def setUp(self) -> None:
        self.kernel = PhysicsDriveKernel(self.world)
        self.dynamics = dict(self.world.dynamics)

    def tearDown(self) -> None:
        self.world.dynamics.clear()
        self.world.dynamics.update(self.dynamics)
        for c in self.world.controllers.values():
            c.dynamics = self.world.dynamics

    def _snapshot(self):
        return (
            {rid: c.motor_state.copy() for rid, c in self.world.controllers.items()},
            self.world.data.ctrl.copy(),
            self.world.data.xfrc_applied.copy(),
        )

    def _restore(self, snapshot) -> None:
        states, ctrl, forces = snapshot
        for rid, c in self.world.controllers.items():
            c.motor_state[:] = states[rid]
        self.world.data.ctrl[:] = ctrl
        self.world.data.xfrc_applied[:] = forces

    def _assert_same(self, dt: float) -> None:
        before = self._snapshot()
        reference_apply(self.world, dt)
        expected = self._snapshot()
        self._restore(before)
        self.kernel.apply(dt)
        actual = self._snapshot()
        for rid in self.world.controllers:
            np.testing.assert_allclose(actual[0][rid], expected[0][rid], rtol=1e-12, atol=1e-12)
        np.testing.assert_allclose(actual[1], expected[1], rtol=1e-12, atol=1e-12)
        np.testing.assert_allclose(actual[2], expected[2], rtol=1e-12, atol=1e-12)

    def test_random_states_yaw_and_stopped_commands(self) -> None:
        rng = np.random.default_rng(19)
        for index in range(40):
            for c in self.world.controllers.values():
                c.motor_state[:] = rng.uniform(-1, 1, 4)
                c.motor_command[:] = rng.uniform(-1, 1, 4)
                if index % 4 == 0:
                    c.motor_command[:] = 0.0
                if index % 4 == 1:
                    c.motor_command[:] = 1e-7
                quat = rng.normal(size=4)
                quat /= np.linalg.norm(quat)
                self.world.data.qpos[c.base_qadr + 3:c.base_qadr + 7] = quat
                self.world.data.qvel[c.base_dadr:c.base_dadr + 6] = rng.uniform(-2, 2, 6)
            self._assert_same(float(self.world.model.opt.timestep))

    def test_dt_and_mutable_dynamics_invalidate_filter(self) -> None:
        for c in self.world.controllers.values():
            c.motor_state[:] = [.2, -.1, .4, -.3]
            c.motor_command[:] = [.8, .1, -.2, -.7]
        dt = float(self.world.model.opt.timestep)
        self._assert_same(dt)
        self._assert_same(dt * 2.0)
        self.world.dynamics["motor_time_constant_s"] *= 1.5
        self.world.dynamics["max_forward_force_n"] *= .7
        self.world.dynamics["stop_linear_damping_n_per_mps"] *= 1.2
        self._assert_same(dt)


def benchmark_fixed_world(iterations: int = 5000) -> tuple[float, float]:
    """Manual bounded microbenchmark; excludes MuJoCo stepping and rendering."""
    world = MultiMasterPiProductionV2(seed=11, render=False)
    try:
        kernel = PhysicsDriveKernel(world)
        before = {rid: c.motor_state.copy() for rid, c in world.controllers.items()}
        dt = float(world.model.opt.timestep)
        timings = []
        for fn in (reference_apply, lambda w, d: kernel.apply(d)):
            for rid, c in world.controllers.items():
                c.motor_state[:] = before[rid]
                c.motor_command[:] = [.2, -.3, .4, -.1]
            start = time.perf_counter()
            for _ in range(iterations):
                fn(world, dt)
            timings.append(time.perf_counter() - start)
        return timings[0], timings[1]
    finally:
        world.close()


if __name__ == "__main__":
    print("reference, kernel seconds:", benchmark_fixed_world())
