"""Bit-exact CPU speedups for sync-SIM runners (execution infrastructure, no behaviour change).

Every item here replaces Python bookkeeping around ``mj_step`` with code that
produces the *same IEEE-754 values in the same order*. Nothing touches the
physics model, timestep, solver, contacts, actuators, cameras, renderer
settings or control rates. A set is opt-in by name and recorded in the run
manifest; ``'none'`` is the original code path.

``exact-v1``
    ``drive_kernel``: the per-controller motor filter and body wrench of
    ``MultiMasterPiProductionV2._physics_step_for`` as scalar Python
    (``sim.physics_drive_kernel.PhysicsDriveKernel`` reused), except that the
    three pattern sums follow ``np.dot``'s own summation order so the result is
    bit-identical, not only ``allclose``. That order depends on the BLAS build
    (Accelerate on arm64 pairs lanes ``(v0*p0 + v2*p2) + (v1*p1 + v3*p3)``), so
    the kernel self-checks against ``np.dot`` at construction and the caller
    falls back to the original path when the check fails (recorded).

    ``contact_prefilter``: the indices of contacts that touch a fixed geom set
    (ascending, as ``range(ncon)`` visits them), from one vectorised mask
    lookup (the pattern of ``sim.contact_audit_kernel`` and the v6-v9 skill
    runners). Loops that ``continue`` on every other contact see exactly the
    same contacts in the same order.
"""
from __future__ import annotations

import math
from collections.abc import Collection

import numpy as np

from sim.masterpi_dynamics_v2 import FORWARD_PATTERN, LEFT_PATTERN, MAX_WHEEL_RAD_S, YAW_LEFT_PATTERN
from sim.physics_drive_kernel import PhysicsDriveKernel

SPEEDUP_SETS: dict[str, tuple[str, ...]] = {
    'none': (),
    'exact-v1': ('drive_kernel', 'contact_prefilter'),
}
SELF_CHECK_SAMPLES = 4096


def resolve(name: str | None) -> tuple[str, tuple[str, ...]]:
    key = 'none' if name in (None, '') else str(name)
    if key not in SPEEDUP_SETS:
        raise ValueError(f'unknown speedup set {key!r}; choose one of {sorted(SPEEDUP_SETS)}')
    return key, SPEEDUP_SETS[key]


def _pairwise_sums(m0: float, m1: float, m2: float, m3: float) -> tuple[float, float, float]:
    """``np.dot(m, P)`` for the three drive patterns in the lane-paired order (sign flips are exact)."""
    return ((m0 + m2) + (m1 + m3), (-m0 + m2) + (m1 - m3), (-m0 - m2) + (m1 + m3))


def dot_order_matches(samples: int = SELF_CHECK_SAMPLES, seed: int = 20260926) -> bool:
    """True when this numpy/BLAS build sums the 4-element pattern dots in the paired order."""
    if not (np.array_equal(FORWARD_PATTERN, [1., 1., 1., 1.]) and np.array_equal(LEFT_PATTERN, [-1., 1., 1., -1.])
            and np.array_equal(YAW_LEFT_PATTERN, [-1., 1., -1., 1.])):
        return False
    rng = np.random.default_rng(seed)
    n = max(64, int(samples))
    vectors = np.concatenate([rng.uniform(-1, 1, (n, 4)), rng.normal(0, 1e-3, (n//4, 4)),
                              np.round(rng.uniform(-1, 1, (n//4, 4)), 3), rng.uniform(-1, 1, (n//8, 4))*1e-9])
    buffer = np.zeros(5)
    for v in vectors:
        buffer[1:] = v                       # also an unaligned view, as motor_state slices could be
        for arr in (v, buffer[1:]):
            want = (float(np.dot(arr, FORWARD_PATTERN)), float(np.dot(arr, LEFT_PATTERN)),
                    float(np.dot(arr, YAW_LEFT_PATTERN)))
            if want != _pairwise_sums(*(float(x) for x in arr)):
                return False
    return True


class ExactDriveKernel(PhysicsDriveKernel):
    """``PhysicsDriveKernel`` with ``np.dot``'s summation order: bit-identical to the original loop."""

    version = 'exact-drive-v1'

    def __init__(self, world) -> None:
        if not dot_order_matches():
            raise RuntimeError('np.dot summation order differs on this build; use the original drive path')
        super().__init__(world)

    def apply(self, dt: float) -> None:
        data = self.world.data
        data.xfrc_applied[:, :] = 0.0
        qpos, qvel, ctrl, force = data.qpos, data.qvel, data.ctrl, data.xfrc_applied
        for c in self.controllers:
            dynamics = c.dynamics
            tau = dynamics['motor_time_constant_s']
            cached = self._filter_cache.get(id(c))
            if cached is None or cached[0] != dt or cached[1] != tau:
                alpha = 1.0 - math.exp(-dt / tau)
                self._filter_cache[id(c)] = (dt, tau, alpha)
            else:
                alpha = cached[2]
            state, command = c.motor_state, c.motor_command
            s0, s1, s2, s3 = float(state[0]), float(state[1]), float(state[2]), float(state[3])
            k0, k1, k2, k3 = float(command[0]), float(command[1]), float(command[2]), float(command[3])
            # numpy: state += alpha * (command - state), elementwise, no fused multiply-add
            m0 = s0 + alpha * (k0 - s0)
            m1 = s1 + alpha * (k1 - s1)
            m2 = s2 + alpha * (k2 - s2)
            m3 = s3 + alpha * (k3 - s3)
            state[0] = m0; state[1] = m1; state[2] = m2; state[3] = m3
            acts = c.wheel_act
            ctrl[acts[0]] = m0 * MAX_WHEEL_RAD_S
            ctrl[acts[1]] = m1 * MAX_WHEEL_RAD_S
            ctrl[acts[2]] = m2 * MAX_WHEEL_RAD_S
            ctrl[acts[3]] = m3 * MAX_WHEEL_RAD_S
            f_sum, l_sum, y_sum = _pairwise_sums(m0, m1, m2, m3)
            fwd, left, yaw_cmd = f_sum / 4.0, l_sum / 4.0, y_sum / 4.0
            dadr = c.base_dadr
            vx_w, vy_w, wz = float(qvel[dadr]), float(qvel[dadr + 1]), float(qvel[dadr + 5])
            qadr = c.base_qadr + 3
            w, x, y, z = float(qpos[qadr]), float(qpos[qadr + 1]), float(qpos[qadr + 2]), float(qpos[qadr + 3])
            # same expression as masterpi_dynamics_v2._quat_to_rpy (yaw only)
            yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
            cy, sy = math.cos(yaw), math.sin(yaw)
            vx_local = cy * vx_w + sy * vy_w
            vy_local = -sy * vx_w + cy * vy_w
            stopped = max(abs(k0), abs(k1), abs(k2), abs(k3)) < 1e-6
            if stopped:
                ld = dynamics['stop_linear_damping_n_per_mps']
                yd = dynamics['stop_yaw_damping_nm_per_radps']
            else:
                ld = dynamics['linear_damping_n_per_mps']
                yd = dynamics['yaw_damping_nm_per_radps']
            fx_l = dynamics['max_forward_force_n'] * fwd - ld * vx_local
            fy_l = dynamics['max_lateral_force_n'] * left - ld * vy_local
            tz = dynamics['max_yaw_torque_nm'] * yaw_cmd - yd * wz
            bid = c.robot_bid
            force[bid, 0] = cy * fx_l - sy * fy_l
            force[bid, 1] = sy * fx_l + cy * fy_l
            force[bid, 5] = tz


def install_drive_kernel(world) -> str:
    """Attach the exact kernel through the world's existing ``_fast_drive_kernel`` hook; returns the status.

    An exact kernel of this version already bound to ``world`` is kept (``'exact_drive_kernel_already_installed'``).
    Any other kernel in the hook (e.g. the ``allclose``-only ``PhysicsDriveKernel`` of another runner, an older
    exact version or one bound to a different world) raises instead of being silently kept or replaced: the run
    would otherwise be recorded as exact while stepping with non-identical arithmetic (Codex review of PR #209).
    """
    current = getattr(world, '_fast_drive_kernel', None)
    if current is not None:
        if (type(current) is ExactDriveKernel and getattr(current, 'version', None) == ExactDriveKernel.version
                and current.world is world):
            return 'exact_drive_kernel_already_installed'
        raise RuntimeError(f'world already has drive kernel {type(current).__module__}.{type(current).__qualname__} '
                           f'(version {getattr(current, "version", None)!r}); refusing to keep or replace it for '
                           f'{ExactDriveKernel.version}: build a new world or leave the hook empty')
    try:
        world._fast_drive_kernel = ExactDriveKernel(world)
    except RuntimeError:
        return 'fallback_original_dot_order_mismatch'
    return 'exact_drive_kernel'


class ContactPrefilter:
    """Ascending indices of the contacts whose geom1 or geom2 is in a fixed set."""

    def __init__(self, ngeom: int, geoms: Collection[int]) -> None:
        self._mask = np.zeros(int(ngeom), dtype=np.bool_)
        if len(geoms):
            self._mask[np.fromiter(geoms, dtype=np.intp)] = True

    def indices(self, data) -> list[int]:
        n = int(data.ncon)
        if n == 0:
            return []
        geom = data.contact.geom[:n]
        mask = self._mask
        return np.flatnonzero(mask[geom[:, 0]] | mask[geom[:, 1]]).tolist()
