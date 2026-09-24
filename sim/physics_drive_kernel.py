"""Allocation-light motor filter and body wrench for the shared MasterPi world.

The caller owns the physics lock and calls ``mj_step`` after ``apply``.  This
kernel only replaces the per-controller arithmetic in the original step loop;
it does not change the timestep, contacts, or actuator model.
"""
from __future__ import annotations

import math

from sim.masterpi_dynamics_v2 import MAX_WHEEL_RAD_S


class PhysicsDriveKernel:
    def __init__(self, world) -> None:
        self.world = world
        self.controllers = tuple(world.controllers.values())
        self._filter_cache: dict[int, tuple[float, float, float]] = {}

    def apply(self, dt: float) -> None:
        data = self.world.data
        data.xfrc_applied[:, :] = 0.0
        for c in self.controllers:
            dynamics = c.dynamics
            tau = dynamics["motor_time_constant_s"]
            cache_key = id(c)
            cached = self._filter_cache.get(cache_key)
            if cached is None or cached[0] != dt or cached[1] != tau:
                alpha = 1.0 - math.exp(-dt / tau)
                self._filter_cache[cache_key] = (dt, tau, alpha)
            else:
                alpha = cached[2]

            state = c.motor_state
            command = c.motor_command
            m0 = float(state[0]) + alpha * (float(command[0]) - float(state[0]))
            m1 = float(state[1]) + alpha * (float(command[1]) - float(state[1]))
            m2 = float(state[2]) + alpha * (float(command[2]) - float(state[2]))
            m3 = float(state[3]) + alpha * (float(command[3]) - float(state[3]))
            state[0] = m0
            state[1] = m1
            state[2] = m2
            state[3] = m3
            acts = c.wheel_act
            data.ctrl[acts[0]] = m0 * MAX_WHEEL_RAD_S
            data.ctrl[acts[1]] = m1 * MAX_WHEEL_RAD_S
            data.ctrl[acts[2]] = m2 * MAX_WHEEL_RAD_S
            data.ctrl[acts[3]] = m3 * MAX_WHEEL_RAD_S

            fwd = (m0 + m1 + m2 + m3) / 4.0
            left = ((-m0 + m1) + m2 - m3) / 4.0
            yaw_cmd = ((-m0 + m1) - m2 + m3) / 4.0
            dadr = c.base_dadr
            vx_w = float(data.qvel[dadr])
            vy_w = float(data.qvel[dadr + 1])
            wz = float(data.qvel[dadr + 5])
            qadr = c.base_qadr + 3
            w = float(data.qpos[qadr])
            x = float(data.qpos[qadr + 1])
            y = float(data.qpos[qadr + 2])
            z = float(data.qpos[qadr + 3])
            yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
            cy, sy = math.cos(yaw), math.sin(yaw)
            vx_local = cy * vx_w + sy * vy_w
            vy_local = -sy * vx_w + cy * vy_w
            stopped = max(abs(float(command[0])), abs(float(command[1])),
                          abs(float(command[2])), abs(float(command[3]))) < 1e-6
            if stopped:
                ld = dynamics["stop_linear_damping_n_per_mps"]
                yd = dynamics["stop_yaw_damping_nm_per_radps"]
            else:
                ld = dynamics["linear_damping_n_per_mps"]
                yd = dynamics["yaw_damping_nm_per_radps"]
            fx_l = dynamics["max_forward_force_n"] * fwd - ld * vx_local
            fy_l = dynamics["max_lateral_force_n"] * left - ld * vy_local
            tz = dynamics["max_yaw_torque_nm"] * yaw_cmd - yd * wz
            force = data.xfrc_applied
            bid = c.robot_bid
            force[bid, 0] = cy * fx_l - sy * fy_l
            force[bid, 1] = sy * fx_l + cy * fy_l
            force[bid, 5] = tz
