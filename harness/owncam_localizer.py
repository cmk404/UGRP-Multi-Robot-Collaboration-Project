"""Own-camera particle-filter localization on a tagged zone map.

Run-time inputs (all robot-owned or static): the robot's ISSUED commands
(base ``mecanum``/``hold``/``drive`` and arm/look servo pulses), its raw wrist
RGB frames, the static map with surveyed wall tags, and fixed calibrations
(camera intrinsics, a command->motion model and measurement noise fitted
OFFLINE on teacher logs of the dev split; see
``scripts/eval_owncam_localization.py``). It never imports the simulator nor
reads simulator poses; ``tests/test_owncam_localizer.py`` enforces this.

Filter structure adapted from PythonRobotics
``Localization/particle_filter/particle_filter.py`` (upstream commit b2020cd,
file sha256 a649994b...): noisy-input prediction, Gaussian importance weights,
weighted covariance, effective particle number and low-variance resampling,
vectorized here. Changes: planar mecanum motion from issued body-velocity
commands with a first-order lag and per-particle slip scales, AprilTag PnP
measurements (bearing, range, face normal), static-wall rejection and sensor
resetting from tag observations.

PythonRobotics license (MIT):
  Copyright (c) 2016 - now Atsushi Sakai and other contributors:
  https://github.com/AtsushiSakai/PythonRobotics/contributors
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions: The above copyright
  notice and this permission notice shall be included in all copies or
  substantial portions of the Software. THE SOFTWARE IS PROVIDED "AS IS",
  WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
  TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
  FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
  TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR
  THE USE OR OTHER DEALINGS IN THE SOFTWARE.
"""
from __future__ import annotations

import copy
import math
from collections.abc import Mapping, Sequence

import numpy as np

from harness.visual_arm import tool_pose
from harness.wall_tags import (angle_between, camera_in_base, observed_tag_in_camera, predicted_tag_in_camera,
                               tag_world_frame, tags_by_id)

SCHEMA = 'ugrp.owncam_localizer.v1'
STEP_S = .05
# Default parameters; the eval script replaces motion/measurement values with
# the offline dev calibration and records its provenance.
DEFAULT_PARAMS = {
    'particles': 2000,
    'motion': {'gain': [[1.15, 0., 0.], [0., .94, 0.], [0., 0., 1.40]], 'tau_s': .12,
               # per-step velocity noise std = rel*|v| + abs (m/s, m/s, rad/s)
               'noise_rel': [.10, .10, .10], 'noise_abs': [.004, .004, .01],
               # persistent per-particle slip scales: initial std and random walk per sqrt(s)
               'scale_std': .05, 'scale_walk': .01},
    # Bearing is split in the camera frame: azimuth atan2(x, z) and elevation
    # atan2(y, z). The commanded-PWM FK camera pitch differs from the settled
    # arm by about 1 deg (gravity compliance), which only biases elevation.
    'measurement': {'azimuth_std_rad': math.radians(.3), 'elevation_std_rad': math.radians(1.5),
                    'range_log_std': .08,
                    'normal_std_rad': math.radians(20.), 'outlier_prob': .05, 'outlier_margin': 6.,
                    'invisible_log_penalty': -20., 'min_side_px': 8., 'max_range_m': None,
                    'range_log_bias': [0., 0.]},
    'map': {'robot_clearance_m': .07, 'wall_log_penalty': -8.},
    'resample_ratio': .5,
    'roughen': [0., 0., 0.],
    'reset': {'min_best_loglik': -12., 'fraction': .2, 'init_pos_std_m': .03, 'init_yaw_bins': 1},
}


GRIP_CLOSED_MAX = 1600       # servo 1 pulse at or below: gripper commanded closed
GRIP_OPEN_MIN = 1800         # servo 1 pulse at or above: gripper commanded open
GRASP_TOOL_Z_M = .06         # commanded tool height of a grasp (floor boxes)


class LoadState:
    """'Loaded' from the robot's own commands only: the gripper was commanded
    closed while the commanded tool point was at grasp height, and has not been
    commanded open since. (A carried box changes the command->motion map: the
    dev fit gave turn gain 0.74 loaded vs 1.49 unloaded.)"""

    def __init__(self):
        self.servo: dict[int, int] = {}
        self.loaded = False

    def command(self, row):
        kind = row['kind']
        if kind == 'initial_servo_command':
            self.servo = {int(k): int(v) for k, v in row['pulses'].items()}
        elif kind == 'look':
            self.servo[6] = int(row['pan_pulse'])
        elif kind == 'arm':
            servo, pulse = int(row['servo_id']), int(row['pulse'])
            self.servo[servo] = pulse
            if servo == 1 and pulse >= GRIP_OPEN_MIN:
                self.loaded = False
            elif servo == 1 and pulse <= GRIP_CLOSED_MAX and not self.loaded:
                try:
                    self.loaded = tool_pose(self.servo).z_m < GRASP_TOOL_Z_M
                except (KeyError, ValueError):
                    pass
        return self.loaded


def wrap(a):
    return (np.asarray(a) + np.pi) % (2*np.pi) - np.pi


class OwnCamLocalizer:
    """Particle filter over (x, y, yaw) + per-particle slip scales."""

    def __init__(self, static_map: Mapping, params: Mapping | None = None, seed: int = 0):
        self.params = copy.deepcopy(dict(DEFAULT_PARAMS if params is None else params))
        self.map = static_map
        self.tags = tags_by_id(static_map)
        self.rng = np.random.default_rng(seed)
        self.n = int(self.params['particles'])
        self.px = np.zeros((self.n, 3))
        self.scale = np.ones((self.n, 3))
        self.logw = np.zeros(self.n)
        self.initialized = False
        self.t = 0.
        self.cmd = np.zeros(3)
        self.cmd_expires = -1.
        self.vel = np.zeros(3)
        self.servo: dict[int, int] = {}
        self.load = LoadState()
        m = self.params['map']
        x0, x1, y0, y1 = static_map['bounds_m']
        c = m['robot_clearance_m']
        self.bounds = (x0 + c, x1 - c, y0 + c, y1 - c)
        self.rects = np.array([[o['center_m'][0], o['center_m'][1], o['half_extents_m'][0] + c,
                                o['half_extents_m'][1] + c] for o in static_map['obstacles']
                               if o.get('kind') == 'wall'], float).reshape(-1, 4)
        self.last_tag_t = None
        self.stats = {'updates': 0, 'resets': 0, 'resamples': 0}

    # ------------------------------------------------------------ commands
    def command(self, row: Mapping) -> None:
        """Feed one issued command (in time order)."""
        t = float(row['t'])
        self.predict_to(t)
        self.load.command(row)
        kind = row['kind']
        if kind == 'initial_servo_command':
            self.servo = {int(k): int(v) for k, v in row['pulses'].items()}
        elif kind == 'arm':
            self.servo[int(row['servo_id'])] = int(row['pulse'])
        elif kind == 'look':
            self.servo[6] = int(row['pan_pulse'])
        elif kind == 'mecanum':
            self.cmd = np.array([row['forward'], row['left'], row['turn']], float)
            self.cmd_expires = t + float(row['duration_s'])
        elif kind == 'drive':
            self.cmd = np.array([row['forward'], 0., row['turn']], float)
            self.cmd_expires = t + float(row['duration_s'])
        else:  # hold / stop / anything else stops the wheels (port semantics)
            self.cmd = np.zeros(3)
            self.cmd_expires = -1.

    # ------------------------------------------------------------ predict
    def predict_to(self, t: float) -> None:
        while self.t < t - 1e-9:
            mp = self.params['motion_loaded'] if self.load.loaded and 'motion_loaded' in self.params \
                else self.params['motion']
            gain = np.asarray(mp['gain'], float)
            rel, ab = np.asarray(mp['noise_rel']), np.asarray(mp['noise_abs'])
            dt = min(STEP_S, t - self.t)
            u = self.cmd if self.t < self.cmd_expires - 1e-9 else np.zeros(3)
            target = gain @ u
            alpha = 1. - math.exp(-dt/max(mp['tau_s'], 1e-6))
            self.vel = self.vel + alpha*(target - self.vel)
            if self.initialized:
                std = rel*np.abs(self.vel) + ab
                v = self.vel[None, :]*self.scale + self.rng.normal(size=(self.n, 3))*std
                c, s = np.cos(self.px[:, 2]), np.sin(self.px[:, 2])
                self.px[:, 0] += (c*v[:, 0] - s*v[:, 1])*dt
                self.px[:, 1] += (s*v[:, 0] + c*v[:, 1])*dt
                self.px[:, 2] = wrap(self.px[:, 2] + v[:, 2]*dt)
                if np.any(np.abs(self.vel) > 1e-6):
                    self.scale += self.rng.normal(size=(self.n, 3))*mp['scale_walk']*math.sqrt(dt)
                self.logw += self._map_logprior(self.px)
            self.t += dt

    def _map_logprior(self, px):
        x, y = px[:, 0], px[:, 1]
        x0, x1, y0, y1 = self.bounds
        bad = (x < x0) | (x > x1) | (y < y0) | (y > y1)
        for cx, cy, hx, hy in self.rects:
            bad |= (np.abs(x - cx) < hx) & (np.abs(y - cy) < hy)
        return np.where(bad, self.params['map']['wall_log_penalty'], 0.)

    # ------------------------------------------------------------ measurement
    def _loglik(self, px, detections, pose):
        mp = self.params['measurement']
        if self.load.loaded and self.params.get('measurement_loaded'):
            # Holding cargo sags the arm below its commanded-PWM FK (offline dev
            # calibration); only the robot's own commands decide 'loaded'.
            mp = {**mp, **self.params['measurement_loaded']}
        total = np.zeros(len(px))
        used = 0
        floor = math.log(mp['outlier_prob'])
        for det in detections:
            tag = self.tags.get(int(det['id']))
            if tag is None:
                continue
            t_obs, n_obs = observed_tag_in_camera(det)
            p_c, n_c = predicted_tag_in_camera(px, tag, pose)
            az = np.arctan2(p_c[:, 0], p_c[:, 2]) - math.atan2(t_obs[0], t_obs[2])
            # observed - predicted elevation minus its calibrated bias (0 in v1)
            el = math.atan2(t_obs[1], t_obs[2]) - np.arctan2(p_c[:, 1], p_c[:, 2]) - mp.get('elevation_bias_rad', 0.)
            r_obs = float(np.linalg.norm(t_obs))
            # Detector range bias (small tags: sub-pixel corner offset), fitted
            # offline on dev as log(r_obs/r_true) = a + b*r_obs.
            bias = mp.get('range_log_bias', [0., 0.])
            rng_err = (np.log(r_obs/np.maximum(np.linalg.norm(p_c, axis=1), 1e-6))
                       - (bias[0] + bias[1]*r_obs))
            normal = np.min([angle_between(np.broadcast_to(n, n_c.shape), n_c) for n in n_obs], axis=0)
            s_az = max(mp['azimuth_std_rad'], mp.get('azimuth_floor_rad', 0.))
            s_el = max(mp['elevation_std_rad'], mp.get('elevation_floor_rad', 0.))
            s_r = max(mp['range_log_std'], mp.get('range_floor', 0.))
            ll = (-.5*(wrap(az)/s_az)**2 - .5*(wrap(el)/s_el)**2
                  - .5*(rng_err/s_r)**2
                  - .5*(np.minimum(normal, math.radians(90.))/mp['normal_std_rad'])**2)
            # Robust per tag: a misdetection or an unmodelled error costs at most
            # log(outlier_prob) - outlier_margin (mixture with a flat outlier term).
            ll = np.logaddexp(math.log(1 - mp['outlier_prob']) + ll, floor - mp['outlier_margin'])
            # The camera must see the tag face from the front, and the tag must be ahead.
            visible = (p_c[:, 2] > .02) & (np.sum(p_c*n_c, axis=1) < 0)
            total += np.where(visible, ll, ll + mp['invisible_log_penalty'])
            used += 1
        # Tags in one frame share the extrinsic (FK) error, so they are not
        # independent: temper the joint log-likelihood by used**(-temper).
        if used > 1:
            total = total * used ** -mp.get('tag_temper', 0.)
        return total

    def _reset_from(self, detections, pose, count):
        """Sensor resetting: particles consistent with one tag's PnP translation.

        For yaw hypotheses spread over the circle, the tag centre seen at t_obs
        fixes the base position; the measurement update then keeps the yaws
        consistent with every tag in view, the face normals and the walls.
        """
        rp = self.params['reset']
        o_bc, r_bc = camera_in_base(pose)
        usable = [d for d in detections if int(d['id']) in self.tags]
        if not usable or count <= 0:
            return None
        picks = self.rng.integers(len(usable), size=count)
        out = np.zeros((count, 3))
        yaw = self.rng.uniform(-np.pi, np.pi, size=count)
        for i, det in enumerate(usable):
            sel = picks == i
            if not np.any(sel):
                continue
            t_obs, _ = observed_tag_in_camera(det)
            c_w, _ = tag_world_frame(self.tags[int(det['id'])])
            v_b = o_bc + r_bc @ t_obs
            th = yaw[sel]
            c, s = np.cos(th), np.sin(th)
            jitter = self.rng.normal(size=(int(sel.sum()), 2))*(rp['init_pos_std_m'] + .03*np.linalg.norm(t_obs))
            out[sel, 0] = c_w[0] - (c*v_b[0] - s*v_b[1]) + jitter[:, 0]
            out[sel, 1] = c_w[1] - (s*v_b[0] + c*v_b[1]) + jitter[:, 1]
            out[sel, 2] = th
        return out

    def update(self, t: float, detections: Sequence[Mapping], commanded_pose: Mapping | None = None) -> dict:
        """Predict to ``t`` and apply this frame's tag detections."""
        self.predict_to(t)
        pose = {int(k): int(v) for k, v in (commanded_pose or self.servo).items()}
        max_range = self.params['measurement'].get('max_range_m')
        dets = [d for d in detections if int(d['id']) in self.tags and d.get('solutions')
                and (not max_range or float(np.linalg.norm(observed_tag_in_camera(d)[0])) <= max_range)]
        if dets:
            self.stats['updates'] += 1
            self.last_tag_t = t
            if not self.initialized:
                self.px = self._reset_from(dets, pose, self.n)
                sd = self.params['motion']['scale_std']
                self.scale = 1. + self.rng.normal(size=(self.n, 3))*sd
                self.logw = self._map_logprior(self.px)
                self.initialized = True
            ll = self._loglik(self.px, dets, pose)
            if ll.max() < self.params['reset']['min_best_loglik']:
                k = int(self.n*self.params['reset']['fraction'])
                idx = self.rng.choice(self.n, size=k, replace=False)
                self.px[idx] = self._reset_from(dets, pose, k)
                self.scale[idx] = 1.
                self.logw[idx] = np.max(self.logw)
                ll[idx] = self._loglik(self.px[idx], dets, pose)
                self.stats['resets'] += 1
            self.logw = self.logw + ll
        if self.initialized:
            self._normalize_and_resample()
        return self.estimate()

    def _normalize_and_resample(self):
        self.logw -= self.logw.max()
        w = np.exp(self.logw)
        w /= w.sum()
        n_eff = 1./np.sum(w*w)
        if n_eff < self.params['resample_ratio']*self.n:
            # low-variance resampling (PythonRobotics re_sampling, vectorized)
            positions = (np.arange(self.n) + self.rng.uniform())/self.n
            idx = np.minimum(np.searchsorted(np.cumsum(w), positions), self.n - 1)
            self.px, self.scale = self.px[idx].copy(), self.scale[idx].copy()
            # Roughening (regularized PF): small jitter so the cloud can move
            # when the likelihood is much sharper than the motion noise.
            rough = np.asarray(self.params.get('roughen', [0., 0., 0.]), float)
            if np.any(rough > 0):
                self.px += self.rng.normal(size=self.px.shape)*rough
                self.px[:, 2] = wrap(self.px[:, 2])
            self.logw = np.zeros(self.n)
            self.stats['resamples'] += 1
        else:
            self.logw = np.log(np.maximum(w, 1e-300))

    def estimate(self) -> dict:
        if not self.initialized:
            return {'t': round(self.t, 4), 'initialized': False}
        w = np.exp(self.logw - self.logw.max())
        w /= w.sum()
        yaw = math.atan2(float(np.sum(w*np.sin(self.px[:, 2]))), float(np.sum(w*np.cos(self.px[:, 2]))))
        mean = np.array([float(np.sum(w*self.px[:, 0])), float(np.sum(w*self.px[:, 1])), yaw])
        d = self.px - mean
        d[:, 2] = wrap(d[:, 2])
        cov = (w[:, None]*d).T @ d / max(1. - float(np.sum(w*w)), 1e-9)   # PythonRobotics calc_covariance
        return {'t': round(self.t, 4), 'initialized': True, 'x': float(mean[0]), 'y': float(mean[1]),
                'yaw': float(mean[2]), 'cov': cov.round(8).tolist(),
                'std_xy_m': float(math.sqrt(max(cov[0, 0] + cov[1, 1], 0.))),
                'std_yaw_rad': float(math.sqrt(max(cov[2, 2], 0.))), 'n_eff': float(1./np.sum(w*w)),
                'since_tag_s': None if self.last_tag_t is None else round(self.t - self.last_tag_t, 3)}
