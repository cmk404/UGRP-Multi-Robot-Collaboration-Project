"""Own-camera pose source for M1: localizer -> ``PoseReport`` -> skill ``PoseEstimate``.

Inputs are exactly the robot's own issued commands and its own ``robot_cam``
frames (plus the static tagged map and fixed calibrations). Nothing here
imports the simulator; ``tests/test_m1_owncam.py`` checks the boundary.

``PoseReport`` carries what a controller needs to decide whether it may trust
the estimate (Codex review #4): the estimate time, initialisation state,
covariance and standard deviations, time since the last tag, the last valid
observation, and the load state inferred from own commands. ``check_limits``
returns every violated limit so the controller can stop and re-look.
"""
from __future__ import annotations

import hashlib
import json
import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field

import numpy as np

from harness.owncam_localizer import OwnCamLocalizer
from harness.wall_tags import TagDetector

SOURCE_PREFIX = 'owncam_pf_v2'


@dataclass(frozen=True)
class PoseReport:
    t_est: float
    initialized: bool
    x_m: float = float('nan')
    y_m: float = float('nan')
    yaw_rad: float = float('nan')
    cov: tuple = ()
    std_xy_m: float = float('inf')
    std_yaw_rad: float = float('inf')
    since_tag_s: float | None = None
    last_valid_obs: Mapping | None = None
    n_eff: float | None = None
    load_state: str = 'unloaded'
    source: str = SOURCE_PREFIX

    def as_dict(self) -> dict:
        return {'t_est': round(self.t_est, 4), 'initialized': self.initialized,
                'xyyaw': [round(self.x_m, 5), round(self.y_m, 5), round(self.yaw_rad, 6)] if self.initialized else None,
                'std_xy_m': None if not self.initialized else round(self.std_xy_m, 5),
                'std_yaw_rad': None if not self.initialized else round(self.std_yaw_rad, 5),
                'since_tag_s': None if self.since_tag_s is None else round(self.since_tag_s, 3),
                'last_valid_obs': self.last_valid_obs, 'n_eff': self.n_eff, 'load_state': self.load_state,
                'source': self.source}


@dataclass(frozen=True)
class PoseLimits:
    """Limits under which the controller may act on the estimate."""
    max_std_xy_m: float
    max_std_yaw_rad: float
    max_since_tag_s: float | None = None     # None: no tag-recency requirement
    max_age_s: float = .3
    max_since_look_s: float | None = None    # require a completed look this recently
    name: str = ''


def check_limits(report: PoseReport, now: float, limits: PoseLimits, *, since_look_s: float | None = None) -> list[str]:
    """Every violated limit (empty list = the estimate may be used)."""
    bad = []
    if not report.initialized:
        return ['not_initialized']
    if now - report.t_est > limits.max_age_s + 1e-9:
        bad.append('stale')
    if not math.isfinite(report.std_xy_m) or report.std_xy_m > limits.max_std_xy_m:
        bad.append('std_xy')
    if not math.isfinite(report.std_yaw_rad) or report.std_yaw_rad > limits.max_std_yaw_rad:
        bad.append('std_yaw')
    if limits.max_since_tag_s is not None and (report.since_tag_s is None or report.since_tag_s > limits.max_since_tag_s):
        bad.append('since_tag')
    if limits.max_since_look_s is not None and (since_look_s is None or since_look_s > limits.max_since_look_s):
        bad.append('since_look')
    return bad


def calibration_label(params: Mapping) -> str:
    blob = json.dumps(params, sort_keys=True, separators=(',', ':')).encode()
    return f'{SOURCE_PREFIX}:{hashlib.sha256(blob).hexdigest()[:8]}'


@dataclass
class OwnCamPoseSource:
    """Feeds the localizer with own commands and own frames; reports ``PoseReport``."""
    static_map: Mapping
    params: Mapping
    seed: int = 0
    width: int = 640
    height: int = 480
    loc: OwnCamLocalizer = field(init=False)
    detector: TagDetector = field(init=False)
    source: str = field(init=False)

    def __post_init__(self):
        self.loc = OwnCamLocalizer(self.static_map, self.params, seed=self.seed)
        self.detector = TagDetector.for_map(self.static_map, self.width, self.height)
        self.source = calibration_label(self.params)
        self.servo: dict[int, int] = {}
        self.last_obs = None
        self.frames = 0

    def on_command(self, row: Mapping) -> None:
        """One own issued command (time ordered, as logged at the robot's port)."""
        self.loc.command(row)
        kind = row['kind']
        if kind == 'initial_servo_command':
            self.servo = {int(k): int(v) for k, v in row['pulses'].items()}
        elif kind == 'arm':
            self.servo[int(row['servo_id'])] = int(row['pulse'])
        elif kind == 'look':
            self.servo[6] = int(row['pan_pulse'])

    def on_frame(self, now: float, rgb: np.ndarray) -> PoseReport:
        """One own ``robot_cam`` frame (RGB array decoded from the JPEG the robot saw)."""
        dets = self.detector.detect(rgb)
        self.frames += 1
        self.loc.update(now, dets, dict(self.servo))
        if dets:
            self.last_obs = {'t': round(now, 4), 'tag_ids': sorted(int(d['id']) for d in dets), 'n_tags': len(dets)}
        return self.report(now)

    def report(self, now: float) -> PoseReport:
        self.loc.predict_to(now)
        est = self.loc.estimate()
        load = 'loaded' if self.loc.load.loaded else 'unloaded'
        if not est.get('initialized'):
            return PoseReport(t_est=float(now), initialized=False, load_state=load, source=self.source,
                              last_valid_obs=self.last_obs)
        w = np.exp(self.loc.logw - self.loc.logw.max())
        n_eff = float(w.sum()**2/np.sum(w*w))
        return PoseReport(t_est=float(est['t']), initialized=True, x_m=est['x'], y_m=est['y'], yaw_rad=est['yaw'],
                          cov=tuple(tuple(r) for r in est['cov']), std_xy_m=est['std_xy_m'],
                          std_yaw_rad=est['std_yaw_rad'], since_tag_s=est.get('since_tag_s'),
                          last_valid_obs=self.last_obs, n_eff=round(n_eff, 1), load_state=load, source=self.source)


def to_skill_estimate(report: PoseReport, pose_estimate_cls):
    """Skill ``PoseEstimate`` from a checked report (raises when not initialised)."""
    if not report.initialized:
        raise ValueError('pose report not initialised')
    return pose_estimate_cls(report.x_m, report.y_m, report.yaw_rad, report.source)


def mean_xy(points: Sequence[Sequence[float]]) -> tuple[float, float]:
    a = np.asarray(points, float)
    return float(a[:, 0].mean()), float(a[:, 1].mean())
