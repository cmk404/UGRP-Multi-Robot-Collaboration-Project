"""Dev-only fit of a small loaded camera-extrinsic correction (camera frame).

Holding the box sags the arm, so the wrist camera sits slightly off its
commanded-PWM FK pose. Model per detection (camera frame, metres):
    t_obs_corr ~= p_c - delta + omega x p_c
with p_c the tag centre predicted from the GT base pose (offline, dev only) and
the commanded servos, t_obs_corr the PnP translation after the frozen range-bias
correction, delta a camera translation and omega a small rotation. Linear
least squares over dev loaded look frames (LOOK_P20, any pan).
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from harness.wall_tags import TagDetector, observed_tag_in_camera, predicted_tag_in_camera, tags_by_id  # noqa: E402
from sim.zone_landmarks import tagged_map  # noqa: E402

RAW = Path('/Users/changmin/projects/ugrp/outputs/owncam-loop-20260925')
CAL = json.loads((Path(__file__).with_name('calibration_loop_v2.json')).read_text())['params']['measurement']


def skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def rows(runs, states=('look_pan',), max_range=None):
    static = tagged_map('zone_wide_door_tags_v2')
    tags, det = tags_by_id(static), TagDetector.for_map(static)
    a, b = CAL['range_log_bias']
    out = []
    for run in runs:
        frames = [json.loads(l) for l in open(run/'inputs/frames.jsonl')]
        ev = {e['frame']: e for e in map(json.loads, open(run/'eval_only/frames_eval.jsonl'))}
        for f in frames:
            e = ev[f['frame']]
            if e['phase'] != 'student' or f['student_state'] not in states:
                continue
            pose = {int(k): int(v) for k, v in f['commanded_servo'].items()}
            img = cv2.cvtColor(cv2.imread(str(run/f['file'])), cv2.COLOR_BGR2RGB)
            for d in det.detect(img):
                t, _ = observed_tag_in_camera(d)
                r = float(np.linalg.norm(t))
                if max_range and r > max_range:
                    continue
                t = t*math.exp(-(a + b*r))
                p, _ = predicted_tag_in_camera(np.array([e['gt']]), tags[d['id']], pose)
                out.append((t, p[0], r, str(run.relative_to(RAW)), pose.get(6)))
    return out


def solve(data):
    A, y = [], []
    for t, p, r, _, _ in data:
        # t - p = -delta + omega x p = -delta - skew(p) omega ; weight 1/r (angular noise ~ r)
        w = 1./max(r, .3)
        A.append(np.hstack([-np.eye(3), -skew(p)])*w); y.append((t - p)*w)
    A, y = np.concatenate(A), np.concatenate(y)
    x = np.linalg.lstsq(A, y, rcond=None)[0]
    return x[:3], x[3:]


def resid(data, delta, omega):
    az, el, rr = [], [], []
    for t, p, r, _, _ in data:
        q = p - delta + np.cross(omega, p)
        az.append(math.degrees(math.atan2(t[0], t[2]) - math.atan2(q[0], q[2])))
        el.append(math.degrees(math.atan2(t[1], t[2]) - math.atan2(q[1], q[2])))
        rr.append(math.log(np.linalg.norm(t)/np.linalg.norm(q)))
    return {k: [round(float(np.mean(v)), 4), round(float(np.std(v)), 4)] for k, v in
            (('az_deg', az), ('el_deg', el), ('rlog', rr))}


def main(out=None):
    runs = sorted(RAW.glob('dev-a[34]/dev-box-s3*'))
    data = rows(runs)
    delta, omega = solve(data)
    res = {'runs': [str(r.relative_to(RAW)) for r in runs], 'detections': len(data),
           'delta_cam_m': delta.round(4).tolist(), 'omega_cam_rad': omega.round(5).tolist(),
           'omega_cam_deg': np.degrees(omega).round(3).tolist(),
           'resid_before': resid(data, np.zeros(3), np.zeros(3)), 'resid_after': resid(data, delta, omega)}
    loro = {}
    for hold in res['runs']:
        tr = [d for d in data if d[3] != hold]; te = [d for d in data if d[3] == hold]
        dh, oh = solve(tr)
        loro[hold] = {'delta_cam_m': dh.round(4).tolist(), 'omega_cam_deg': np.degrees(oh).round(3).tolist(),
                      'resid_heldout': resid(te, dh, oh)}
    res['leave_one_run_out'] = loro
    print(json.dumps(res, indent=1))
    if out:
        Path(out).write_text(json.dumps(res, indent=2) + '\n')


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else None)
