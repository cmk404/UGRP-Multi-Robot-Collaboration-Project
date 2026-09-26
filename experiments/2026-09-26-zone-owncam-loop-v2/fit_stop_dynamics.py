"""Dev-only fit of the mecanum motion lag with separate drive / stop time constants.

Data: dev closed-loop runs of the first loop cohort (outputs/owncam-loop-20260925/dev-a*),
student phase only. GT (eval_only/gt_trajectory.jsonl, 20 Hz) is used here offline to fit
calibration constants; it never reaches the student. Test runs (test/, seeds 41-43) and
the new cohort's seeds are never read.

Model (per axis, body frame): vel' = (G u - vel) / tau, tau = tau_drive while a motion
command is active and tau_stop once the command is zero/expired (hold). For fixed taus
the displacement is linear in G, so the forward/lateral rows of G are least-squares fits.
Windows: each drive start (standstill) to the next drive start, so the post-stop coast
and the look that follows are included (the v1 refit ended at the last drive frame).
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

RAW = Path('/Users/changmin/projects/ugrp/outputs/owncam-loop-20260925')
DT = .01


def load(run: Path):
    cmds = [json.loads(l) for l in open(run/'inputs/commands.jsonl')]
    gt = [json.loads(l) for l in open(run/'eval_only/gt_trajectory.jsonl')]
    gt = [g for g in gt if g['phase'] == 'student']
    ev = [json.loads(l) for l in open(run/'eval_only/frames_eval.jsonl')]
    ev = [e for e in ev if e['phase'] == 'student']
    return cmds, gt, ev


def drive_starts(ev):
    out, prev = [], None
    for e in ev:
        if e['student_state'] == 'drive' and prev != 'drive':
            out.append(e['t'])
        prev = e['student_state']
    return out


def command_schedule(cmds, t0, t1):
    """Piecewise-constant issued command u(t) on [t0, t1) at DT (port semantics)."""
    n = int(round((t1 - t0)/DT))
    u = np.zeros((n, 3))
    cmd, exp = np.zeros(3), -1.
    rows = [c for c in cmds if c['t'] < t1]
    k = 0
    for i in range(n):
        t = t0 + i*DT
        while k < len(rows) and rows[k]['t'] <= t + 1e-9:
            c = rows[k]; k += 1
            if c['kind'] == 'mecanum':
                cmd, exp = np.array([c['forward'], c['left'], c['turn']]), c['t'] + c['duration_s']
            elif c['kind'] == 'drive':
                cmd, exp = np.array([c['forward'], 0., c['turn']]), c['t'] + c['duration_s']
            elif c['kind'] in ('hold', 'stop'):
                cmd, exp = np.zeros(3), -1.
        u[i] = cmd if t < exp - 1e-9 else 0.
    return u


def lagged(u, tau_drive, tau_stop):
    a_d, a_s = 1 - math.exp(-DT/tau_drive), 1 - math.exp(-DT/tau_stop)
    lag = np.zeros_like(u)
    cur = np.zeros(3)
    for i in range(len(u)):
        a = a_s if not np.any(u[i]) else a_d
        cur = cur + a*(u[i] - cur)
        lag[i] = cur
    return lag


def windows(runs):
    """(features builder inputs, GT body displacement samples) per window."""
    out = []
    for run in runs:
        cmds, gt, ev = load(run)
        starts = drive_starts(ev) + [ev[-1]['t']]
        gts = np.array([[g['t'], g['x'], g['y'], g['yaw']] for g in gt])
        for t0, t1 in zip(starts[:-1], starts[1:]):
            if t1 - t0 < 1.:
                continue
            u = command_schedule(cmds, t0, t1)
            sel = (gts[:, 0] >= t0) & (gts[:, 0] < t1)
            g = gts[sel]
            if len(g) < 5:
                continue
            x0, y0, th0 = g[0, 1:]
            c, s = math.cos(th0), math.sin(th0)
            body = np.stack([c*(g[:, 1]-x0) + s*(g[:, 2]-y0), -s*(g[:, 1]-x0) + c*(g[:, 2]-y0)], 1)
            idx = np.clip(np.round((g[:, 0] - g[0, 0])/DT).astype(int), 0, len(u) - 1)
            out.append((u, idx, body, str(run.relative_to(RAW))))
    return out


def fit(ws, tau_drive, tau_stop):
    X, Y = [], []
    for u, idx, body, _ in ws:
        U = np.cumsum(lagged(u, tau_drive, tau_stop), 0)*DT       # integral of lagged command
        X.append(U[idx]); Y.append(body)
    X, Y = np.concatenate(X), np.concatenate(Y)
    G = np.linalg.lstsq(X, Y, rcond=None)[0].T                    # 2x3: fwd/lat rows
    r = Y - X @ G.T
    return G, np.sqrt((r**2).mean(0)), X, Y


def post_stop_error(ws, G, tau_drive, tau_stop):
    """Mean predicted-minus-GT forward displacement at the window end (after the stop)."""
    e = []
    for u, idx, body, _ in ws:
        U = np.cumsum(lagged(u, tau_drive, tau_stop), 0)*DT
        e.append((U[idx[-1]] @ G.T) - body[-1])
    return np.array(e)


def main(out_path=None):
    result = {}
    for key, pat in (('motion_loaded', 'dev-a*/dev-box-s3*'), ('motion', 'dev-a*/dev-nobox-s3*')):
        runs = sorted(RAW.glob(pat))
        ws = windows(runs)
        best = None
        grid = []
        for td in (.2, .3, .44, .6, .8, 1.0):
            for ts in (.03, .05, .08, .12, .2, .3, .44, 1.0):
                G, rms, _, _ = fit(ws, td, ts)
                score = float(np.sqrt((rms**2).sum()))
                grid.append([td, ts, round(score, 5)])
                if best is None or score < best[0]:
                    best = (score, td, ts, G, rms)
        score, td, ts, G, rms = best
        # leave-one-run-out check of the chosen taus
        loro = []
        for hold in sorted({w[3] for w in ws}):
            tr = [w for w in ws if w[3] != hold]; te = [w for w in ws if w[3] == hold]
            Gh, _, _, _ = fit(tr, td, ts)
            loro += list(np.abs(post_stop_error(te, Gh, td, ts)[:, 0]))
        end = post_stop_error(ws, G, td, ts)
        result[key] = {'tau_drive_s': td, 'tau_stop_s': ts, 'gain_rows_fwd_lat': G.round(4).tolist(),
                       'rms_m_fwd_lat': rms.round(4).tolist(), 'windows': len(ws), 'runs': [str(r.relative_to(RAW)) for r in runs],
                       'window_end_fwd_err_m': {'mean': round(float(end[:, 0].mean()), 4),
                                                'p90_abs': round(float(np.percentile(np.abs(end[:, 0]), 90)), 4)},
                       'loro_window_end_fwd_abs_p90_m': round(float(np.percentile(loro, 90)), 4),
                       'grid_tau_drive_tau_stop_score': grid}
        print(key, 'tau_drive', td, 'tau_stop', ts, 'rms', rms.round(4), 'windows', len(ws),
              'end fwd err mean %.4f p90 %.4f LORO p90 %.4f' % (end[:, 0].mean(), np.percentile(np.abs(end[:, 0]), 90),
                                                               np.percentile(loro, 90)))
        print('  G fwd/lat rows', G.round(4).tolist())
    if out_path:
        Path(out_path).write_text(json.dumps(result, indent=2) + '\n')


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else None)
