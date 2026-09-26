"""POST-HOC diagnostic of the test failures (written after the test run; not pre-registered).

For test frames in the door zone and the carry phase where the boundary PF
error is >= 15 cm (every 6th frame), compare the scan log-likelihood at the
eval-only GT pose with the likelihood at the (wrong) estimate. If the GT pose
scores lower, the measurement itself prefers the wrong pose (ambiguity or a
false band), which a better filter alone would not fix. Also reports the error
direction in the door zone per episode. Evaluation only; nothing here feeds a
localizer.
"""
from __future__ import annotations

import argparse
import collections
import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[1]))

import markerless_probe as mp  # noqa: E402
import run_probe as rp  # noqa: E402


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--estimates', type=Path, required=True)
    ap.add_argument('--calibration', type=Path, default=HERE/'calibration_dev.json')
    ap.add_argument('--every', type=int, default=6)
    ap.add_argument('--min-err', type=float, default=.15)
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args(argv)
    view, _ = mp.load_m1_map()
    geo = mp.MapGeometry(view)
    meas = {**mp.DEFAULT_MEASUREMENT, **json.loads(args.calibration.read_text())['measurement']}
    cols = mp.column_positions(mp.DEFAULT_DETECTOR['columns'], mp.DEFAULT_DETECTOR['strip_half_px'])
    amb = collections.defaultdict(list)
    door = {}
    for ep in rp.EPISODES['test']:
        d = rp.DATA_ROOT/ep
        frames = mp.read_jsonl(d/'inputs'/'frames.jsonl')
        ev = {r['frame']: r for r in mp.read_jsonl(d/'eval_only'/'frames_eval.jsonl')}
        est = {r['frame']: r for r in mp.read_jsonl(args.estimates/f"{ep.replace('/', '__')}.estimates.jsonl")}
        comp = []
        for k, row in enumerate(frames):
            g = ev[row['frame']]['gt']
            r = est[row['frame']]
            grp = rp.group_of(r, g)
            x, y, yaw = r['boundary']['xyyaw']
            err = math.hypot(x - g[0], y - g[1])
            if grp == 'door_zone':
                comp.append((x - g[0], y - g[1], math.degrees((yaw - g[2] + math.pi) % (2*math.pi) - math.pi), err,
                             r['boundary']['n_cols']))
            if grp not in ('door_zone', 'carry') or err < args.min_err or k % args.every:
                continue
            servo = {int(a): int(b) for a, b in row['commanded_servo'].items()}
            loaded = row['report'].get('load_state') == 'loaded'
            b = mp.elevation_bias(meas['bias_rad']['loaded' if loaded else 'unloaded'], servo)
            und = mp.undistort(cv2.imread(str(d/row['file'])))
            cm = mp.column_model(servo, b, cols)
            st = mp.carried_mask_top(und, cols, mp.DEFAULT_DETECTOR['strip_half_px']) if loaded else None
            scan = mp.detect_boundaries(und, cm, None, st)
            if scan.detected.sum() < int(meas['min_columns']):
                amb[grp].append({'too_few_columns': True, 'err': err})
                continue
            poses = np.array([g, [x, y, yaw]])
            vb, vt, vtf, _, _ = geo.expected_rows(poses, cm)
            ll = mp.boundary_loglik(vb, vt, scan, meas, vtf)
            sq = mp.edge_errors(vb, vt, scan, float(meas['sigma_px']), True, vtf_exp=vtf)
            matched = np.isfinite(sq) & (sq < 9.)
            amb[grp].append({'ll_gt_minus_est': float(ll[0] - ll[1]), 'matched_gt': float(matched[0].mean()),
                             'matched_est': float(matched[1].mean()), 'err': err})
        if comp:
            a = np.asarray(comp)
            door[ep] = {'frames': int(len(a)), 'dx_median_m': round(float(np.median(a[:, 0])), 3),
                        'dy_median_m': round(float(np.median(a[:, 1])), 3),
                        'dyaw_median_deg': round(float(np.median(a[:, 2])), 2),
                        'err_median_m': round(float(np.median(a[:, 3])), 3),
                        'share_err_gt_30cm': round(float(np.mean(a[:, 3] > .3)), 3),
                        'scan_columns_median': float(np.median(a[:, 4]))}
    summary = {}
    for grp, rows in amb.items():
        ok = [r for r in rows if 'll_gt_minus_est' in r]
        summary[grp] = {'frames': len(rows), 'too_few_columns': len(rows) - len(ok),
                        'share_gt_scores_higher': round(float(np.mean([r['ll_gt_minus_est'] > 0 for r in ok])), 3),
                        'median_ll_gt_minus_est': round(float(np.median([r['ll_gt_minus_est'] for r in ok])), 3),
                        'mean_matched_columns_gt': round(float(np.mean([r['matched_gt'] for r in ok])), 3),
                        'mean_matched_columns_est': round(float(np.mean([r['matched_est'] for r in ok])), 3)}
    out = {'schema': 'ugrp.markerless_probe.posthoc.v1', 'posthoc': True,
           'note': 'written after the test run; diagnostic only, not a pre-registered result',
           'selection': f'door_zone/carry frames, boundary error >= {args.min_err} m, every {args.every}th frame',
           'likelihood_at_gt_vs_estimate': summary, 'door_zone_error_direction': door}
    args.output.write_text(json.dumps(out, indent=1) + '\n')
    print(json.dumps(out, indent=1))


if __name__ == '__main__':
    main()
