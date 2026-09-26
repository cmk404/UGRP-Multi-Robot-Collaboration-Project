"""Post-hoc offline diagnostics (NOT pre-registered; no physics, no model calls).

1. B5 scoring addendum: the pre-registered B5 check (b5_offline.py) could not score the relabelled beam position
   because the referee record carries no pose. Here the beam pose is read from PR #169's observer-only post-run
   replay (states.npz, evaluation only) and compared with the `top_cargo_v2_track` label position.
2. N1 (new blocker found in this cohort): for every new-seed run, the start TOP box detections
   (`top_zone_v2`, the box detector used by `top_cargo_v2`/`top_cargo_v2_track`) are counted per colour against the
   scenario setup (evaluation only). For a colour that is missed, the saved start TOP JPEG is inspected with the
   same hue range and a relaxed saturation floor to show where the box is and why the shipped gate rejects it.
   No threshold is changed; this only documents the root cause.

  .venv-sim-worker-mac/bin/python experiments/2026-09-26-zone-teacher-fix/offline_addendum.py \\
      --raw outputs/zone-teacher-fix-20260926 --b5 <b5-offline.json> --output <json>
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

import cv2
import mujoco
import numpy as np

from harness import zone_color_boxes as zcb
from sim.zone_arena import top_views

A2_S12 = Path('/Users/changmin/projects/ugrp-worktrees/zone-team-a2/outputs/zone-team-a2-20260925/a-two-dynamic-s12')
RUNS = [f'mix-{m}-s{s}' for s in (21, 22, 23) for m in ('dynamic', 'independent')] + \
    [f'tri-dynamic-s{s}' for s in (21, 22)]
RELAXED_S_MIN = 60


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def beam_replay_pose(src, t_s=1800.):
    rep = src/'replay'
    m = mujoco.MjModel.from_binary_path(str(rep/'model.mjb'))
    z = np.load(rep/'states.npz')
    j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, 'cargo_long_beam_0_free')
    a = m.jnt_qposadr[j]
    i = min(int(np.searchsorted(z['time'], t_s)), len(z['time'])-1)
    q = z['qpos'][i, a:a+7]
    w, x, y, zq = q[3:7]
    return {'t_s': float(z['time'][i]), 'xy_m': [round(float(q[0]), 4), round(float(q[1]), 4)],
            'yaw_rad': round(math.atan2(2*(w*zq + x*y), 1 - 2*(y*y + zq*zq)), 4),
            'files_sha256': {'states.npz': sha(rep/'states.npz'), 'model.mjb': sha(rep/'model.mjb')},
            'scope': 'observer-only post-run replay; evaluation only'}


def relaxed_components(jpeg, kind):
    frame = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    strict = zcb._mask(hsv, zcb.TOP_ZONE_HSV[kind])
    relaxed = zcb._mask(hsv, [((lo[0], RELAXED_S_MIN, lo[2]), hi) for lo, hi in zcb.TOP_ZONE_HSV[kind]])
    k = np.ones((3, 3), np.uint8)
    rel = cv2.morphologyEx(cv2.morphologyEx(relaxed, cv2.MORPH_OPEN, k), cv2.MORPH_CLOSE, k)
    n, lab, stats, cent = cv2.connectedComponentsWithStats(rel)
    out = []
    for c in range(1, n):
        area = int(stats[c, cv2.CC_STAT_AREA])
        if not zcb.TOP_ZONE_AREA_PX[0] <= area <= zcb.TOP_ZONE_AREA_PX[1]:
            continue
        px = lab == c
        s = hsv[..., 1][px]
        out.append({'centroid_px': [round(float(v), 1) for v in cent[c]], 'relaxed_area_px': area,
                    'strict_mask_px_in_component': int((strict[px] > 0).sum()),
                    'strict_area_gate_px': zcb.TOP_ZONE_AREA_PX[0],
                    'hue_median': float(np.median(hsv[..., 0][px])), 'sat_median': float(np.median(s)),
                    'sat_p10_p90': [float(np.percentile(s, 10)), float(np.percentile(s, 90))],
                    'val_median': float(np.median(hsv[..., 2][px])),
                    'strict_sat_min': int(zcb.TOP_ZONE_HSV[kind][0][0][1])})
    return out


def n1(raw):
    rows = {}
    for run in RUNS:
        d = raw/run
        if not (d/'result.json').is_file():
            rows[run] = {'status': 'no_result'}
            continue
        setup = json.loads((d/'episode-setup-only.json').read_text())
        truth = {}
        for o in setup['setup_only']['objects'].values():
            truth[o['kind']] = truth.get(o['kind'], 0) + 1
        seen, per_cam, files = {}, {}, {}
        cams = {c['name']: c for c in setup['static_map']['top_cameras']}
        for name, _, _, suffix, _ in top_views(setup['static_map']):
            camera = cams[name]
            f = d/'rgb'/f'001-start-{suffix}.jpg'
            jpeg = f.read_bytes()
            files[f.name] = hashlib.sha256(jpeg).hexdigest()
            dets = zcb.detect_top(jpeg, camera, tuple(truth), profile=zcb.TOP_PROFILE_ZONE)
            per_cam[suffix] = [(x['kind'], [round(v, 3) for v in x['floor_xy_m']]) for x in dets]
            for x in dets:
                seen.setdefault(x['kind'], []).append(x['floor_xy_m'])
        # TOP views overlap: count distinct positions (>= 0.1 m apart) per colour
        counts = {}
        for kind, pts in seen.items():
            uniq = []
            for p in pts:
                if all(math.dist(p, q) > .1 for q in uniq):
                    uniq.append(p)
            counts[kind] = len(uniq)
        missed = {k: n - counts.get(k, 0) for k, n in truth.items() if counts.get(k, 0) < n}
        row = {'setup_box_counts': truth, 'start_top_zone_v2_counts': counts, 'missed': missed,
               'labels_at_start': json.loads((d/'result.json').read_text())['labels'],
               'detections_per_camera': per_cam, 'files_sha256': files}
        if missed:
            gt = {oid: o for oid, o in setup['setup_only']['objects'].items() if o['kind'] in missed}
            row['missed_setup_positions_eval_only'] = {oid: {'kind': o['kind'], 'xy_m': o['position_m'][:2]}
                                                       for oid, o in gt.items()}
            row['relaxed_inspection'] = {}
            for _, _, _, suffix, _ in top_views(setup['static_map']):
                jpeg = (d/'rgb'/f'001-start-{suffix}.jpg').read_bytes()
                for kind in missed:
                    comps = [c for c in relaxed_components(jpeg, kind) if c['strict_mask_px_in_component']
                             < c['strict_area_gate_px']]
                    if comps:
                        row['relaxed_inspection'][f'{suffix}/{kind}'] = comps
        rows[run] = row
    return rows


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--raw', type=Path, required=True)
    p.add_argument('--b5', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    b5 = json.loads(args.b5.read_text())
    gt = beam_replay_pose(A2_S12)
    track = b5['profiles']['top_cargo_v2_track']['long_beam-1_rgb_xy_after']
    base = b5['profiles']['top_cargo_v2']['long_beam-1_rgb_xy_after']
    out = {'scope': 'post-hoc offline diagnostics, not pre-registered; no physics; evaluation data only for scoring',
           'b5_scoring': {'beam_replay_pose': gt,
                          'track_label_xy_m': track, 'track_error_m': round(math.dist(track, gt['xy_m']), 4),
                          'baseline_label_xy_m': base, 'baseline_error_m': round(math.dist(base, gt['xy_m']), 4)},
           'n1_start_box_detection': n1(args.raw), 'relaxed_sat_min_for_inspection_only': RELAXED_S_MIN}
    args.output.write_text(json.dumps(out, indent=2) + '\n')
    print(json.dumps(out['b5_scoring'] | {'beam_replay_pose': gt['xy_m']}))
    for run, r in out['n1_start_box_detection'].items():
        print(run, r.get('missed'), r.get('relaxed_inspection') and
              {k: [(c['centroid_px'], c['sat_median'], c['strict_mask_px_in_component']) for c in v]
               for k, v in r['relaxed_inspection'].items()})


if __name__ == '__main__':
    main()
