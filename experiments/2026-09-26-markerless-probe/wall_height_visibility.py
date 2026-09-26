"""How taller walls change what the wrist camera can measure (evaluation-only analysis).

For every recorded test frame (eval-only GT pose, own commanded servo pose and
own load state, and the carried-box mask detected in that frame) and for wall
heights H in {0.10, 0.30, 0.50} m (same footprints, door posts dropped), count
the image columns in which

  * the wall's floor contact (bottom edge) is in view and not hidden by the
    carried box, and
  * the wall's top edge (near edge of the top face) is in view and not hidden.

This is geometry only: it does not render taller walls and does not run the
detector. It answers which measurements a taller wall adds or removes along
the M1 trajectories, per phase. GT is used because this is an evaluation of
visibility, not a localization input.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[1]))

import markerless_probe as mp  # noqa: E402
import run_probe as rp  # noqa: E402

HEIGHTS = (.10, .30, .50)


def walls_only(view, h):
    m = json.loads(json.dumps(view))
    for o in m['obstacles']:
        if o.get('kind') == 'wall':
            o['height_m'] = h
    m['landmarks'] = {'tags': [], 'door_posts': []}
    return m


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--episodes', nargs='+', default=rp.EPISODES['test'])
    ap.add_argument('--calibration', type=Path, required=True)
    ap.add_argument('--every', type=int, default=2)
    ap.add_argument('--output', required=True)
    args = ap.parse_args(argv)
    m1 = mp.load_m1_localizer()
    view, _ = mp.load_m1_map()
    geos = {h: mp.MapGeometry(walls_only(view, h), include_posts=False) for h in HEIGHTS}
    cal = json.loads(args.calibration.read_text())
    bias = cal['measurement']['bias_rad']
    cols = mp.column_positions(mp.DEFAULT_DETECTOR['columns'], mp.DEFAULT_DETECTOR['strip_half_px'])
    stats: dict = {}
    for ep in args.episodes:
        d = rp.DATA_ROOT/ep
        frames = mp.read_jsonl(d/'inputs'/'frames.jsonl')
        cmds = mp.read_jsonl(d/'inputs'/'commands.jsonl')
        gt = {r['frame']: r['gt'] for r in mp.read_jsonl(d/'eval_only'/'frames_eval.jsonl')}
        load = m1.LoadState()
        ci = 0
        for k, row in enumerate(frames):
            while ci < len(cmds) and float(cmds[ci]['t']) < float(row['t']) - 1e-9:
                load.command(cmds[ci])
                ci += 1
            if k % args.every:
                continue
            servo = {int(a): int(b) for a, b in row['commanded_servo'].items()}
            state = 'loaded' if load.loaded else 'unloaded'
            cm = mp.column_model(servo, mp.elevation_bias(bias[state], servo), cols)
            if load.loaded:
                und = mp.undistort(cv2.imread(str(d/row['file'])))
                limit = mp.carried_mask_top(und, cols, mp.DEFAULT_DETECTOR['strip_half_px']).astype(float) - 3.
            else:
                limit = np.full(len(cols), mp.HEIGHT - 1.)
            limit = np.minimum(limit, mp.HEIGHT - 1.)
            grp = rp.group_of(row, gt[row['frame']])
            pose = np.array([gt[row['frame']]])
            for h, geo in geos.items():
                vb, vt, _, _, _ = geo.expected_rows(pose, cm, wall_height_m=h)
                vb, vt = vb[0], vt[0]
                with np.errstate(invalid='ignore'):
                    b_in = np.isfinite(vb) & (vb >= 0) & (vb <= limit)
                    t_in = np.isfinite(vt) & (vt >= 0) & (vt <= limit)
                for key in ('all', grp):
                    s = stats.setdefault(f'{h:.2f}', {}).setdefault(key, {'frames': 0, 'bottom_cols': 0, 'top_cols': 0,
                                                                          'top_only_cols': 0, 'frames_ge4_bottom': 0,
                                                                          'frames_ge4_any': 0, 'frames_none': 0})
                    s['frames'] += 1
                    s['bottom_cols'] += int(b_in.sum())
                    s['top_cols'] += int(t_in.sum())
                    s['top_only_cols'] += int((t_in & ~b_in).sum())
                    s['frames_ge4_bottom'] += int(b_in.sum() >= 4)
                    s['frames_ge4_any'] += int((b_in | t_in).sum() >= 4)
                    s['frames_none'] += int(not (b_in | t_in).any())
    out = {'schema': 'ugrp.markerless_probe.wall_height_visibility.v1', 'episodes': args.episodes,
           'every': args.every, 'columns': len(cols), 'bias_rad': bias,
           'note': 'geometry only (eval-only GT pose, own servo/load, carried-box mask from the frame); '
                   'no rendering of taller walls, no detector', 'heights': {}}
    for h, groups in stats.items():
        out['heights'][h] = {g: {'frames': s['frames'],
                                 'mean_bottom_cols': round(s['bottom_cols']/s['frames'], 2),
                                 'mean_top_cols': round(s['top_cols']/s['frames'], 2),
                                 'mean_top_only_cols': round(s['top_only_cols']/s['frames'], 2),
                                 'share_ge4_bottom': round(s['frames_ge4_bottom']/s['frames'], 4),
                                 'share_ge4_any': round(s['frames_ge4_any']/s['frames'], 4),
                                 'share_no_edge': round(s['frames_none']/s['frames'], 4)} for g, s in groups.items()}
    Path(args.output).write_text(json.dumps(out, indent=1))
    for h, groups in out['heights'].items():
        print(h, json.dumps(groups['all']))
        for g in ('search_approach', 'door_zone', 'carry', 'manipulate', 'look_back'):
            if g in groups:
                print('   ', g, groups[g])


if __name__ == '__main__':
    main()
