"""Offline tag-free localization probe on the M1 wrist-camera recordings.

Subcommands (see README.md):

  localize   robot inputs only (own frames, own commands, static map, fixed
             calibration). Writes per-frame estimates of every filter. Never
             opens ``eval_only/``.
  calibrate  dev split only: fits the boundary elevation bias per own load state
             from dev frames and the eval-only GT pose (offline fit, like the
             M1 tag calibration) and writes a calibration JSON.
  score      joins estimates with ``eval_only/frames_eval.jsonl`` (GT and the
             online tag PF errors of the recorded run) and writes metrics.

Filters:
  boundary   tag-free: floor/wall-boundary pseudo scan + M1 motion model.
  deadreck   same PF, no measurement (issued-command dead reckoning).
  tag_replay the M1 tag PF (AprilTag PnP) replayed offline on the same frames.
The online tag PF of the recorded run is scored from ``frames_eval.jsonl``.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[1]))

import markerless_probe as mp  # noqa: E402
from harness.wall_tags import TagDetector  # noqa: E402

DATA_ROOT = Path('/Users/changmin/projects/ugrp/outputs/m1-owncam-20260926')
EPISODES = {
    'dev': ['dev-a8/m1dev-s93', 'dev-a8/m1devdiag-s95', 'dev-a6/m1devdiag-s94'],
    'test': [f'test/m1test-s{s}' for s in range(101, 107)],
}
# Static layout docks (sim/zone_arena LAYOUTS['zone_wide']): robots start at their own dock.
DOCK_X, DOCK_YAW = -.85, 0.
DOCK_STD = (.15, .15, math.radians(10.))


def load_cal(path: Path | None) -> dict:
    return json.loads(Path(path).read_text()) if path else {}


class TagFilter:
    """The M1 tag PF as the M1 pose source drives it (detections on the RGB frame)."""

    def __init__(self, m1, full_map, params, seed):
        self.loc = m1.OwnCamLocalizer(full_map, params, seed=seed)
        self.det = TagDetector.for_map(full_map, 640, 480)
        self.servo = {}

    def command(self, row):
        self.loc.command(row)
        if row['kind'] == 'initial_servo_command':
            self.servo = {int(k): int(v) for k, v in row['pulses'].items()}
        elif row['kind'] == 'arm':
            self.servo[int(row['servo_id'])] = int(row['pulse'])
        elif row['kind'] == 'look':
            self.servo[6] = int(row['pan_pulse'])

    def set_motion_profile(self, t, name):
        if name != self.loc.motion_profile:
            self.loc.set_motion_profile(t, name)

    def frame(self, t, bgr, row):
        dets = self.det.detect(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
        self.last = self.loc.update(t, dets, dict(self.servo))
        self.last['n_tags'] = len(dets)


class BoundaryFilter:
    """Tag-free PF: boundary scans (or none, for dead reckoning) + the M1 motion model."""

    def __init__(self, m1, view_map, params, cal, seed, init, use_scan=True, keep_scans=False):
        meas = cal.get('measurement', {})
        det = cal.get('detector', {})
        self.loc = mp.make_boundary_pf(m1, view_map, params, meas, det, seed)
        self.use_scan = use_scan
        self.keep_scans = keep_scans
        self.init = init
        self.servo = {}
        self.global_frames = 0

    def start(self, dock_y):
        if self.init == 'dock':
            self.loc.init_gaussian((DOCK_X, dock_y, DOCK_YAW), DOCK_STD)
        elif self.init == 'global':
            self.loc.init_uniform(20000)
        else:
            raise ValueError(self.init)

    def command(self, row):
        self.loc.command(row)
        if row['kind'] == 'initial_servo_command':
            self.servo = {int(k): int(v) for k, v in row['pulses'].items()}
        elif row['kind'] == 'arm':
            self.servo[int(row['servo_id'])] = int(row['pulse'])
        elif row['kind'] == 'look':
            self.servo[6] = int(row['pan_pulse'])

    def set_motion_profile(self, t, name):
        if name != self.loc.motion_profile:
            self.loc.set_motion_profile(t, name)

    def frame(self, t, bgr, row):
        scan = None
        if self.use_scan:
            und = mp.undistort(bgr)
            scan = self.loc.scan(und, self.servo)
            self.last = self.loc.update_scan(t, scan, self.servo)
            if self.init == 'global' and self.loc.n > 2000:
                self.global_frames += 1
                if self.global_frames >= 50:          # ~10 s at 5 Hz, then track with 2000
                    self.loc.resize(2000)
        else:
            self.loc.predict_to(t)
            self.last = self.loc.estimate()
        self.last['n_cols'] = 0 if scan is None else int(scan.detected.sum())
        self.last['scan'] = scan.as_dict() if (scan is not None and self.keep_scans) else None


def servo_check(filters, row):
    want = {int(k): int(v) for k, v in row['commanded_servo'].items()}
    return all(getattr(f, 'servo', want) == want for f in filters.values())


def localize(args):
    m1 = mp.load_m1_localizer()
    view_map, map_prov = mp.load_m1_map()
    full_map = json.loads(mp.git_blob(mp.M1_SHA, mp.M1_MAP))
    m1_cal, m1_cal_prov = mp.load_m1_calibration()
    params = m1_cal['params']
    cal = load_cal(args.calibration)
    out_dir = Path(args.output)
    out_dir.mkdir(parents=True, exist_ok=True)
    wanted = args.filters.split(',')
    for ep in args.episodes:
        ep_dir = DATA_ROOT/ep
        inputs = mp.episode_inputs(ep_dir)
        spec = inputs['manifest']['spec']
        seed = int(spec['seed'])
        filters = {}
        if 'boundary' in wanted:
            filters['boundary'] = BoundaryFilter(m1, view_map, params, cal, seed, 'dock', keep_scans=args.keep_scans)
        if 'boundary_global' in wanted:
            filters['boundary_global'] = BoundaryFilter(m1, view_map, params, cal, seed, 'global')
        if 'deadreck' in wanted:
            filters['deadreck'] = BoundaryFilter(m1, view_map, params, cal, seed, 'dock', use_scan=False)
        if 'tag_replay' in wanted:
            filters['tag_replay'] = TagFilter(m1, full_map, params, seed)
        for f in filters.values():
            if hasattr(f, 'start'):
                f.start(float(spec['spawn_y']))
        rows_out = []
        mismatches = 0
        t0 = time.time()

        def on_frame(k, row):
            nonlocal mismatches
            mismatches += 0 if servo_check(filters, row) else 1
            rec = {'frame': row['frame'], 't': row['t'], 'phase': row['phase'], 'skill_phase': row['skill_phase'],
                   'load_state': row['report'].get('load_state'), 's3': int(row['commanded_servo']['3']),
                   's6': int(row['commanded_servo']['6'])}
            for name, f in filters.items():
                e = f.last
                rec[name] = None if not e.get('initialized') else {
                    'xyyaw': [round(e['x'], 5), round(e['y'], 5), round(e['yaw'], 6)],
                    'std_xy_m': round(e['std_xy_m'], 5), 'std_yaw_rad': round(e['std_yaw_rad'], 5),
                    **{k2: e[k2] for k2 in ('n_cols', 'n_tags', 'since_scan_s', 'since_tag_s') if k2 in e},
                    **({'scan': e['scan']} if e.get('scan') else {})}
            rows_out.append(rec)
        n = mp.replay(ep_dir, filters, on_frame=on_frame)
        name = ep.replace('/', '__')
        with open(out_dir/f'{name}.estimates.jsonl', 'w') as fh:
            for r in rows_out:
                fh.write(json.dumps(r) + '\n')
        stats = {k: getattr(f.loc, 'stats', {}) for k, f in filters.items()}
        meta = {'schema': mp.SCHEMA, 'episode': ep, 'frames': n, 'servo_mismatch_frames': mismatches,
                'filters': list(filters), 'seed': seed, 'dock': [DOCK_X, float(spec['spawn_y']), DOCK_YAW],
                'dock_std': list(DOCK_STD), 'wall_s': round(time.time() - t0, 1), 'stats': stats,
                'load_average': list(os.getloadavg()), 'map': map_prov, 'm1_calibration': m1_cal_prov,
                'm1_localizer': {'source': f'{mp.M1_SHA}:{mp.M1_LOCALIZER}', 'sha256': mp.M1_LOCALIZER_SHA256},
                'probe_calibration': {'path': str(args.calibration) if args.calibration else None,
                                      'sha256': mp.sha256(Path(args.calibration).read_bytes()) if args.calibration else None},
                'probe_module_sha256': mp.sha256((HERE/'markerless_probe.py').read_bytes()),
                'runner_sha256': mp.sha256(Path(__file__).read_bytes())}
        (out_dir/f'{name}.meta.json').write_text(json.dumps(meta, indent=1))
        print(f'{ep}: {n} frames, {meta["wall_s"]} s, servo mismatches {mismatches}', flush=True)


# ----------------------------------------------------------------------------- calibrate (dev, GT offline)
def calibrate(args):
    """Fit the boundary elevation bias per own load state and arm pose on dev frames (GT offline).

    Arm poses are the settled families of the M1 controller (commanded servo 3,
    4, 5 exactly equal to a family); frames in between (macro interpolation)
    are not used for the fit and get the bias interpolated over servo 3.
    """
    if any(not ep.startswith('dev') for ep in args.episodes):
        raise SystemExit('calibrate reads dev episodes only')
    m1 = mp.load_m1_localizer()
    view_map, _ = mp.load_m1_map()
    geo = mp.MapGeometry(view_map)
    base = load_cal(args.base)
    det_params = {**mp.DEFAULT_DETECTOR, **base.get('detector', {})}
    seed_bias = base.get('measurement', {}).get('bias_rad', {})
    cols = mp.column_positions(det_params['columns'], int(det_params['strip_half_px']))
    grid = np.round(np.arange(-.07, .00501, .00125), 5)
    resid: dict = {}
    signed: dict = {}
    frames_used: dict = {}
    for ep in args.episodes:
        ep_dir = DATA_ROOT/ep
        frames = mp.read_jsonl(ep_dir/'inputs'/'frames.jsonl')
        gt = {r['frame']: r['gt'] for r in mp.read_jsonl(ep_dir/'eval_only'/'frames_eval.jsonl')}
        cmds = mp.read_jsonl(ep_dir/'inputs'/'commands.jsonl')
        load = m1.LoadState()
        ci = 0
        for k, row in enumerate(frames):
            while ci < len(cmds) and float(cmds[ci]['t']) < float(row['t']) - 1e-9:     # as mp.replay
                load.command(cmds[ci])
                ci += 1
            if k % args.every:
                continue
            servo = {int(a): int(b) for a, b in row['commanded_servo'].items()}
            family = (servo[3], servo[4], servo[5])
            if family not in POSE_FAMILIES:
                continue
            state = 'loaded' if load.loaded else 'unloaded'
            und = mp.undistort(cv2.imread(str(ep_dir/row['file'])))
            st = mp.carried_mask_top(und, cols, int(det_params['strip_half_px'])) if load.loaded else None
            b0 = mp.elevation_bias(seed_bias.get(state, -.0187), servo)
            scan = mp.detect_boundaries(und, mp.column_model(servo, b0, cols), det_params, st)
            if not scan.detected.any():
                continue
            key = f'{state}:{servo[3]}'
            frames_used[key] = frames_used.get(key, 0) + 1
            pose = np.array([gt[row['frame']]])
            limit = np.minimum((st if st is not None else np.full(len(cols), mp.HEIGHT)) - 3., mp.HEIGHT - 1.)
            for b in grid:
                vb, vt, vtf, _, _ = geo.expected_rows(pose, mp.column_model(servo, float(b), cols))
                d = np.sqrt(mp.edge_errors(vb, vt, scan, 1., use_top_edge=False, vtf_exp=vtf))[0]   # px
                resid.setdefault(key, {}).setdefault(float(b), []).extend(d.tolist())   # inf: no match
                # signed bottom-edge residual of the nearest candidate where the bottom is in view
                with np.errstate(invalid='ignore'):
                    vis = scan.detected & np.isfinite(vb[0]) & (vb[0] >= 0) & (vb[0] <= limit)
                diff = scan.vb[vis] - vb[0][vis][:, None]
                a = np.where(np.isfinite(diff), np.abs(diff), np.inf)
                pick = diff[np.arange(len(diff)), a.argmin(1)] if len(diff) else np.zeros(0)
                signed.setdefault(key, {}).setdefault(float(b), []).extend(pick[np.abs(pick) < 6.].tolist())
    out = {'schema': 'ugrp.markerless_probe.calibration.v2', 'split_used': args.episodes, 'every': args.every,
           'method': 'per (own load state, settled arm pose): grid over the elevation bias; per detected column '
                     'the best candidate |row error| vs the eval-only GT pose (bottom edge, or top edge when the '
                     'bottom is hidden); choose the bias with the largest share of |error| < 3 px',
           'pose_families_s3_s4_s5': [list(f) for f in sorted(POSE_FAMILIES)], 'frames_used': frames_used,
           'grid': {}, 'detector': base.get('detector', {})}
    table: dict = {}
    for key, per_b in resid.items():
        rows = []
        for b, e in sorted(per_b.items()):
            e = np.asarray(e)
            fin = e[np.isfinite(e)]
            rows.append({'bias_rad': b, 'n': int(e.size), 'share_lt3px': round(float(np.mean(e < 3)), 4),
                         'unmatched': int((~np.isfinite(e)).sum()),
                         'median_matched_px': None if fin.size == 0 else round(float(np.median(fin)), 3)})
        out['grid'][key] = rows
        state, s3 = key.split(':')
        if frames_used[key] < args.min_frames:
            continue
        # zero crossing of the median signed bottom residual (the residual falls ~FY px/rad
        # with the bias); the share criterion is the fallback when few bottoms are in view
        bs = sorted(signed.get(key, {}))
        med = [(b, float(np.median(signed[key][b])), len(signed[key][b])) for b in bs if len(signed[key][b]) >= 200]
        share_best = max(rows, key=lambda r: r['share_lt3px'])['bias_rad']
        crossings = [round(b0 + (b1 - b0)*m0/(m0 - m1_), 5) for (b0, m0, _), (b1, m1_, _) in zip(med, med[1:])
                     if m0 >= 0. >= m1_ and m0 != m1_]
        crossings = [c for c in crossings if abs(c - share_best) <= .006]      # the crossing at the optimum
        if crossings:
            choice, how = min(crossings, key=lambda c: abs(c - share_best)), 'median_signed_zero'
        else:
            choice, how = share_best, 'share_lt3px'
        share = min(rows, key=lambda r: abs(r['bias_rad'] - choice))['share_lt3px']
        out.setdefault('signed_median_px', {})[key] = [[b, round(m, 3), n] for b, m, n in med]
        table.setdefault(state, []).append((int(s3), choice, share, how))
    bias = {}
    for state, entries in table.items():
        entries.sort()
        bias[state] = {'s3': [e[0] for e in entries], 'bias': [e[1] for e in entries],
                       'share_lt3px_near': [e[2] for e in entries], 'fit': [e[3] for e in entries]}
    out['measurement'] = {**base.get('measurement', {}), 'bias_rad': bias}
    Path(args.output).write_text(json.dumps(out, indent=1))
    print(json.dumps(bias, indent=1), json.dumps(frames_used))


# Settled arm poses of the M1 controller (commanded servo 3, 4, 5): search, carry,
# look (also used loaded). Pan (servo 6) varies within a family.
POSE_FAMILIES = {(740, 2320, 1320), (777, 2053, 1646), (1072, 2400, 1482)}


# ----------------------------------------------------------------------------- score (GT)
def ang(a):
    return (a + math.pi) % (2*math.pi) - math.pi


def pct(a, q):
    return None if len(a) == 0 else round(float(np.percentile(a, q)), 4)


def summary(errs):
    e = np.asarray([x[0] for x in errs])
    y = np.asarray([x[1] for x in errs])
    return {'n': int(e.size), 'pos_p50_m': pct(e, 50), 'pos_p90_m': pct(e, 90), 'pos_p99_m': pct(e, 99),
            'pos_max_m': None if e.size == 0 else round(float(e.max()), 4),
            'yaw_p50_deg': pct(y, 50), 'yaw_p90_deg': pct(y, 90),
            'share_pos_lt_5cm': None if e.size == 0 else round(float(np.mean(e < .05)), 4),
            'share_pos_lt_10cm': None if e.size == 0 else round(float(np.mean(e < .10)), 4)}


def group_of(rec, gt):
    sp = rec['skill_phase'] or ''
    near_door = abs(gt[0] - 2.2) < .6 and -.45 < gt[1] < .55
    if near_door:
        return 'door_zone'
    if sp in ('nav_preplace', 'to_carry_posture', 'pre_release'):
        return 'carry'
    if sp in ('grasp', 'nav_pregrasp', 'release'):
        return 'manipulate'
    if sp == 'look_back':
        return 'look_back'
    return 'search_approach'


def score(args):
    est_dir = Path(args.estimates)
    report = {'schema': 'ugrp.markerless_probe.metrics.v1', 'episodes': {}, 'pooled': {}}
    pooled: dict = {}
    for ep in args.episodes:
        name = ep.replace('/', '__')
        est = mp.read_jsonl(est_dir/f'{name}.estimates.jsonl')
        ev = {r['frame']: r for r in mp.read_jsonl(DATA_ROOT/ep/'eval_only'/'frames_eval.jsonl')}
        per: dict = {}
        for rec in est:
            e = ev[rec['frame']]
            gt = e['gt']
            grp = group_of(rec, gt)
            cands = {k: v for k, v in rec.items() if isinstance(v, dict) and 'xyyaw' in v}
            if e.get('pos_err_m') is not None:
                cands['tag_online'] = {'_err': (float(e['pos_err_m']), float(e['yaw_err_deg']))}
            for fname, v in cands.items():
                if '_err' in v:
                    err = v['_err']
                else:
                    x, y, yaw = v['xyyaw']
                    err = (math.hypot(x - gt[0], y - gt[1]), abs(math.degrees(ang(yaw - gt[2]))))
                for key in ('all', grp):
                    per.setdefault(fname, {}).setdefault(key, []).append(err)
                    pooled.setdefault(fname, {}).setdefault(key, []).append(err)
        report['episodes'][ep] = {f: {g: summary(v) for g, v in d.items()} for f, d in per.items()}
    report['pooled'] = {f: {g: summary(v) for g, v in d.items()} for f, d in pooled.items()}
    Path(args.output).write_text(json.dumps(report, indent=1))
    for f, d in report['pooled'].items():
        a = d['all']
        print(f"{f:16s} n={a['n']:6d} p50={a['pos_p50_m']} p90={a['pos_p90_m']} p99={a['pos_p99_m']} "
              f"max={a['pos_max_m']} yaw_p90={a['yaw_p90_deg']}")
        for g in ('search_approach', 'door_zone', 'carry', 'manipulate', 'look_back'):
            if g in d:
                s = d[g]
                print(f"    {g:16s} n={s['n']:6d} p50={s['pos_p50_m']} p90={s['pos_p90_m']} max={s['pos_max_m']}")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    a = sub.add_parser('localize')
    a.add_argument('--episodes', nargs='+', required=True)
    a.add_argument('--calibration', type=Path)
    a.add_argument('--filters', default='boundary,deadreck,tag_replay')
    a.add_argument('--keep-scans', action='store_true')
    a.add_argument('--output', required=True)
    c = sub.add_parser('calibrate')
    c.add_argument('--episodes', nargs='+', required=True)
    c.add_argument('--base', type=Path)
    c.add_argument('--every', type=int, default=5)
    c.add_argument('--min-frames', type=int, default=20)
    c.add_argument('--output', required=True)
    s = sub.add_parser('score')
    s.add_argument('--episodes', nargs='+', required=True)
    s.add_argument('--estimates', required=True)
    s.add_argument('--output', required=True)
    args = ap.parse_args(argv)
    {'localize': localize, 'calibrate': calibrate, 'score': score}[args.cmd](args)


if __name__ == '__main__':
    main()
