"""Offline own-camera localization: detect, calibrate (dev), localize, score.

Stages are separated so the run-time path cannot see simulator truth:

* ``detect``    inputs/ frames -> derived/detections.jsonl (robot inputs only)
* ``calibrate`` DEV episodes only: fit the issued-command motion model and the
                tag measurement noise against eval_only/ truth (teacher logs
                used as training data; provenance recorded)
* ``localize``  inputs/ + derived/ + static map + calibration -> derived/
                estimates.jsonl. Never opens eval_only/.
* ``score``     joins estimates with eval_only/ truth -> metrics JSON, checked
                against thresholds fixed before the test split was scored.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

DOOR_REGION = {'half_x_m': .6, 'half_y_m': .5}


def read_jsonl(path):
    with open(path) as fh:
        return [json.loads(line) for line in fh if line.strip()]


def write_jsonl(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(''.join(json.dumps(r) + '\n' for r in rows))


def file_sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def episode_dirs(root, ids):
    return [Path(root)/i for i in ids]


def static_for(ep):
    from sim.zone_landmarks import tagged_map
    manifest = json.loads((ep/'manifest.json').read_text())
    static = tagged_map(manifest['map_id'])
    from sim.research_dispatch_arena import digest
    if digest(static) != manifest['static_map_sha256']:
        raise ValueError(f'{ep}: static map hash differs from the recording')
    return manifest, static


# ------------------------------------------------------------------ detect
def detect(ep):
    import cv2
    from harness.wall_tags import TagDetector
    _, static = static_for(ep)
    det = TagDetector.for_map(static)
    rows = []
    for f in read_jsonl(ep/'inputs'/'frames.jsonl'):
        data = (ep/f['file']).read_bytes()
        if hashlib.sha256(data).hexdigest() != f['sha256']:
            raise ValueError(f'{ep}/{f["file"]}: frame hash mismatch')
        bgr = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        rows.append({'frame': f['frame'], 't': f['t'], 'detections': det.detect(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))})
    write_jsonl(ep/'derived'/'detections.jsonl', rows)
    return rows


# ------------------------------------------------------------------ localize (robot inputs only)
def load_inputs(ep):
    """Everything the localizer may read: its own inputs and derived detections."""
    return {'commands': read_jsonl(ep/'inputs'/'commands.jsonl'),
            'frames': read_jsonl(ep/'inputs'/'frames.jsonl'),
            'detections': {r['frame']: r['detections'] for r in read_jsonl(ep/'derived'/'detections.jsonl')}}


def localize(static, inputs, params, seed=0):
    from harness.owncam_localizer import OwnCamLocalizer
    loc = OwnCamLocalizer(static, params, seed=seed)
    commands = sorted(inputs['commands'], key=lambda c: c['t'])
    ci, out = 0, []
    for f in sorted(inputs['frames'], key=lambda r: r['t']):
        # A command stamped with the frame's SIM time was issued in the next
        # physics step, after the capture (recorder order: step, then capture).
        while ci < len(commands) and commands[ci]['t'] < f['t'] - 1e-6:
            loc.command(commands[ci])
            ci += 1
        own = {int(k): int(v) for k, v in loc.servo.items()}
        frame_pose = {int(k): int(v) for k, v in f['commanded_servo'].items()}
        if own != frame_pose:
            raise ValueError(f'frame {f["frame"]}: commanded servo differs from the command log replay')
        dets = inputs['detections'].get(f['frame'], [])
        est = loc.update(f['t'], dets, own)
        out.append({'frame': f['frame'], 'posture': f['posture'], 'tags': [d['id'] for d in dets], **est})
    return out, loc.stats


# ------------------------------------------------------------------ calibrate (dev only, uses eval truth)
def _active_series(commands, times):
    cmds = sorted(commands, key=lambda c: c['t'])
    u, exp, ci, out = np.zeros(3), -1., 0, []
    for t in times:
        while ci < len(cmds) and cmds[ci]['t'] <= t + 1e-9:
            c = cmds[ci]
            if c['kind'] == 'mecanum':
                u, exp = np.array([c['forward'], c['left'], c['turn']], float), c['t'] + c['duration_s']
            elif c['kind'] == 'drive':
                u, exp = np.array([c['forward'], 0., c['turn']], float), c['t'] + c['duration_s']
            elif c['kind'] not in ('arm', 'look', 'initial_servo_command'):
                u, exp = np.zeros(3), -1.
            ci += 1
        out.append(u if t < exp - 1e-9 else np.zeros(3))
    return np.array(out)


def _gt_body_velocity(gt):
    t = np.array([g['t'] for g in gt])
    x, y, yaw = (np.array([g[k] for g in gt]) for k in ('x', 'y', 'yaw'))
    dt = np.diff(t)
    dx, dy = np.diff(x), np.diff(y)
    c, s = np.cos(yaw[:-1]), np.sin(yaw[:-1])
    dyaw = (np.diff(yaw) + np.pi) % (2*np.pi) - np.pi
    return t[:-1], dt, np.stack(((c*dx + s*dy)/dt, (-s*dx + c*dy)/dt, dyaw/dt), axis=1)


def _lagged(u, dt, tau):
    v, out = np.zeros(3), []
    for ui, d in zip(u, dt):
        out.append(v.copy())
        v = v + (1 - math.exp(-d/max(tau, 1e-6)))*(ui - v)
    return np.array(out)


def calibrate(eps, base_params):
    from harness.wall_tags import angle_between, observed_tag_in_camera, predicted_tag_in_camera, tags_by_id
    series = []
    for ep in eps:
        gt = read_jsonl(ep/'eval_only'/'gt_trajectory.jsonl')
        t, dt, v = _gt_body_velocity(gt)
        u = _active_series(read_jsonl(ep/'inputs'/'commands.jsonl'), t)
        series.append((u, dt, v))
    best = None
    for tau in np.arange(0., .61, .02):
        U = np.vstack([_lagged(u, dt, tau) for u, dt, _ in series])
        V = np.vstack([v for _, _, v in series])
        gain_t, *_ = np.linalg.lstsq(U, V, rcond=None)
        res = V - U @ gain_t
        score = float(np.sum(res**2))
        if best is None or score < best[0]:
            best = (score, float(tau), gain_t.T, U, V, res)
    _, tau, gain, U, V, res = best
    vm = U @ gain.T
    # noise std = rel*|v| + abs per axis: regress |res| * sqrt(pi/2) on |v_model|
    rel, ab = [], []
    for k in range(3):
        a = np.column_stack((np.abs(vm[:, k]), np.ones(len(vm))))
        coef, *_ = np.linalg.lstsq(a, np.abs(res[:, k])*math.sqrt(math.pi/2), rcond=None)
        rel.append(max(float(coef[0]), 0.)); ab.append(max(float(coef[1]), 1e-4))
    # persistent slip: ratio of true to modelled displacement over 2 s moving windows
    ratios = []
    for u, dt, v in series:
        m = _lagged(u, dt, tau) @ gain.T
        step = int(round(2./float(np.median(dt))))
        for i in range(0, len(v) - step, step):
            dm, dg = (m[i:i+step]*dt[i:i+step, None]).sum(0), (v[i:i+step]*dt[i:i+step, None]).sum(0)
            for k, thr in ((0, .05), (1, .05), (2, .15)):
                if abs(dm[k]) > thr:
                    ratios.append((k, dg[k]/dm[k]))
    scale_std = float(np.std([r for _, r in ratios])) if ratios else .05
    # measurement noise from dev detections vs truth-predicted tag geometry
    errs = {'bearing': [], 'azimuth': [], 'elevation': [], 'range_log': [], 'normal': [], 'range_m': []}
    for ep in eps:
        _, static = static_for(ep)
        tags = tags_by_id(static)
        frames = {f['frame']: f for f in read_jsonl(ep/'inputs'/'frames.jsonl')}
        truth = {g['frame']: g for g in read_jsonl(ep/'eval_only'/'frames_gt.jsonl')}
        for row in read_jsonl(ep/'derived'/'detections.jsonl'):
            g, f = truth[row['frame']], frames[row['frame']]
            pose = {int(k): int(v) for k, v in f['commanded_servo'].items()}
            if f['posture'] == 'arm_moving':
                continue  # the arm may lag its issued setpoint while moving
            for d in row['detections']:
                t_obs, n_obs = observed_tag_in_camera(d)
                p_c, n_c = predicted_tag_in_camera(np.array([[g['x'], g['y'], g['yaw']]]), tags[d['id']], pose)
                errs['bearing'].append(float(angle_between(t_obs[None], p_c)[0]))
                q = p_c[0]
                errs['azimuth'].append(math.atan2(t_obs[0], t_obs[2]) - math.atan2(q[0], q[2]))
                errs['elevation'].append(math.atan2(t_obs[1], t_obs[2]) - math.atan2(q[1], q[2]))
                errs['range_log'].append(float(math.log(np.linalg.norm(t_obs)/np.linalg.norm(p_c[0]))))
                errs['normal'].append(float(min(angle_between(n[None], n_c)[0] for n in n_obs)))
                errs['range_m'].append(float(np.linalg.norm(p_c[0])))
    def robust(values):
        a = np.asarray(values)
        return float(1.4826*np.median(np.abs(a - np.median(a))))
    def rms(values):
        a = np.asarray(values)
        return float(np.sqrt(np.median(a**2)) + 1.4826*np.median(np.abs(a - np.median(a))))
    b = np.asarray(errs['bearing'])
    params = copy.deepcopy(base_params)
    params['motion'].update(gain=np.round(gain, 4).tolist(), tau_s=round(tau, 3),
                            noise_rel=[round(v, 4) for v in rel], noise_abs=[round(v, 5) for v in ab],
                            scale_std=round(scale_std, 4))
    params['measurement'].update(
        # RMS about zero (bias included: the filter has no bias term)
        azimuth_std_rad=round(rms(errs['azimuth']), 5),
        elevation_std_rad=round(rms(errs['elevation']), 5),
        range_log_std=round(rms(errs['range_log']), 4),
        normal_std_rad=round(max(float(np.median(errs['normal'])), math.radians(5.)), 4))
    rng = np.asarray(errs['range_m'])
    report = {'tau_s': tau, 'gain': gain.round(4).tolist(), 'velocity_residual_rms': res.std(0).round(4).tolist(),
              'slip_ratio_std': scale_std, 'detections': len(b),
              'bearing_deg': {'median': math.degrees(float(np.median(b))), 'p90': math.degrees(float(np.percentile(b, 90)))},
              'azimuth_deg': {'mean': math.degrees(float(np.mean(errs['azimuth']))),
                              'std': math.degrees(float(np.std(errs['azimuth'])))},
              'elevation_deg': {'mean': math.degrees(float(np.mean(errs['elevation']))),
                                'std': math.degrees(float(np.std(errs['elevation'])))},
              'range_log': {'median': float(np.median(errs['range_log'])), 'robust_std': robust(errs['range_log'])},
              'normal_deg': {'median': math.degrees(float(np.median(errs['normal']))),
                             'p90': math.degrees(float(np.percentile(errs['normal'], 90)))},
              'by_range': {f'{lo:.1f}-{hi:.1f}m': {'n': int(np.sum((rng >= lo) & (rng < hi))),
                                                   'bearing_p50_deg': math.degrees(float(np.median(b[(rng >= lo) & (rng < hi)]))) if np.any((rng >= lo) & (rng < hi)) else None,
                                                   'range_log_p50_abs': float(np.median(np.abs(np.asarray(errs['range_log'])[(rng >= lo) & (rng < hi)]))) if np.any((rng >= lo) & (rng < hi)) else None}
                           for lo, hi in ((0, .5), (.5, 1), (1, 2), (2, 3), (3, 5), (5, 9))}}
    return params, report


# ------------------------------------------------------------------ score (eval truth)
def in_door_region(x, y, static):
    for p in static.get('passages', []):
        if p['kind'] == 'door' and abs(x - p['center_m'][0]) <= DOOR_REGION['half_x_m'] and \
                abs(y - p['center_m'][1]) <= DOOR_REGION['half_y_m']:
            return True
    return False


def posture_group(label):
    if label.endswith(('_c', '_l', '_r')):
        return label  # e.g. search_c, level_l, carry_level_r
    return label


def score_episode(ep, estimates):
    manifest, static = static_for(ep)
    truth = {g['frame']: g for g in read_jsonl(ep/'eval_only'/'frames_gt.jsonl')}
    rows = []
    for e in estimates:
        g = truth[e['frame']]
        row = {'episode': ep.name, 'mode': manifest['spec']['mode'], 'split': manifest['spec']['split'],
               'frame': e['frame'], 't': e['t'], 'posture': e['posture'], 'visible': bool(e['tags']),
               'n_tags': len(e['tags']), 'door_region': in_door_region(g['x'], g['y'], static),
               'initialized': e['initialized'], 'box_z': g['box_z']}
        if e['initialized']:
            row.update(pos_err_m=math.hypot(e['x'] - g['x'], e['y'] - g['y']),
                       yaw_err_deg=abs(math.degrees((e['yaw'] - g['yaw'] + math.pi) % (2*math.pi) - math.pi)),
                       std_xy_m=e['std_xy_m'], std_yaw_deg=math.degrees(e['std_yaw_rad']),
                       since_tag_s=e['since_tag_s'])
        rows.append(row)
    return rows


def summarize(rows):
    def stats(sel):
        ok = [r for r in sel if r['initialized']]
        out = {'frames': len(sel), 'initialized': len(ok),
               'visible_rate': round(sum(r['visible'] for r in sel)/len(sel), 3) if sel else None}
        if ok:
            p = np.array([r['pos_err_m'] for r in ok]); y = np.array([r['yaw_err_deg'] for r in ok])
            out.update(pos_m={'p50': round(float(np.median(p)), 4), 'p90': round(float(np.percentile(p, 90)), 4),
                              'max': round(float(p.max()), 4)},
                       yaw_deg={'p50': round(float(np.median(y)), 3), 'p90': round(float(np.percentile(y, 90)), 3),
                                'max': round(float(y.max()), 3)})
        return out
    groups = {}
    for key in ('mode', 'posture', 'episode'):
        for value in sorted({r[key] for r in rows}):
            groups[f'{key}={value}'] = stats([r for r in rows if r[key] == value])
    for mode in sorted({r['mode'] for r in rows}):
        groups[f'door_region&mode={mode}'] = stats([r for r in rows if r['door_region'] and r['mode'] == mode])
    groups['all'] = stats(rows)
    return groups


def gates(rows, thresholds):
    out = {}
    for gate in thresholds['gates']:
        sel = [r for r in rows if r['mode'] in gate['modes'] and (not gate.get('door_region') or r['door_region'])
               and (not gate.get('stationary_look') or r['posture'].endswith(('_c', '_l', '_r')))]
        missing = sum(not r['initialized'] for r in sel)
        ok = [r for r in sel if r['initialized']]
        res = {'frames': len(sel), 'uninitialized': missing}
        if ok:
            p = np.array([r['pos_err_m'] for r in ok]); y = np.array([r['yaw_err_deg'] for r in ok])
            q = gate['quantile']
            res.update(pos_m=round(float(np.percentile(p, q)), 4), yaw_deg=round(float(np.percentile(y, q)), 3))
            res['pass'] = bool(missing == 0 and res['pos_m'] < gate['pos_m'] and res['yaw_deg'] < gate['yaw_deg'])
        else:
            res['pass'] = False
        out[gate['id']] = {**gate, **res}
    return out


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('stage', choices=('detect', 'calibrate', 'localize', 'score'))
    p.add_argument('--data', required=True, help='root with one directory per episode')
    p.add_argument('--episodes', required=True, help='comma-separated episode ids')
    p.add_argument('--calibration', help='calibration JSON (written by calibrate, read by localize)')
    p.add_argument('--thresholds', help='pre-registered thresholds JSON (score)')
    p.add_argument('--output', help='metrics JSON (score)')
    p.add_argument('--seed', type=int, default=0)
    args = p.parse_args(argv)
    eps = episode_dirs(args.data, [e for e in args.episodes.split(',') if e])
    if args.stage == 'detect':
        for ep in eps:
            rows = detect(ep)
            print(ep.name, len(rows), sum(bool(r['detections']) for r in rows), flush=True)
    elif args.stage == 'calibrate':
        splits = {json.loads((ep/'manifest.json').read_text())['spec']['split'] for ep in eps}
        if splits != {'dev'}:
            raise SystemExit('calibration uses dev episodes only')
        from harness.owncam_localizer import DEFAULT_PARAMS
        params, report = calibrate(eps, DEFAULT_PARAMS)
        value = {'schema': 'ugrp.owncam_loc_calibration.v1', 'params': params, 'report': report,
                 'provenance': {'episodes': [ep.name for ep in eps],
                                'files': {f'{ep.name}/{name}': file_sha(ep/name) for ep in eps for name in (
                                    'inputs/commands.jsonl', 'eval_only/gt_trajectory.jsonl',
                                    'eval_only/frames_gt.jsonl', 'derived/detections.jsonl')},
                                'note': 'teacher logs (simulator truth) used offline as training data only'}}
        Path(args.calibration).write_text(json.dumps(value, indent=2) + '\n')
        print(json.dumps(report, indent=1))
    elif args.stage == 'localize':
        params = json.loads(Path(args.calibration).read_text())['params']
        for ep in eps:
            _, static = static_for(ep)
            est, st = localize(static, load_inputs(ep), params, seed=args.seed)
            write_jsonl(ep/'derived'/'estimates.jsonl', est)
            print(ep.name, len(est), st, flush=True)
    else:
        thresholds = json.loads(Path(args.thresholds).read_text())
        rows = []
        for ep in eps:
            rows += score_episode(ep, read_jsonl(ep/'derived'/'estimates.jsonl'))
        value = {'schema': 'ugrp.owncam_loc_metrics.v1', 'episodes': [ep.name for ep in eps],
                 'thresholds_sha256': file_sha(args.thresholds), 'gates': gates(rows, thresholds),
                 'summary': summarize(rows),
                 'estimates_sha256': {ep.name: file_sha(ep/'derived'/'estimates.jsonl') for ep in eps}}
        Path(args.output).write_text(json.dumps(value, indent=2) + '\n')
        print(json.dumps(value['gates'], indent=1))


if __name__ == '__main__':
    main()
