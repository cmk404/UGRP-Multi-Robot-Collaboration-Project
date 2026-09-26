#!/usr/bin/env python3
"""Tag-free vision localization on the environment-v3 teacher renders (offline replay).

Subcommands:

  calibrate  TRAIN split only, GT offline: per (own load state, settled arm pose) the
             camera elevation sag and height offset of the true camera pose against
             the commanded-PWM FK, and the settling time after own arm/pan commands.
  train      segmentation network on TRAIN frames (targets: teacher segmentation
             renders), validation on DEV (``seg_model.train``).
  segment    robot inputs only: own frames -> network -> per-column interval
             observations (npz per episode). Never opens ``eval_only/``.
  oracle     EVAL-ONLY DIAGNOSTIC: the same column observations from the teacher's
             segmentation renders (perfect perception upper bound; never a student).
  localize   robot inputs only (+ the saved observations): PF variants
               vision    learned segmentation + interval likelihood (the student)
               boundary  PR #210 hand-built boundary detector + its likelihood
               deadreck  same PF, no measurement
               oracle    diagnostic, observations from the teacher labels
  score      GT (``eval_only/frames_eval.jsonl``) -> metrics per group; false
             detections of the learned observations against the oracle ones.
  bench      inference cost per frame (CPU / MPS), under the agent lock.
"""
from __future__ import annotations

import argparse
import hashlib
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
import vision_loc as vl  # noqa: E402

mp = vl.mp
PRIMARY_OUT = Path('/Users/changmin/projects/ugrp/outputs/vision-loc-20260926')
RENDER_ROOT = Path(os.environ.get('VL_RENDER_ROOT', PRIMARY_OUT/'render'))
MAP_FILE = HERE/'maps'/'zone_wide_door_walls_v3_notags.json'
DOCK_X, DOCK_YAW = -.85, 0.
DOCK_STD = (.15, .15, math.radians(10.))
DOOR_X, DOOR_Y0, DOOR_Y1 = 2.2, -.2, .3          # door_1 opening (static map)
POSE_FAMILIES = {(740, 2320, 1320): 'search', (777, 2053, 1646): 'carry', (1072, 2400, 1482): 'look_p20'}


def sha_file(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def episodes_table() -> dict:
    return json.loads((HERE/'episodes.json').read_text())


def split_of(ep: str) -> str:
    return next(e['split'] for e in episodes_table()['episodes'] if e['episode_id'] == ep)


def spawn_y(ep: str) -> float:
    return float(next(e['spawn_y'] for e in episodes_table()['episodes'] if e['episode_id'] == ep))


def load_map() -> dict:
    return json.loads(MAP_FILE.read_text())


def load_json(path):
    return json.loads(Path(path).read_text()) if path else {}


def require_frozen(episodes, *, checkpoint=None, config=None, calibration=None):
    """Test episodes only with the pre-registered student: prereg.json present and every frozen hash unchanged."""
    if not any(split_of(e) == 'test' for e in episodes):
        return None
    path = HERE/'prereg.json'
    if not path.exists():
        raise SystemExit('test refused: no prereg.json (register the gate and the frozen student first)')
    pre = json.loads(path.read_text())
    st = pre['student']
    bad = [f for f, h in st['frozen_files_sha256'].items() if sha_file(HERE/f) != h]
    if checkpoint is not None and sha_file(checkpoint) != st['model']['sha256']:
        bad.append('checkpoint')
    if config is not None and sha_file(config) != st['config']['sha256']:
        bad.append('config')
    if calibration is not None and sha_file(calibration) != st['calibration']['sha256']:
        bad.append('calibration')
    if bad:
        raise SystemExit(f'test refused: differs from prereg.json: {bad}')
    return {'prereg_sha256': sha_file(path)}


# ----------------------------------------------------------------------------- calibrate (TRAIN, GT offline)
class OwnState:
    """Own servo pulses, own load state and own servo-command time from own commands."""

    def __init__(self, m1):
        self.load = m1.LoadState()
        self.servo: dict[int, int] = {}
        self.last_servo_cmd_t = -1e9

    def command(self, row):
        self.load.command(row)
        k = row['kind']
        if k == 'initial_servo_command':
            self.servo = {int(a): int(b) for a, b in row['pulses'].items()}
            self.last_servo_cmd_t = float(row['t'])
        elif k == 'arm':
            self.servo[int(row['servo_id'])] = int(row['pulse'])
            self.last_servo_cmd_t = float(row['t'])
        elif k == 'look':
            self.servo[6] = int(row['pan_pulse'])
            self.last_servo_cmd_t = float(row['t'])

    def set_motion_profile(self, t, name):
        pass


def true_camera_in_base(label_row):
    x, y, yaw = label_row['base_gt']
    r_w = np.asarray(label_row['cam_xmat'], float).reshape(3, 3) @ np.diag([1., -1., -1.])  # MuJoCo -> optical
    c, s = math.cos(yaw), math.sin(yaw)
    rz = np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])
    return rz.T @ r_w, rz.T @ (np.asarray(label_row['cam_pos_m'], float) - np.array([x, y, 0.]))


def calibrate(args):
    eps = args.episodes
    if any(split_of(e) != 'train' for e in eps):
        raise SystemExit('calibrate reads TRAIN episodes only')
    m1 = mp.load_m1_localizer()
    rows_by_key: dict = {}
    settle_rows = []
    segments: list = []
    for ep in eps:
        ep_dir = RENDER_ROOT/ep
        labels = {r['frame_id']: r for r in vl.read_jsonl(ep_dir/'eval_only'/'labels.jsonl')}
        own = OwnState(m1)
        frames = vl.read_jsonl(ep_dir/'inputs'/'frames.jsonl')
        cmds = vl.read_jsonl(ep_dir/'inputs'/'commands.jsonl')
        ci = prev_ci = 0
        seg = None
        for row in frames:
            t = float(row['t'])
            while ci < len(cmds) and float(cmds[ci]['t']) < t - 1e-9:
                own.command(cmds[ci])
                ci += 1
            servo = {int(a): int(b) for a, b in row['commanded_servo'].items()}
            if servo != own.servo:
                raise RuntimeError(f'{ep} frame {row["frame"]}: own servo replay differs from the frame record')
            lab = labels[row['frame_id']]
            r_b, p_b = true_camera_in_base(lab)
            b, dz, az = vl.elevation_and_dz(r_b, p_b, servo)
            fam = POSE_FAMILIES.get((servo[3], servo[4], servo[5]))
            dt = t - own.last_servo_cmd_t
            state = 'loaded' if own.load.loaded else 'unloaded'
            settle_rows.append((dt, abs(math.degrees(az)), fam, state, b))
            if fam is not None and dt >= args.settled_s:
                rows_by_key.setdefault((state, servo[3], fam), []).append((b, dz, az))
            # pan -> chassis yaw coupling: hold segments (no wheel command since the last frame)
            wheels = any(c['kind'] in ('mecanum', 'drive') for c in cmds[prev_ci:ci])
            prev_ci = ci
            arm = (servo[3], servo[4], servo[5])
            if wheels or seg is None or seg['state'] != state or seg['arm'] != arm:
                seg = {'state': state, 'arm': arm, 'rows': []}
                segments.append(seg)
            if dt >= args.settled_s:
                seg['rows'].append((servo[6], float(lab['base_gt'][2])))
    table: dict = {}
    fits = {}
    for (state, s3, fam), vals in sorted(rows_by_key.items()):
        a = np.asarray(vals)
        fits[f'{state}:{fam}:{s3}'] = {'n': int(len(a)), 'bias_median_rad': round(float(np.median(a[:, 0])), 5),
                                       'bias_p10_p90_rad': [round(float(np.percentile(a[:, 0], q)), 5) for q in (10, 90)],
                                       'dz_median_m': round(float(np.median(a[:, 1])), 5),
                                       'az_abs_p95_deg': round(float(np.percentile(np.abs(np.degrees(a[:, 2])), 95)), 3)}
        if len(a) >= args.min_frames:
            table.setdefault(state, []).append((s3, float(np.median(a[:, 0])), float(np.median(a[:, 1]))))
    sag = {}
    for state, entries in table.items():
        entries.sort()
        sag[state] = {'s3': [e[0] for e in entries], 'bias': [round(e[1], 5) for e in entries],
                      'dz': [round(e[2], 5) for e in entries]}
    for state in ('unloaded', 'loaded'):
        if state not in sag:
            raise SystemExit(f'no settled {state} frames for the sag table')
    # settling after own arm / pan commands: azimuth lag (pan) and elevation deviation (arm)
    bins = [0., .1, .2, .3, .4, .5, .6, .8, 1.2, 1e9]
    settle = []
    ref = {(st, fam): np.median([v[0] for k, vs in rows_by_key.items() if k[0] == st and k[2] == fam for v in vs])
           for st, _, fam in rows_by_key}
    for lo, hi in zip(bins, bins[1:]):
        sel = [r for r in settle_rows if lo <= r[0] < hi]
        az = np.asarray([r[1] for r in sel])
        el = np.asarray([abs(math.degrees(r[4] - ref[(r[3], r[2])])) for r in sel if (r[3], r[2]) in ref])
        settle.append({'since_cmd_s': [lo, None if hi > 1e8 else hi], 'n': len(sel),
                       'az_abs_p95_deg': None if az.size == 0 else round(float(np.percentile(az, 95)), 3),
                       'el_dev_abs_p95_deg': None if el.size == 0 else round(float(np.percentile(el, 95)), 3)})
    # chassis yaw vs own pan pulse within hold segments (reference: the segment's pan-1500 frames)
    pan_fit, pan_k = {}, {}
    for state in ('unloaded', 'loaded'):
        xs, ys = [], []
        for sg in segments:
            if sg['state'] != state:
                continue
            ref = [yv for pan, yv in sg['rows'] if pan == 1500]
            if not ref:
                continue
            r0 = float(np.median(ref))
            for pan, yv in sg['rows']:
                if pan != 1500:
                    xs.append(pan - 1500.)
                    ys.append(((yv - r0 + math.pi) % (2*math.pi)) - math.pi)
        xs, ys = np.asarray(xs), np.asarray(ys)
        k = float(np.sum(xs*ys)/np.sum(xs*xs)) if xs.size >= 10 else 0.
        res = ys - k*xs if xs.size else np.zeros(0)
        pan_k[state] = round(k, 8)
        pan_fit[state] = {'n': int(xs.size), 'rad_per_pwm': round(k, 8),
                          'residual_abs_p95_deg': None if res.size == 0 else round(float(np.degrees(np.percentile(np.abs(res), 95))), 3),
                          'offset_at_pan_2030_deg': round(math.degrees(k*530.), 3)}
    out = {'schema': 'ugrp.vision_loc.calibration.v1', 'split_used': eps,
           'method': ('true camera pose (MuJoCo, teacher render, eval_only/labels.jsonl) vs commanded-PWM FK '
                      '(harness.wall_tags.camera_in_base) on settled frames (own arm/pan command >= settled_s ago) '
                      'of the named M1 arm poses; median elevation rotation about the FK optical x axis and median '
                      'camera height offset per (own load state, servo 3 pulse); interpolated over servo 3'),
           'settled_s': args.settled_s, 'fits': fits, 'sag': sag, 'settle_analysis': settle,
           'pan_base_yaw': pan_k, 'pan_base_yaw_fit': pan_fit,
           'pan_base_yaw_method': ('GT chassis yaw (eval_only/labels.jsonl base_gt) in hold segments (no own wheel '
                                   'command, same own arm pose and load state) minus the segment median at pan 1500, '
                                   'regressed through the origin on (pan pulse - 1500), settled frames'),
           'files_sha256': {ep: sha_file(RENDER_ROOT/ep/'eval_only'/'labels.jsonl') for ep in eps}}
    Path(args.output).write_text(json.dumps(out, indent=1))
    print(json.dumps({'sag': sag, 'settle': settle, 'pan_base_yaw_fit': pan_fit}, indent=1))


# ----------------------------------------------------------------------------- motion refit (TRAIN, GT offline)
def fit_motion_cmd(args):
    """Refit the M1 command->motion model (unloaded and loaded) on TRAIN teacher logs.

    Reuses ``scripts/eval_owncam_localization.py`` (``fit_motion``: lagged-command
    least squares, noise and slip-scale fit; ``_gt_body_velocity``) exactly as the
    own-camera localizer calibration did; only the episodes differ (the teacher
    here strafes with mecanum commands far more than the M1 student did). The
    M1 'fine' manipulation profile and everything else stay as calibrated.
    """
    if any(split_of(e) != 'train' for e in args.episodes):
        raise SystemExit('fit-motion reads TRAIN episodes only')
    import copy
    from scripts import eval_owncam_localization as eol
    m1 = mp.load_m1_localizer()
    m1_cal, m1_prov = mp.load_m1_calibration()
    base = copy.deepcopy(m1_cal['params'])
    raw = []
    for ep in args.episodes:
        gt = vl.read_jsonl(RENDER_ROOT/ep/'eval_only'/'gt_trajectory.jsonl')
        t, dt, v = eol._gt_body_velocity(gt)
        cmds = sorted(vl.read_jsonl(RENDER_ROOT/ep/'inputs'/'commands.jsonl'), key=lambda c: c['t'])
        u, loaded, ci, cur, exp, load = [], [], 0, np.zeros(3), -1., m1.LoadState()
        for tt in t:
            while ci < len(cmds) and cmds[ci]['t'] <= tt + 1e-9:
                c = cmds[ci]
                load.command(c)
                if c['kind'] == 'mecanum':
                    cur, exp = np.array([c['forward'], c['left'], c['turn']], float), c['t'] + c['duration_s']
                elif c['kind'] == 'drive':
                    cur, exp = np.array([c['forward'], 0., c['turn']], float), c['t'] + c['duration_s']
                elif c['kind'] not in ('arm', 'look', 'initial_servo_command'):
                    cur, exp = np.zeros(3), -1.
                ci += 1
            u.append(cur if tt < exp - 1e-9 else np.zeros(3))
            loaded.append(load.loaded)
        raw.append((np.array(u), dt, v, np.array(loaded)))
    params = copy.deepcopy(base)
    mot, rep_u = eol.fit_motion([(u, dt, v, ~m) for u, dt, v, m in raw], {k: v for k, v in base['motion'].items()
                                                                          if k != 'tau_stop_s'})
    mot_l, rep_l = eol.fit_motion([(u, dt, v, m) for u, dt, v, m in raw], {k: v for k, v in base['motion_loaded'].items()
                                                                           if k != 'tau_stop_s'})
    params['motion'], params['motion_loaded'] = mot, mot_l
    out = {'schema': 'ugrp.vision_loc.motion_refit.v1', 'split_used': args.episodes, 'base': m1_prov,
           'method': 'scripts/eval_owncam_localization.py fit_motion on TRAIN teacher logs (own commands + GT '
                     'trajectory); motion and motion_loaded replaced, tau_stop_s dropped (single-lag fit), fine '
                     'profile and all other parameters from the M1 calibration',
           'report': {'unloaded': rep_u, 'loaded': rep_l}, 'params': params}
    Path(args.output).write_text(json.dumps(out, indent=1))
    print(json.dumps(out['report'], indent=1))


# ----------------------------------------------------------------------------- train
def train_cmd(args):
    import seg_model
    tab = episodes_table()['episodes']
    train_eps = [RENDER_ROOT/e['episode_id'] for e in tab if e['split'] == 'train']
    val_eps = [RENDER_ROOT/e['episode_id'] for e in tab if e['split'] == 'dev']
    for ep in train_eps + val_eps:
        if not (ep/'teacher_manifest.json').exists():
            raise SystemExit(f'{ep} is not rendered yet')
    out = Path(args.output)
    log_path = out/'train_log.jsonl'
    out.mkdir(parents=True, exist_ok=True)
    fh = open(log_path, 'a')

    def log(line):
        print(line, flush=True)
        fh.write(line + '\n')
        fh.flush()
    info = seg_model.train(train_eps, val_eps, out, epochs=args.epochs, batch=args.batch, lr=args.lr,
                           every=args.every, val_every=args.val_every, workers=args.workers, seed=args.seed, log=log)
    info['load_average_end'] = list(os.getloadavg())
    info['train_episode_hashes'] = {str(e): sha_file(e/'teacher_manifest.json') for e in train_eps}
    (out/'train_info.json').write_text(json.dumps(info, indent=1))
    print(json.dumps({k: info[k] for k in ('sha256', 'params', 'train_frames', 'val_frames', 'wall_s')}))


# ----------------------------------------------------------------------------- observations
def save_obs(path: Path, frames_idx, obs_list, extra: dict):
    arr = {k: np.stack([getattr(o, k) for o in obs_list]) for k in ('b_kind', 'b_lo', 'b_hi', 't_kind', 't_lo', 't_hi')}
    arr['b_kind'] = arr['b_kind'].astype(np.int8)
    arr['t_kind'] = arr['t_kind'].astype(np.int8)
    for k in ('b_lo', 'b_hi', 't_lo', 't_hi'):
        arr[k] = arr[k].astype(np.float32)
    np.savez_compressed(path, frame=np.asarray(frames_idx, np.int32), columns=obs_list[0].columns, **arr,
                        meta=np.asarray(json.dumps(extra)))


def load_obs(path: Path) -> tuple[dict, dict]:
    z = np.load(path, allow_pickle=False)
    cols = z['columns']
    out = {}
    for i, f in enumerate(z['frame']):
        out[int(f)] = vl.ColumnObs(cols, z['b_kind'][i].astype(int), z['b_lo'][i].astype(float),
                                   z['b_hi'][i].astype(float), z['t_kind'][i].astype(int), z['t_lo'][i].astype(float),
                                   z['t_hi'][i].astype(float))
    return out, json.loads(str(z['meta']))


def segment(args):
    import seg_model
    require_frozen(args.episodes, checkpoint=args.checkpoint, config=args.config)
    cfg = load_json(args.config)
    seg = seg_model.Segmenter(Path(args.checkpoint), args.device, tuple(cfg.get('infer_size', (320, 240))))
    obs_params = {**vl.DEFAULT_OBS, **cfg.get('obs', {})}
    cols = vl.column_positions(int(obs_params['columns']), int(obs_params['strip_half_px']))
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=True)
    for ep in args.episodes:
        ep_dir = RENDER_ROOT/ep
        frames = vl.read_jsonl(ep_dir/'inputs'/'frames.jsonl')      # student view (no eval_only)
        obs, idx, counts = [], [], []
        t0 = time.time()
        for row in frames:
            bgr = cv2.imread(str(ep_dir/row['file']), cv2.IMREAD_COLOR)
            probs = seg.probs(bgr)
            und = vl.mp.undistort(bgr) if obs_params.get('refine_px') else None
            obs.append(vl.column_observations(probs, cols, obs_params, und))
            idx.append(int(row['frame']))
            counts.append(np.bincount(probs.argmax(2).ravel(), minlength=5).tolist())
        wall = time.time() - t0
        meta = {'episode': ep, 'checkpoint_sha256': seg.sha256, 'obs_params': obs_params, 'device': str(seg.dev),
                'infer_size': list(seg.infer_size),
                'frames': len(idx), 'wall_s': round(wall, 1), 'load_average': list(os.getloadavg()),
                'source': 'own frames only (inputs/frames.jsonl + frames/)'}
        save_obs(out/f'{ep}.obs.npz', idx, obs, meta)
        (out/f'{ep}.classcounts.json').write_text(json.dumps(counts))
        print(f'{ep}: {len(idx)} frames, {wall:.0f} s ({1000*wall/max(len(idx),1):.0f} ms/frame incl. decode)', flush=True)


def oracle(args):
    """EVAL-ONLY DIAGNOSTIC: observations from the teacher's segmentation renders."""
    obs_params = {**vl.DEFAULT_OBS, **load_json(args.config).get('obs', {})}
    cols = vl.column_positions(int(obs_params['columns']), int(obs_params['strip_half_px']))
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=True)
    for ep in args.episodes:
        ep_dir = RENDER_ROOT/ep
        frames = vl.read_jsonl(ep_dir/'inputs'/'frames.jsonl')
        labels = {r['frame_id']: r for r in vl.read_jsonl(ep_dir/'eval_only'/'labels.jsonl')}
        obs, idx = [], []
        for row in frames:
            lab = cv2.imread(str(ep_dir/labels[row['frame_id']]['label']), cv2.IMREAD_UNCHANGED)
            obs.append(vl.column_observations(vl.one_hot(lab), cols, obs_params))
            idx.append(int(row['frame']))
        save_obs(out/f'{ep}.obs.npz', idx, obs, {'episode': ep, 'obs_params': obs_params,
                                                 'source': 'EVAL-ONLY teacher segmentation renders (diagnostic)'})
        print(f'{ep}: oracle {len(idx)} frames', flush=True)


# ----------------------------------------------------------------------------- localize
class Sink:
    """Feeds one PF variant; ``frame`` applies that variant's measurement."""

    def __init__(self, loc, kind, obs=None, boundary=None):
        self.loc, self.kind, self.obs, self.boundary = loc, kind, obs, boundary
        self.servo: dict[int, int] = {}
        self.last = {}

    def command(self, row):
        self.loc.command(row)
        k = row['kind']
        if k == 'initial_servo_command':
            self.servo = {int(a): int(b) for a, b in row['pulses'].items()}
        elif k == 'arm':
            self.servo[int(row['servo_id'])] = int(row['pulse'])
        elif k == 'look':
            self.servo[6] = int(row['pan_pulse'])

    def set_motion_profile(self, t, name):
        if name != self.loc.motion_profile:
            self.loc.set_motion_profile(t, name)

    def frame(self, t, bgr, row):
        loc = self.loc
        if self.kind in ('vision', 'oracle'):
            self.last = loc.update_obs(t, self.obs[int(row['frame'])], self.servo)
            self.last['n_cols'] = int(self.obs[int(row['frame'])].informative.sum())
        elif self.kind == 'boundary':
            self.last = self.boundary_update(t, bgr)
        else:
            loc.predict_to(t)
            loc._normalize_and_resample()
            self.last = loc.estimate()
            self.last['measured'] = False

    def boundary_update(self, t, bgr):
        """PR #210 detector + likelihood (wall height 0.40), same PF, sag table, settle gate and geometry."""
        loc, det, meas = self.loc, self.boundary['detector'], self.boundary['measurement']
        loc.predict_to(t)
        used = False
        n = 0
        if loc.initialized and loc.settled(t):
            und = mp.undistort(bgr)
            cm = loc.column_model_for(self.servo, self.boundary['columns'])
            self_top = mp.carried_mask_top(und, self.boundary['columns'], int(det['strip_half_px'])) \
                if loc.load.loaded else None
            scan = mp.detect_boundaries(und, cm, det, self_top)
            n = int(scan.detected.sum())
            if n >= int(meas['min_columns']):
                vb, vt = vl.expected_rows(loc.geometry, loc.px, cm)
                loc.logw = loc.logw + mp.boundary_loglik(vb, vt, scan, meas, vtf_exp=vt)
                used = True
        if loc.initialized:
            loc._normalize_and_resample()
        est = loc.estimate()
        est['measured'], est['n_cols'] = used, n
        return est


def localize(args):
    frozen = require_frozen(args.episodes, config=args.config, calibration=args.calibration)
    if frozen and args.motion:
        raise SystemExit('test refused: the registered student uses the M1 motion model')
    m1 = mp.load_m1_localizer()
    m1_cal, m1_prov = mp.load_m1_calibration()
    params = m1_cal['params']
    if args.motion:
        refit = load_json(args.motion)
        params = refit['params']
        m1_prov = {**m1_prov, 'motion_refit': {'path': str(args.motion), 'sha256': sha_file(args.motion)}}
    static = load_map()
    cal = load_json(args.calibration)
    cfg = load_json(args.config)
    wanted = args.filters.split(',')
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=True)
    pr210 = json.loads((mp.ROOT/'experiments'/'2026-09-26-markerless-probe'/'calibration_dev.json').read_text())
    for ep in args.episodes:
        seed = int(next(e['seed'] for e in episodes_table()['episodes'] if e['episode_id'] == ep))
        sinks = {}

        def pf():
            return vl.make_vision_pf(m1, static, params, cfg.get('measurement', {}), cfg.get('obs', {}), cal['sag'],
                                     seed, cal.get('pan_base_yaw') if cfg.get('pan_coupling', True) else None)
        for name in wanted:
            if name == 'vision':
                obs, meta = load_obs(Path(args.obs)/f'{ep}.obs.npz')
                sinks[name] = Sink(pf(), 'vision', obs)
            elif name == 'oracle':
                obs, _ = load_obs(Path(args.oracle_obs)/f'{ep}.obs.npz')
                sinks[name] = Sink(pf(), 'oracle', obs)
            elif name == 'boundary':
                det = {**mp.DEFAULT_DETECTOR, **pr210.get('detector', {}), 'wall_height_m': vl.WALL_HEIGHT_M}
                meas = {**mp.DEFAULT_MEASUREMENT, **{k: v for k, v in pr210['measurement'].items() if k != 'bias_rad'}}
                cols = mp.column_positions(det['columns'], int(det['strip_half_px']))
                sinks[name] = Sink(pf(), 'boundary', boundary={'detector': det, 'measurement': meas, 'columns': cols})
            elif name == 'deadreck':
                sinks[name] = Sink(pf(), 'deadreck')
            else:
                raise SystemExit(f'unknown filter {name}')
        for s in sinks.values():
            s.loc.init_gaussian((DOCK_X, spawn_y(ep), DOCK_YAW), DOCK_STD)
        rows_out = []
        t0 = time.time()

        def on_frame(k, row):
            any_sink = next(iter(sinks.values()))
            rec = {'frame': row['frame'], 't': row['t'], 'phase': row['phase'], 'skill_phase': row['skill_phase'],
                   'loaded': bool(any_sink.loc.load.loaded), 's3': int(row['commanded_servo']['3']),
                   's6': int(row['commanded_servo']['6']), 'settled': bool(any_sink.loc.settled(float(row['t'])))}
            for name, s in sinks.items():
                e = s.last
                rec[name] = None if not e.get('initialized') else {
                    'xyyaw': [round(e['x'], 5), round(e['y'], 5), round(e['yaw'], 6)],
                    'std_xy_m': round(e['std_xy_m'], 5), 'std_yaw_rad': round(e['std_yaw_rad'], 5),
                    'measured': bool(e.get('measured')), 'n_cols': e.get('n_cols')}
            rows_out.append(rec)
        n = vl.replay(RENDER_ROOT/ep, list(sinks.values()), on_frame=on_frame)
        with open(out/f'{ep}.estimates.jsonl', 'w') as fh:
            for r in rows_out:
                fh.write(json.dumps(r) + '\n')
        meta = {'schema': vl.SCHEMA, 'episode': ep, 'frames': n, 'filters': list(sinks), 'seed': seed,
                'dock': [DOCK_X, spawn_y(ep), DOCK_YAW], 'dock_std': list(DOCK_STD), 'wall_s': round(time.time() - t0, 1),
                'stats': {k: dict(s.loc.stats) for k, s in sinks.items()}, 'load_average': list(os.getloadavg()),
                'map': {'file': str(MAP_FILE.relative_to(mp.ROOT)), 'sha256': sha_file(MAP_FILE)},
                'm1_calibration': m1_prov,
                'm1_localizer': {'source': f'{mp.M1_SHA}:{mp.M1_LOCALIZER}', 'sha256': mp.M1_LOCALIZER_SHA256},
                'calibration': {'path': str(args.calibration), 'sha256': sha_file(args.calibration)},
                'config': {'path': str(args.config) if args.config else None,
                           'sha256': sha_file(args.config) if args.config else None, 'value': cfg},
                'obs_dirs': {'vision': args.obs, 'oracle': args.oracle_obs},
                'module_sha256': {f: sha_file(HERE/f) for f in ('vision_loc.py', 'vision_loc_cli.py')},
                'pr210_probe_sha256': sha_file(vl._PROBE)}
        (out/f'{ep}.meta.json').write_text(json.dumps(meta, indent=1))
        print(f'{ep}: {n} frames, {meta["wall_s"]} s', flush=True)


# ----------------------------------------------------------------------------- score (GT)
def ang(a):
    return (a + math.pi) % (2*math.pi) - math.pi


def pct(a, q):
    return None if len(a) == 0 else round(float(np.percentile(a, q)), 4)


def summary(errs):
    e = np.asarray([x[0] for x in errs])
    y = np.asarray([x[1] for x in errs])
    lat = np.asarray([x[2] for x in errs])
    return {'n': int(e.size), 'pos_p50_m': pct(e, 50), 'pos_p90_m': pct(e, 90), 'pos_p95_m': pct(e, 95),
            'pos_p99_m': pct(e, 99), 'pos_max_m': None if e.size == 0 else round(float(e.max()), 4),
            'lat_abs_p90_m': pct(lat, 90), 'lat_abs_p99_m': pct(lat, 99),
            'yaw_p50_deg': pct(y, 50), 'yaw_p90_deg': pct(y, 90),
            'share_pos_lt_5cm': None if e.size == 0 else round(float(np.mean(e < .05)), 4),
            'share_pos_lt_10cm': None if e.size == 0 else round(float(np.mean(e < .10)), 4)}


def near_door(gt) -> bool:
    """PR #210's door-vicinity box around door_1 (GT position)."""
    return abs(gt[0] - DOOR_X) < .6 and -.45 < gt[1] < .55


def groups_of(rec, gt) -> list[str]:
    g = ['all', 'loaded' if rec['loaded'] else 'unloaded']
    if near_door(gt):
        g += ['door_zone', 'door_loaded' if rec['loaded'] else 'door_unloaded']
    else:                                       # PR #210 groups (door zone first)
        sp = rec['skill_phase'] or ''
        g.append('carry' if sp in ('nav_preplace', 'to_carry_posture', 'pre_release') else
                 'manipulate' if sp in ('grasp', 'nav_pregrasp', 'release') else
                 'look_back' if sp == 'look_back' else 'search_approach')
    return g


def false_detections(vis: vl.ColumnObs, orc: vl.ColumnObs, tol_px: float) -> dict:
    """Column-level disagreement of the learned observations with the teacher-label ones."""
    out = {}
    for part in ('b', 't'):
        kv, ko = getattr(vis, f'{part}_kind'), getattr(orc, f'{part}_kind')
        lv, hv = getattr(vis, f'{part}_lo'), getattr(vis, f'{part}_hi')
        lo, ho = getattr(orc, f'{part}_lo'), getattr(orc, f'{part}_hi')
        det = kv == vl.EDGE
        with np.errstate(invalid='ignore'):
            # a learned sharp edge counts as false when the teacher interval (+- tol) does not contain it
            bad = det & ~((ko != vl.NONE) & (lv >= np.nan_to_num(lo, nan=vl.NEG_INF) - tol_px)
                          & (lv <= np.nan_to_num(ho, nan=vl.POS_INF) + tol_px))
            missed = (ko == vl.EDGE) & (kv == vl.NONE)
        out[part] = {'edges': int(det.sum()), 'false_edges': int(bad.sum()), 'teacher_edges': int((ko == vl.EDGE).sum()),
                     'missed_edges': int(missed.sum())}
    return out


def score(args):
    require_frozen(args.episodes)
    est_dir = Path(args.estimates)
    report = {'schema': 'ugrp.vision_loc.metrics.v1', 'episodes': {}, 'pooled': {}, 'false_detections': {}}
    pooled: dict = {}
    fd_pool = {'b': {}, 't': {}}
    for ep in args.episodes:
        est = vl.read_jsonl(est_dir/f'{ep}.estimates.jsonl')
        ev = {r['frame']: r for r in vl.read_jsonl(RENDER_ROOT/ep/'eval_only'/'frames_eval.jsonl')}
        per: dict = {}
        for rec in est:
            gt = ev[rec['frame']]['gt']
            for fname, v in rec.items():
                if not isinstance(v, dict) or 'xyyaw' not in v:
                    continue
                x, y, yaw = v['xyyaw']
                err = (math.hypot(x - gt[0], y - gt[1]), abs(math.degrees(ang(yaw - gt[2]))), abs(y - gt[1]))
                for key in groups_of(rec, gt):
                    per.setdefault(fname, {}).setdefault(key, []).append(err)
                    pooled.setdefault(fname, {}).setdefault(key, []).append(err)
        report['episodes'][ep] = {f: {g: summary(v) for g, v in d.items()} for f, d in per.items()}
        if args.obs and args.oracle_obs:
            vis, _ = load_obs(Path(args.obs)/f'{ep}.obs.npz')
            orc, _ = load_obs(Path(args.oracle_obs)/f'{ep}.obs.npz')
            tot = {'b': {}, 't': {}}
            for f in vis:
                d = false_detections(vis[f], orc[f], args.tol_px)
                for part in d:
                    for k, v in d[part].items():
                        tot[part][k] = tot[part].get(k, 0) + v
                        fd_pool[part][k] = fd_pool[part].get(k, 0) + v
            report['false_detections'][ep] = tot
    report['pooled'] = {f: {g: summary(v) for g, v in d.items()} for f, d in pooled.items()}
    if args.obs and args.oracle_obs:
        report['false_detections']['pooled'] = fd_pool
        for part, d in fd_pool.items():
            d['false_edge_rate'] = round(d.get('false_edges', 0)/max(d.get('edges', 0), 1), 5)
            d['missed_edge_rate'] = round(d.get('missed_edges', 0)/max(d.get('teacher_edges', 0), 1), 5)
        report['false_detections']['tol_px'] = args.tol_px
    report['groups'] = {'door_zone': '|x - 2.2| < 0.6 and -0.45 < y < 0.55 (GT), as PR #210',
                        'loaded': 'own-command load state (LoadState of the M1 localizer)',
                        'lateral': '|y_est - y_gt| (door_1 is crossed along x)'}
    Path(args.output).write_text(json.dumps(report, indent=1))
    for f, d in report['pooled'].items():
        a = d['all']
        print(f"{f:9s} n={a['n']:6d} p50={a['pos_p50_m']} p90={a['pos_p90_m']} p99={a['pos_p99_m']} "
              f"max={a['pos_max_m']} yaw_p90={a['yaw_p90_deg']}")
        for g in ('door_zone', 'door_loaded', 'door_unloaded', 'loaded', 'unloaded', 'search_approach', 'carry',
                  'manipulate', 'look_back'):
            if g in d:
                s = d[g]
                print(f"    {g:15s} n={s['n']:6d} p50={s['pos_p50_m']} p90={s['pos_p90_m']} p99={s['pos_p99_m']} "
                      f"lat_p99={s['lat_abs_p99_m']} yaw_p90={s['yaw_p90_deg']}")
    if report['false_detections'].get('pooled'):
        print(json.dumps(report['false_detections']['pooled']))


# ----------------------------------------------------------------------------- bench
def bench(args):
    import seg_model
    import torch
    ep_dir = RENDER_ROOT/args.episodes[0]
    frames = vl.read_jsonl(ep_dir/'inputs'/'frames.jsonl')[:args.n]
    imgs = [cv2.imread(str(ep_dir/r['file']), cv2.IMREAD_COLOR) for r in frames]
    cfg = load_json(args.config)
    obs_params = {**vl.DEFAULT_OBS, **cfg.get('obs', {})}
    cols = vl.column_positions(int(obs_params['columns']), int(obs_params['strip_half_px']))
    res = {'episode': args.episodes[0], 'infer_size': cfg.get('infer_size', (320, 240)), 'n': len(imgs), 'torch': torch.__version__,
           'threads': torch.get_num_threads(), 'load_average_start': list(os.getloadavg())}
    for dev in args.devices.split(','):
        seg = seg_model.Segmenter(Path(args.checkpoint), dev, tuple(cfg.get('infer_size', (320, 240))))
        for im in imgs[:5]:
            seg.probs(im)
        t_net, t_obs = [], []
        for im in imgs:
            a = time.perf_counter()
            p = seg.probs(im)
            b = time.perf_counter()
            vl.column_observations(p, cols, obs_params, vl.mp.undistort(im) if obs_params.get('refine_px') else None)
            t_net.append(b - a)
            t_obs.append(time.perf_counter() - b)
        res[dev] = {'net_incl_undistort_ms_p50': round(1e3*float(np.median(t_net)), 2),
                    'net_ms_p90': round(1e3*float(np.percentile(t_net, 90)), 2),
                    'column_obs_ms_p50': round(1e3*float(np.median(t_obs)), 2)}
    res['load_average_end'] = list(os.getloadavg())
    Path(args.output).write_text(json.dumps(res, indent=1))
    print(json.dumps(res, indent=1))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    c = sub.add_parser('calibrate')
    c.add_argument('--episodes', nargs='+', required=True)
    c.add_argument('--settled-s', type=float, default=.8)
    c.add_argument('--min-frames', type=int, default=20)
    c.add_argument('--output', required=True)
    fm = sub.add_parser('fit-motion')
    fm.add_argument('--episodes', nargs='+', required=True)
    fm.add_argument('--output', required=True)
    t = sub.add_parser('train')
    t.add_argument('--output', required=True)
    t.add_argument('--epochs', type=int, default=4)
    t.add_argument('--batch', type=int, default=16)
    t.add_argument('--lr', type=float, default=1e-3)
    t.add_argument('--every', type=int, default=2)
    t.add_argument('--val-every', type=int, default=10)
    t.add_argument('--workers', type=int, default=3)
    t.add_argument('--seed', type=int, default=0)
    for name, fn in (('segment', segment), ('oracle', oracle)):
        s = sub.add_parser(name)
        s.add_argument('--episodes', nargs='+', required=True)
        s.add_argument('--config')
        s.add_argument('--output', required=True)
        if name == 'segment':
            s.add_argument('--checkpoint', required=True)
            s.add_argument('--device', default=None)
    lz = sub.add_parser('localize')
    lz.add_argument('--episodes', nargs='+', required=True)
    lz.add_argument('--calibration', required=True)
    lz.add_argument('--config')
    lz.add_argument('--filters', default='vision,boundary,deadreck')
    lz.add_argument('--motion', help='motion refit JSON (fit-motion); default: the M1 calibration')
    lz.add_argument('--obs')
    lz.add_argument('--oracle-obs')
    lz.add_argument('--output', required=True)
    sc = sub.add_parser('score')
    sc.add_argument('--episodes', nargs='+', required=True)
    sc.add_argument('--estimates', required=True)
    sc.add_argument('--obs')
    sc.add_argument('--oracle-obs')
    sc.add_argument('--tol-px', type=float, default=5.)
    sc.add_argument('--output', required=True)
    b = sub.add_parser('bench')
    b.add_argument('--episodes', nargs=1, required=True)
    b.add_argument('--checkpoint', required=True)
    b.add_argument('--config')
    b.add_argument('--devices', default='cpu,mps')
    b.add_argument('--n', type=int, default=200)
    b.add_argument('--output', required=True)
    args = ap.parse_args(argv)
    {'calibrate': calibrate, 'fit-motion': fit_motion_cmd, 'train': train_cmd, 'segment': segment, 'oracle': oracle, 'localize': localize,
     'score': score, 'bench': bench}[args.cmd](args)


if __name__ == '__main__':
    main()
