#!/usr/bin/env python3
"""Evaluation-only offline analysis of the v61 Y1/Y2 regrasp support failure.

Scope (see README.md in this directory):
- reads the existing raw run outputs only (pair-decisions, issued commands,
  recorded RGB, observer replay qpos); it never steps physics and never
  produces control input;
- recomputes every recorded fine-stage decision from the recorded RGB with
  the exact saved stage models;
- compares measured (replay) joint/base/wheel state of the failing model slot
  between the first approach and each re-approach;
- renders the fixed TOP camera from replay qpos, optionally with one factor
  (wheel angles, base pose, arm, beam, ...) replaced by the first-approach
  value, and re-runs the same geometry support test on the rendered image.

Replay qpos is observer-only privileged state. It is used here purely as an
evaluation instrument and is never fed back into any robot or controller.
"""
from __future__ import annotations

import argparse
import hashlib
import io
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from harness.camera_varied_start_student import predict_stage  # noqa: E402
from harness.dispatch_skill_binding import canonical_pair_top  # noqa: E402

RAW_ROOT = Path('/Users/changmin/projects/ugrp/outputs/beam-regrasp-20260925')
STAGE_MODELS = Path('/Users/changmin/projects/ugrp-worktrees/team-recovery-fix/outputs/'
                    'dispatch-models/e78a5a5777f5bc48/models/varied')
REFERENCE_TOP = ROOT / 'tests/fixtures/camera_goal_transport/reference-top.jpg'
SLOTS = ('r1', 'r3')
ARM = ('arm_yaw', 'shoulder', 'elbow', 'wrist_pitch')
GRIP = ('left_gripper_close', 'right_gripper_close')
WHEELS = ('wheel_fl_joint', 'wheel_fr_joint', 'wheel_rl_joint', 'wheel_rr_joint')


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def wrap_deg(x):
    return (np.asarray(x) + 180.0) % 360.0 - 180.0


def yaw_deg(quat):
    w, x, y, z = quat
    return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def action_key(a):
    return tuple(round(float(a.get(k, 0.0)), 9) for k in ('forward', 'left', 'turn', 'duration_s')) + (a.get('kind'),)


class Replay:
    def __init__(self, run_dir: Path):
        import mujoco
        self.mujoco = mujoco
        rp = run_dir / 'replay'
        self.manifest = json.loads((rp / 'replay.json').read_text())
        self.files_ok = {name: sha(rp / name) == digest
                         for name, digest in self.manifest['files_sha256'].items()}
        arrays = np.load(rp / 'states.npz')
        self.time, self.qpos = arrays['time'], arrays['qpos']
        self.model = mujoco.MjModel.from_binary_path(str(rp / 'model.mjb'))
        self.data = mujoco.MjData(self.model)
        self.renderer = None

    def adr(self, joint: str) -> int:
        jid = self.mujoco.mj_name2id(self.model, self.mujoco.mjtObj.mjOBJ_JOINT, joint)
        if jid < 0:
            raise KeyError(joint)
        return int(self.model.jnt_qposadr[jid])

    def nearest(self, t: float) -> tuple[int, float]:
        index = int(np.argmin(np.abs(self.time - t)))
        return index, float(self.time[index] - t)

    def robot_state(self, q: np.ndarray, rid: str) -> dict:
        base = self.adr(f'{rid}__base_free')
        return {'base_xy_m': q[base:base + 2].tolist(), 'base_z_m': float(q[base + 2]),
                'yaw_deg': yaw_deg(q[base + 3:base + 7]),
                'arm_deg': {j: math.degrees(q[self.adr(f'{rid}__{j}')]) for j in ARM},
                'gripper_mm': {j: 1000 * float(q[self.adr(f'{rid}__{j}')]) for j in GRIP},
                'wheel_deg': {j: math.degrees(q[self.adr(f'{rid}__{j}')]) for j in WHEELS}}

    def project_top(self, q: np.ndarray, point, size) -> list[float]:
        m, d = self.model, self.data
        d.qpos[:] = q
        self.mujoco.mj_forward(m, d)
        cid = self.mujoco.mj_name2id(m, self.mujoco.mjtObj.mjOBJ_CAMERA, 'cctv_top')
        pos, mat = d.cam_xpos[cid], d.cam_xmat[cid].reshape(3, 3)
        local = mat.T @ (np.asarray(point, float) - pos)
        w, h = size
        f = (h / 2) / math.tan(math.radians(m.cam_fovy[cid]) / 2)
        return [w / 2 + f * local[0] / -local[2], h / 2 - f * local[1] / -local[2]]

    def render_top_jpeg(self, q: np.ndarray, size) -> bytes:
        from PIL import Image
        mj = self.mujoco
        if self.renderer is None:
            self.renderer = mj.Renderer(self.model, height=size[1], width=size[0])
            self.option = mj.MjvOption()
            self.option.geomgroup[:] = 1
        self.data.qpos[:] = q
        mj.mj_forward(self.model, self.data)
        self.renderer.update_scene(self.data, camera='cctv_top', scene_option=self.option)
        rgb = self.renderer.render().copy()
        buf = io.BytesIO()
        Image.fromarray(rgb).save(buf, format='JPEG', quality=95)  # same encoder as render_team_jpeg
        return buf.getvalue()


def load_models(stage_dir: Path) -> tuple[dict, dict]:
    skill = json.loads((stage_dir / 'varied-start-skill.json').read_text())
    models, hashes = {}, {'varied-start-skill.json': sha(stage_dir / 'varied-start-skill.json')}
    for slot, stages in skill['models'].items():
        for stage, ref in stages.items():
            path = stage_dir / ref['path']
            digest = sha(path)
            if digest != ref['sha256']:
                raise RuntimeError(f'model hash mismatch {path}')
            hashes[ref['path']] = digest
            models.setdefault(slot, {})[stage] = json.loads(path.read_text())
    return models, hashes


def slot_mapping(run_dir: Path, decisions: list, replay: Replay | None, command_times: dict) -> dict:
    plan = json.loads((run_dir / 'committed-plan.json').read_text())
    bindings = json.loads((run_dir / 'skill-bindings.json').read_text())
    participants = next(t['participants'] for t in plan['plan']['tasks'] if t['object'] == 'beam')
    upper, lower = participants  # harness/dispatch_skill_binding.py SkillBindings.__init__
    code_rule = {'r1': lower, 'r3': upper}
    image_bindings = {json.dumps(x['pair_binding'], sort_keys=True)
                      for x in decisions if x['kind'] == 'image_binding'}
    receipts = {json.dumps({s: x['source_rgb'][s]['physical_robot_id'] for s in SLOTS}, sort_keys=True)
                for x in decisions if x['kind'] == 'coarse_command_receipt'}
    own_suffix = set()
    for x in decisions:
        if x['kind'] == 'learned_approach':
            for call in x['report']['approach_calls']:
                own_suffix.add((call['robot_id'], call['images']['own']['path'].rsplit('-', 1)[1]))
    result = {'beam_participants_plan_order': participants,
              'code_rule': 'SkillBindings: upper, lower = participants; pair = {r1: lower, r3: upper}',
              'code_rule_mapping': code_rule,
              'skill_bindings_pair_model_slots': bindings['pair_model_slots'],
              'image_binding_pair_binding_values': sorted(image_bindings),
              'coarse_receipt_physical_ids': sorted(receipts),
              'fine_call_own_rgb_file_suffix': sorted(own_suffix)}
    # Geometric check: project physical bases at the first coarse decision into TOP.
    if replay is not None:
        first = next(x for x in decisions if x['kind'] == 'coarse')
        t = command_times['first_coarse']
        index, dt = replay.nearest(t)
        raw = run_dir / next(x for x in decisions if x['kind'] == 'image_binding')['raw_top']['path']
        import cv2
        h, w = cv2.imdecode(np.frombuffer(raw.read_bytes(), np.uint8), cv2.IMREAD_COLOR).shape[:2]
        projected = {}
        for rid in ('r1', 'r3'):
            base = replay.adr(f'{rid}__base_free')
            projected[rid] = replay.project_top(replay.qpos[index], replay.qpos[index][base:base + 3], (w, h))
        wheel = {s: first['decisions'][s]['wheel_center_px'] for s in SLOTS}
        nearest = {s: min(projected, key=lambda rid: abs(projected[rid][1] - wheel[s][1])) for s in SLOTS}
        result['geometric_check'] = {'sim_time_s': t, 'replay_dt_s': dt,
                                     'coarse_wheel_center_px_by_slot': wheel,
                                     'projected_base_px_by_physical': projected,
                                     'nearest_physical_by_image_row': nearest}
    consistent = (code_rule == bindings['pair_model_slots']
                  and image_bindings == {json.dumps(code_rule, sort_keys=True)}
                  and receipts == {json.dumps(code_rule, sort_keys=True)}
                  and own_suffix == {(s, f'{code_rule[s]}.jpg') for s in SLOTS}
                  and (replay is None or result['geometric_check']['nearest_physical_by_image_row'] == code_rule))
    result['consistent'] = bool(consistent)
    return result


def link_calls(decisions: list, commands: dict, mapping: dict) -> tuple[list, dict]:
    """Attach issued_at_s to every fine call by contiguous action-sequence match."""
    approaches = [x['report'] for x in decisions if x['kind'] == 'learned_approach']
    linked = []
    for slot in SLOTS:
        stream = [e for e in commands[mapping[slot]] if e['stage'] == 'APPROACH']
        keys = [action_key(e['action']) for e in stream]
        cursor = 0
        for ai, report in enumerate(approaches):
            calls = [c for c in report['approach_calls'] if c['robot_id'] == slot]
            want = [action_key(c['action']) for c in calls]
            start = next((s for s in range(cursor, len(keys) - len(want) + 1)
                          if keys[s:s + len(want)] == want), None)
            if start is None:
                raise RuntimeError(f'no contiguous command match for {slot} approach {ai}')
            for j, call in enumerate(calls):
                linked.append({'approach': ai, 'slot': slot, 'physical': mapping[slot], 'call': call,
                               'issued_at_s': float(stream[start + j]['issued_at_s'])})
            cursor = start + len(want)
    first_coarse = next(e for e in commands[mapping['r1']] if e['stage'] == 'APPROACH')['issued_at_s']
    return linked, {'first_coarse': float(first_coarse)}


def recompute(run_dir: Path, linked: list, models: dict) -> dict:
    images, image_ok = {}, 0
    max_abs = {'orthogonal_residual': 0.0, 'nearest_support_distance': 0.0, 'image_derived_error': 0.0}
    decision_mismatch = []
    for row in linked:
        call = row['call']
        for kind in ('own', 'top'):
            ref = call['images'][kind]
            if ref['path'] not in images:
                data = (run_dir / ref['path']).read_bytes()
                images[ref['path']] = data
                image_ok += hashlib.sha256(data).hexdigest() == ref['sha256']
        got = predict_stage(models[row['slot']][call['stage']], images[call['images']['own']['path']],
                            images[call['images']['top']['path']])
        want = call['decision']
        same = (got['ok'] == want['ok'] and got['reason'] == want['reason']
                and abs(got['command'] - want['command']) < 1e-12)
        for key in max_abs:
            if key in want['diagnostics']:
                diff = abs(got['diagnostics'][key] - want['diagnostics'][key])
                max_abs[key] = max(max_abs[key], diff)
                same &= diff < 1e-9
        row['recomputed'] = got
        if not same:
            decision_mismatch.append({'approach': row['approach'], 'slot': row['slot'],
                                      'frame_id': call['frame_id'], 'stage': call['stage']})
    return {'calls': len(linked), 'unique_images': len(images), 'image_sha256_ok': image_ok,
            'decision_mismatches': decision_mismatch, 'max_abs_diagnostic_diff': max_abs}


def residual(row) -> float | None:
    return row['call']['decision']['diagnostics'].get('orthogonal_residual')


def pose_comparison(linked: list, replay: Replay, mapping: dict, slot: str = 'r3') -> list:
    phys = mapping[slot]
    rows = [r for r in linked if r['slot'] == slot]
    for r in rows:
        r['replay_index'], r['replay_dt_s'] = replay.nearest(r['issued_at_s'])
    out = []
    for ai in sorted({r['approach'] for r in rows if r['approach'] > 0}):
        fails = [r for r in rows if r['approach'] == ai and r['call']['decision']['ok'] is False]
        if not fails:
            continue
        f = fails[0]
        stage = f['call']['stage']
        firsts = [r for r in rows if r['approach'] == 0 and r['call']['stage'] == stage]
        qf = replay.qpos[f['replay_index']]
        base = replay.adr(f'{phys}__base_free')

        def dist(r):
            return float(np.linalg.norm(replay.qpos[r['replay_index']][base:base + 2] - qf[base:base + 2]))
        g = min(firsts, key=dist)
        sf, sg = replay.robot_state(qf, phys), replay.robot_state(replay.qpos[g['replay_index']], phys)
        first_res = [residual(r) for r in firsts if residual(r) is not None]
        out.append({
            'approach': ai, 'stage': stage, 'physical_robot': phys, 'model_slot': slot,
            'failure': {'frame_id': f['call']['frame_id'], 'issued_at_s': f['issued_at_s'],
                        'replay_index': f['replay_index'], 'replay_dt_s': f['replay_dt_s'],
                        'orthogonal_residual': residual(f),
                        'nearest_support_distance': f['call']['decision']['diagnostics']['nearest_support_distance'],
                        'state': sf},
            'nearest_first_approach': {'frame_id': g['call']['frame_id'], 'issued_at_s': g['issued_at_s'],
                                       'replay_index': g['replay_index'], 'replay_dt_s': g['replay_dt_s'],
                                       'orthogonal_residual': residual(g), 'state': sg},
            'first_approach_same_stage_residual': {'n': len(first_res), 'median': float(np.median(first_res)),
                                                   'max': float(np.max(first_res))},
            'difference': {
                'base_distance_mm': 1000 * dist(g),
                'yaw_deg': float(wrap_deg(sf['yaw_deg'] - sg['yaw_deg'])),
                'arm_max_abs_deg': max(abs(sf['arm_deg'][j] - sg['arm_deg'][j]) for j in ARM),
                'gripper_max_abs_mm': max(abs(sf['gripper_mm'][j] - sg['gripper_mm'][j]) for j in GRIP),
                'wheel_wrapped_deg': {j: float(wrap_deg(sf['wheel_deg'][j] - sg['wheel_deg'][j])) for j in WHEELS},
                'wheel_raw_deg': {j: sf['wheel_deg'][j] - sg['wheel_deg'][j] for j in WHEELS}},
            '_f': f, '_g': g})
    return out


def counterfactuals(run_dir: Path, decisions: list, comparisons: list, replay: Replay,
                    models: dict, mapping: dict, reference: bytes, save_dir: Path | None,
                    figure_rows: list | None = None) -> list:
    import cv2
    binding = {x['frame_id']: x for x in decisions if x['kind'] == 'image_binding'}
    phys_all = ('r1', 'r2', 'r3')
    results = []

    def adrs(names):
        out = []
        for n in names:
            jid = replay.mujoco.mj_name2id(replay.model, replay.mujoco.mjtObj.mjOBJ_JOINT, n)
            a = int(replay.model.jnt_qposadr[jid])
            width = 7 if replay.model.jnt_type[jid] == 0 else 1
            out.extend(range(a, a + width))
        return out

    for comp in comparisons:
        f, g = comp.pop('_f'), comp.pop('_g')
        phys, slot, stage = comp['physical_robot'], comp['model_slot'], comp['stage']
        model = models[slot][stage]
        qf, qg = replay.qpos[f['replay_index']].copy(), replay.qpos[g['replay_index']].copy()
        bf = binding[f['call']['frame_id']]
        bg = binding[g['call']['frame_id']]
        raw_f = (run_dir / bf['raw_top']['path']).read_bytes()
        raw_g = (run_dir / bg['raw_top']['path']).read_bytes()
        size = cv2.imdecode(np.frombuffer(raw_f, np.uint8), cv2.IMREAD_COLOR).shape[1::-1]
        own = (run_dir / f['call']['images']['own']['path']).read_bytes()
        groups = {
            'wheels': adrs([f'{phys}__{j}' for j in WHEELS]),
            'base_pose': adrs([f'{phys}__base_free']),
            'arm_gripper': adrs([f'{phys}__{j}' for j in ARM + GRIP]),
            'robot_all': adrs([f'{phys}__base_free'] + [f'{phys}__{j}' for j in WHEELS + ARM + GRIP]),
            'beam': adrs(['team_beam_free']),
            'beam_position_only': adrs(['team_beam_free'])[:3],
            'beam_orientation_only': adrs(['team_beam_free'])[3:],
            'other_robots_all': adrs([f'{o}__base_free' for o in phys_all if o != phys]
                                     + [f'{o}__{j}' for o in phys_all if o != phys for j in WHEELS + ARM + GRIP]),
        }
        beam = groups['beam']
        beam_delta = {'position_mm': (1000 * (qf[beam[:3]] - qg[beam[:3]])).tolist(),
                      'yaw_deg': float(wrap_deg(yaw_deg(qf[beam[3:]]) - yaw_deg(qg[beam[3:]]))),
                      'first_replay_frame_to_F_position_mm':
                          (1000 * (qf[beam[:3]] - replay.qpos[0][beam[:3]])).tolist()}
        canon_cache = {}

        def score(q, translation, label, save=True):
            jpeg = replay.render_top_jpeg(q, size)
            canon, _t = canonical_pair_top(jpeg, reference, translation_px=translation)
            pred = predict_stage(model, own, canon)
            if save:
                canon_cache[label] = canon
            if save and save_dir is not None:
                save_dir.mkdir(parents=True, exist_ok=True)
                (save_dir / f'a{comp["approach"]}-{label}-canonical-top.jpg').write_bytes(canon)
            return {'orthogonal_residual': pred['diagnostics']['orthogonal_residual'],
                    'nearest_support_distance': pred['diagnostics']['nearest_support_distance'],
                    'ok': pred['ok'], 'reason': pred['reason'], 'render_sha256': hashlib.sha256(jpeg).hexdigest()}, jpeg

        tf, tg = bf['transform']['translation_px'], bg['transform']['translation_px']
        rows = {}
        rows['F_render'], jf = score(qf, tf, 'F')
        rows['G_render'], jg = score(qg, tg, 'G')
        rows['G_render_with_F_translation'], _ = score(qg, tf, 'G-transF')

        def fidelity(rendered, recorded):
            a = cv2.imdecode(np.frombuffer(rendered, np.uint8), cv2.IMREAD_COLOR).astype(int)
            b = cv2.imdecode(np.frombuffer(recorded, np.uint8), cv2.IMREAD_COLOR).astype(int)
            d = np.abs(a - b)
            return {'mean_abs': float(d.mean()), 'p99_abs': float(np.quantile(d, .99)), 'max_abs': int(d.max())}
        rows['F_render']['raw_top_fidelity_vs_recorded'] = fidelity(jf, raw_f)
        rows['G_render']['raw_top_fidelity_vs_recorded'] = fidelity(jg, raw_g)
        raw_variants = {'F': jf}
        for name, idx in groups.items():
            q = qf.copy(); q[idx] = qg[idx]
            rows[f'F_with_G_{name}'], raw_variants[f'F-G{name}'] = score(q, tf, f'F-G{name}')
            q = qg.copy(); q[idx] = qf[idx]
            rows[f'G_with_F_{name}'], _ = score(q, tg, f'G-F{name}')
        # Wheel-angle sweep on the failure frame: rotate all four wheels of the
        # failing robot together by delta from the recorded value.
        sweep = []
        for delta in range(0, 360, 15):
            q = qf.copy(); q[groups['wheels']] += math.radians(delta)
            s, _ = score(q, tf, 'sweep', save=False)
            sweep.append({'delta_deg': delta, 'orthogonal_residual': s['orthogonal_residual'], 'ok': s['ok']})
        attribution = {label: residual_attribution(model['geometry'], canon_cache[label], slot)
                       for label in ('F', 'F-Gbeam', 'G')}
        # Link to the v60 record: hue<=24 / hue<=35 beam masks on the raw TOP
        # render with the actual beam versus the beam moved back to G's pose.
        from harness.dispatch_skill_binding import beam_feature
        beam_masks = {}
        for label in ('F', 'F-Gbeam'):
            f24, f35 = (beam_feature(raw_variants[label], hue_upper=h) for h in (24, 35))
            beam_masks[label] = {'hue24_length_px': f24['length_px'],
                                 'hue24_center_y_px': f24['center'][1] * f24['image_size'][1],
                                 'hue35_length_px': f35['length_px'],
                                 'hue35_center_y_px': f35['center'][1] * f35['image_size'][1]}
        if figure_rows is not None:
            figure_rows.append((comp['approach'], stage, model['geometry'], slot,
                                [(lab, canon_cache[lab], rows[key]['orthogonal_residual'])
                                 for lab, key in (('G', 'G_render'), ('F', 'F_render'),
                                                  ('F-Gbeam', 'F_with_G_beam'))]))
        results.append({**comp, 'translation_px': {'F': tf, 'G': tg},
                        'recorded': {'F': residual(f), 'G': residual(g)},
                        'beam_pose_F_minus_G': beam_delta,
                        'counterfactual': rows, 'wheel_sweep_all_four_on_F': sweep,
                        'residual_top_features': attribution,
                        'raw_top_beam_masks': beam_masks,
                        'limit': model['geometry']['ood_residual_limit'],
                        'nearest_limit': model['geometry']['ood_nearest_limit']})
    return results


def feature_names() -> list[str]:
    """Names in the order of harness.camera_varied_start_geometry._features."""
    moments = ['cx', 'cy', 'x2', 'y2', 'xy', 'sin2a', 'cos2a', 'frac', 'wsum']
    names = [f'fg_{m}' for m in moments] + [f'yellow_{m}' for m in moments]
    for mask in ('fg', 'yellow'):
        names += [f'{mask}_comp{i}_{k}' for i in range(8) for k in ('area', 'cx', 'cy', 'w', 'h')]
        names += [f'{mask}_q{q}_{a}' for q in (5, 10, 25, 50, 75, 90, 95) for a in ('x', 'y')]
    for mask in ('fg', 'yellow'):
        names += [f'{mask}_grid_r{r}c{c}' for r in range(6) for c in range(8)]
    return names


def _geometry_masks(geometry: dict, canonical_top: bytes, slot: str):
    import base64
    import cv2
    from harness.camera_varied_start_geometry import _decode_top, _features, _roi
    background = cv2.imdecode(np.frombuffer(base64.b64decode(geometry['background_png_base64']), np.uint8),
                              cv2.IMREAD_COLOR)
    roi = _roi(_decode_top(canonical_top), slot)
    # Same thresholds as _features; recomputed only for the figure.
    diff = np.max(cv2.absdiff(roi, background), axis=2)
    fg = cv2.morphologyEx((diff > 18).astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    fg = cv2.morphologyEx(fg, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    yellow = ((hsv[:, :, 0] >= 18) & (hsv[:, :, 0] <= 38) & (hsv[:, :, 1] > 80)
              & (hsv[:, :, 2] > 70) & (fg > 0)).astype(np.uint8)
    return roi, fg, yellow, _features(roi, background)


def residual_attribution(geometry: dict, canonical_top: bytes, slot: str, top: int = 6) -> dict:
    _roi_img, _fg, _yellow, features = _geometry_masks(geometry, canonical_top, slot)
    mean, scale = np.asarray(geometry['feature_mean']), np.asarray(geometry['feature_scale'])
    components = np.asarray(geometry['ood_components'])
    z = (features - mean) / scale
    r = z - (z @ components.T) @ components
    names = feature_names()
    order = np.argsort(-np.abs(r))[:top]
    return {'orthogonal_residual': float(np.linalg.norm(r)),
            'top': [{'feature': names[i], 'residual_component': float(r[i]), 'raw': float(features[i]),
                     'train_mean': float(mean[i]), 'train_scale': float(scale[i])} for i in order]}


def write_figure(path: Path, figure_rows: list) -> None:
    import cv2
    rows = []
    for approach, stage, geometry, slot, tiles in figure_rows:
        row = []
        for label, canon, value in tiles:
            roi, fg, yellow, _ = _geometry_masks(geometry, canon, slot)
            vis = roi.copy()
            vis[fg > 0] = (0.5 * vis[fg > 0] + 0.5 * np.array([255, 0, 0])).astype(np.uint8)
            vis[yellow > 0] = (0, 0, 255)
            top = roi.copy()
            cv2.putText(top, f'a{approach} {stage} {label} res {value:.2f}', (4, 14),
                        cv2.FONT_HERSHEY_SIMPLEX, .42, (255, 255, 255), 1)
            row.append(np.vstack([top, vis]))
        rows.append(np.hstack(row))
    cv2.imwrite(str(path), np.vstack(rows), [cv2.IMWRITE_JPEG_QUALITY, 90])


def out_of_support_calls(linked: list) -> list:
    return [{'approach': r['approach'], 'slot': r['slot'], 'physical': r['physical'],
             'stage': r['call']['stage'], 'phase_index': r['call']['phase_index'],
             'frame_id': r['call']['frame_id'], 'issued_at_s': r['issued_at_s'],
             'orthogonal_residual': residual(r),
             'nearest_support_distance': r['call']['decision']['diagnostics'].get('nearest_support_distance')}
            for r in linked if r['call']['decision']['ok'] is False]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--raw-root', type=Path, default=RAW_ROOT)
    ap.add_argument('--runs', nargs='+', default=['Y1-grasp', 'Y2-grasp'])
    ap.add_argument('--stage-model-dir', type=Path, default=STAGE_MODELS)
    ap.add_argument('--reference-top', type=Path, default=REFERENCE_TOP)
    ap.add_argument('--no-render', action='store_true')
    ap.add_argument('--save-renders', type=Path)
    ap.add_argument('--figure', type=Path, help='ROI/mask figure (first run only)')
    ap.add_argument('--out', type=Path)
    args = ap.parse_args()
    models, model_hashes = load_models(args.stage_model_dir)
    reference = args.reference_top.read_bytes()
    report = {'schema': 'ugrp.r3_regrasp_pose_analysis.v1',
              'scope': 'evaluation-only offline analysis of recorded runs; no physics stepping; '
                       'replay qpos is observer-only and never control input',
              'inputs': {'raw_root': str(args.raw_root), 'stage_model_dir': str(args.stage_model_dir),
                         'stage_model_sha256': model_hashes,
                         'reference_top': str(args.reference_top.relative_to(ROOT)),
                         'reference_top_sha256': hashlib.sha256(reference).hexdigest()},
              'runs': {}}
    qpos_by_run = {}
    for run in args.runs:
        run_dir = args.raw_root / run
        decisions = json.loads((run_dir / 'pair-decisions.json').read_text())
        commands = json.loads((run_dir / 'issued-commands.json').read_text())
        replay = Replay(run_dir)
        qpos_by_run[run] = replay.qpos
        bindings = json.loads((run_dir / 'skill-bindings.json').read_text())['pair_model_slots']
        linked, times = link_calls(decisions, commands, bindings)
        mapping = slot_mapping(run_dir, decisions, replay, times)
        check = recompute(run_dir, linked, models)
        comps = pose_comparison(linked, replay, bindings)
        all_dt = [abs(replay.nearest(r['issued_at_s'])[1]) for r in linked]
        entry = {'input_sha256': {n: sha(run_dir / n) for n in
                                  ('pair-decisions.json', 'issued-commands.json', 'result.json',
                                   'skill-bindings.json', 'committed-plan.json')},
                 'replay_manifest_files_ok': replay.files_ok,
                 'replay_frames': int(len(replay.time)),
                 'slot_mapping': mapping, 'linkage': {'linked_calls': len(linked),
                                                      'max_abs_replay_dt_s': float(max(all_dt))},
                 'recompute': check, 'out_of_support_calls': out_of_support_calls(linked)}
        if args.no_render:
            for c in comps:
                c.pop('_f'), c.pop('_g')
            entry['pose_comparison'] = comps
        else:
            figure_rows = [] if args.figure is not None and run == args.runs[0] else None
            entry['pose_comparison'] = counterfactuals(run_dir, decisions, comps, replay, models,
                                                       bindings, reference,
                                                       args.save_renders / run if args.save_renders else None,
                                                       figure_rows)
            if figure_rows:
                write_figure(args.figure, figure_rows)
                entry['figure'] = {'path': args.figure.name, 'sha256': sha(args.figure)}
        report['runs'][run] = entry
    runs = list(qpos_by_run)
    report['replay_identical_across_runs'] = all(
        qpos_by_run[runs[0]].shape == qpos_by_run[r].shape and np.array_equal(qpos_by_run[runs[0]], qpos_by_run[r])
        for r in runs[1:])
    text = json.dumps(report, indent=1, ensure_ascii=False, default=float)
    if args.out:
        args.out.write_text(text + '\n')
    else:
        print(text)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
