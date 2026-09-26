"""Offline evaluation of the v3.1 wrist-RGB-only judgments (render / score / adversarial).

v3.1 (``harness/zone_own_perception_v3_1.py``) is v3 behind an image
information gate. This script adds, and never edits, the v3 evaluator
(``scripts/eval_zone_own_perception_v3.py``, imported read-only):

``render``      v3's renderer, unchanged, pointed at the **v3.1 split file**
                (new seeds; the v3 test split was already used once). With
                ``--lighting`` a visual-only lighting profile is applied to the
                built world (light diffuse/ambient/position and the headlight) -
                the shadow stress set. Physics, cameras, robot appearance, paint,
                the catalogue and ``maps/zones/*`` are unchanged; the profile and
                the before/after light values are written to ``render-v3_1.json``.
``score``       v3.1 and v3 on the same frames, scored against ``eval-labels/``,
                multi-tick views through the v1 commit policy (v3 tracker).
``adversarial`` synthetic information-free and occluded versions of every view
                of a rendered split (black, near-black V 0-39, uniform grey,
                heavy blur, lens covers, partial covers), run through every
                v3.1 judgment in every posture, and v3 side by side.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import random
import subprocess
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from scripts import eval_zone_own_perception_v2 as ev2  # noqa: E402  (read-only reuse)
from scripts import eval_zone_own_perception_v3 as ev3  # noqa: E402  (read-only reuse)
from scripts.eval_zone_color_detection import sha256  # noqa: E402

SPLIT_FILE = ROOT/'experiments/2026-09-26-zone-own-perception-v3-1/split.json'
SCHEMA = 'ugrp.zone_own_perception_v3_1_eval.v1'
CONFIDENT = ev2.CONFIDENT

# --- shadow stress lighting (PREREGISTERED, visual only) ---------------------------
# The standard scene has four shadow-casting lights (three key lights and the
# ``dispatch_ceiling`` light) plus the MuJoCo headlight.
LIGHTING = {
    'standard': {'note': 'scene lighting unchanged'},
    'dim_all': {'note': 'all light diffuse and ambient x0.4, headlight x0.4',
                'diffuse_scale': .4, 'ambient_scale': .4, 'headlight_scale': .4},
    'ceiling_only': {'note': 'only the overhead ceiling light (diffuse 1.0); key lights, ambient and '
                             'headlight off: hard shadow of the own arm straight down',
                     'only_light': 3, 'only_diffuse': 1., 'ambient_scale': 0., 'headlight_scale': 0.},
    'low_side_key': {'note': 'key light 0 moved low to the side (1.0 m high, 30 deg elevation); other lights '
                             'x0.25, headlight off: long raking shadows',
                     'move_light': 0, 'pos': (0., -3.4, 1.0), 'dir': (0., .866, -.5),
                     'others_scale': .25, 'headlight_scale': 0.},
}

# --- adversarial frames (PREREGISTERED) ---------------------------------------------
# ``INFO_FREE`` must be ``unknown`` for every v3.1 judgment in every posture.
# ``OCCLUDED`` must not produce a confident error; ``OCCLUDED_LIT`` is reported only.
INFO_FREE = ('black', 'black_10', 'near_black_uniform', 'near_black_noise', 'dimmed_scene', 'uniform_grey',
             'blur_s4', 'blur_s8', 'blur_s16', 'lens_cover_dark', 'lens_cover_lit')
OCCLUDED = ('partial_cover_dark',)
OCCLUDED_LIT = ('partial_cover_lit',)
TRANSFORMS = INFO_FREE + OCCLUDED + OCCLUDED_LIT
POSTURES = {'carry': dict(ev3.CARRY), 'held_check': dict(ev3.HELD_CHECK)}
V3_MATRIX_EVERY = 4


def _git(*args):
    return subprocess.check_output(['git', *args], cwd=ROOT, text=True).strip()


def load_split():
    return json.loads(SPLIT_FILE.read_text())


# ------------------------------------------------------------------ adversarial frames

def _vignette(height, width):
    yy, xx = np.mgrid[0:height, 0:width]
    return np.clip(np.hypot((xx-width/2)/(width/2), (yy-height/2)/(height/2))/1.3, 0, 1)


def adversarial_frame(name: str, frame: np.ndarray, rng: random.Random) -> tuple[np.ndarray, dict]:
    """One adversarial version of a BGR frame. ``rng`` fixes every random parameter."""
    import cv2
    height, width = frame.shape[:2]
    noise = np.random.default_rng(rng.randrange(1 << 30))
    params: dict = {}
    if name == 'black':
        out = np.zeros_like(frame)
    elif name == 'black_10':
        out = np.full_like(frame, 10)                         # the Codex review repro
    elif name == 'near_black_uniform':
        level = rng.randint(0, 39)
        params['level'] = level
        out = np.full_like(frame, level)
    elif name == 'near_black_noise':
        level = rng.randint(0, 39)
        params['level'] = level
        out = np.clip(level+noise.normal(0, 4, frame.shape), 0, 39).astype(np.uint8)
    elif name == 'dimmed_scene':
        out = (frame.astype(np.float32)*39./255.).astype(np.uint8)   # textured, brightness 0-39
    elif name == 'uniform_grey':
        level = rng.randint(40, 250)
        params['level'] = level
        out = np.full_like(frame, level)
    elif name.startswith('blur_s'):
        sigma = float(name[6:])
        params['sigma'] = sigma
        out = cv2.GaussianBlur(frame, (0, 0), sigma)
    elif name in ('lens_cover_dark', 'lens_cover_lit'):
        if name == 'lens_cover_dark':
            centre, tint = rng.uniform(20, 60), (rng.uniform(.7, 1.), rng.uniform(.7, 1.), 1.)
        else:
            centre, tint = rng.uniform(120, 230), (.6, .75, 1.)          # skin / translucent cover
        params.update(centre=round(centre, 1), tint=[round(v, 3) for v in tint])
        base = centre*(1-.75*_vignette(height, width))
        out = np.clip(base[..., None]*np.asarray(tint)+noise.normal(0, 2, frame.shape), 0, 255).astype(np.uint8)
    elif name in ('partial_cover_dark', 'partial_cover_lit'):
        share = rng.uniform(.5, .9)
        level = rng.uniform(5, 39) if name == 'partial_cover_dark' else rng.uniform(120, 220)
        params.update(covered_rows_share=round(share, 3), level=round(level, 1))
        yy, xx = np.mgrid[0:height, 0:width]
        edge = height*(1-share)+.08*height*np.sin(xx/width*math.pi*rng.uniform(1, 3))
        alpha = cv2.GaussianBlur((yy >= edge).astype(np.float32), (0, 0), 15)[..., None]
        cover = np.clip(level*(1-.3*_vignette(height, width))[..., None]
                        + noise.normal(0, 2, frame.shape), 0, 255)
        out = (frame*(1-alpha)+cover*alpha).astype(np.uint8)
    else:
        raise ValueError(name)
    return out, params


def _jpeg(frame):
    import cv2
    ok, buf = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    assert ok
    return buf.tobytes()


# ------------------------------------------------------------------ render (v3 renderer, v3.1 split)

def _apply_lighting(world, profile_name):
    import mujoco  # noqa: F401  (eval-side only)
    profile = LIGHTING[profile_name]
    m = world.model
    before = {'light_pos': m.light_pos.tolist(), 'light_dir': m.light_dir.tolist(),
              'light_diffuse': m.light_diffuse.tolist(), 'light_ambient': m.light_ambient.tolist(),
              'headlight': {'ambient': [float(v) for v in m.vis.headlight.ambient],
                           'diffuse': [float(v) for v in m.vis.headlight.diffuse]}}
    if 'diffuse_scale' in profile:
        m.light_diffuse[:] *= profile['diffuse_scale']
    if 'ambient_scale' in profile:
        m.light_ambient[:] *= profile['ambient_scale']
    if 'only_light' in profile:
        keep = profile['only_light']
        for i in range(m.nlight):
            m.light_diffuse[i] = profile['only_diffuse'] if i == keep else 0.
            m.light_specular[i] = m.light_specular[i] if i == keep else 0.
    if 'move_light' in profile:
        i = profile['move_light']
        m.light_pos[i] = profile['pos']
        d = np.asarray(profile['dir'], float)
        m.light_dir[i] = d/np.linalg.norm(d)
        for j in range(m.nlight):
            if j != i:
                m.light_diffuse[j] *= profile['others_scale']
    if 'headlight_scale' in profile:
        m.vis.headlight.ambient[:] = np.asarray(m.vis.headlight.ambient)*profile['headlight_scale']
        m.vis.headlight.diffuse[:] = np.asarray(m.vis.headlight.diffuse)*profile['headlight_scale']
    after = {'light_pos': m.light_pos.tolist(), 'light_dir': m.light_dir.tolist(),
             'light_diffuse': m.light_diffuse.tolist(), 'light_ambient': m.light_ambient.tolist(),
             'headlight': {'ambient': [float(v) for v in m.vis.headlight.ambient],
                           'diffuse': [float(v) for v in m.vis.headlight.diffuse]}}
    return {'profile': profile_name, **profile, 'before': before, 'after': after}


def render(args):
    lighting_log = []
    original = ev2.build_world

    def build_world(variant, seed, rng):
        world, items, info = original(variant, seed, rng)
        if args.lighting != 'standard':
            lighting_log.append({'variant': variant, 'seed': seed, **_apply_lighting(world, args.lighting)})
        return world, items, info

    ev3.SPLIT_FILE = SPLIT_FILE                   # module global read by ev3.load_split and the manifest
    ev2.build_world = build_world
    try:
        ev3.render(SimpleNamespace(split=args.split, output=args.output, families=args.families,
                                   max_scenes=args.max_scenes, grasp_posture=args.grasp_posture))
    finally:
        ev2.build_world = original
    sidecar = {'schema': SCHEMA+'.render', 'renderer': 'scripts/eval_zone_own_perception_v3.py (unchanged)',
               'renderer_sha256': sha256(ROOT/'scripts/eval_zone_own_perception_v3.py'),
               'wrapper_sha256': sha256(Path(__file__)), 'split_file': str(SPLIT_FILE.relative_to(ROOT)),
               'split_file_sha256': sha256(SPLIT_FILE), 'split': args.split, 'lighting': args.lighting,
               'lighting_profile': LIGHTING[args.lighting], 'lighting_applied': lighting_log,
               'source_sha': _git('rev-parse', 'HEAD'), 'source_dirty': bool(_git('status', '--porcelain'))}
    (Path(args.output)/'render-v3_1.json').write_text(json.dumps(sidecar, indent=1))


# ------------------------------------------------------------------ score

def _answers_for_tick(family, posture, frame, pose, question, v3, v31):
    expected = question['expected_kind']
    rows = []
    for version, mod in (('v3_1', v31), ('v3', v3)):
        if family == 'held':
            rows.append((version, 'held_item_at_grip', mod.judge_held_item(frame, pose, expected_kind=expected)))
        elif family == 'carry':
            rows.append((version, 'team_cargo_at_grip',
                         mod.judge_team_cargo_at_grip(frame, pose, expected_kind=expected)))
        else:
            both = mod.judge_team_cargo_grasp_stage(frame, pose, expected_kind=expected)
            rows.append((version, 'team_cargo_identity', {**both['identity'], 'judgment': 'team_cargo_identity'}))
            rows.append((version, 'team_cargo_handle', {**both['handle'], 'judgment': 'team_cargo_handle'}))
    return rows


def _gate_info(answer):
    info = answer.get('image_information')
    return None if info is None else {k: info[k] for k in ('sufficient', 'failed', 'lit_share', 'value_range',
                                                           'edge_share', 'sharpness')}


def score_views(frames_dir, transform=None, transform_seed='v3_1'):
    """Score every view; with ``transform`` each tick frame is first replaced by its adversarial version."""
    import cv2
    from harness import zone_own_outcome_v3 as outcome
    from harness import zone_own_perception_v3 as v3
    from harness import zone_own_perception_v3_1 as v31
    frames_dir = Path(frames_dir)
    manifest = json.loads((frames_dir/'manifest.json').read_text())
    records = []
    for view in manifest['views']:
        if 'skipped' in view:
            continue
        vid = view['view_id']
        labels = json.loads((frames_dir/'eval-labels'/f'{vid}.json').read_text())
        inputs = json.loads((frames_dir/'frames'/vid/'actor-inputs.json').read_text())
        family, posture, question = inputs['family'], labels['posture'], inputs['question']
        rng = random.Random(f'{transform_seed}:{transform}:{vid}')
        series = {}
        for tick_in, tick_lab in zip(inputs['ticks'], labels['ticks']):
            frame = (frames_dir/'frames'/vid/tick_in['frame']).read_bytes()
            params = None
            if transform is not None:
                bgr = cv2.imdecode(np.frombuffer(frame, np.uint8), 1)
                bgr, params = adversarial_frame(transform, bgr, random.Random(f'{rng.random()}'))
                frame = _jpeg(bgr)
            pose = {int(k): int(v) for k, v in tick_in['commanded_arm_pwm'].items()}
            for version, judgment, answer in _answers_for_tick(family, posture, frame, pose, question, v3, v31):
                series.setdefault((version, judgment), []).append(answer)
                records.append({'record': 'tick', 'view_id': vid, 'case': labels['case'], 'family': family,
                                'posture': posture, 'version': version, 'judgment': judgment,
                                'transform': transform, 'transform_params': params,
                                'variant': inputs['variant'], 'seed': inputs['seed'], 'tick': tick_in['tick'],
                                'answer': answer['answer'], 'confidence': answer['confidence'],
                                'reason': answer['reason'], 'observed': answer.get('observed'),
                                'cue': answer.get('cue'), 'v3_reason': answer.get('v3_reason'),
                                'image_information': _gate_info(answer),
                                'truth': tick_lab['truth'][judgment],
                                'truth_observed': tick_lab['truth_observed'][judgment],
                                'observable': tick_lab['observable'][judgment]})
        final = labels['ticks'][-1]
        for (version, judgment), observations in series.items():
            name = observations[0]['judgment']
            decision = outcome.track(name, [{**o, 'judgment': name} for o in observations], question=question)
            records.append({'record': 'view', 'view_id': vid, 'case': labels['case'], 'family': family,
                            'posture': posture, 'version': version, 'judgment': judgment,
                            'transform': transform, 'variant': inputs['variant'], 'seed': inputs['seed'],
                            'ticks': len(observations), 'status': decision['status'],
                            'answer': decision['answer'], 'confidence': decision['confidence'],
                            'reason': decision['reason'], 'observed': decision.get('observed'),
                            'unknown_ticks': decision['unknown_ticks'],
                            'truth': final['truth'][judgment], 'truth_observed': final['truth_observed'][judgment],
                            'observable': final['observable'][judgment]})
    return records, manifest


def summarize(records):
    return ev3.summarize(records)


def _load():
    return {'start': os.getloadavg()[0]}


def score(args):
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=False)
    load = _load()
    started = time.monotonic()
    records, manifest = score_views(args.frames)
    (out/'records.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in records))
    load['end'] = os.getloadavg()[0]
    sidecar = Path(args.frames)/'render-v3_1.json'
    summary = {'schema': SCHEMA+'.score', 'frames_dir': str(args.frames), 'split': manifest['split'],
               'split_file_sha256': manifest['split_file_sha256'],
               'lighting': json.loads(sidecar.read_text())['lighting'] if sidecar.exists() else 'standard',
               'render_source_sha': manifest['source_sha'], 'render_source_dirty': manifest['source_dirty'],
               'score_source_sha': _git('rev-parse', 'HEAD'),
               'score_source_dirty': bool(_git('status', '--porcelain')),
               'confident_threshold': CONFIDENT, 'load_average_1min': load,
               'wall_s': round(time.monotonic()-started, 1), 'results': summarize(records),
               'information_gate': gate_rates(records)}
    (out/'summary.json').write_text(json.dumps(summary, indent=1))
    print(json.dumps({'written': str(out/'summary.json'), 'load_average_1min': load}))


def gate_rates(records):
    """Share of v3.1 tick answers stopped by the information gate, per family/posture."""
    out = {}
    rows = [r for r in records if r['record'] == 'tick' and r['version'] == 'v3_1']
    for key in sorted({(r['family'], r['posture'], r['judgment']) for r in rows}):
        sel = [r for r in rows if (r['family'], r['posture'], r['judgment']) == key]
        stopped = [r for r in sel if r['reason'] == 'INSUFFICIENT_IMAGE_INFORMATION']
        out[':'.join(key)] = {'ticks': len(sel), 'stopped_by_gate': len(stopped),
                              'stopped_cases': sorted({r['case'] for r in stopped})}
    return out


# ------------------------------------------------------------------ adversarial

def _all_judgments(mod, frame, pose):
    """Every judgment of ``mod`` for every ordered kind on one frame and pose."""
    rows = []
    for kind in mod.SOLO_KINDS:
        rows.append(('held_item_at_grip', kind, mod.judge_held_item(frame, pose, expected_kind=kind)))
    for kind in mod.TEAM_KINDS:
        rows.append(('team_cargo_at_grip', kind, mod.judge_team_cargo_at_grip(frame, pose, expected_kind=kind)))
        rows.append(('team_cargo_handle', kind, mod.judge_team_cargo_handle(frame, pose, expected_kind=kind)))
        both = mod.judge_team_cargo_grasp_stage(frame, pose, expected_kind=kind)
        rows.append(('team_cargo_grasp_stage', kind, both))
        rows.append(('team_cargo_grasp_stage.identity', kind, both['identity']))
        rows.append(('team_cargo_grasp_stage.handle', kind, both['handle']))
    return rows


def adversarial(args):
    import cv2
    from harness import zone_own_perception_v3 as v3
    from harness import zone_own_perception_v3_1 as v31
    frames_dir = Path(args.frames)
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=False)
    load = _load()
    started = time.monotonic()
    manifest = json.loads((frames_dir/'manifest.json').read_text())
    poses = {**POSTURES, 'grasp_look_v3': dict(v31.GRASP_LOOK_POSTURE)}
    matrix = []
    # 1) every v3.1 judgment x ordered kind x posture on tick 0 of every view, information-free
    #    transforms. v3 side by side (0.2 s per call) on every 4th view in the frame's own posture.
    views = [v for v in manifest['views'] if 'skipped' not in v]
    for index, view in enumerate(views):
        vid = view['view_id']
        inputs = json.loads((frames_dir/'frames'/vid/'actor-inputs.json').read_text())
        tick0 = inputs['ticks'][0]
        own = {int(k): int(v) for k, v in tick0['commanded_arm_pwm'].items()}
        bgr = cv2.imread(str(frames_dir/'frames'/vid/tick0['frame']))
        for name in INFO_FREE:
            adv, params = adversarial_frame(name, bgr, random.Random(f'v3_1:matrix:{name}:{vid}'))
            jpeg = _jpeg(adv)
            info = v31.image_information(jpeg)
            runs = [('v3_1', v31, posture, pose) for posture, pose in poses.items()]
            if index % V3_MATRIX_EVERY == 0:
                runs.append(('v3', v3, 'own:'+view.get('posture', ''), own))
            for version, mod, posture, pose in runs:
                for judgment, kind, answer in _all_judgments(mod, jpeg, pose):
                    matrix.append({'view_id': vid, 'transform': name, 'params': params, 'posture': posture,
                                   'version': version, 'judgment': judgment, 'expected_kind': kind,
                                   'answer': answer['answer'], 'confidence': answer['confidence'],
                                   'reason': answer['reason'], 'gate_failed': info['failed']})
    (out/'matrix.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in matrix))
    # 2) the view's own judgments over all ticks, through the tracker, every transform
    records = []
    for name in TRANSFORMS:
        rows, _ = score_views(frames_dir, transform=name)
        records += rows
    (out/'records.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in records))
    load['end'] = os.getloadavg()[0]
    summary = {'schema': SCHEMA+'.adversarial', 'frames_dir': str(frames_dir), 'split': manifest['split'],
               'split_file_sha256': manifest['split_file_sha256'], 'render_source_sha': manifest['source_sha'],
               'score_source_sha': _git('rev-parse', 'HEAD'),
               'score_source_dirty': bool(_git('status', '--porcelain')),
               'transforms': {'info_free': INFO_FREE, 'occluded': OCCLUDED, 'occluded_lit_report_only': OCCLUDED_LIT},
               'load_average_1min': load, 'wall_s': round(time.monotonic()-started, 1),
               'results': summarize_adversarial(matrix, records)}
    (out/'summary.json').write_text(json.dumps(summary, indent=1))
    print(json.dumps({'written': str(out/'summary.json'), 'load_average_1min': load,
                      'wall_s': summary['wall_s']}))


def summarize_adversarial(matrix, records):
    res = {'matrix': {}, 'views': {}}
    for version in ('v3_1', 'v3'):
        for name in INFO_FREE:
            sel = [r for r in matrix if r['version'] == version and r['transform'] == name]
            decided = [r for r in sel if r['answer'] != 'unknown']
            confident = [r for r in decided if r['confidence'] >= CONFIDENT]
            res['matrix'][f'{version}:{name}'] = {
                'answers': len(sel), 'not_unknown': len(decided), 'confident_not_unknown': len(confident),
                'examples': sorted({(r['judgment'], r['posture'], r['expected_kind'], r['answer'], r['reason'])
                                    for r in confident})[:8]}
    views = [r for r in records if r['record'] == 'view']
    for version in ('v3_1', 'v3'):
        for name in TRANSFORMS:
            sel = [r for r in views if r['version'] == version and r['transform'] == name]
            confirmed = [r for r in sel if r['status'] == 'confirmed']
            wrong = [r for r in confirmed if r['answer'] != r['truth']]
            res['views'][f'{version}:{name}'] = {
                'views': len(sel), 'confirmed': len(confirmed), 'confirmed_wrong': len(wrong),
                'unknown': sum(r['answer'] == 'unknown' for r in sel),
                'confirmed_cases': sorted({(r['case'], r['judgment'], r['answer']) for r in confirmed})[:10],
                'confirmed_wrong_cases': sorted({(r['case'], r['judgment'], r['answer'], r['truth'])
                                                 for r in wrong})[:10]}
    return res


# ------------------------------------------------------------------ cli

def parser():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = p.add_subparsers(dest='command', required=True)
    r = sub.add_parser('render', help='render wrist frames + eval labels for one v3.1 split (v3 renderer)')
    r.add_argument('--split', required=True)
    r.add_argument('--output', type=Path, required=True)
    r.add_argument('--families', action='append', choices=('held', 'carry', 'grasp'))
    r.add_argument('--max-scenes', type=int, default=0)
    r.add_argument('--grasp-posture', action='append', choices=sorted(ev3.GRASP_CANDIDATES)+['grasp_look_v3'])
    r.add_argument('--lighting', choices=sorted(LIGHTING), default='standard')
    s = sub.add_parser('score', help='run v3.1 and v3 on a rendered split and score them')
    s.add_argument('--frames', type=Path, required=True)
    s.add_argument('--output', type=Path, required=True)
    a = sub.add_parser('adversarial', help='information-free / occluded versions of a rendered split')
    a.add_argument('--frames', type=Path, required=True)
    a.add_argument('--output', type=Path, required=True)
    return p


def main(argv=None):
    args = parser().parse_args(argv)
    {'render': render, 'score': score, 'adversarial': adversarial}[args.command](args)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
