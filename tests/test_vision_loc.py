"""Tests for the tag-free vision localization (experiments/2026-09-26-vision-loc).

Synthetic only (no rendered episodes, no simulator): ray-traced label images of
the tag-free walls_v3 map, the column interval observations, the camera-level
map ray cast, the particle filter on synthetic labels, the student input
boundary (no ``eval_only``/``teacher`` reads), the teacher output split, the
episode splits and the tag-free map itself. The segmentation network test runs
only where torch is installed.
"""
from __future__ import annotations

import builtins
import json
import math
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
VL_DIR = ROOT/'experiments'/'2026-09-26-vision-loc'
sys.path.insert(0, str(VL_DIR))
sys.path.insert(0, str(ROOT))

import vision_loc as vl  # noqa: E402

MAP = json.loads((VL_DIR/'maps'/'zone_wide_door_walls_v3_notags.json').read_text())
SEARCH = {1: 2000, 3: 740, 4: 2320, 5: 1320, 6: 1500}
LOOK_LEFT = {1: 1500, 3: 1072, 4: 2400, 5: 1482, 6: 2030}
LOOK_P20 = {1: 1500, 3: 1072, 4: 2400, 5: 1482, 6: 1500}
COLS = vl.column_positions(96, 2)


def render_labels(pose, servo, bias=0., dz=0., occluder_rows=None):
    """Ray-traced label image (floor 0, wall 1, background 4) of the tag-free map; optional occluder rows (object)."""
    from harness.wall_tags import camera_in_base
    mp = vl.mp
    o, r_bc = camera_in_base(servo)
    o = o + np.array([0., 0., dz])
    rot = r_bc @ mp.bias_rotation(bias).T
    vs, us = np.mgrid[0:vl.HEIGHT, 0:vl.WIDTH].astype(float)
    rays = np.stack([us, vs, np.ones_like(us)], -1) @ mp.K_INV.T @ rot.T
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    ow = np.array([x + c*o[0] - s*o[1], y + s*o[0] + c*o[1], o[2]])
    dw = np.stack([c*rays[..., 0] - s*rays[..., 1], s*rays[..., 0] + c*rays[..., 1], rays[..., 2]], -1)
    t_best = np.full(us.shape, np.inf)
    for w in MAP['obstacles']:
        if w.get('kind') != 'wall':
            continue
        lo = np.array([w['center_m'][0] - w['half_extents_m'][0], w['center_m'][1] - w['half_extents_m'][1], 0.])
        hi = np.array([w['center_m'][0] + w['half_extents_m'][0], w['center_m'][1] + w['half_extents_m'][1],
                       w['height_m']])
        with np.errstate(divide='ignore', invalid='ignore'):
            t1, t2 = (lo - ow)/dw, (hi - ow)/dw
        tmin = np.nanmax(np.minimum(t1, t2), -1)
        tmax = np.nanmin(np.maximum(t1, t2), -1)
        hit = (tmax >= np.maximum(tmin, 0)) & (tmin > 0)
        t_best = np.where(hit & (tmin < t_best), tmin, t_best)
    with np.errstate(divide='ignore', invalid='ignore'):
        t_floor = np.where(dw[..., 2] < 0, -ow[2]/dw[..., 2], np.inf)
    lab = np.full(us.shape, vl.BACKGROUND, np.uint8)
    lab[np.isfinite(t_best)] = vl.WALL
    lab[t_floor < t_best] = vl.FLOOR
    if occluder_rows is not None:
        lab[occluder_rows[0]:occluder_rows[1]] = vl.OBJECT
    return lab


def test_tagfree_map_has_no_tags_and_tall_walls():
    base = json.loads((ROOT/'maps'/'zones'/'zone_wide_door.json').read_text())
    assert MAP['landmarks']['tags'] == [] and 'door_posts' not in MAP['landmarks']
    assert MAP['wall_profile']['id'] == 'walls_v3' and MAP['wall_profile']['height_m'] == .40
    walls = [o for o in MAP['obstacles'] if o['kind'] == 'wall']
    assert walls and all(o['height_m'] == .40 for o in walls)
    strip = lambda m: [{k: v for k, v in o.items() if k != 'height_m'} for o in m['obstacles']]
    assert strip(MAP) == strip(base)
    for key in ('bounds_m', 'passages', 'regions', 'zone_slots', 'top_cameras'):
        assert MAP[key] == base[key], key
    from sim.research_dispatch_arena import digest
    assert MAP['base_map']['static_map_sha256'] == digest(base)


@pytest.mark.parametrize('pose,servo', [((-.5, -.6, 0.), SEARCH), ((1.5, .05, 0.), SEARCH),
                                        ((1.8, .0, .1), LOOK_P20), ((-.6, -2.25, 0.), LOOK_LEFT),
                                        ((3.0, .2, math.pi/2), SEARCH)])
def test_observations_match_expected_rows(pose, servo):
    lab = render_labels(pose, servo)
    obs = vl.column_observations(vl.one_hot(lab), COLS)
    cm = vl.column_model(servo, 0., 0., COLS)
    geo = vl.mp.MapGeometry(MAP, include_posts=False)
    vb, vt = vl.expected_rows(geo, np.array([pose]), cm)
    edge = obs.b_kind == vl.EDGE
    assert edge.sum() >= 20
    res = np.abs(obs.b_lo[edge] - vb[0][edge])
    assert np.median(res) < 1. and np.percentile(res, 90) < 2.5
    # every interval observation contains the expected row
    iv = obs.b_kind == vl.INTERVAL
    assert np.all((vb[0][iv] >= obs.b_lo[iv] - 2) & (vb[0][iv] <= obs.b_hi[iv] + 2))
    # the likelihood prefers the true pose over 10 cm / 5 deg perturbations
    poses = np.array([pose]) + np.array([[0, 0, 0], [.1, 0, 0], [-.1, 0, 0], [0, .1, 0], [0, -.1, 0],
                                         [0, 0, .09], [0, 0, -.09]])
    vb, vt = vl.expected_rows(geo, poses, cm)
    ll = vl.column_loglik(vb, vt, obs, {})
    assert ll[0] >= ll.max() - 1e-9


def test_wall_behind_camera_is_not_the_first_footprint():
    """West wall 0.13 m behind the chassis: the first footprint is the one in front (PR #210 cast from 0.6 m back)."""
    pose, servo = (-.92, -.85, 0.), SEARCH
    geo = vl.mp.MapGeometry(MAP, include_posts=False)
    cm = vl.column_model(servo, 0., 0., COLS)
    vb, _ = vl.expected_rows(geo, np.array([pose]), cm)
    obs = vl.column_observations(vl.one_hot(render_labels(pose, servo)), COLS)
    edge = obs.b_kind == vl.EDGE
    assert edge.sum() > 30 and np.median(np.abs(obs.b_lo[edge] - vb[0][edge])) < 1.
    old, _, _, _, _ = geo.expected_rows(np.array([pose]), cm, wall_height_m=.40)
    assert np.isnan(old[0][edge]).mean() > .5          # the old cast hit the wall behind the robot


def test_column_intervals_for_occlusion_free_floor_and_near_wall():
    h = vl.HEIGHT
    col = np.full(h, vl.WALL, np.uint8)
    col[300:] = vl.FLOOR
    lab = np.repeat(col[:, None], vl.WIDTH, 1)
    o = vl.column_observations(vl.one_hot(lab), COLS)
    assert np.all(o.b_kind == vl.EDGE) and np.allclose(o.b_lo, 299.5, atol=.6)
    lab2 = lab.copy()
    lab2[280:340] = vl.OBJECT                       # occluder over the boundary
    o2 = vl.column_observations(vl.one_hot(lab2), COLS)
    assert np.all(o2.b_kind == vl.INTERVAL) and np.allclose(o2.b_lo, 279.5) and np.allclose(o2.b_hi, 339.5)
    free = np.full((h, vl.WIDTH), vl.FLOOR, np.uint8)
    free[:40] = vl.BACKGROUND
    o3 = vl.column_observations(vl.one_hot(free), COLS)
    assert np.all(o3.b_kind == vl.INTERVAL) and np.all(o3.b_lo == vl.NEG_INF) and np.allclose(o3.b_hi, 39.5)
    near = np.full((h, vl.WIDTH), vl.WALL, np.uint8)
    o4 = vl.column_observations(vl.one_hot(near), COLS)
    assert np.all(o4.b_kind == vl.INTERVAL) and np.all(o4.b_hi == vl.POS_INF)
    assert np.all(o4.t_kind == vl.INTERVAL) and np.all(o4.t_lo == vl.NEG_INF)


def test_interval_prob():
    kind = np.array([vl.EDGE, vl.INTERVAL, vl.INTERVAL])
    lo = np.array([100., 50., vl.NEG_INF])
    hi = np.array([100., 80., 20.])
    p = vl.interval_prob(np.array([[100., 65., 0.], [110., 120., 60.]]), kind, lo, hi, 2.5)
    assert np.allclose(p[0], [1., 1., 1.], atol=1e-6)
    assert np.all(p[1] < 1e-3)


def test_vision_pf_tracks_synthetic_labels():
    import harness.owncam_localizer as base
    params = json.loads(json.dumps(base.DEFAULT_PARAMS))
    params['particles'] = 600
    sag = {s: {'s3': [740], 'bias': [0.], 'dz': [0.]} for s in ('loaded', 'unloaded')}
    pf = vl.make_vision_pf(base, MAP, params, {'settle_s': 0.}, {'columns': 48}, sag, seed=3)
    true = np.array([-.5, -.6, 0.])
    pf.init_gaussian(true + [.1, -.08, math.radians(5)], (.15, .15, math.radians(10)))
    pf.command({'t': 0., 'kind': 'initial_servo_command', 'pulses': SEARCH})
    t, errs = 0., []
    for k in range(40):
        cmd = {'t': t, 'kind': 'mecanum', 'forward': .06, 'left': 0., 'turn': .15 if k % 20 < 10 else -.15,
               'duration_s': .2}
        pf.command(cmd)
        g = np.asarray(params['motion']['gain']) @ np.array([.06, 0., cmd['turn']])
        c, s = math.cos(true[2]), math.sin(true[2])
        true = true + .2*np.array([c*g[0] - s*g[1], s*g[0] + c*g[1], g[2]])
        t += .2
        obs = vl.column_observations(vl.one_hot(render_labels(true, SEARCH)), pf.columns)
        est = pf.update_obs(t, obs, SEARCH)
        errs.append(math.hypot(est['x'] - true[0], est['y'] - true[1]))
    assert max(errs[-15:]) < .05


def test_settle_gate_skips_frames_right_after_own_servo_commands():
    import harness.owncam_localizer as base
    params = json.loads(json.dumps(base.DEFAULT_PARAMS))
    params['particles'] = 100
    sag = {s: {'s3': [740], 'bias': [0.], 'dz': [0.]} for s in ('loaded', 'unloaded')}
    pf = vl.make_vision_pf(base, MAP, params, {'settle_s': .4}, {'columns': 48}, sag, seed=1)
    pf.init_gaussian((-.5, -.6, 0.), (.05, .05, .05))
    pf.command({'t': 0., 'kind': 'initial_servo_command', 'pulses': SEARCH})
    obs = vl.column_observations(vl.one_hot(render_labels((-.5, -.6, 0.), SEARCH)), pf.columns)
    assert pf.update_obs(.2, obs, SEARCH)['measured'] is False
    assert pf.update_obs(.5, obs, SEARCH)['measured'] is True
    pf.command({'t': .6, 'kind': 'look', 'pan_pulse': 1560})
    assert pf.update_obs(.8, obs, {**SEARCH, 6: 1560})['measured'] is False


def _fake_episode(tmp_path):
    import cv2
    ep = tmp_path/'ep'
    for d in ('inputs', 'eval_only', 'teacher', 'frames'):
        (ep/d).mkdir(parents=True)
    (ep/'eval_only'/'frames_eval.jsonl').write_text('{}\n')
    (ep/'teacher'/'controller_events.jsonl').write_text('{}\n')
    (ep/'inputs'/'commands.jsonl').write_text(json.dumps({'t': 0., 'kind': 'hold'}) + '\n')
    (ep/'inputs'/'motion_profile.jsonl').write_text(json.dumps({'t': 0., 'profile': 'fine'}) + '\n')
    cv2.imwrite(str(ep/'frames'/'00000.jpg'), np.zeros((vl.HEIGHT, vl.WIDTH, 3), np.uint8))
    (ep/'inputs'/'frames.jsonl').write_text(json.dumps({'frame': 0, 't': .1, 'file': 'frames/00000.jpg'}) + '\n')
    return ep


def test_student_replay_never_reads_eval_only_or_teacher(monkeypatch, tmp_path):
    ep = _fake_episode(tmp_path)
    real_open = builtins.open

    def guarded(path, *a, **k):
        if 'eval_only' in str(path) or 'teacher' in str(path):
            raise AssertionError(f'student path read {path}')
        return real_open(path, *a, **k)
    monkeypatch.setattr(builtins, 'open', guarded)

    class Sink:
        def __init__(self):
            self.seen = []

        def command(self, row):
            self.seen.append(('cmd', row['t']))

        def set_motion_profile(self, t, name):
            self.seen.append(('profile', name))

        def frame(self, t, bgr, row):
            self.seen.append(('frame', t, bgr.shape))
    sink = Sink()
    assert vl.replay(ep, [sink]) == 1
    assert sorted(sink.seen[:2]) == [('cmd', 0.), ('profile', 'fine')]
    assert sink.seen[-1] == ('frame', .1, (vl.HEIGHT, vl.WIDTH, 3))


def test_vision_module_has_no_simulator_or_truth_access():
    code = (VL_DIR/'vision_loc.py').read_text()
    for banned in ('import mujoco', 'from mujoco', 'sim.zone_scene', 'sim.zone_arena', "'eval_only'", '"eval_only"',
                   'frames_eval', 'gt_trajectory', 'labels.jsonl', 'xpos', 'qpos', "'teacher'"):
        assert banned not in code, banned
    out = subprocess.run([sys.executable, '-c', (
        'import sys, builtins\n'
        'real = builtins.__import__\n'
        'def block(name, *a, **k):\n'
        '    if name == "mujoco" or name.startswith("mujoco.") or name == "torch":\n'
        '        raise ImportError("blocked")\n'
        '    return real(name, *a, **k)\n'
        'builtins.__import__ = block\n'
        f'sys.path.insert(0, {str(VL_DIR)!r}); sys.path.insert(0, {str(ROOT)!r})\n'
        'import vision_loc\n'
        'print("ok")\n')], capture_output=True, text=True, cwd=ROOT)
    assert out.returncode == 0 and 'ok' in out.stdout, out.stderr


def test_teacher_split_moves_reports_and_teacher_files(tmp_path):
    import run_vl_teacher_render as rt
    out = tmp_path/'ep'
    (out/'inputs').mkdir(parents=True)
    (out/'eval_only').mkdir()
    rows = [{'frame': 0, 't': .1, 'file': 'frames/00000.jpg', 'report': {'xyyaw': [1, 2, 3]}, 'phase': 'init'}]
    (out/'inputs'/'frames.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in rows))
    events = [{'t': 1., 'event': 'motion_profile', 'profile': 'fine'}, {'t': 2., 'event': 'sweep_start'}]
    (out/'controller_events.jsonl').write_text(''.join(json.dumps(e) + '\n' for e in events))
    (out/'skill_events.jsonl').write_text('')
    (out/'macros.jsonl').write_text('')
    moved = {}
    rt.split_outputs(out, [{'frame_id': 1}], moved)
    student = [json.loads(x) for x in (out/'inputs'/'frames.jsonl').read_text().splitlines()]
    assert 'report' not in student[0] and student[0]['phase'] == 'init'
    assert json.loads((out/'teacher'/'frames_with_teacher_report.jsonl').read_text())['report']
    assert not (out/'controller_events.jsonl').exists() and (out/'teacher'/'controller_events.jsonl').exists()
    assert json.loads((out/'inputs'/'motion_profile.jsonl').read_text()) == {'t': 1., 'profile': 'fine'}
    assert (out/'eval_only'/'labels.jsonl').exists()


def test_episode_splits_are_disjoint_and_preregistered():
    tab = json.loads((VL_DIR/'episodes.json').read_text())
    eps = tab['episodes']
    seeds = [e['seed'] for e in eps]
    assert len(set(seeds)) == len(seeds)
    count = {s: sum(e['split'] == s for e in eps) for s in ('train', 'dev', 'test')}
    assert count == {'train': 8, 'dev': 3, 'test': 6}
    assert all(e['episode_id'] == f"vl-{e['split']}-s{e['seed']}" for e in eps)
    assert tab['controller']['contact_profile'] == 'cargo_noslip_v1'
    assert 'teacher_gt_eval_only' in tab['controller']['pose_source']


def test_segmenter_shapes_when_torch_is_available(tmp_path):
    torch = pytest.importorskip('torch')
    pytest.importorskip('torchvision')
    import seg_model
    model = seg_model.build(pretrained_backbone=False)
    path = tmp_path/'m.pt'
    torch.save({'schema': seg_model.SCHEMA, 'state_dict': model.state_dict(), 'classes': vl.CLASSES}, path)
    seg = seg_model.Segmenter(path, 'cpu')
    p = seg.probs(np.zeros((vl.HEIGHT, vl.WIDTH, 3), np.uint8))
    assert p.shape == (vl.HEIGHT, vl.WIDTH, 5) and np.allclose(p.sum(2), 1., atol=1e-4)
