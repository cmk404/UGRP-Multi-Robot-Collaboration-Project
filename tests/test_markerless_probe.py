"""Tests for the tag-free localization probe (experiments/2026-09-26-markerless-probe).

Synthetic only (no recorded frames, no simulator): the floor-trace geometry,
the map ray cast, the band detector on a ray-traced synthetic image of the
static map, particle-filter tracking on synthetic scans, and the input boundary
(no ``eval_only`` reads, no simulator imports in the localization path).
"""
from __future__ import annotations

import builtins
import importlib.util
import json
import math
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
PROBE_DIR = ROOT/'experiments'/'2026-09-26-markerless-probe'
MAP = json.loads((ROOT/'maps'/'zones'/'zone_wide_door_tags_v1.json').read_text())
SEARCH = {1: 2000, 3: 740, 4: 2320, 5: 1320, 6: 1500}
LOOK_LEFT = {1: 1500, 3: 1072, 4: 2400, 5: 1482, 6: 2030}


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope='module')
def probe():
    sys.path.insert(0, str(PROBE_DIR))
    return _load('markerless_probe', PROBE_DIR/'markerless_probe.py')


def _markerless_view(static_map):
    view = json.loads(json.dumps(static_map))
    view['landmarks'] = {'tags': [], 'door_posts': []}
    return view


def render(probe, pose, servo, bias=0., wall_level=60., floor_level=120., outside_level=90.):
    """Ray-traced undistorted image: floor, 0.10 m wall faces (uniform) and what lies beyond."""
    from harness.wall_tags import camera_in_base
    o, r_bc = camera_in_base(servo)
    rot = r_bc @ probe.bias_rotation(bias).T
    vs, us = np.mgrid[0:probe.HEIGHT, 0:probe.WIDTH].astype(float)
    rays = np.stack([us, vs, np.ones_like(us)], -1) @ probe.K_INV.T @ rot.T      # base frame
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    ow = np.array([x + c*o[0] - s*o[1], y + s*o[0] + c*o[1], o[2]])
    dw = np.stack([c*rays[..., 0] - s*rays[..., 1], s*rays[..., 0] + c*rays[..., 1], rays[..., 2]], -1)
    t_best = np.full(us.shape, np.inf)
    for w in MAP['obstacles']:
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
    img = np.full(us.shape, outside_level)
    floor = t_floor < t_best
    fx, fy = ow[0] + t_floor*dw[..., 0], ow[1] + t_floor*dw[..., 1]
    x0, x1, y0, y1 = MAP['bounds_m']
    inside = (fx > x0) & (fx < x1) & (fy > y0) & (fy < y1)
    img = np.where(floor & inside, floor_level, img)
    img = np.where(np.isfinite(t_best) & ~floor, wall_level, img)
    return np.repeat(img[..., None], 3, -1).astype(np.uint8)


def test_floor_trace_round_trip(probe):
    cols = probe.column_positions(24)
    cm = probe.column_model(SEARCH, -.02, cols)
    v = np.linspace(200., 470., 5)[:, None]*np.ones((1, len(cols)))
    t = cm.t_of_row(v)
    assert np.allclose(cm.rows(t), v, atol=1e-6)
    # floor-trace points and wall-height-trace points project into their own column
    from harness.wall_tags import camera_in_base
    o, r_bc = camera_in_base(SEARCH)
    rot = r_bc @ probe.bias_rotation(-.02).T
    s_ = np.linspace(0., 3., 7)
    for h in (0., .1):
        q0, _ = (cm.q0, None) if h == 0. else cm.trace_at(h)
        for sv in s_:
            pts = np.concatenate([q0 + sv*cm.d, np.full((len(cols), 1), h)], 1)
            cam = (pts - o) @ rot
            assert np.allclose(probe.CX + probe.FX*cam[:, 0]/cam[:, 2], cols, atol=1e-6)
            rows = cm.rows(np.full(len(cols), sv)) if h == 0. else cm.rows_at(np.full(len(cols), sv), h)
            assert np.allclose(probe.CY + probe.FY*cam[:, 1]/cam[:, 2], rows, atol=1e-6)


def test_raycast_axis_aligned(probe):
    geo = probe.MapGeometry(_markerless_view(MAP))
    t, h = geo.raycast(np.array([0., 0.]), np.array([-1., 0.]), np.array([1., 0.]), np.array([0., 1.]))
    assert t[0] == pytest.approx(2.175) and h[0] == pytest.approx(.10)     # divider face at x 2.175
    assert t[1] == pytest.approx(1.425)                                   # north wall inner face
    t, _ = geo.raycast(0., .05, 1., 0.)                                   # through the door
    assert t == pytest.approx(5.375)


@pytest.mark.parametrize('pose,servo', [((-.4, -.8, .1), SEARCH), ((3.6, -2.2, -.3), SEARCH),
                                        ((.5, .6, .6), LOOK_LEFT)])
def test_detector_recovers_rendered_walls(probe, pose, servo):
    geo = probe.MapGeometry(_markerless_view(MAP))
    cols = probe.column_positions(48)
    img = render(probe, pose, servo, bias=-.02)
    cm = probe.column_model(servo, -.02, cols)
    scan = probe.detect_boundaries(img, cm)
    vb, vt, vtf, _, _ = geo.expected_rows(np.array([pose]), cm)
    visible = np.isfinite(vb[0]) & (vb[0] > 5) & (vb[0] < probe.HEIGHT - 10)
    assert visible.sum() >= 10
    det = scan.detected & visible
    assert det.sum() >= .8*visible.sum()
    err = np.abs(scan.vb[det, 0] - vb[0, det])
    assert np.median(err) < 1. and np.percentile(err, 90) < 2.
    top = det & scan.top_visible[:, 0] & np.isfinite(vt[0]) & np.isfinite(vtf[0])
    if top.sum() >= 5:          # measured top edge vs the exact near/far edges of the wall's top face
        near, far = np.abs(scan.vt[top, 0] - vt[0, top]), np.abs(scan.vt[top, 0] - vtf[0, top])
        assert np.median(np.minimum(near, far)) < 1.5
    # the pseudo range agrees with the ray-cast floor distance
    t = cm.t_of_row(vb[0])
    rng, _ = cm.range_bearing(np.where(np.isfinite(t), t, 0.))
    rel = np.abs(scan.range_m[det, 0] - rng[det])/rng[det]
    assert np.median(rel) < .02


def test_detector_ignores_floor_only_image(probe):
    img = np.full((probe.HEIGHT, probe.WIDTH, 3), 110, np.uint8)
    img[300:, :] = 90                              # one floor edge, no wall band above it
    scan = probe.detect_boundaries(img, probe.column_model(SEARCH, -.02, probe.column_positions(48)))
    assert scan.detected.sum() == 0


def test_edge_errors_use_top_edge_when_bottom_is_hidden(probe):
    cols = probe.column_positions(4)
    scan = probe.Scan(cols, np.array([[100., np.nan]]*4), np.full((4, 2), np.nan), np.zeros((4, 2), bool),
                      np.ones((4, 2)), np.zeros((4, 2)), np.ones((4, 2)), np.zeros((4, 2)),
                      np.array([150, 150, 480, 480]))
    vb_exp = np.array([[200., 200., 100., 200.]])      # columns 0-1: bottom hidden behind the carried box
    vt_exp = np.array([[101., 140., 60., 60.]])
    e = probe.edge_errors(vb_exp, vt_exp, scan, 1., use_top_edge=False)[0]
    assert e[0] == pytest.approx(1.) and e[1] == pytest.approx(1600.)   # matched to the top edge
    assert e[2] == pytest.approx(0.)                                    # bottom visible: bottom edge
    assert not np.isinf(e[3]) and e[3] == pytest.approx(1e4)


def test_boundary_pf_tracks_synthetic_scans(probe):
    import harness.owncam_localizer as base
    params = json.loads(json.dumps(base.DEFAULT_PARAMS))
    params['particles'] = 800
    view = _markerless_view(MAP)
    geo = probe.MapGeometry(view)
    pf = probe.make_boundary_pf(base, view, params, {'bias_rad': {'unloaded': -.02, 'loaded': -.02}}, {}, seed=3)
    rng = np.random.default_rng(7)
    true = np.array([-.5, -.6, 0.])
    pf.init_gaussian(true + [.12, -.1, math.radians(6)], (.15, .15, math.radians(10)))
    pf.command({'t': 0., 'kind': 'initial_servo_command', 'pulses': SEARCH})
    t = 0.
    errs = []
    for k in range(60):
        cmd = {'t': t, 'kind': 'mecanum', 'forward': .06, 'left': 0., 'turn': .15 if k % 20 < 10 else -.15,
               'duration_s': .2}
        pf.command(cmd)
        # truth follows the same nominal gains (the default motion model)
        g = np.asarray(params['motion']['gain']) @ np.array([.06, 0., cmd['turn']])
        c, s = math.cos(true[2]), math.sin(true[2])
        true = true + .2*np.array([c*g[0] - s*g[1], s*g[0] + c*g[1], g[2]])
        t += .2
        cm = probe.column_model(SEARCH, -.02, pf.columns)
        vb, vt, _, _, _ = geo.expected_rows(true[None, :], cm)
        noisy = vb[0] + rng.normal(0, 1., vb.shape[1])
        keep = np.isfinite(noisy) & (noisy > 3) & (noisy < probe.HEIGHT - 3) & (rng.random(vb.shape[1]) > .3)
        vbo = np.where(keep, noisy, np.nan)[:, None]
        scan = probe.Scan(pf.columns, np.hstack([vbo, np.full_like(vbo, np.nan)]), np.full((len(vbo), 2), np.nan),
                          np.zeros((len(vbo), 2), bool), np.ones((len(vbo), 2)), np.zeros((len(vbo), 2)),
                          np.ones((len(vbo), 2)), np.zeros((len(vbo), 2)), np.full(len(vbo), probe.HEIGHT))
        est = pf.update_scan(t, scan, SEARCH)
        errs.append(math.hypot(est['x'] - true[0], est['y'] - true[1]))
    assert max(errs[-20:]) < .05


def test_localize_path_never_reads_eval_only(probe, monkeypatch, tmp_path):
    """``episode_inputs``/``replay`` read inputs only (a fake episode; eval_only must stay unread)."""
    ep = tmp_path/'ep'
    (ep/'inputs').mkdir(parents=True)
    (ep/'eval_only').mkdir()
    (ep/'eval_only'/'frames_eval.jsonl').write_text('{}\n')
    (ep/'manifest.json').write_text(json.dumps({'spec': {'seed': 1, 'spawn_y': 0.}}))
    (ep/'controller_events.jsonl').write_text(json.dumps({'t': 0., 'event': 'motion_profile', 'profile': 'fine'}) + '\n')
    (ep/'inputs'/'commands.jsonl').write_text(json.dumps({'t': 0., 'kind': 'hold'}) + '\n')
    import cv2
    (ep/'frames').mkdir()
    cv2.imwrite(str(ep/'frames'/'00000.jpg'), np.zeros((probe.HEIGHT, probe.WIDTH, 3), np.uint8))
    (ep/'inputs'/'frames.jsonl').write_text(json.dumps({'frame': 0, 't': .1, 'file': 'frames/00000.jpg'}) + '\n')
    real_open = builtins.open

    def guarded(path, *a, **k):
        if 'eval_only' in str(path):
            raise AssertionError('localization read eval_only')
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
    assert probe.replay(ep, {'s': sink}) == 1
    assert sink.seen[:2] == [('cmd', 0.), ('profile', 'fine')] or sink.seen[:2] == [('profile', 'fine'), ('cmd', 0.)]
    assert sink.seen[-1] == ('frame', .1, (probe.HEIGHT, probe.WIDTH, 3))


def test_probe_module_has_no_simulator_imports():
    code = (PROBE_DIR/'markerless_probe.py').read_text()
    for banned in ('import mujoco', 'from mujoco', 'sim.zone_scene', 'sim.zone_arena', "'eval_only'", '"eval_only"',
                   'frames_eval', 'gt_trajectory', 'xpos', 'qpos'):
        assert banned not in code, banned
    out = subprocess.run([sys.executable, '-c', (
        'import sys, builtins\n'
        'real = builtins.__import__\n'
        'def block(name, *a, **k):\n'
        '    if name == "mujoco" or name.startswith("mujoco."):\n'
        '        raise ImportError("blocked")\n'
        '    return real(name, *a, **k)\n'
        'builtins.__import__ = block\n'
        f'sys.path.insert(0, {str(PROBE_DIR)!r}); sys.path.insert(0, {str(ROOT)!r})\n'
        'import markerless_probe\n'
        'print("ok")\n')], capture_output=True, text=True, cwd=ROOT)
    assert out.returncode == 0 and 'ok' in out.stdout, out.stderr
