"""Kind-aware TOP cargo detection (harness/zone_cargo_perception.py, profile top_cargo_v1)."""
import itertools
import json
import math
from pathlib import Path

import cv2
import numpy as np
import pytest

from harness import zone_cargo_perception as zc
from harness import zone_color_boxes as zcb
from harness import zone_perception as zp
from sim.zone_arena import authored_map

ROOT = Path(__file__).resolve().parents[1]
STATIC = authored_map('zone_wide')
CAM = {c['name']: c for c in STATIC['top_cameras']}
SW = CAM['cctv_top']
# Rendered colours (OpenCV HSV medians on dev TOP renders).
HSV = {'can': (133, 181, 167), 'tile': (162, 189, 172), 'heavy_crate': (170, 93, 193),
       'long_beam': (40, 200, 169), 'tri_frame': (23, 71, 191), 'dark': (0, 0, 30),
       'yellow': (25, 227, 220), 'red': (1, 223, 200), 'floor': (108, 40, 70), 'paint_C': (142, 161, 132)}


def _bgr(hsv):
    return tuple(int(c) for c in cv2.cvtColor(np.uint8([[hsv]]), cv2.COLOR_HSV2BGR)[0, 0])


def _floor():
    frame = np.zeros((720, 960, 3), np.uint8)
    frame[:] = _bgr(HSV['floor'])
    return frame


def _px(camera, xy, z):
    cx, cy, cz = camera['position_m']
    s = 720/(2*(cz-z)*math.tan(math.radians(camera['fov_y_deg'])/2))
    return ((xy[0]-cx)*s + 479.5, -(xy[1]-cy)*s + 359.5)


def _poly(camera, pose, local, z):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([_px(camera, (x+c*a-s*b, y+s*a+c*b), z) for a, b in local], np.float32)


def _rect(frame, camera, pose, half, z, colour, offset=(0., 0.)):
    ox, oy = offset
    pts = _poly(camera, pose, [(ox-half[0], oy-half[1]), (ox+half[0], oy-half[1]),
                               (ox+half[0], oy+half[1]), (ox-half[0], oy+half[1])], z)
    cv2.fillConvexPoly(frame, np.rint(pts).astype(np.int32), _bgr(HSV[colour]))


def _crate(frame, camera, pose):
    _rect(frame, camera, pose, (.03, .02), .046, 'dark', (-.09, 0.))
    _rect(frame, camera, pose, (.03, .02), .046, 'dark', (.09, 0.))
    _rect(frame, camera, pose, (.07, .05), .06, 'heavy_crate')


def _beam(frame, camera, pose):
    _rect(frame, camera, pose, (.30, .02), .032, 'long_beam')
    for x in (-.27, .27):
        _rect(frame, camera, pose, (.018, .0205), .032, 'dark', (x, 0.))


def _frame(frame, camera, pose):
    r = .2
    verts = [(r*math.cos(a), r*math.sin(a)) for a in (0., 2*math.pi/3, 4*math.pi/3)]
    for i in range(3):
        (x0, y0), (x1, y1) = verts[i], verts[(i+1) % 3]
        pts = _poly(camera, pose, [(x0, y0), (x1, y1)], .024)
        cv2.line(frame, tuple(np.rint(pts[0]).astype(int)), tuple(np.rint(pts[1]).astype(int)),
                 _bgr(HSV['tri_frame']), 8)
        lug = (1.1*x0, 1.1*y0)
        _rect(frame, camera, (pose[0]+math.cos(pose[2])*lug[0]-math.sin(pose[2])*lug[1],
                              pose[1]+math.sin(pose[2])*lug[0]+math.cos(pose[2])*lug[1],
                              pose[2]+math.atan2(y0, x0)), (.03, .02), .046, 'dark')


def _disc(frame, camera, xy, radius, z, colour):
    u, v = _px(camera, xy, z)
    s = _px(camera, (xy[0]+radius, xy[1]), z)[0]-u
    cv2.circle(frame, (int(round(u)), int(round(v))), int(round(s)), _bgr(HSV[colour]), -1)


def _jpeg(frame):
    ok, data = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
    assert ok
    return data.tobytes()


def _yaw_err(a, b, sym):
    p = math.radians(sym)
    d = (a-b) % p
    return math.degrees(min(d, p-d))


def _one(rows, kind):
    got = [r for r in rows if r['kind'] == kind]
    assert len(got) == 1, rows
    return got[0]


# ------------------------------------------------------------ regressions: existing profiles untouched

def test_existing_profile_tables_are_unchanged():
    assert zp.HSV_RANGES == {
        'cyan': [((80, 110, 60), (100, 255, 255))],
        'red': [((0, 150, 70), (6, 255, 255)), ((172, 150, 70), (179, 255, 255))],
        'green': [((45, 120, 50), (75, 255, 255))],
        'yellow': [((22, 150, 120), (32, 255, 255))]}
    assert (zp.MIN_AREA_PX, zp.MAX_AREA_PX, zp.MIN_FILL, zp.BOX_TOP_Z_M) == (30, 260, .55, .032)
    assert zcb.TOP_ZONE_HSV == {
        'cyan': (((84, 90, 40), (96, 255, 255)),),
        'red': (((0, 120, 50), (6, 255, 255)), ((172, 120, 50), (179, 255, 255))),
        'green': (((55, 100, 40), (72, 255, 255)),),
        'yellow': (((22, 140, 80), (33, 255, 255)),)}
    assert (zcb.TOP_ZONE_AREA_PX, zcb.TOP_ZONE_MIN_RECT_FILL, zcb.TOP_ZONE_MAX_ASPECT) == ((50, 300), .70, 2.2)


def test_existing_box_outputs_unchanged_and_new_profile_keeps_boxes_off_cargo():
    frame = _floor()
    boxes = [((.3, -1.6), 'yellow'), ((.9, -1.9), 'red')]
    for xy, colour in boxes:
        _rect(frame, SW, (*xy, .2), (.017, .020), .032, colour)
    jpeg = _jpeg(frame)
    before = (zp.detect_boxes(jpeg, SW, STATIC['box_kinds']),
              zcb.detect_top(jpeg, SW, zcb.KINDS, profile=zcb.TOP_PROFILE_ZONE))
    rows = zc.detect_cargo_top(jpeg, SW)
    # Same calls return the same bytes after the cargo detector ran.
    assert json.dumps(before) == json.dumps((zp.detect_boxes(jpeg, SW, STATIC['box_kinds']),
                                             zcb.detect_top(jpeg, SW, zcb.KINDS, profile=zcb.TOP_PROFILE_ZONE)))
    got = [r for r in rows if r['kind'] == 'box']
    ref = before[1]
    assert [(r['colour'], r['floor_xy_m']) for r in got] == [(r['kind'], r['floor_xy_m']) for r in ref]
    assert not [r for r in rows if r['kind'] != 'box']


def test_module_is_outside_the_bundle_source_closure():
    from harness.rgb_execution_bundle import source_closure
    closure = source_closure()
    assert 'harness/zone_cargo_perception.py' not in closure
    assert 'scripts/eval_zone_cargo_perception.py' not in closure


def test_geometry_constants_match_the_catalogue():
    from sim.zone_cargo import CATALOGUE, visual_spec
    can, tile, beam, crate, frame = (CATALOGUE[k] for k in zc.CARGO_KINDS)
    assert can.parts[0].size[0] == zc.CAN_RADIUS_M and 2*can.parts[0].size[1] == zc.TOP_Z_M['can']
    assert (2*tile.parts[0].size[0], 2*tile.parts[0].size[1]) == zc.TILE_DIMS_M
    assert 2*beam.parts[0].size[0] == pytest.approx(zc.BEAM_LENGTH_M)
    assert 2*beam.parts[0].size[1] == pytest.approx(zc.BEAM_WIDTH_M)
    bands = [p for p in beam.parts if p.role == 'marker']
    assert min(abs(p.center[0])-p.size[0] for p in bands) == pytest.approx(zc.BEAM_BAND_INNER_M)
    body = next(p for p in crate.parts if p.name == 'box')
    assert (2*body.size[0], 2*body.size[1]) == pytest.approx(zc.CRATE_BODY_M)
    assert next(p for p in crate.parts if p.name == 'lug_east').center[0] == pytest.approx(zc.CRATE_LUG_CENTRE_M)
    assert frame.extra['vertex_radius_m'] == zc.FRAME_VERTEX_RADIUS_M
    assert {k: CATALOGUE[k].colour for k in zc.CARGO_KINDS} == zc.COLOUR
    for k in zc.CARGO_KINDS:
        assert visual_spec(CATALOGUE[k])['colour'] == zc.COLOUR[k]


# ------------------------------------------------------------ kinds and poses (synthetic TOP frames)

@pytest.mark.parametrize('yaw', [0., .5, 1.3, -.9])
def test_crate_beam_frame_tile_can_kind_xy_yaw(yaw):
    frame = _floor()
    _crate(frame, SW, (.2, -2.4, yaw))
    _beam(frame, SW, (1.0, -1.4, yaw))
    _frame(frame, SW, (.0, -1.2, yaw))
    _rect(frame, SW, (.8, -2.3, yaw), (.03, .02), .012, 'tile')
    _disc(frame, SW, (1.2, -2.3), .019, .05, 'can')
    rows = zc.detect_cargo_top(_jpeg(frame), SW)
    assert sorted(r['kind'] for r in rows) == sorted(zc.CARGO_KINDS)
    truth = {'heavy_crate': (.2, -2.4), 'long_beam': (1.0, -1.4), 'tri_frame': (0., -1.2), 'tile': (.8, -2.3),
             'can': (1.2, -2.3)}
    for kind, xy in truth.items():
        r = _one(rows, kind)
        assert math.dist(r['floor_xy_m'], xy) < .012, (kind, r)
        assert 0 < r['confidence'] <= 1
        if kind == 'can':
            assert r['yaw_rad'] is None
        else:
            assert _yaw_err(r['yaw_rad'], yaw, r['yaw_symmetry_deg']) < (8 if kind == 'tile' else 3), (kind, r)
    assert _one(rows, 'heavy_crate')['evidence']['lug_dark_fraction'] > .5


def test_beam_split_by_an_occluder_is_merged_into_one_item():
    frame = _floor()
    _beam(frame, SW, (.6, -1.8, .4))
    # a dark robot-sized blob over the middle of the beam
    u, v = _px(SW, (.6, -1.8), .032)
    cv2.circle(frame, (int(u), int(v)), 22, _bgr(HSV['dark']), -1)
    rows = zc.detect_cargo_top(_jpeg(frame), SW)
    beam = _one(rows, 'long_beam')
    assert beam['evidence']['fragments'] >= 2
    assert beam['evidence']['centre_mode'] == 'both_ends'
    assert math.dist(beam['floor_xy_m'], (.6, -1.8)) < .015


def test_beam_across_two_tops_is_united_in_the_floor_frame():
    nw = CAM['cctv_top_north']
    # seam between cctv_top (south) and cctv_top_north lies at the cameras' mid y
    seam_y = (SW['position_m'][1]+nw['position_m'][1])/2
    pose = (.4, seam_y+.02, math.pi/2-.2)
    tops = {}
    for c in STATIC['top_cameras']:
        f = _floor()
        if c['name'] in ('cctv_top', 'cctv_top_north'):
            _beam(f, c, pose)
        tops[c['name']] = _jpeg(f)
    per_cam = [zc.detect_cargo_top(tops[n], CAM[n]) for n in ('cctv_top', 'cctv_top_north')]
    assert all(len([r for r in rows if r['kind'] == 'long_beam']) == 1 for rows in per_cam)
    items = zc.detect_all_cargo(tops, STATIC)['items']
    beam = _one(items, 'long_beam')
    assert math.dist(beam['floor_xy_m'], pose[:2]) < .03
    assert _yaw_err(beam['yaw_rad'], pose[2], 180) < 3


def test_can_on_zone_c_paint_is_found_and_paint_is_not():
    frame = _floor()
    c = STATIC['regions']['zone_C']
    se = CAM['cctv_top_east']
    (cx, cy), (hx, hy) = c['center_m'], c['half_extents_m']
    pts = np.array([_px(se, (cx+a*hx, cy+b*hy), 0.) for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1))], np.float32)
    cv2.fillConvexPoly(frame, np.rint(pts).astype(np.int32), _bgr(HSV['paint_C']))
    _disc(frame, se, (cx, cy+.1), .019, .05, 'can')
    rows = zc.detect_cargo_top(_jpeg(frame), se)
    assert [r['kind'] for r in rows] == ['can']
    assert math.dist(rows[0]['floor_xy_m'], (cx, cy+.1)) < .01


def test_yellow_blob_on_a_beam_is_not_reported_as_a_box():
    frame = _floor()
    pose = (1.0, -1.6, .3)
    _beam(frame, SW, pose)
    _rect(frame, SW, pose, (.02, .018), .032, 'yellow', (.12, 0.))   # yellowish highlight on the bar
    jpeg = _jpeg(frame)
    assert [d['kind'] for d in zcb.detect_top(jpeg, SW, zcb.KINDS, profile=zcb.TOP_PROFILE_ZONE)] == ['yellow']
    rows = zc.detect_cargo_top(jpeg, SW)
    assert [r['kind'] for r in rows] == ['long_beam']


# ------------------------------------------------------------ grasp handles from pose + static catalogue

@pytest.mark.parametrize('kind', ['long_beam', 'heavy_crate', 'tri_frame', 'tile', 'can'])
def test_grasp_handles_match_the_catalogue_up_to_symmetry(kind):
    from sim.zone_cargo import CargoInstance, world_grasps
    pose = (1.3, -.4, .7)
    truth = sorted(tuple(round(v, 4) for v in g['grip_xyz'][:2])
                   for g in world_grasps(CargoInstance('x', kind, pose)).values())
    sym = zc.YAW_SYMMETRY_DEG[kind]
    for extra in ((0.,) if sym is None else (0., math.radians(sym))):
        item = {'kind': kind, 'floor_xy_m': list(pose[:2]), 'yaw_rad': None if sym is None else pose[2]+extra}
        out = zc.grasp_handles(item)
        got = sorted(tuple(round(v, 4) for v in h['grip_xyz_m'][:2]) for h in out['handles'])
        assert len(got) == len(truth)
        for a, b in zip(got, truth):
            assert math.dist(a, b) < 1e-3
        if kind in ('long_beam', 'heavy_crate', 'tri_frame'):
            assert out['role_names_interchangeable_under_symmetry'] is True
            for h in out['handles']:
                bx, by, byaw = h['approach_base_xyyaw']
                gx, gy, _ = h['grip_xyz_m']
                assert math.hypot(gx-bx, gy-by) == pytest.approx(.155, abs=1e-3)
                assert math.atan2(gy-by, gx-bx) == pytest.approx(byaw, abs=1e-3) or \
                    abs(abs(math.atan2(gy-by, gx-bx)-byaw)-2*math.pi) < 1e-3
    assert zc.grasp_handles({'kind': 'box', 'floor_xy_m': [0, 0], 'yaw_rad': None})['handles'] == []


# ------------------------------------------------------------ evaluation bookkeeping

def test_split_is_fixed_and_disjoint():
    split = json.loads((ROOT/'experiments/2026-09-25-zone-cargo-perception/split.json').read_text())
    dev, test = set(split['splits']['dev']), set(split['splits']['test'])
    assert dev and test and not dev & test
    prior = json.loads((ROOT/'experiments/2026-09-25-zone-rgb-color/split.json').read_text())
    used = {s['seed'] for part in prior['splits'].values() for s in part}
    assert not (dev | test) & used


def test_eval_matching_prefers_same_class_and_scores_symmetric_yaw():
    from scripts import eval_zone_cargo_perception as ev
    assert ev._yaw_error(math.pi+.1, .1, 180.) == pytest.approx(0., abs=1e-9)
    assert ev._yaw_error(2*math.pi/3+.05, 0., 120.) == pytest.approx(math.degrees(.05))
    assert ev._yaw_error(None, 0., 180.) is None
    assert ev._cls('box', 'red') == 'box_red' and ev._cls('can') == 'can'
    for a, b in itertools.combinations(ev.CARGO_SET, 2):
        assert a[0] != b[0]
