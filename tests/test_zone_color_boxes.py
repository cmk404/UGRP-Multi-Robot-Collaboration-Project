"""Colour-generalised zone box detection (harness/zone_color_boxes.py)."""
import base64
import hashlib
import json
import math
from pathlib import Path

import cv2
import numpy as np
import pytest

from harness import markerless_box as mb
from harness import zone_color_boxes as z
from harness import zone_perception as zp
from harness.visual_arm import camera_extrinsics
from sim.masterpi_camera_profile import CAMERA_FISHEYE_D, scaled_camera_matrix

ROOT = Path(__file__).resolve().parents[1]
POSE = {3: 740, 4: 2320, 5: 1320, 6: 1500}     # folded arm (teacher/zone reset pose)
SIZE = (960, 720)                                # zone runner robot camera size
# Box paints as rendered (OpenCV HSV), measured on the dev split.
BOX_HSV = {'cyan': (93, 181, 200), 'red': (1, 223, 200), 'green': (63, 193, 180), 'yellow': (25, 227, 220)}
FLOOR_HSV = (108, 40, 70)


def _bgr(hsv, v_scale=1.):
    h, s, v = hsv
    px = np.uint8([[[h, s, int(min(255, v*v_scale))]]])
    return tuple(int(c) for c in cv2.cvtColor(px, cv2.COLOR_HSV2BGR)[0, 0])


def _floor():
    frame = np.zeros((SIZE[1], SIZE[0], 3), np.uint8)
    frame[:] = _bgr(FLOOR_HSV)
    return frame


def _project(points):
    origin, axes = camera_extrinsics(POSE)
    return mb._project_points(np.asarray(points, float), np.asarray(origin), np.asarray(axes),
                              scaled_camera_matrix(*SIZE), np.asarray(CAMERA_FISHEYE_D).reshape(4, 1))


def _draw_box(frame, hsv, center, yaw=0.):
    pixels = _project(mb._cuboid_corners(center, yaw, z.BOX_DIMS_M))
    assert pixels is not None
    cv2.fillConvexPoly(frame, cv2.convexHull(pixels.astype(np.float32)).astype(np.int32), _bgr(hsv, .7))
    top = pixels[4:][[0, 1, 3, 2]]
    cv2.fillConvexPoly(frame, np.rint(top).astype(np.int32), _bgr(hsv))
    return frame


def _jpeg(frame):
    ok, data = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
    assert ok
    return data.tobytes()


# ------------------------------------------------------------ regression: cyan path unchanged

def test_markerless_box_source_matches_the_runnable_bundle_pin():
    from harness.rgb_execution_bundle import REGISTRY, RUNNABLE_ID
    bundle = json.loads((ROOT/REGISTRY/f'{RUNNABLE_ID}.json').read_text())
    pinned = bundle['source_files_sha256']['harness/markerless_box.py']
    assert hashlib.sha256((ROOT/'harness/markerless_box.py').read_bytes()).hexdigest() == pinned


def test_new_module_is_outside_the_bundle_source_closure():
    from harness.rgb_execution_bundle import source_closure
    closure = source_closure()
    assert 'harness/zone_color_boxes.py' not in closure
    assert 'scripts/eval_zone_color_detection.py' not in closure


def test_production_profile_cyan_is_the_production_extractor():
    frame = _draw_box(_floor(), BOX_HSV['cyan'], (.32, -.03), .3)
    ours, clipped = z._colour_components(frame, 'cyan')
    theirs, their_clipped = mb._cyan_components(frame)
    assert clipped == their_clipped and len(ours) == len(theirs) == 1
    assert ours[0]['bbox'] == theirs[0]['bbox'] and ours[0]['area'] == theirs[0]['area']


def test_default_profiles_are_the_production_and_zc_baselines():
    assert z.detect_own.__kwdefaults__['profile'] == z.OWN_PROFILE_PRODUCTION
    assert z.detect_top.__kwdefaults__['profile'] == z.TOP_PROFILE_BASELINE
    camera, frame = _top_scene([('red', (.40, -2.0), 0.), ('green', (.60, -2.1), 0.)])
    assert z.detect_top(_jpeg(frame), camera) == zp.detect_boxes(_jpeg(frame), camera, z.KINDS)


def test_single_target_observer_and_multi_colour_detector_agree_on_cyan():
    frame = _draw_box(_floor(), BOX_HSV['cyan'], (.32, -.03), .3)
    single = mb.observe_ground_box(base64.b64encode(_jpeg(frame)).decode(), POSE)
    many = z.detect_own(_jpeg(frame), POSE, ('cyan',))['detections']
    assert single['visible'] and len(many) == 1
    assert math.dist(single['estimated_box_center_base_m'][:2], many[0]['estimated_box_center_base_m'][:2]) < .01


# ------------------------------------------------------------ own RGB

@pytest.mark.parametrize('profile', [z.OWN_PROFILE_PRODUCTION, z.OWN_PROFILE_ZONE])
@pytest.mark.parametrize('kind', z.KINDS)
def test_own_rgb_finds_each_kind_once_with_its_floor_position(kind, profile):
    frame = _draw_box(_floor(), BOX_HSV[kind], (.32, -.03), .4)
    dets = z.detect_own(_jpeg(frame), POSE, profile=profile)['detections']
    assert [d['kind'] for d in dets] == [kind]
    assert dets[0]['range_class'] == 'near'
    assert math.dist(dets[0]['estimated_box_center_base_m'][:2], (.32, -.03)) < .015


def test_own_rgb_reports_adjacent_boxes_of_different_colours_separately():
    frame = _floor()
    for kind, xy in (('red', (.34, -.05)), ('yellow', (.34, .005)), ('green', (.40, -.025))):
        _draw_box(frame, BOX_HSV[kind], xy, 0.)
    dets = z.detect_own(_jpeg(frame), POSE, profile=z.OWN_PROFILE_ZONE)['detections']
    assert sorted(d['kind'] for d in dets) == ['green', 'red', 'yellow']


def test_own_rgb_rejects_beam_orange_and_zone_floor_paint():
    frame = _floor()
    # Orange beam (H 14) as a long bar, zone A (H 12) and C (H 142) paint patches.
    bar = _project([[.25, -.12, 0], [.25, .12, 0], [.29, .12, .03], [.29, -.12, .03]])
    cv2.fillConvexPoly(frame, cv2.convexHull(bar.astype(np.float32)).astype(np.int32), _bgr((14, 230, 200)))
    cv2.rectangle(frame, (80, 60), (300, 200), _bgr((12, 177, 52)), -1)
    cv2.rectangle(frame, (620, 60), (880, 200), _bgr((142, 177, 84)), -1)
    for profile in (z.OWN_PROFILE_PRODUCTION, z.OWN_PROFILE_ZONE):
        assert z.detect_own(_jpeg(frame), POSE, profile=profile)['detections'] == []


def test_far_boxes_are_zone_profile_only_and_keep_their_bearing():
    target = (1.8, .35)
    frame = _draw_box(_floor(), BOX_HSV['green'], target, .2)
    assert z.detect_own(_jpeg(frame), POSE)['detections'] == []
    dets = z.detect_own(_jpeg(frame), POSE, profile=z.OWN_PROFILE_ZONE)['detections']
    assert len(dets) == 1 and dets[0]['range_class'] == 'far_coarse'
    est = dets[0]['estimated_box_center_base_m']
    assert abs(math.atan2(est[1], est[0])-math.atan2(target[1], target[0])) < math.radians(1.)


def test_unknown_profile_and_kind_are_rejected():
    with pytest.raises(ValueError):
        z.detect_own(_jpeg(_floor()), POSE, profile='nope')
    with pytest.raises(ValueError):
        z.detect_own(_jpeg(_floor()), POSE, ('purple',))
    with pytest.raises(ValueError):
        z.detect_top(_jpeg(_floor()), {}, profile='nope')


# ------------------------------------------------------------ TOP RGB

def _top_scene(boxes, extra=None):
    from sim.zone_arena import authored_map
    camera = authored_map('zone_wide')['top_cameras'][0]
    h, w = 720, 960
    frame = np.zeros((h, w, 3), np.uint8)
    frame[:] = _bgr((106, 104, 128))             # zone pickup paint
    cx, cy, cz = camera['position_m']
    scale = h/(2*(cz-zp.BOX_TOP_Z_M)*math.tan(math.radians(camera['fov_y_deg'])/2))
    for kind, (x, y), yaw in boxes:
        u, v = (x-cx)*scale+(w-1)/2, -(y-cy)*scale+(h-1)/2
        rect = ((u, v), (z.BOX_DIMS_M[0]*scale, z.BOX_DIMS_M[1]*scale), math.degrees(yaw))
        cv2.fillConvexPoly(frame, np.rint(cv2.boxPoints(rect)).astype(np.int32), _bgr(BOX_HSV[kind]))
    if extra is not None:
        extra(frame)
    return camera, frame


def test_top_zone_profile_finds_rotated_boxes_that_the_baseline_fill_gate_drops():
    boxes = [(k, (.1+.3*i, -2.4+.4*j), yaw) for i, k in enumerate(z.KINDS)
             for j, yaw in enumerate((0., math.radians(45)))]
    camera, frame = _top_scene(boxes)
    zone = z.detect_top(_jpeg(frame), camera, profile=z.TOP_PROFILE_ZONE)
    assert len(zone) == len(boxes)
    for kind, xy, _ in boxes:
        assert any(d['kind'] == kind and math.dist(d['floor_xy_m'], xy) < .01 for d in zone)
    base = z.detect_top(_jpeg(frame), camera)
    aligned = [xy for _, xy, yaw in boxes if yaw == 0.]
    assert all(any(math.dist(d['floor_xy_m'], xy) < .01 for d in base) for xy in aligned)
    assert len(base) < len(boxes)          # the bbox-fill gate drops some 45-degree boxes


def test_top_zone_profile_rejects_a_real_yellow_roller_and_keeps_a_real_yellow_box():
    # 40x40 crops of dev-split TOP frames (zone_wide seed 101, cctv_top). The roller
    # crop is detected by the looser dev variant (min area 30, rect fill 0.6).
    fixtures = ROOT/'tests/fixtures/zone_color'
    for name, expected in (('top-yellow-roller.png', 0), ('top-yellow-box.png', 1)):
        def paste(frame, crop=cv2.imread(str(fixtures/name))):
            frame[340:380, 460:500] = crop
        camera, frame = _top_scene([], paste)
        found = [d for d in z.detect_top(_jpeg(frame), camera, profile=z.TOP_PROFILE_ZONE) if d['kind'] == 'yellow']
        assert len(found) == expected, name


# ------------------------------------------------------------ evaluation protocol

def test_split_is_zone_wide_only_with_disjoint_dev_and_test_seeds():
    split = json.loads((ROOT/'experiments/2026-09-25-zone-rgb-color/split.json').read_text())
    scenes = {s['scene'] for v in split['splits'].values() for s in v}
    assert scenes == {'zones/zone_wide', 'dispatch/open'}
    dev = {(s['scene'], s['seed']) for s in split['splits']['dev']}
    test = {(s['scene'], s['seed']) for s in split['splits']['test']}
    assert dev and test and not dev & test
    assert {s['seed'] for s in split['splits']['dev']}.isdisjoint({s['seed'] for s in split['splits']['test']})


def test_scorer_matching_prefers_the_same_colour_and_grades_visibility():
    from scripts.eval_zone_color_detection import _match, _visibility
    rows = {'a': {'kind': 'red', 'seg_bbox_px': [10, 10, 20, 20], 'visible_px': 90},
            'b': {'kind': 'yellow', 'seg_bbox_px': [15, 15, 25, 25], 'visible_px': 90}}
    assert _match((18, 18), rows, 'yellow')[2] == 'b'
    assert _match((7, 7), rows, 'yellow')[:1] == (True,)         # colour confusion
    assert _match((60, 60), rows, 'red') is None
    row = {'visible_px': 100, 'partial_frame': False, 'visible_fraction': .5}
    assert _visibility(row, True) == 'occluded' and _visibility({**row, 'visible_fraction': .9}, True) == 'full'
    assert _visibility({**row, 'visible_px': 10}, True) == 'not_visible'
