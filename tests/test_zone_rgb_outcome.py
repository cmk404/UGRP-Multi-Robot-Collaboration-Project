"""RGB-only zone job outcome check (harness/zone_rgb_outcome.py).

Small drawn TOP images (no SIM): a grey floor, 12x12 px coloured box tops at
calibrated floor positions, dark discs as robots. Checks each outcome rule,
the safety rules against false "delivered", the re-check policy and that the
module reads no simulator/teacher state.
"""
from __future__ import annotations

import inspect
import io
import math
import tokenize
from pathlib import Path

import cv2
import numpy as np
import pytest

from harness import zone_perception as zp
from harness import zone_rgb_outcome as zro

ROOT = Path(__file__).resolve().parents[1]
W, H = 960, 720
CAM = {'name': 'cctv_top', 'position_m': [1.5, 0., 2.5], 'quaternion_wxyz': [1, 0, 0, 0], 'fov_y_deg': 55.}
STATIC = {
    'top_cameras': [CAM],
    'box_kinds': ['cyan', 'green', 'red', 'yellow'],
    'regions': {'pickup': {'center_m': [.5, 0.], 'half_extents_m': [.6, .8]},
                'zone_A': {'center_m': [2.6, 0.], 'half_extents_m': [.3, .7]}},
    'zone_slots': {'A': [{'slot_id': 'A1', 'center_m': [2.6, -.4], 'half_extents_m': [.06, .06]},
                         {'slot_id': 'A2', 'center_m': [2.6, 0.], 'half_extents_m': [.06, .06]},
                         {'slot_id': 'A3', 'center_m': [2.6, .4], 'half_extents_m': [.06, .06]}]},
}
BGR = {'red': (0, 0, 210), 'yellow': (0, 210, 230), 'green': (40, 200, 40), 'cyan': (210, 200, 0)}
SOURCE = [.4, .2]
TARGET = zro.slot_target(STATIC, 'A2')
JOB = zro.job_spec(robot_ids='r1', item='red-1', kind='red', source_xy_m=SOURCE, zone='A', target=TARGET)


def top(boxes=(), robots=()):
    img = np.full((H, W, 3), (95, 105, 110), np.uint8)
    # a little floor texture so the ring check sees a real floor
    img[::40, :] = (90, 100, 104)
    for kind, (x, y) in boxes:
        u, v, _ = zro.floor_to_pixel(x, y, CAM, img.shape, height=zp.BOX_TOP_Z_M)
        cv2.rectangle(img, (int(u)-6, int(v)-6), (int(u)+5, int(v)+5), BGR[kind], -1)
    for x, y in robots:
        u, v, s = zro.floor_to_pixel(x, y, CAM, img.shape)
        cv2.circle(img, (int(u), int(v)), int(.17*s), (25, 25, 25), -1)
    ok, enc = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 95])
    assert ok
    return {'cctv_top': enc.tobytes()}


BEFORE = top([('red', SOURCE), ('red', (.4, -.6)), ('yellow', (.8, .2))])


def run(after, job=JOB, **kw):
    return zro.job_outcome(job, BEFORE, after, STATIC, before_names={'cctv_top': 'b.jpg'},
                           after_names={'cctv_top': 'a.jpg'}, **kw)


def test_floor_to_pixel_inverts_the_authored_calibration():
    for x, y in ((.4, .2), (2.6, -.4), (1.5, 0.)):
        u, v, _ = zro.floor_to_pixel(x, y, CAM, (H, W), height=zp.BOX_TOP_Z_M)
        fx, fy = zp.pixel_to_floor(u, v, CAM, (H, W))
        assert math.isclose(fx, x, abs_tol=1e-9) and math.isclose(fy, y, abs_tol=1e-9)


def test_drawn_boxes_are_detected_at_their_floor_position():
    rows = zro.sightings(BEFORE, STATIC, ['red', 'yellow'])
    assert sorted(r['kind'] for r in rows) == ['red', 'red', 'yellow']
    red = min((r for r in rows if r['kind'] == 'red'), key=lambda r: math.dist(r['floor_xy_m'], SOURCE))
    assert math.dist(red['floor_xy_m'], SOURCE) < .01


def test_delivered_needs_a_new_item_in_the_own_area_and_a_visibly_empty_source():
    res = run(top([('red', (2.61, .01)), ('red', (.4, -.6)), ('yellow', (.8, .2))], robots=[(2.1, 0.)]))
    assert res['outcome'] == 'delivered' and res['rule'] == 'delivered_source_empty'
    assert res['confidence'] >= zro.COMMIT_CONFIDENCE
    assert res['evidence']['images'] == ['a.jpg', 'b.jpg']
    assert res['evidence']['source']['visibility']['visible']


def test_still_at_source_tracked_and_pushed():
    same = run(BEFORE)
    assert same['outcome'] == 'still_at_source' and same['rule'] == 'still_at_source_tracked'
    pushed = run(top([('red', (.49, .2)), ('red', (.4, -.6)), ('yellow', (.8, .2))]))
    assert pushed['outcome'] == 'still_at_source' and pushed['rule'] == 'still_at_source_near'
    assert 'source_item_moved_but_near' in pushed['flags']


def test_dropped_on_the_way_is_seen_elsewhere():
    res = run(top([('red', (1.5, .1)), ('red', (.4, -.6)), ('yellow', (.8, .2))], robots=[(1.05, .1)]))
    assert res['outcome'] == 'seen_elsewhere'
    assert res['evidence']['new_elsewhere'] and math.dist(res['evidence']['new_elsewhere'][0], (1.5, .1)) < .02


def test_lost_item_is_not_seen():
    res = run(top([('red', (.4, -.6)), ('yellow', (.8, .2))]))
    assert res['outcome'] == 'not_seen' and 'source_empty_item_not_found' in res['flags']


def test_robot_over_the_source_is_occlusion_not_absence():
    res = run(top([('red', (.4, -.6)), ('yellow', (.8, .2))], robots=[(.38, .2)]))
    assert res['outcome'] == 'not_seen' and 'source_occluded' in res['flags']
    assert not res['evidence']['source']['visibility']['visible']


def test_same_kind_box_in_own_area_with_hidden_source_is_never_committed():
    # Colour cannot tell two red boxes apart: a peer's red box in the own slot
    # while the own box is hidden under a robot must not become "delivered".
    res = run(top([('red', (2.6, 0.)), ('red', (.4, -.6)), ('yellow', (.8, .2))], robots=[(.38, .2)]))
    assert res['rule'] == 'delivered_source_occluded'
    assert res['confidence'] < zro.COMMIT_CONFIDENCE
    _, committed = zro.commit([res])
    assert committed['confidence'] < zro.COMMIT_CONFIDENCE


def test_wrong_kind_in_the_target_is_not_delivered():
    res = run(top([('yellow', (2.6, 0.)), ('red', (.4, -.6))]))
    assert res['outcome'] != 'delivered'


def test_same_kind_in_the_zone_with_hidden_target_is_not_evidence():
    after = top([('red', (2.6, .4)), ('red', (.4, -.6)), ('yellow', (.8, .2))], robots=[(2.55, 0.)])
    res = run(after)
    assert res['outcome'] == 'not_seen'
    assert 'new_same_kind_in_zone_while_target_hidden' in res['flags']


def test_zone_only_job_uses_the_zone_count_with_lower_confidence():
    job = zro.job_spec(robot_ids=['r1', 'r2'], item='red-1', kind='red', source_xy_m=SOURCE, zone='A')
    res = run(top([('red', (2.5, .3)), ('red', (.4, -.6)), ('yellow', (.8, .2))]), job=job)
    assert res['outcome'] == 'delivered' and res['rule'] == 'delivered_zone_count'
    assert res['confidence'] == zro.CONFIDENCE['delivered_zone_count']
    assert res['job']['robot_ids'] == ['r1', 'r2']


def test_commit_takes_the_first_confident_decision():
    ns = {'outcome': 'not_seen', 'confidence': 0.}
    low = {'outcome': 'delivered', 'confidence': .5}
    hit = {'outcome': 'still_at_source', 'confidence': .95}
    assert zro.commit([ns, low, hit, ns]) == (2, hit)
    assert zro.commit([ns, low]) == (1, low)


def test_receipt_replaces_teacher_text():
    res = run(BEFORE)
    rc = zro.receipt(res)
    assert rc['status'] == zro.RECEIPT_TEXT['still_at_source']
    assert 'finished' not in rc['status'] and 'executor' not in rc['status']


def test_module_reads_no_simulator_or_teacher_state():
    src = (ROOT/'harness'/'zone_rgb_outcome.py').read_text()
    # Names and calls only: drop comments and every string literal.
    code = ' '.join(t.string for t in tokenize.generate_tokens(io.StringIO(src).readline)
                    if t.type not in (tokenize.COMMENT, tokenize.STRING))
    for token in ('mujoco', 'qpos', 'xpos', 'setup_only', 'body (', 'placed_by_teacher', 'grasp_failed',
                  'dropped_in_transit', 'teacher', 'inject', 'contact', 'referee'):
        assert token not in code, token
    params = set(inspect.signature(zro.job_outcome).parameters)
    assert params == {'job', 'before_tops', 'after_tops', 'static_map', 'own_rgb', 'own_servo_pose',
                      'before_names', 'after_names', 'own_name', 'profile', 'before_rows'}


def test_static_map_given_has_no_item_poses():
    static = __import__('json').loads((ROOT/'maps'/'zones'/'zone_wide.json').read_text())
    assert 'objects' not in static and 'setup_only' not in static


def test_new_modules_are_outside_the_bundle_source_closure():
    from harness.rgb_execution_bundle import source_closure
    closure = source_closure()
    assert 'harness/zone_rgb_outcome.py' not in closure
    assert 'scripts/eval_zone_rgb_outcome.py' not in closure


def test_catalogue_kinds_need_the_cargo_detector():
    job = zro.job_spec(robot_ids=['r1', 'r2'], item='beam-1', kind='long_beam', source_xy_m=SOURCE, zone='A')
    if zro._zcp is None:
        with pytest.raises(RuntimeError, match='zone_cargo_perception'):
            run(BEFORE, job=job)
    else:
        assert run(BEFORE, job=job)['outcome'] in zro.OUTCOMES
