"""RGB-only zone job outcome check (harness/zone_rgb_outcome.py).

Small drawn TOP images (no SIM): a grey floor, 12x12 px coloured box tops at
calibrated floor positions, dark discs as robots. Checks each outcome rule,
the source-emptiness proof (unchanged is not unoccluded), the decision policy
(explicit ``unconfirmed``, two stable ticks, gripper gate, teacher-free
deadline, command-evidenced releases), cargo footprint/yaw checks, and that the
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
REFERENCE = BEFORE
NAMES = {'cctv_top': 'a.jpg'}


def run(after, job=JOB, reference=None, before=None, **kw):
    return zro.observe(job, reference or REFERENCE, before or BEFORE, after, STATIC, names=NAMES, **kw)


def track(frames, job=JOB, reference=None, before=None, commands=None, assigned_at=0., deadline_s=zro.DEADLINE_S):
    tr = zro.JobTracker(job, reference or REFERENCE, before or BEFORE, STATIC, assigned_at=assigned_at,
                        deadline_s=deadline_s)
    d = None
    for i, f in enumerate(frames):
        d = tr.update(assigned_at + (i+1)*zro.CADENCE_S, f, commands=commands)
    return d


def cmd(t, pulse, rid='r1'):
    return {'robot_id': rid, 'sim_time_s': t, 'kind': 'arm', 'servo_id': 1, 'pulse': pulse}


DELIVERED = top([('red', (2.61, .01)), ('red', (.4, -.6)), ('yellow', (.8, .2))], robots=[(2.1, 0.)])


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


def test_delivered_observation_needs_a_proven_empty_source():
    res = run(DELIVERED)
    assert res['outcome'] == 'delivered' and res['rule'] == 'delivered_source_proven'
    view = res['evidence']['source']['state']['views'][0]
    assert view['item_seen_here_in_reference'] and view['unchanged'] and view['centre_bare'] >= .7


def test_still_at_source_tracked_and_pushed():
    assert run(BEFORE)['rule'] == 'still_at_source_tracked'
    pushed = run(top([('red', (.49, .2)), ('red', (.4, -.6)), ('yellow', (.8, .2))]))
    assert pushed['rule'] == 'still_at_source_near' and 'source_item_moved_but_near' in pushed['flags']


def test_dropped_on_the_way_is_seen_elsewhere():
    res = run(top([('red', (1.5, .1)), ('red', (.4, -.6)), ('yellow', (.8, .2))], robots=[(1.05, .1)]))
    assert res['outcome'] == 'seen_elsewhere'


def test_robot_over_the_source_is_not_proof_of_absence():
    res = run(top([('red', (.4, -.6)), ('yellow', (.8, .2))], robots=[(.38, .2)]))
    assert res['outcome'] == 'not_seen' and 'source_emptiness_unproven' in res['flags']


def test_static_occluder_since_assignment_is_not_proof():
    # Review item 2: a robot already over the source at assignment and still
    # there now leaves the ring unchanged; a peer's red box in the own slot must
    # not make this "delivered".
    occluded = top([('red', (.4, -.6)), ('yellow', (.8, .2))], robots=[(.4, .2)])
    now = top([('red', (2.6, 0.)), ('red', (.4, -.6)), ('yellow', (.8, .2))], robots=[(.4, .2)])
    res = run(now, before=occluded)
    assert res['rule'] == 'delivered_source_unproven'
    # v1 compared with the assignment frame (unchanged -> "visible"); v2 compares
    # with the label frame and needs the item seen there, so this is unproven.
    assert not res['evidence']['source']['state']['proven_empty']
    old_style = zro.point_state(occluded, now, STATIC, SOURCE)
    assert old_style['unchanged'] and not old_style['proven_empty']
    d = track([now, now, now], before=occluded, deadline_s=1.5)
    assert d['status'] == 'confirmed' and d['outcome'] != 'delivered'


def test_static_occluder_since_reference_is_not_proof():
    # The occluder is in the label frame too: the reference ring is not floor
    # and the item was never seen there.
    occluded = top([('red', (.4, -.6)), ('yellow', (.8, .2))], robots=[(.4, .2)])
    now = top([('red', (2.6, 0.)), ('red', (.4, -.6)), ('yellow', (.8, .2))], robots=[(.4, .2)])
    res = run(now, reference=occluded, before=occluded)
    assert res['outcome'] != 'delivered' or res['rule'] == 'delivered_source_unproven'
    assert not res['evidence']['source']['state']['proven_empty']


def test_occluder_on_the_ring_only_since_reference_is_not_proof():
    ringed = top([('red', SOURCE), ('red', (.4, -.6))], robots=[(.4, .36)])
    now = top([('red', (2.6, 0.)), ('red', (.4, -.6))], robots=[(.4, .36)])
    res = run(now, reference=ringed, before=ringed)
    assert not res['evidence']['source']['state']['proven_empty']


def test_wrong_kind_in_the_target_is_not_delivered():
    assert run(top([('yellow', (2.6, 0.)), ('red', (.4, -.6))]))['outcome'] != 'delivered'


def test_same_kind_in_the_zone_with_changed_target_is_not_evidence():
    res = run(top([('red', (2.6, .4)), ('red', (.4, -.6)), ('yellow', (.8, .2))], robots=[(2.55, 0.)]))
    assert res['outcome'] == 'not_seen' and 'new_same_kind_in_zone_while_target_changed' in res['flags']


def test_zone_only_job_uses_the_zone_count():
    job = zro.job_spec(robot_ids=['r1', 'r2'], item='red-1', kind='red', source_xy_m=SOURCE, zone='A')
    res = run(top([('red', (2.5, .3)), ('red', (.4, -.6)), ('yellow', (.8, .2))]), job=job)
    assert res['rule'] == 'delivered_zone_count'


# ---------------------------------------------------------------- decisions

def test_one_tick_is_unconfirmed_and_receipt_never_claims_success():
    d = track([DELIVERED])
    assert d['status'] == 'unconfirmed' and d['outcome'] == zro.UNCONFIRMED
    rc = zro.receipt(d)
    assert rc['status'] == zro.RECEIPT_TEXT[zro.UNCONFIRMED] and not rc['confirmed'] and rc['rgb_outcome'] == 'unconfirmed'


def test_two_stable_ticks_confirm_delivered():
    d = track([DELIVERED, DELIVERED])
    assert d['status'] == 'confirmed' and d['outcome'] == 'delivered'
    assert zro.receipt(d)['status'] == zro.RECEIPT_TEXT['delivered']


def test_low_confidence_delivered_is_never_confirmed():
    now = top([('red', (2.6, 0.)), ('red', (.4, -.6)), ('yellow', (.8, .2))], robots=[(.38, .2)])
    d = track([now]*4, deadline_s=2.5)
    assert d['outcome'] != 'delivered'
    assert zro.receipt(d)['status'] != zro.RECEIPT_TEXT['delivered']


def test_closed_gripper_blocks_delivered():
    commands = {'r1': [cmd(0.1, 2000), cmd(0.2, 1500)]}
    d = track([DELIVERED, DELIVERED], commands=commands)
    assert d['status'] == 'unconfirmed' and 'target_item_but_carrier_gripper_closed' in d['flags']
    commands = {'r1': [cmd(0.1, 2000), cmd(0.2, 1500), cmd(0.5, 2000)]}
    assert track([DELIVERED, DELIVERED], commands=commands)['outcome'] == 'delivered'


def test_non_delivered_waits_for_the_teacher_free_deadline():
    assert track([BEFORE]*3)['status'] == 'unconfirmed'
    d = track([BEFORE]*3, deadline_s=2.5)
    assert d['status'] == 'confirmed' and d['outcome'] == 'still_at_source' and d['rule'].startswith('deadline_')


def test_deadline_without_evidence_confirms_not_seen():
    lost = top([('red', (.4, -.6)), ('yellow', (.8, .2))], robots=[(.38, .2)])
    d = track([lost]*3, deadline_s=2.5)
    assert d['outcome'] == 'not_seen' and d['rule'] == 'deadline_not_seen'


def test_three_command_evidenced_releases_at_the_source_confirm_early():
    commands = {'r1': [cmd(.05, 2000), cmd(.1, 1500), cmd(.3, 2000), cmd(1.1, 1500), cmd(1.3, 2000),
                       cmd(2.1, 1500), cmd(2.3, 2000)]}
    d = track([BEFORE]*5, commands=commands)
    assert d['status'] == 'confirmed' and d['rule'] == 'released_at_source'


def test_issued_arm_and_releases_follow_the_command_log():
    log = [{'robot_id': 'r1', 'sim_time_s': 0., 'kind': 'initial', 'pulses': {'1': 2000, '3': 740}},
           cmd(1., 1500), {'robot_id': 'r1', 'sim_time_s': 1.5, 'kind': 'look', 'pan_pulse': 1400}, cmd(2., 2000)]
    assert zro.issued_arm(log, 1.6) == {1: 1500, 3: 740, 6: 1400}
    assert zro.gripper_open(log, .5) is True and zro.gripper_open(log, 1.2) is False
    assert zro.releases(log, 0., 3.) == [2.]
    assert zro.gripper_open([], 1.) is None


# ---------------------------------------------------------------- cargo

def _cargo_job(yaw=0.):
    area = {'landing_center_m': [2.6, 0.], 'landing_half_extents_m': [.36, .06], 'item_pose': [2.6, 0., yaw]}
    return zro.job_spec(robot_ids=['r1', 'r2'], item='beam-1', kind='long_beam', source_xy_m=SOURCE, zone='A',
                        target=zro.landing_target(area))


@pytest.mark.parametrize('row,ok', [
    ({'floor_xy_m': [2.6, 0.], 'yaw_rad': 0., 'clipped': False}, True),
    ({'floor_xy_m': [2.6, 0.], 'yaw_rad': math.pi, 'clipped': False}, True),     # 180 deg symmetric
    ({'floor_xy_m': [2.6, 0.], 'yaw_rad': .5, 'clipped': False}, False),         # wrong yaw
    ({'floor_xy_m': [2.75, 0.], 'yaw_rad': 0., 'clipped': False}, False),        # footprint leaves the area
    ({'floor_xy_m': [2.6, 0.], 'yaw_rad': 0., 'clipped': True}, False),          # clipped view
    ({'floor_xy_m': [2.6, 0.], 'yaw_rad': None, 'clipped': False}, False),       # yaw unknown
])
def test_cargo_target_checks_full_footprint_and_yaw(row, ok):
    assert zro._in_target({'kind': 'long_beam', **row}, _cargo_job()) is ok


def test_cargo_needs_known_open_grippers_to_confirm(monkeypatch):
    job = _cargo_job()
    obs = {'outcome': 'delivered', 'confidence': .95, 'rule': 'delivered_source_proven', 'flags': [],
           'target_sighting': {'floor_xy_m': [2.6, 0.], 'yaw_rad': 0.}, 'images': []}
    hist = [dict(obs, t=1.), dict(obs, t=2.)]
    d = zro.decide(job, hist, assigned_at=0., now=2.)
    assert d['status'] == 'unconfirmed' and 'cargo_lifted_state_unknown' in d['flags']
    lifted = {'r1': [cmd(.5, 1500)], 'r2': [cmd(.5, 1500, 'r2')]}
    assert zro.decide(job, hist, assigned_at=0., now=2., commands=lifted)['status'] == 'unconfirmed'
    placed = {'r1': [cmd(.5, 1500), cmd(.8, 2000)], 'r2': [cmd(.5, 1500, 'r2'), cmd(.8, 2000, 'r2')]}
    assert zro.decide(job, hist, assigned_at=0., now=2., commands=placed)['outcome'] == 'delivered'


def test_catalogue_items_need_a_landing_area():
    with pytest.raises(ValueError):
        zro.job_spec(robot_ids=['r1', 'r2'], item='beam-1', kind='long_beam', source_xy_m=SOURCE, zone='A')


# ---------------------------------------------------------------- boundaries

def test_module_reads_no_simulator_or_teacher_state():
    src = (ROOT/'harness'/'zone_rgb_outcome.py').read_text()
    code = ' '.join(t.string for t in tokenize.generate_tokens(io.StringIO(src).readline)
                    if t.type not in (tokenize.COMMENT, tokenize.STRING))
    for token in ('mujoco', 'qpos', 'xpos', 'setup_only', 'body (', 'placed_by_teacher', 'grasp_failed',
                  'dropped_in_transit', 'teacher', 'inject', 'contact', 'referee', 'phase'):
        assert token not in code, token
    assert set(inspect.signature(zro.observe).parameters) == {
        'job', 'reference_tops', 'before_tops', 'current_tops', 'static_map', 'own_rgb', 'own_arm_pulses',
        'profile', 'reference_rows', 'before_rows', 'names'}
    assert set(inspect.signature(zro.JobTracker.update).parameters) == {'self', 't', 'current_tops', 'commands',
                                                                       'own_rgb', 'names'}


def test_eval_refuses_to_overwrite(tmp_path):
    from scripts import eval_zone_rgb_outcome as ev
    (tmp_path/'x').mkdir()
    with pytest.raises(SystemExit, match='refusing to overwrite'):
        ev.new_dir(tmp_path/'x')
    assert ev.new_dir(tmp_path/'y').is_dir()


def test_static_map_given_has_no_item_poses():
    static = __import__('json').loads((ROOT/'maps'/'zones'/'zone_wide.json').read_text())
    assert 'objects' not in static and 'setup_only' not in static


def test_new_modules_are_outside_the_bundle_source_closure():
    from harness.rgb_execution_bundle import source_closure
    closure = source_closure()
    assert 'harness/zone_rgb_outcome.py' not in closure
    assert 'scripts/eval_zone_rgb_outcome.py' not in closure
