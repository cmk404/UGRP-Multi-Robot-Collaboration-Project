"""TOP cargo detection profile top_cargo_v2 (harness/zone_cargo_perception_v2.py)."""
import hashlib
import math
from pathlib import Path

import pytest

from harness import zone_cargo_perception as v1
from harness import zone_cargo_perception_v2 as v2
from tests.test_zone_cargo_perception import CAM, STATIC, SW, _beam, _floor, _frame, _jpeg, _rect, _yaw_err

ROOT = Path(__file__).resolve().parents[1]
# harness/zone_cargo_perception.py as merged in PR #168 (44b8132). top_cargo_v1
# results are only valid for these bytes; a change needs a new profile.
V1_SHA256 = '57335d615209f68658cb272360735404b2321c48dbd03b7eae27045733794290'


def test_v1_module_is_byte_identical_to_the_evaluated_version():
    assert hashlib.sha256((ROOT/'harness/zone_cargo_perception.py').read_bytes()).hexdigest() == V1_SHA256
    assert v1.PROFILE == 'top_cargo_v1' and v2.PROFILE == 'top_cargo_v2'


def test_v2_module_is_outside_the_bundle_source_closure():
    from harness.rgb_execution_bundle import source_closure
    assert 'harness/zone_cargo_perception_v2.py' not in source_closure()


def _pair(sep, yaw=.4, along=.05, centre=(1.0, -1.6)):
    n = (-math.sin(yaw), math.cos(yaw))
    a = (centre[0], centre[1], yaw)
    b = (centre[0]+n[0]*sep+along*math.cos(yaw), centre[1]+n[1]*sep+along*math.sin(yaw), yaw)
    return a, b


def _match(rows, truth, tol):
    rows = [r for r in rows if r['kind'] == 'long_beam']
    assert len(rows) == len(truth), rows
    for t in truth:
        best = min(rows, key=lambda r: math.dist(r['floor_xy_m'], t[:2]))
        assert math.dist(best['floor_xy_m'], t[:2]) < tol, (t, best)
        assert _yaw_err(best['yaw_rad'], t[2], 180) < 3


@pytest.mark.parametrize('sep', [.045, .05, .06, .08, .12])
def test_parallel_beams_in_one_view_stay_two_items(sep):
    a, b = _pair(sep)
    f = _floor()
    _beam(f, SW, a)
    _beam(f, SW, b)
    rows = v2.detect_cargo_top(_jpeg(f), SW)
    _match(rows, [a, b], .01)
    if sep < .055:
        assert all(r['evidence']['split_from_wider_blob'] for r in rows if r['kind'] == 'long_beam')


def test_v1_loses_fused_parallel_beams_documenting_the_fix():
    a, b = _pair(.045)
    f = _floor()
    _beam(f, SW, a)
    _beam(f, SW, b)
    assert len([r for r in v1.detect_cargo_top(_jpeg(f), SW) if r['kind'] == 'long_beam']) < 2


def _seam_tops(poses):
    tops = {}
    for c in STATIC['top_cameras']:
        f = _floor()
        if c['name'] in ('cctv_top', 'cctv_top_north'):
            for p in poses:
                _beam(f, c, p)
        tops[c['name']] = _jpeg(f)
    return tops


@pytest.mark.parametrize('sep', [.045, .06, .09])
def test_parallel_beams_across_a_seam_are_paired_by_line_not_united(sep):
    nw = CAM['cctv_top_north']
    seam = (SW['position_m'][1]+nw['position_m'][1])/2
    a, b = _pair(sep, yaw=math.pi/2-.15, along=0., centre=(.4, seam))
    items = [i for i in v2.detect_all_cargo(_seam_tops([a, b]), STATIC)['items'] if i['kind'] == 'long_beam']
    # 30 mm: one view may include a 12 mm end cap the other misses (known v1 bias).
    _match(items, [a, b], .03)
    for i in items:
        assert i['evidence']['merged_views'] == 2
        assert i['evidence']['visible_length_m'] <= v2.MERGE_MAX_LENGTH_M


def test_single_beam_across_a_seam_is_still_united():
    nw = CAM['cctv_top_north']
    seam = (SW['position_m'][1]+nw['position_m'][1])/2
    pose = (.4, seam+.02, math.pi/2-.2)
    items = [i for i in v2.detect_all_cargo(_seam_tops([pose]), STATIC)['items'] if i['kind'] == 'long_beam']
    assert len(items) == 1 and math.dist(items[0]['floor_xy_m'], pose[:2]) < .03


def _piece(camera, seg, clipped, length):
    return {'kind': 'long_beam', 'camera': camera, 'segment_floor_m': seg, 'segment_end_clipped': clipped,
            'evidence': {'visible_length_m': length}, 'confidence': 1.}


def test_merge_rules_same_camera_length_and_visible_ends():
    a = _piece('cam_a', [[0., 0.], [.35, 0.]], [False, True], .35)
    # same camera: never
    assert not v2._compatible([a], _piece('cam_a', [[.2, .01], [.55, .01]], [True, False], .35))
    # collinear, overlapping, other camera, union .55 m: yes
    assert v2._compatible([a], _piece('cam_b', [[.2, .005], [.55, .005]], [True, False], .35))
    # 4.5 cm apart laterally: no (v1 accepted up to 5 cm)
    assert not v2._compatible([a], _piece('cam_b', [[.2, .045], [.55, .045]], [True, False], .35))
    # union longer than the static beam: no
    assert not v2._compatible([a], _piece('cam_b', [[.3, 0.], [.70, 0.]], [True, False], .40))
    # runs past a's visible (unclipped) end at t=0: no
    assert not v2._compatible([a], _piece('cam_b', [[-.10, 0.], [.2, 0.]], [False, True], .30))
    # disjoint along the axis (views overlap, so one beam's pieces must too): no
    assert not v2._compatible([a], _piece('cam_b', [[.42, 0.], [.58, 0.]], [True, False], .16))


def test_same_camera_crates_are_never_deduplicated():
    f = _floor()
    from tests.test_zone_cargo_perception import _crate
    _crate(f, SW, (.30, -1.6, 0.))
    _crate(f, SW, (.30, -1.52, 0.))       # 8 cm apart on the short axis (< v1 dedupe 10 cm)
    tops = {c['name']: _jpeg(f if c['name'] == 'cctv_top' else _floor()) for c in STATIC['top_cameras']}
    crates = [i for i in v2.detect_all_cargo(tops, STATIC)['items'] if i['kind'] == 'heavy_crate']
    per_cam = [r for r in v2.detect_cargo_top(tops['cctv_top'], SW) if r['kind'] == 'heavy_crate']
    assert len(crates) == len(per_cam)


# ------------------------------------------------------------ boxes and the frame

def _frame_scene():
    f = _floor()
    pose = (.3, -1.8, .3)
    _frame(f, SW, pose)
    return f, pose


def test_box_inside_the_frame_is_kept_box_on_a_bar_is_suppressed():
    f, pose = _frame_scene()
    _rect(f, SW, (pose[0], pose[1], .2), (.017, .02), .032, 'red')          # open interior
    # a yellow highlight on bar 0 (between vertices 0 and 1), centred on the bar
    r = .2
    v0 = (r*math.cos(pose[2]), r*math.sin(pose[2]))
    v1_ = (r*math.cos(pose[2]+2*math.pi/3), r*math.sin(pose[2]+2*math.pi/3))
    mid = (pose[0]+(v0[0]+v1_[0])/2, pose[1]+(v0[1]+v1_[1])/2)
    _rect(f, SW, (mid[0], mid[1], pose[2]+math.radians(150)), (.012, .012), .032, 'yellow')
    jpeg = _jpeg(f)
    got2 = [(r['kind'], r.get('colour')) for r in v2.detect_cargo_top(jpeg, SW)]
    got1 = [(r['kind'], r.get('colour')) for r in v1.detect_cargo_top(jpeg, SW)]
    assert ('box', 'red') in got2 and ('box', 'yellow') not in got2 and ('tri_frame', 'cream') in got2
    assert ('box', 'red') not in got1        # the v1 defect this profile fixes


def test_box_touching_the_outside_of_a_bar_is_kept():
    f, pose = _frame_scene()
    # outward normal of bar 0 at its midpoint, box centre 4 cm from the bar centre-line
    r, a0, a1 = .2, pose[2], pose[2]+2*math.pi/3
    mid = ((math.cos(a0)+math.cos(a1))*r/2, (math.sin(a0)+math.sin(a1))*r/2)
    k = .1 + .04
    out = (pose[0]+mid[0]/.1*k, pose[1]+mid[1]/.1*k)
    _rect(f, SW, (out[0], out[1], 0.), (.017, .02), .032, 'red')
    got = [(r['kind'], r.get('colour')) for r in v2.detect_cargo_top(_jpeg(f), SW)]
    assert ('box', 'red') in got


def test_v2_matches_v1_where_v1_was_not_at_fault():
    f = _floor()
    from tests.test_zone_cargo_perception import _crate, _disc
    _crate(f, SW, (.2, -2.4, .7))
    _beam(f, SW, (1.0, -1.4, .7))
    _rect(f, SW, (.8, -2.3, .7), (.03, .02), .012, 'tile')
    _disc(f, SW, (1.2, -2.3), .019, .05, 'can')
    _rect(f, SW, (.5, -1.9, .1), (.017, .02), .032, 'red')
    jpeg = _jpeg(f)
    strip = lambda rows: sorted((r['kind'], r.get('colour'), tuple(r['floor_xy_m']), r['yaw_rad']) for r in rows)
    assert strip(v1.detect_cargo_top(jpeg, SW)) == strip(v2.detect_cargo_top(jpeg, SW))
