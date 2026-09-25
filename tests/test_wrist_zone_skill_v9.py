"""wrist_zone_skill_v9: the two approach fixes from the v8 cohort (583 fit-model flip, 587 close-range fit loss)."""
import base64
import json
import math
import subprocess
from pathlib import Path

from harness import visual_box_skill as n7
from harness import wrist_zone_skill as v1
from harness import wrist_zone_skill_v2 as v2
from harness import wrist_zone_skill_v5 as v5
from harness import wrist_zone_skill_v8 as v8
from harness import wrist_zone_skill_v9 as v9
from harness.markerless_box import observe_ground_box

ROOT = Path(__file__).resolve().parents[1]
FIX = ROOT / 'tests' / 'fixtures' / 'wrist_zone_skill_v9'
FRAMES = {f['file']: f for f in json.loads((FIX / 'frames.json').read_text())['frames']}


def _pose(name):
    return {int(k): v for k, v in FRAMES[name]['own_servo_pwm'].items()}


def _fit(name, sat=150):
    b64 = base64.b64encode((FIX / name).read_bytes()).decode()
    return observe_ground_box(b64, _pose(name), 'small_box_01', min_saturation=sat)


def _skill():
    return v9.WristOnlyBoxSkillV9(robot_id='r1', cargo_id='small_box_01', **v1.BOX_SKILL_OPTIONS)


def test_frozen_versions_are_unchanged():
    for rev, path in (('ac34651', 'harness/wrist_zone_skill_v6.py'), ('57e2fd1', 'harness/wrist_zone_skill_v7.py'),
                      ('39f214c', 'harness/wrist_zone_skill_v8.py'), ('39f214c', 'scripts/run_zone_owncam_skill_v8.py')):
        assert subprocess.check_output(['git', 'show', f'{rev}:{path}'], cwd=ROOT) == (ROOT / path).read_bytes(), path


def test_fixture_hashes():
    import hashlib
    for name, f in FRAMES.items():
        assert hashlib.sha256((FIX / name).read_bytes()).hexdigest() == f['sha256']


def test_583_fit_model_flip_no_longer_flips_the_vertical_error():
    """583: the two wrist commands give different N7 fit models; v2's centroid error flips sign, v9's does not."""
    a, b = 'v8cohort-s583_0296.jpg', 'v8cohort-s583_0297.jpg'
    fa, fb = _fit(a), _fit(b)
    assert {fa['reason'], fb['reason']} == {'FLOOR_CUBOID_HYPOTHESIS_VALIDATED', 'MEASURED_TOP_FACE_FLOOR_HYPOTHESIS_VALIDATED'}
    assert abs(fa['estimated_box_center_base_m'][0] - fb['estimated_box_center_base_m'][0]) < .01
    v2_err = [f['pixel_centroid'][1] - 218.7 for f in (fa, fb)]
    assert min(v2_err) < -v2.NEAR_VERTICAL_TOL_PX and max(v2_err) > v2.NEAR_VERTICAL_TOL_PX    # v8: dithers
    v9_err = [v9.projected_centre_px(f, _pose(n))[1] - 218.7 for f, n in ((fa, a), (fb, b))]
    assert all(abs(e) <= v2.NEAR_VERTICAL_TOL_PX for e in v9_err)                                 # v9: holds


def test_face_approach_no_progress_guard_forces_one_step(monkeypatch):
    s = _skill()
    s._face_approach = True
    s.last_fused = {'x': .29, 'y': .005, 'n': 5, 'good': 5}
    monkeypatch.setattr(v8.WristOnlyBoxSkillV8, '_approach', lambda self, box, target, pose: {'kind': 'pose', 'pulses': {3: 545}})
    box = {'visible': True, 'estimated_box_center_base_m': [.295, .005, .016], 'pixel_centroid': [298., 187.]}
    pose = _pose('v8cohort-s583_0297.jpg')
    kinds = [s._approach(box, (.295, .005, .016), pose)['kind'] for _ in range(v9.FACE_NO_PROGRESS_CORRECTIONS)]
    assert kinds[:-1] == ['pose'] * (v9.FACE_NO_PROGRESS_CORRECTIONS - 1) and kinds[-1] == 'drive'
    assert s.v9_stats['face_forced_steps'] == 1 and s._face_pose_corrections == 0
    s.last_fused = {'x': .29, 'y': .10}                                   # box not ahead: no forced step
    assert all(s._approach(box, (.29, .10, .016), pose)['kind'] == 'pose' for _ in range(8))


def _str_pose(name):
    return {str(k): v for k, v in _pose(name).items()}


def test_587_close_range_fit_loss_enters_look_down_and_sweeps_until_fit(monkeypatch):
    good, lost = 'v8cohort-s587_0088.jpg', 'v8cohort-s587_0089.jpg'
    assert FRAMES[good]['logged_box']['visible'] and not FRAMES[lost]['logged_box']['visible']
    s = _skill()

    def parent(self, box, target, pose):
        return self._drive_macro(.15, 0., 1.) if target is not None else {'kind': 'wait', 'duration': .1}
    monkeypatch.setattr(v8.WristOnlyBoxSkillV8, '_approach', parent)
    box = dict(FRAMES[good]['logged_box'])
    target = tuple(box['estimated_box_center_base_m'])
    first = s._approach(box, target, _str_pose(good))                      # valid fit at 0.377 m -> N7 0.15 m macro
    assert first['kind'] == 'drive' and first['fwd'] == .15
    sweep = [s._approach({'visible': False}, None, _str_pose(lost)) for _ in range(2)]
    pan = _pose(good)[6]
    assert sweep == [{'kind': 'pose', 'pulses': {4: v9.LOOK_DOWN_ELBOW, 3: w, 6: pan}}
                     for w in v9.LOOK_DOWN_WRIST_SWEEP[:2]]
    again = s._approach(box, target, _str_pose(good))                      # fit valid again in look-down: creep only
    assert again['kind'] == 'drive' and again['fwd'] * again['duration'] <= v9.CAPPED_FORWARD[0] * v9.CAPPED_FORWARD[1] + 1e-9
    st = s.v9_stats
    assert st['look_down_entries'] == 1 and st['sweep_frames'] == 2 and st['capped_forwards'] == 1 and st['backoff_steps'] == 0
    assert [e['event'] for e in s.v9_events] == ['close_range_fit_lost_look_down', 'look_down_fit_valid']


def test_look_down_sweep_then_reverse_steps_then_restore_then_n7(monkeypatch):
    s = _skill()
    monkeypatch.setattr(v8.WristOnlyBoxSkillV8, '_approach', lambda self, box, target, pose: {'kind': 'wait', 'duration': .1})
    s._last_valid_fit = {'fx': .38, 'fy': 0., 'pose': {3: 611, 4: 2320, 6: 1534}}
    s._last_forward = (.15, 1., .38)
    pose = {'3': 623, '4': 2320, '6': 1534}
    n = len(v9.LOOK_DOWN_WRIST_SWEEP)
    acts = [s._approach({'visible': False}, None, pose) for _ in range((n + 1) * v9.MAX_BACKOFF_STEPS + n + 2)]
    kinds = [a['kind'] for a in acts]
    assert kinds[:n] == ['pose'] * n and acts[n] == v9.BACKOFF_STEP
    assert sum(a == v9.BACKOFF_STEP for a in acts) == v9.MAX_BACKOFF_STEPS
    restore = acts[(n + 1) * v9.MAX_BACKOFF_STEPS + n]
    assert restore == {'kind': 'pose', 'pulses': {3: 611, 4: 2320, 6: 1534}}
    assert acts[-1] == {'kind': 'wait', 'duration': .1} and s._look_down_done   # N7 unchanged afterwards


def test_no_look_down_when_far_or_without_a_forward_macro(monkeypatch):
    s = _skill()
    monkeypatch.setattr(v8.WristOnlyBoxSkillV8, '_approach', lambda self, box, target, pose: {'kind': 'wait', 'duration': .1})
    s._last_valid_fit = {'fx': .60, 'fy': 0., 'pose': {3: 740}}
    s._last_forward = (.15, 1., .60)
    assert s._approach({'visible': False}, None, {'3': 740, '6': 1500})['kind'] == 'wait'
    s._last_forward = None
    s._last_valid_fit['fx'] = .38
    assert s._approach({'visible': False}, None, {'3': 740, '6': 1500})['kind'] == 'wait'
    assert s.v9_stats['look_down_entries'] == 0


def test_look_down_holds_the_vertical_within_tolerance(monkeypatch):
    s = _skill()
    seen = {}

    def parent(self, box, target, pose):
        seen['row'] = box['pixel_centroid'][1]
        return {'kind': 'wait', 'duration': .1}
    monkeypatch.setattr(v8.WristOnlyBoxSkillV8, '_approach', parent)
    name = 'v8cohort-s583_0296.jpg'
    fit = _fit(name)
    projected = v9.projected_centre_px(fit, _pose(name))
    s._approach(fit, tuple(fit['estimated_box_center_base_m']), _str_pose(name))
    assert seen['row'] == projected[1]                                     # normal mode: projected centre
    s._look_down = True
    s._approach(fit, tuple(fit['estimated_box_center_base_m']), _str_pose(name))
    assert abs(projected[1] - v9.TARGET_ROW_PX) <= v9.LOOK_DOWN_VERTICAL_HOLD_PX and seen['row'] == v9.TARGET_ROW_PX


def test_v9_keeps_the_v6_to_v8_api():
    order = v5.CoarseOrderSheet('cyan', 'W2', (.60, -.85), (.15, .25), 'B2', (4.6, -2.1))
    skill = v9.WristZoneDeliveryV9(order, mode='m1', robot_id='r2', static_keepouts=[], static_bounds_m=(-1.05, 5.4, -3.15, 1.45))
    assert isinstance(skill.box, v9.WristOnlyBoxSkillV9) and skill.box.robot_id == 'r2'
    for name in ('approach_point', 'reanchor_after_probe', 'decide', 'confirm_placement', 'summary', 'planner_discs'):
        assert callable(getattr(skill, name))
    assert skill.summary()['profile'] == 'wrist_zone_skill_v9'
