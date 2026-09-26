"""wrist_zone_skill_v6: head-on own-RGB face yaw, public carry re-anchor hook, static keep-outs."""
import base64
import hashlib
import json
import math
import subprocess
from pathlib import Path

import cv2
import pytest

from harness import m1_contract
from harness import wrist_zone_skill as v1
from harness import wrist_zone_skill_v5 as v5
from harness import wrist_zone_skill_v6 as v6

ROOT = Path(__file__).resolve().parents[1]
FIX = ROOT / 'tests' / 'fixtures' / 'wrist_zone_skill_v6'
FACE = json.loads((FIX / 'frames.json').read_text())['frames']
PROBE = json.loads((FIX / 'probe' / 'frames.json').read_text())['sets']
ORDER = v5.CoarseOrderSheet('cyan', 'E1', (3.55, -2.35), (.25, .25), 'B2', (4.6, -2.1))
OWN = v1.PoseEstimate(3.05, -2.35, 0., 'own_rgb_apriltag_ekf')
GT = v1.PoseEstimate(3.05, -2.35, 0., 'gt_stub_eval_only')
PINNED = {'harness/wrist_zone_skill.py': '964e337eccdaef0bad17fec204b54234875bbb6a9e6e3300847e4c967d5312cd',
          'harness/wrist_zone_skill_v2.py': '70f9f9a2b0d81e0477a87cc9fa1a07f90eae0c2365e76244b5e28636be90ae6a',
          'harness/wrist_zone_skill_v3.py': '81114b8053b0d7f687c205f55d036e8a33fb5d66f50c846450b934654d8a8689'}


def _wrap90(deg):
    return (deg + 45.) % 90. - 45.


def _face_obs(frame, frame_id=1):
    payload = (FIX / frame['file']).read_bytes()
    return {'robot_id': 'r1', 'frame_id': frame_id, 'sim_time': float(frame_id), 'camera': 'robot_cam',
            'image': base64.b64encode(payload).decode(), 'sha256': hashlib.sha256(payload).hexdigest(),
            'actuator_state': {'servo_pulses': dict(frame['own_servo_pwm'])}}


def _probe_obs(entry, frame_id=None, pwm=None):
    payload = (FIX / 'probe' / entry['file']).read_bytes()
    return {'robot_id': 'r1', 'frame_id': entry['frame_id'] if frame_id is None else frame_id,
            'sim_time': float(entry['sim_time']), 'camera': 'robot_cam',
            'image': base64.b64encode(payload).decode(), 'sha256': hashlib.sha256(payload).hexdigest(),
            'actuator_state': {'servo_pulses': dict(pwm or entry['own_servo_pwm'])}}


def test_earlier_versions_are_unchanged():
    for path, digest in PINNED.items():
        assert hashlib.sha256((ROOT / path).read_bytes()).hexdigest() == digest, path
    for rev, path in (('6664425', 'harness/wrist_zone_skill_v4.py'), ('4884226', 'harness/wrist_zone_skill_v5.py'),
                      ('4884226', 'scripts/run_zone_owncam_skill_v5.py'), ('4884226', 'harness/m1_contract.py')):
        recorded = subprocess.check_output(['git', 'show', f'{rev}:{path}'], cwd=ROOT)
        assert recorded == (ROOT / path).read_bytes(), path


def test_fixture_hashes_match_their_manifest():
    for frame in FACE:
        assert hashlib.sha256((FIX / frame['file']).read_bytes()).hexdigest() == frame['sha256']
    for frames in PROBE.values():
        for entry in frames:
            assert hashlib.sha256((FIX / 'probe' / entry['file']).read_bytes()).hexdigest() == entry['sha256']


# ---------------- (a) head-on face yaw ----------------
@pytest.mark.parametrize('yaw', [-30., -20., -10., -5., -2., 0., 2., 5., 10., 20., 30.])
def test_top_edge_yaw_across_minus30_to_plus30_including_head_on(yaw):
    frames = [f for f in FACE if f['label_setup_yaw_deg'] == yaw]
    assert len(frames) == 3
    accepted = 0
    for frame in frames:
        image = cv2.imread(str(FIX / frame['file']))
        est = v6.top_edge_yaw(image, frame['own_servo_pwm'], frame['n7_target_xy_m'])
        if not est['ok']:
            # the only refusal on these renders is the close 0.30 m view whose far edge leaves the image
            assert est['reason'] == 'TOP_EDGE_CLIPPED' and frame['view']['range_m'] == .30
            continue
        accepted += 1
        err = abs(_wrap90(math.degrees(est['yaw_mod90_rad']) - frame['label_relative_yaw_deg_evaluation_only']))
        assert err <= 3., (frame['file'], err)
    assert accepted >= 2


def test_n7_cuboid_yaw_is_the_head_on_failure_v6_replaces():
    # recorded with each fixture: v5's evidence at exactly 0 deg is off by up to 30 deg
    head_on = [f for f in FACE if f['label_setup_yaw_deg'] == 0.]
    errors = [abs(_wrap90(f['n7_cuboid_yaw_mod90_deg'] - f['label_relative_yaw_deg_evaluation_only'])) for f in head_on]
    assert max(errors) > 7.


@pytest.mark.parametrize('yaw', [-2., 0., 2.])
def test_aligner_becomes_ready_on_axis_aligned_boxes(yaw):
    aligner = v6.EdgeYawFaceAligner()
    usable = [f for f in FACE if f['label_setup_yaw_deg'] == yaw and f['view']['range_m'] != .30]
    result = None
    for i, frame in enumerate(usable * 2):
        aligner.frame = _face_obs(frame, i + 1)
        result = aligner.observe({'visible': True}, frame['n7_target_xy_m'])
    assert result['ready'] and result['normal_source'].startswith('own_rgb')
    assert aligner.used_fallback is False
    label = usable[0]['label_relative_yaw_deg_evaluation_only']
    assert abs(_wrap90(result['evidence']['yaw_mod90_deg'] - label)) <= 3.
    # outward face toward the chassis: the box is ahead (+x), so the normal points back (-x)
    assert result['normal_xy'][0] < -.99


def test_aligner_never_votes_the_cuboid_yaw_and_has_no_frame_fallback():
    aligner = v6.EdgeYawFaceAligner()
    for _ in range(6):
        result = aligner.observe({'visible': True, 'estimated_yaw_mod_pi_rad': .3,
                                  'floor_hypothesis_projection_iou': .99}, (.38, 0.))
    assert not result['ready'] and result['reason'] == 'EDGE_YAW_EVIDENCE_NOT_ACCEPTED'
    assert result['evidence']['edge']['reason'] == 'NO_CURRENT_OWN_FRAME'


def test_box_skill_feeds_the_current_validated_frame_to_the_aligner(monkeypatch):
    box = v6.WristOnlyBoxSkillV6(robot_id='r1', cargo_id='small_box_01', **v1.BOX_SKILL_OPTIONS)
    seen = []
    monkeypatch.setattr(v6.v5.WristOnlyBoxSkillV5, 'decide',
                        lambda self, obs: seen.append(self._face_aligner.frame) or {'kind': 'wait', 'duration': .1})
    obs = _face_obs(FACE[0], 3)
    box.decide(obs)
    assert seen == [obs] and box._face_aligner.frame is None


# ---------------- (b) public re-anchor hook ----------------
def _carrying_box():
    box = v6.WristOnlyBoxSkillV6(robot_id='r1', cargo_id='small_box_01', **v1.BOX_SKILL_OPTIONS)
    box.phase, box.held = 'carry', True
    box._attachment_image = box._carry_previous_image = 'OLD'
    return box


def test_reanchor_accepts_the_recorded_held_probe_and_updates_the_anchor():
    box = _carrying_box()
    frames = [_probe_obs(e) for e in PROBE['held_post_lift']]
    box._validate_observation(frames[0])          # home_before may be the last frame the skill validated
    result = box.reanchor_after_probe(*frames)
    assert result['attached'] and result['anchor_updated']
    assert result['pans_pwm'] == [1520, 1580, 1460, 1520]
    assert box._attachment_image == frames[3]['image'] == box._carry_previous_image
    assert box._last_frame_id == frames[3]['frame_id']


def test_reanchor_rejects_a_box_standing_on_the_floor():
    box = _carrying_box()
    frames = [_probe_obs(e) for e in PROBE['floor_after_release']]
    result = box.reanchor_after_probe(*frames)
    assert not result['attached'] and not result['anchor_updated']
    assert box._attachment_image == 'OLD' == box._carry_previous_image


def test_reanchor_rejects_stale_frames_bad_geometry_and_wrong_phase():
    frames = [_probe_obs(e) for e in PROBE['held_post_lift']]
    box = _carrying_box()
    box._validate_observation(frames[1])           # left already consumed -> not a new frame
    with pytest.raises(ValueError):
        box.reanchor_after_probe(*frames)
    box = _carrying_box()
    swapped = [frames[0], frames[2], frames[1], frames[3]]
    with pytest.raises(ValueError):                # frame order / pan direction
        box.reanchor_after_probe(*swapped)
    box = _carrying_box()
    moved = dict(frames[2]['actuator_state']['servo_pulses'], **{'4': 1700})
    with pytest.raises(ValueError, match='arm servo 4'):
        box.reanchor_after_probe(frames[0], frames[1], _probe_obs(PROBE['held_post_lift'][2], pwm=moved), frames[3])
    box = _carrying_box()
    tampered = dict(frames[3], sha256='0' * 64)
    with pytest.raises(ValueError):
        box.reanchor_after_probe(frames[0], frames[1], frames[2], tampered)
    assert box._attachment_image == 'OLD'
    box = _carrying_box()
    box.phase = 'approach'
    with pytest.raises(RuntimeError):
        box.reanchor_after_probe(*frames)


def test_delivery_wrapper_keeps_its_own_gate_monotonic():
    skill = v6.WristZoneDeliveryV6(ORDER, mode='m1')
    skill.box.phase, skill.box.held = 'carry', True
    frames = [_probe_obs(e) for e in PROBE['held_post_lift']]
    result = skill.reanchor_after_probe(*frames)
    assert result['attached']
    assert skill._gate._last_frame_id == frames[3]['frame_id']
    assert skill._validated['frame_id'] == frames[3]['frame_id']
    assert skill.events[-1]['event'] == 'reanchor_after_probe'


# ---------------- (c) static keep-outs ----------------
def test_approach_point_is_v5s_without_keepouts_for_any_bay_half():
    for half in ((.25, .25), (.15, .25)):             # (.15, .25) = M1 amendment A2
        order = v5.CoarseOrderSheet('cyan', 'b', (-.19, -1.2), half, 'B2', (4.6, -2.1))
        choice = v6.WristZoneDeliveryV6(order).approach_point()
        assert choice['is_v5_point'] and choice['goal_xy_m'] == [round(-.19 - half[0] - .25, 4), -1.2]


def test_approach_point_moves_off_an_idle_peer_spot_or_blocks():
    order = v5.CoarseOrderSheet('cyan', 'b', (-.19, -2.25), (.25, .25), 'B2', (4.6, -2.1))
    peer = v6.StaticKeepout('r2_spawn', (-.85, -2.25), .17, 'static_layout_idle_spawn')   # s91 geometry
    choice = v6.WristZoneDeliveryV6(order, static_keepouts=[peer]).approach_point()
    assert not choice["blocked"] and not choice["is_v5_point"] and choice["side"] != "west"
    gx, gy = choice['goal_xy_m']
    assert math.hypot(gx + .85, gy + 2.25) >= v6.ROBOT_RADIUS_M + .17 + v6.KEEPOUT_MARGIN_M
    # a peer spot that only blocks the v5 point leaves a west candidate
    near = v6.StaticKeepout('r2_spawn', (-.69, -2.55), .17, 'static_layout_idle_spawn')
    west = v6.WristZoneDeliveryV6(order, static_keepouts=[near]).approach_point()
    assert west['side'] == 'west' and not west['is_v5_point']
    ring = [v6.StaticKeepout(f'p{i}', (-.19 + .75 * math.cos(a), -2.25 + .75 * math.sin(a)), .3, 'static_map_parking')
            for i, a in enumerate(k * math.pi / 4 for k in range(8))]
    wall = ring
    skill = v6.WristZoneDeliveryV6(order, static_keepouts=wall)
    assert skill.approach_point()['blocked']
    skill.phase = 'nav_pregrasp'
    assert skill._nav_pregrasp(None, OWN) == {'kind': 'finish', 'reason': 'BAY_APPROACH_BLOCKED'}


def test_approach_candidates_stay_inside_the_static_bounds():
    order = v5.CoarseOrderSheet('cyan', 'b', (-.19, -2.25), (.25, .25), 'B2', (4.6, -2.1))
    peer = v6.StaticKeepout('r2_spawn', (-.69, -2.25), .17, 'static_layout_idle_spawn')
    free = v6.WristZoneDeliveryV6(order, static_keepouts=[peer]).approach_point()
    tight = v6.WristZoneDeliveryV6(order, static_keepouts=[peer], static_bounds_m=(-1.05, 5.4, -2.80, 1.45))
    choice = tight.approach_point()
    assert free['side'] == 'south' and choice['side'] == 'north'
    assert choice["goal_xy_m"][1] - v6.ROBOT_RADIUS_M - v6.KEEPOUT_MARGIN_M >= -2.80


def test_keepouts_must_be_static_records():
    with pytest.raises(ValueError):
        v6.StaticKeepout('r2', (0., 0.), .17, 'live_peer_pose')
    with pytest.raises(TypeError):
        v6.WristZoneDeliveryV6(ORDER, static_keepouts=[((0., 0.), .17)])


def test_guard_stops_motion_into_a_keepout_but_allows_turns_and_retreat():
    peer = v6.StaticKeepout('r2', (3.40, -2.35), .17, 'static_layout_idle_spawn')     # 0.35 m ahead of OWN
    skill = v6.WristZoneDeliveryV6(ORDER, static_keepouts=[peer])
    assert skill._guard({'kind': 'drive', 'fwd': .1, 'turn': 0., 'duration': .5}, OWN) == 'r2'
    assert skill._guard({'kind': 'mecanum', 'forward': .05, 'left': .02, 'turn': 0., 'duration': .5}, OWN) == 'r2'
    assert skill._guard({'kind': 'drive', 'fwd': -.1, 'turn': 0., 'duration': .5}, OWN) is None
    assert skill._guard({'kind': 'mecanum', 'forward': 0., 'left': 0., 'turn': .1, 'duration': .5}, OWN) is None
    assert skill._guard({'kind': 'pose', 'pulses': {6: 1500}}, OWN) is None


def test_v6_keeps_the_m1_contract():
    skill = v6.WristZoneDeliveryV6(ORDER, mode='m1')
    frame = _face_obs(FACE[0], 1)
    with pytest.raises(m1_contract.ContractViolation):
        skill.decide(frame, GT)
    assert isinstance(skill.box, v6.WristOnlyBoxSkillV6)
    assert v6.WristZoneDeliveryV6(ORDER, mode='m1').decide(frame, OWN)['kind'] in ('mecanum', 'wait', 'drive')
