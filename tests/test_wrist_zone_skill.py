"""wrist_zone_skill_v1 and the wrist-camera view geometry (pure, no simulator)."""
import base64
import hashlib
import math
from pathlib import Path

import cv2
import numpy as np
import pytest

from harness import owncam_view as ov
from harness import wrist_zone_skill as wz
from harness.markerless_face import MarkerlessFaceAligner
from harness.visual_box_skill import SEARCH, VisualBoxSkill

ROOT = Path(__file__).resolve().parents[1]
ORDER = wz.OrderSheet('cyan', (3.5, -1.8), 'C1', (3.0, -1.25))


def _obs(frame_id, camera='robot_cam', pose=None):
    jpeg = cv2.imencode('.jpg', np.zeros((480, 640, 3), np.uint8))[1].tobytes()
    return {'robot_id': 'r1', 'frame_id': frame_id, 'sim_time': float(frame_id), 'camera': camera,
            'image': base64.b64encode(jpeg).decode(), 'sha256': hashlib.sha256(jpeg).hexdigest(),
            'actuator_state': {'servo_pulses': {str(k): v for k, v in (pose or SEARCH).items()}}}


# ---------------- view geometry ----------------
def test_carry_posture_constant_matches_posture_ik():
    assert ov.posture(.14, .18, -30.) == wz.CARRY_POSTURE


def test_higher_tool_pitch_raises_the_view():
    low = ov.view_metrics(ov.posture(.155, .14, -45.))
    mid = ov.view_metrics(ov.posture(.14, .18, -30.))
    high = ov.view_metrics(ov.posture(.14, .18, -20.))
    assert low['optical_axis_pitch_deg'] < mid['optical_axis_pitch_deg'] < high['optical_axis_pitch_deg']
    assert low['floor']['centre_line_x_m'][0] < mid['floor']['centre_line_x_m'][0]
    assert high['sees_above_horizon'] and not low['sees_above_horizon']


def test_sim_wrist_image_is_the_remapped_pinhole_region():
    mask = ov.valid_pixel_mask(8)
    assert 0.8 < mask.mean() < 1.0 and mask[30, 40] and not mask[0, 0]


def test_occlusion_mask_removes_view():
    pose = ov.posture(.14, .18, -30.)
    occ = np.zeros((480, 640), bool)
    occ[120:, :] = True                       # a held box covering the lower 3/4
    free = ov.view_metrics(pose)
    held = ov.view_metrics(pose, occlusion=occ)
    assert held['unoccluded_fraction_of_valid'] < .35 < free['unoccluded_fraction_of_valid']
    assert held['elevation_deg'][0] > free['elevation_deg'][0]


def test_tag_pixel_side_shrinks_with_distance():
    pose = ov.posture(.14, .18, -20.)
    near = ov.tag_pixel_side(pose, distance_m=.6, height_m=.2, size_m=.08)
    far = ov.tag_pixel_side(pose, distance_m=1.5, height_m=.2, size_m=.08)
    assert near['in_view'] and far['in_view'] and near['side_px'] > 1.8 * far['side_px']


def test_posture_rejects_unreachable():
    with pytest.raises(ValueError):
        ov.posture(.30, .30, 0.)


# ---------------- skill boundary ----------------
def test_pose_estimate_requires_source_and_finite_values():
    with pytest.raises(ValueError):
        wz.PoseEstimate(0., 0., 0., '')
    with pytest.raises(ValueError):
        wz.PoseEstimate(math.nan, 0., 0., 'gt_stub_eval_only')


def test_order_sheet_is_cyan_only():
    with pytest.raises(ValueError):
        wz.OrderSheet('red', (0., 0.), 'A1', (1., 1.))


def test_module_has_no_nav_cam_top_or_world_access():
    text = (ROOT / 'harness/wrist_zone_skill.py').read_text()
    code = '\n'.join(line for line in text.splitlines() if not line.lstrip().startswith('#'))
    body = code.split('"""', 2)[2]            # skip the module docstring
    for token in ('nav_cam', 'cctv', 'render_team', 'world.', '.data.', 'xpos', 'mujoco'):
        assert token not in body, token


def test_nav_cam_observation_is_rejected():
    skill = wz.WristZoneDelivery(ORDER)
    skill.phase = 'grasp'
    with pytest.raises(ValueError):
        skill.decide(_obs(1, camera='nav_cam'), wz.PoseEstimate(3.1, -1.8, 0., 'gt_stub_eval_only'))


def test_navigation_reaches_pregrasp_then_hands_over_to_rgb_grasp():
    skill = wz.WristZoneDelivery(ORDER)
    far = skill.decide(_obs(1), wz.PoseEstimate(2.6, -1.8, 0., 'stub'))
    assert far['kind'] == 'mecanum' and far['forward'] > 0 and skill.phase == 'nav_pregrasp'
    at = skill.decide(_obs(2), wz.PoseEstimate(3.5 - wz.PREGRASP_STANDOFF_M, -1.8, 0., 'stub'))
    assert at == {'kind': 'wait', 'duration': .1} and skill.phase == 'grasp'
    assert skill.pose_sources == {'stub'}


def test_small_errors_use_the_minimum_command():
    skill = wz.WristZoneDelivery(ORDER)
    action = skill.decide(_obs(1), wz.PoseEstimate(3.5 - wz.PREGRASP_STANDOFF_M - .03, -1.8, 0., 'stub'))
    assert action['forward'] == pytest.approx(wz.NAV_MIN_COMMAND)


def test_face_fallback_only_after_unready_rgb_and_is_labelled():
    aligner = wz.FaceAlignerWithMapFallback()
    aligner.map_normal_base = (-1., 0.)
    weak = {'visible': True, 'estimated_yaw_mod_pi_rad': .1, 'floor_hypothesis_projection_iou': .6}
    for _ in range(wz.FACE_FALLBACK_AFTER):
        assert not aligner.observe(weak, (.38, 0.))['ready']
    result = aligner.observe(weak, (.38, 0.))
    assert result['ready'] and result['normal_source'] == 'static_map_approach_convention+pose_estimate'
    assert aligner.used_fallback


def test_face_rgb_fits_win_when_available():
    aligner = wz.FaceAlignerWithMapFallback()
    aligner.map_normal_base = (0., 1.)
    good = {'visible': True, 'estimated_yaw_mod_pi_rad': .05, 'floor_hypothesis_projection_iou': .9}
    results = [aligner.observe(good, (.38, 0.)) for _ in range(3)]
    assert results[-1]['ready'] and results[-1]['normal_source'] == 'own_rgb_markerless_face_fits'
    assert results[-1]['normal_xy'][0] < -.9 and not aligner.used_fallback


def test_n7_class_is_unchanged():
    assert isinstance(VisualBoxSkill()._face_aligner, MarkerlessFaceAligner)
    assert isinstance(wz.WristOnlyBoxSkill()._face_aligner, wz.FaceAlignerWithMapFallback)


def test_placement_confirmation_maps_own_rgb_box_with_the_estimate(monkeypatch):
    import harness.zone_color_boxes as zcb
    monkeypatch.setattr(zcb, 'detect_own', lambda *a, **k: {'detections': [
        {'range_class': 'near', 'estimated_box_center_base_m': [.16, 0., .016],
         'floor_hypothesis_projection_iou': .8}]})
    skill = wz.WristZoneDelivery(ORDER)
    inside = skill.confirm_placement(_obs(1), wz.PoseEstimate(2.84, -1.25, 0., 'gt_stub_eval_only'))
    assert inside['in_slot'] and inside['pose_source'] == 'gt_stub_eval_only'
    outside = skill.confirm_placement(_obs(2), wz.PoseEstimate(2.70, -1.25, 0., 'gt_stub_eval_only'))
    assert not outside['in_slot'] and outside['reason'] == 'OUTSIDE_SLOT'


def test_runner_scenarios_are_preregistered_and_never_m1():
    from scripts import run_zone_owncam_skill as run
    assert set(run.SCENARIOS) == {401, 402, 501, 502, 503, 504, 505}
    assert run.DEV_SEEDS == (401, 402) and run.POSE_SOURCE == 'gt_stub_eval_only'
    text = (ROOT / 'scripts/run_zone_owncam_skill.py').read_text()
    assert "'counts_as_m1': False" in text and "camera='nav_cam'" not in text and 'render_team' not in text
    for scenario in run.SCENARIOS.values():
        assert scenario['start'][0] > 2.3 and scenario['pickup_xy'][0] > 2.3   # east of the divider: no door
