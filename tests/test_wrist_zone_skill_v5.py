"""wrist_zone_skill_v5 / runner v5: Codex own-camera review fixes (issues 1, 2, 3, 5, 6, 8)."""
import base64
import hashlib
import importlib.util
import math
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from harness import m1_contract
from harness import wrist_zone_skill as v1
from harness import wrist_zone_skill_v5 as v5
from harness.visual_box_skill import SEARCH

ROOT = Path(__file__).resolve().parents[1]
PINNED = {'harness/wrist_zone_skill.py': '964e337eccdaef0bad17fec204b54234875bbb6a9e6e3300847e4c967d5312cd',
          'harness/wrist_zone_skill_v2.py': '70f9f9a2b0d81e0477a87cc9fa1a07f90eae0c2365e76244b5e28636be90ae6a',
          'harness/wrist_zone_skill_v3.py': '81114b8053b0d7f687c205f55d036e8a33fb5d66f50c846450b934654d8a8689'}
ORDER = v5.CoarseOrderSheet('cyan', 'E1', (3.55, -2.35), (.25, .25), 'B2', (4.6, -2.1))
GT = v1.PoseEstimate(3.05, -2.35, 0., 'gt_stub_eval_only')
OWN = v1.PoseEstimate(3.05, -2.35, 0., 'own_rgb_apriltag_ekf')


def _obs(frame_id, camera='robot_cam', robot='r1', tamper=False):
    jpeg = cv2.imencode('.jpg', np.zeros((480, 640, 3), np.uint8))[1].tobytes()
    digest = hashlib.sha256(jpeg).hexdigest()
    if tamper:
        digest = '0' * 64
    return {'robot_id': robot, 'frame_id': frame_id, 'sim_time': float(frame_id), 'camera': camera,
            'image': base64.b64encode(jpeg).decode(), 'sha256': digest,
            'actuator_state': {'servo_pulses': {str(k): v for k, v in SEARCH.items()}}}


def test_earlier_versions_are_unchanged():
    for path, digest in PINNED.items():
        assert hashlib.sha256((ROOT / path).read_bytes()).hexdigest() == digest, path
    # v4 as recorded by the v4 cohort (6664425)
    v4_sha = subprocess.check_output(['git', 'show', '6664425:harness/wrist_zone_skill_v4.py'], cwd=ROOT)
    assert hashlib.sha256(v4_sha).hexdigest() == hashlib.sha256((ROOT / 'harness/wrist_zone_skill_v4.py').read_bytes()).hexdigest()


# ---------------- issue 1: M1 contract ----------------
@pytest.mark.parametrize('label, ok', [('gt_stub_eval_only', False), ('stub', False), ('own_rgb_tags+gt', False),
                                       ('own_rgb_top_fusion', False), ('own_rgb_nav_cam_vo', False),
                                       ('own_rgb_apriltag_ekf', True), ('owncam_pf_v2:3fa91c0d', True),
                                       ('owncam_pf_v2+gt', False)])
def test_m1_pose_source_allow_list(label, ok):
    assert m1_contract.is_m1_pose_source(label) is ok


def test_executor_rejects_gt_pose_in_m1_mode():
    skill = v5.WristZoneDeliveryV5(ORDER, mode='m1')
    with pytest.raises(m1_contract.ContractViolation):
        skill.decide(_obs(1), GT)
    ok = v5.WristZoneDeliveryV5(ORDER, mode='m1')
    assert ok.decide(_obs(1), OWN)['kind'] in ('mecanum', 'wait')


def test_judge_rejects_gt_pose_in_m1_mode():
    skill = v5.WristZoneDeliveryV5(ORDER, mode='m1')
    skill.decide(_obs(1), OWN)
    with pytest.raises(m1_contract.ContractViolation):
        skill.confirm_placement(_obs(1), GT)


def test_outcome_fields_never_turn_gt_into_m1_success():
    contract = {'camera': 'robot_cam'}
    diag = m1_contract.outcome_fields(mode='diagnostic', pose_sources_seen={'gt_stub_eval_only'},
                                      diagnostic_success=True, input_contract=contract)
    assert diag['diagnostic_success'] and not diag['counts_as_m1'] and not diag['m1_success']
    m1_gt = m1_contract.outcome_fields(mode='m1', pose_sources_seen={'gt_stub_eval_only'},
                                       diagnostic_success=True, input_contract=contract)
    assert not m1_gt['m1_success']
    other_cam = m1_contract.outcome_fields(mode='m1', pose_sources_seen={'own_rgb_apriltag_ekf'},
                                           diagnostic_success=True, input_contract=contract,
                                           cameras_seen={'robot_cam', 'nav_cam'})
    assert not other_cam['counts_as_m1']
    clean = m1_contract.outcome_fields(mode='m1', pose_sources_seen={'own_rgb_apriltag_ekf'},
                                       diagnostic_success=True, input_contract=contract)
    assert clean['counts_as_m1'] and clean['m1_success']


def test_exporter_validation_rejects_missing_or_inconsistent_fields():
    good = m1_contract.outcome_fields(mode='diagnostic', pose_sources_seen={'gt_stub_eval_only'},
                                      diagnostic_success=True, input_contract={})
    m1_contract.validate_outcome({**good, 'success': False})
    with pytest.raises(m1_contract.ContractViolation):
        m1_contract.validate_outcome({**good, 'success': True})          # the v2 s518 derived-result mistake
    with pytest.raises(m1_contract.ContractViolation):
        m1_contract.validate_outcome({k: v for k, v in good.items() if k != 'm1_success'})
    with pytest.raises(m1_contract.ContractViolation):
        m1_contract.validate_outcome({**good, 'counts_as_m1': True, 'm1_success': True})


def test_runner_refuses_m1_with_only_the_gt_stub():
    spec = importlib.util.spec_from_file_location('runner_v5', ROOT / 'scripts/run_zone_owncam_skill_v5.py')
    runner = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runner)
    with pytest.raises(m1_contract.ContractViolation):
        runner.select_pose_source('m1', None)
    proc = subprocess.run([sys.executable, str(ROOT / 'scripts/run_zone_owncam_skill_v5.py'), '--seed', '541',
                           '--output', '/nonexistent/never-created', '--mode', 'm1'],
                          capture_output=True, text=True, cwd=ROOT)
    assert proc.returncode != 0 and 'REFUSED' in proc.stderr
    assert not Path('/nonexistent/never-created').exists()


# ---------------- issue 2: strict validation at every stage ----------------
@pytest.mark.parametrize('camera', ['nav_cam', 'cctv_top', 'cctv_top_east'])
def test_look_back_rejects_other_cameras(camera):
    skill = v5.WristZoneDeliveryV5(ORDER)
    skill.decide(_obs(1), GT)                    # nav step with a valid own frame
    skill.phase = 'look_back'
    action = skill.decide(_obs(2, camera=camera), GT)
    assert action['kind'] == 'finish' and action['reason'].startswith('OBSERVATION_REJECTED')
    assert skill.placement is None and skill.rejected_observations[-1]['camera'] == camera


@pytest.mark.parametrize('kwargs, frame', [({'tamper': True}, 2), ({'robot': 'r2'}, 2), ({}, 1)])
def test_look_back_rejects_hash_robot_and_stale_frames(kwargs, frame):
    skill = v5.WristZoneDeliveryV5(ORDER)
    skill.decide(_obs(1), GT)
    skill.phase = 'look_back'
    action = skill.decide(_obs(frame, **kwargs), GT)
    assert action['reason'].startswith('OBSERVATION_REJECTED') and skill.placement is None


def test_navigation_phase_also_rejects_other_cameras():
    skill = v5.WristZoneDeliveryV5(ORDER)
    assert skill.decide(_obs(1, camera='nav_cam'), GT)['reason'].startswith('OBSERVATION_REJECTED')


def test_confirm_placement_only_accepts_the_last_validated_frame():
    skill = v5.WristZoneDeliveryV5(ORDER)
    skill.decide(_obs(1), GT)
    with pytest.raises(ValueError):
        skill.confirm_placement(_obs(1, camera='cctv_top'), GT)
    with pytest.raises(ValueError):
        skill.confirm_placement(_obs(7), GT)
    result = skill.confirm_placement(_obs(1), GT)
    assert result['observation_validated'] and result['reason'] == 'NO_UNIQUE_CYAN_BOX'


# ---------------- issue 3: coarse order, own-RGB orientation ----------------
def test_v5_refuses_an_exact_pickup_order():
    with pytest.raises(TypeError):
        v5.WristZoneDeliveryV5(v1.OrderSheet('cyan', (3.5, -1.8), 'C1', (3.0, -1.25)))
    with pytest.raises(ValueError):
        v5.CoarseOrderSheet('cyan', 'X', (3.5, -1.8), (.02, .02), 'C1', (3.0, -1.25))
    assert not hasattr(ORDER, 'pickup_xy_m')


def test_bay_approach_uses_only_the_bay():
    skill = v5.WristZoneDeliveryV5(ORDER)
    action = skill.decide(_obs(1), v1.PoseEstimate(3.05, -2.35, 0., 'gt_stub_eval_only'))
    assert action['kind'] == 'wait' and skill.phase == 'grasp'
    assert skill.events[-1]['event'] == 'bay_approach_point_reached' and skill.events[-1]['goal'] == [3.05, -2.35]


def _fit(deg, iou=.94):
    return {'visible': True, 'estimated_yaw_mod_pi_rad': math.radians(deg), 'floor_hypothesis_projection_iou': iou}


def test_face_vote_tolerates_the_recorded_outliers():
    # v4 P/531 face inspection: 85, 64 (IoU .71), 86, 85 -> N7's rule never got 3 consecutive; v5 votes.
    aligner = v5.RobustFaceAligner()
    results = [aligner.observe(_fit(d, i), (.40, .0)) for d, i in ((85, .94), (64, .71), (86, .94), (85, .93))]
    assert [r['ready'] for r in results] == [False, False, False, True]
    nx, ny = results[-1]['normal_xy']
    assert nx < -.99 and abs(math.degrees(math.atan2(ny, nx)) % 90 - 85.3) < 1.
    mixed = v5.RobustFaceAligner()
    seq = [mixed.observe(_fit(d, i), (.25, .0))['ready'] for d, i in ((1, .94), (88, .94), (21, .79), (1, .94), (89, .95))]
    assert seq[-1]


def test_face_vote_refuses_scattered_fits_and_has_no_fallback():
    aligner = v5.RobustFaceAligner()
    assert not any(aligner.observe(_fit(d), (.4, 0.))['ready'] for d in (0, 30, 60, 15, 45, 75))
    box = v5.WristOnlyBoxSkillV5(robot_id='r1', cargo_id='small_box_01', **v1.BOX_SKILL_OPTIONS)
    box.set_map_face_normal((-1., 0.))
    assert box.map_normal_offers_ignored == 1 and box._face_aligner.used_fallback is False
    assert not box._face_aligner.observe({'visible': False}, (.4, 0.))['ready']


def test_face_unobservable_relooks_then_aborts(monkeypatch):
    skill = v5.WristZoneDeliveryV5(ORDER)
    skill.phase = 'grasp'
    frame = [0]

    def unobservable(self, obs):
        return {'kind': 'finish', 'reason': 'BOX_FACE_ALIGNMENT_UNOBSERVABLE'}
    monkeypatch.setattr(v5.WristOnlyBoxSkillV5, 'decide', unobservable)
    reasons = []
    for _ in range(v5.MAX_FACE_RELOOKS + 1):
        skill.phase = 'grasp'
        frame[0] += 1
        action = skill.decide(_obs(frame[0]), GT)
        reasons.append(skill.phase if action['kind'] != 'finish' else action['reason'])
    assert reasons == ['backoff'] * v5.MAX_FACE_RELOOKS + ['GRASP_FACE_UNOBSERVABLE_AFTER_RELOOKS']
    assert skill.face_relooks == v5.MAX_FACE_RELOOKS


# ---------------- issue 5: per-physics-step contacts ----------------
def test_contact_log_counts_cargo_wall_every_physics_step():
    import mujoco
    xml = """<mujoco><option timestep="0.001"/><worldbody>
      <geom name="floor" type="plane" size="1 1 .1"/>
      <geom name="zone_wall_a" type="box" pos=".032 0 .05" size=".01 .2 .05"/>
      <body name="cargo_box_00" pos="0 0 .02"><freejoint/><geom name="cargo_box_00_geom" type="box" size=".02 .02 .016" mass=".05"/></body>
      </worldbody></mujoco>"""
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    data.qvel[0] = .5                         # slide into the wall
    spec = importlib.util.spec_from_file_location('runner_v5b', ROOT / 'scripts/run_zone_owncam_skill_v5.py')
    runner = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runner)
    log = runner.PhysicsContactLog(SimpleNamespace(model=model, data=data), 'cargo_box_00')
    for _ in range(300):
        mujoco.mj_step(model, data)
        log.step(float(data.time))
    summary = log.summary()
    cw = summary['classes']['cargo_wall']
    assert summary['physics_steps'] == 300 and cw['steps'] > 50 and cw['max_normal_force_n'] > 0
    assert cw['min_dist_m'] is not None and summary['episode_counts']['cargo_wall'] >= 1


# ---------------- issue 8: aggregator enforces the pre-registered seed list ----------------
def test_aggregator_marks_missing_seeds_and_blocks():
    spec = importlib.util.spec_from_file_location(
        'build_results', ROOT / 'experiments/2026-09-25-zone-owncam-skill/build_results.py')
    agg = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(agg)
    runs, missing = agg.collect_preregistered(lambda s: {'seed': s} if s != 543 else None, (541, 542, 543))
    assert [r['seed'] for r in runs] == [541, 542] and missing == [{'seed': 543, 'status': 'missing/infrastructure_failure'}]
    with pytest.raises(agg.CohortIncomplete):
        agg.require_complete('v5', missing)


def _square_box(yaw_deg):
    box = v5.WristOnlyBoxSkillV5(robot_id='r1', cargo_id='small_box_01', **v1.BOX_SKILL_OPTIONS)
    box._face_inspection_reached = True
    box.last_face_alignment = {'ready': True, 'normal_source': 'own_rgb_markerless_face_inlier_vote',
                               'evidence': {'yaw_mod90_deg': yaw_deg % 90}}
    return box


def test_face_squaring_rotates_toward_the_own_rgb_face_then_strafes_then_hands_over():
    pose = {'1': 2000, '3': 640, '4': 2320, '5': 1320, '6': 1500}
    box = _square_box(25.)
    action = box._approach({'visible': True}, (.39, .0, .016), pose)
    assert action['kind'] == 'mecanum' and action['turn'] > 0 and action['forward'] == 0
    box = _square_box(-12.)
    assert box._approach({'visible': True}, (.39, .0, .016), pose)['turn'] < 0
    box = _square_box(2.)
    action = box._approach({'visible': True}, (.39, .03, .016), pose)
    assert action == {'kind': 'pose', 'pulses': {6: 1549}}          # keep the box in view first
    action = box._approach({'visible': True}, (.39, .03, .016), {**pose, '6': 1549})
    assert action['kind'] == 'mecanum' and action['left'] > 0 and action['turn'] == 0
    assert box._face_aligner is not None and box.face_square_stats['strafes'] == 1
    action = box._approach({'visible': True}, (.39, .004, .016), pose)
    assert action['kind'] == 'wait' and box._face_approach and box.face_square_stats['squared']['yaw_err_deg'] == 2.


def test_face_squaring_gives_up_after_the_move_cap():
    pose = {'1': 2000, '3': 640, '4': 2320, '5': 1320, '6': 1500}
    box = _square_box(25.)
    box.face_square_stats['turns'] = v5.FACE_SQUARE_MAX_TURNS
    action = box._approach({'visible': True}, (.39, .0, .016), pose)
    assert action['kind'] == 'finish' and action['reason'] == 'BOX_FACE_SQUARING_NO_CONVERGENCE'


def _tb_builder():
    spec = importlib.util.spec_from_file_location(
        'build_tb_v5', ROOT / 'experiments/2026-09-25-zone-owncam-skill/build_tensorboard_v5.py')
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _legacy_result(tmp_path):
    import json
    src = tmp_path / 'src'
    src.mkdir()
    (src / 'result.json').write_text(json.dumps({
        'profile': 'wrist_zone_skill_v2', 'pose_source': 'gt_stub_eval_only', 'pose_sources_seen': ['gt_stub_eval_only'],
        'counts_as_m1': False, 'reason': 'OWN_RGB_PLACEMENT_IN_SLOT', 'claim_scope': 'skill isolation',
        'sim_seconds': 100., 'wall_seconds': 150., 'steps': 200, 'seed': 518, 'development_seed': False,
        'source_sha': 'x', 'evaluation_only': {'place_in_slot_gt': True, 'skill_claim_in_slot': True}}))
    return src


def test_exporter_relabels_gt_success_as_diagnostic_only(tmp_path, monkeypatch):
    import json
    tb = _tb_builder()
    monkeypatch.setattr(tb, 'VIEW', tmp_path / 'view')
    name = tb.derived_run(_legacy_result(tmp_path), 'v2-s518', 'relabel', 'diagnostic')
    derived = json.loads((tmp_path / 'view' / name / 'result.json').read_text())
    assert derived['success'] is False and derived['m1_success'] is False
    assert derived['diagnostic_success'] is True and derived['counts_as_m1'] is False
    assert derived['pose_source'] == 'gt_stub_eval_only' and 'EXACT pickup_xy' in derived['input_contract']['order_sheet']


def test_exporter_refuses_gt_results_in_an_m1_view(tmp_path, monkeypatch):
    tb = _tb_builder()
    monkeypatch.setattr(tb, 'VIEW', tmp_path / 'view')
    with pytest.raises(m1_contract.ContractViolation):
        tb.derived_run(_legacy_result(tmp_path), 'v2-s518', 'relabel', 'm1')
