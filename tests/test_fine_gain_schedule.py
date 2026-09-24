from unittest.mock import patch

import pytest

from harness import fine_gain_schedule as schedule
from harness.camera_varied_start_pose_student import SCHEMA as POSE_SCHEMA, TOLERANCES
from scripts.run_camera_varied_start_student import run_approach
from tests.test_camera_varied_start_runner import FakeScene


def _model(stage='forward'):
    return {'schema': POSE_SCHEMA, 'stage': stage, 'robot_id': 'r1'}


def _decision(error, command, *, ok=True, ready=False, precision='fine'):
    return {'ok': ok, 'ready': ready, 'stationary_ready': ready, 'command': command,
            'precision': precision, 'diagnostics': {'image_derived_error': error}}


def test_far_forward_error_uses_existing_bound_and_keeps_model_output():
    out = schedule.schedule_decision(_model(), _decision(.17, .034))
    assert out['command'] == .15
    assert out['model_command'] == .034
    assert out['gain_schedule']['applied'] is True


def test_previous_command_carry_over_shrinks_command_before_handover():
    # Recorded B1 overshoot point: error .0596 after a .15 command.
    out = schedule.schedule_decision(_model(), _decision(.0596, .012), previous_command=.15)
    room = .0596 - .15 * schedule.CARRY_RESPONSE_HIGH['forward'] - 8 * TOLERANCES['forward']
    assert room < 0 and out['command'] == .012 and out['gain_schedule']['applied'] is False
    mid = schedule.schedule_decision(_model(), _decision(.0951, .019), previous_command=.15)
    expected = (.0951 - .15 * .24 - .032) / schedule.TOTAL_RESPONSE_HIGH['forward']
    assert mid['command'] == pytest.approx(max(.019, expected))
    # Upper-bound landing point stays outside the hand-over distance.
    landing = .0951 - .15 * .24 - mid['command'] * schedule.TOTAL_RESPONSE_HIGH['forward']
    assert landing >= .032 - 1e-12


@pytest.mark.parametrize('stage', ['yaw', 'lateral', 'forward'])
def test_handover_ready_unsupported_and_latched_decisions_are_unchanged(stage):
    near = schedule.HANDOVER_TOLERANCES * TOLERANCES[stage]
    for decision in (_decision(near, .01), _decision(-near, -.01),
                     _decision(.2, 0., ready=True), _decision(.2, 0., ok=False),
                     _decision(.2, .04, precision='coarse')):
        out = schedule.schedule_decision(_model(stage), decision)
        assert out['command'] == decision['command']
        assert out['gain_schedule']['applied'] is False
    latched = schedule.schedule_decision(_model(stage), _decision(.2, .01), latched=True)
    assert latched['command'] == .01 and latched['gain_schedule']['latched'] is True


def test_schedule_never_lowers_or_reverses_the_model_command():
    strong = schedule.schedule_decision(_model('lateral'), _decision(.2, .06))
    assert strong['command'] == .06
    opposed = schedule.schedule_decision(_model(), _decision(.2, -.02))
    assert opposed['command'] == -.02 and opposed['gain_schedule']['applied'] is False


def test_negative_forward_respects_reverse_bound():
    assert schedule.schedule_decision(_model(), _decision(-.2, -.04))['command'] == -.05


def test_stage_scheduler_latches_after_sign_flip():
    s = schedule.StageScheduler(['r1'])
    first = s.decide('r1', _model(), _decision(.17, .034))
    assert first['command'] == .15
    s.issued('r1', first['command'])
    flipped = s.decide('r1', _model(), _decision(-.006, -.01))
    assert flipped['gain_schedule']['latched'] is True
    s.issued('r1', flipped['command'])
    again = s.decide('r1', _model(), _decision(.17, .034))
    assert again['command'] == .034 and again['gain_schedule']['latched'] is True


def test_legacy_models_are_rejected():
    with pytest.raises(ValueError):
        schedule.schedule_decision({'schema': 'other', 'stage': 'forward'}, _decision(.2, .04))


def _models():
    return {r: {s: {'schema': POSE_SCHEMA, 'stage': s, 'robot_id': r}
                for s in ('yaw', 'lateral', 'forward')} for r in ('r1', 'r3')}


def _run(enabled):
    far = _decision(.17, .034)
    ready = _decision(0., 0., ready=True)
    calls = {'forward': 0}
    def predict(model, own, top):
        if model['stage'] != 'forward':
            return dict(ready)
        calls['forward'] += 1
        return dict(far) if calls['forward'] <= 4 else dict(ready)
    scene = FakeScene()
    with patch('harness.camera_varied_start_student.predict_stage', side_effect=predict):
        result = run_approach(scene, _models(), fine_gain_schedule=enabled)
    return scene, result


def test_run_approach_applies_schedule_only_when_requested_and_tracks_own_commands():
    scene, result = _run(False)
    forward = [d for d, _ in scene.drives if any(c['forward'] for c in d.values())]
    assert forward[0]['r1']['forward'] == .034
    assert 'fine_gain_schedule' not in result
    scene, result = _run(True)
    forward = [d for d, _ in scene.drives if any(c['forward'] for c in d.values())]
    assert forward[0]['r1']['forward'] == .15
    # The second slice sees its own previous .15 command as carry-over.
    second = [c for c in result['approach_calls'] if c['action']['forward'] and c['robot_id'] == 'r1'][1]
    assert second['decision']['gain_schedule']['previous_command'] == .15
    assert second['decision']['gain_schedule']['owed_high'] == pytest.approx(.15 * .24)
    assert result['fine_gain_schedule']['schema'] == schedule.SCHEMA


def test_run_approach_rejects_schedule_for_non_pose_models():
    with pytest.raises(ValueError, match='visual pose stage models'):
        run_approach(FakeScene(), {'r1': {'forward': {}}, 'r3': {'forward': {}}},
                     fine_gain_schedule=True)


def test_dispatch_cli_rejects_schedule_with_realtime_control(tmp_path):
    from scripts.run_dispatch_e2e import main
    with pytest.raises(SystemExit):
        main(['--output', str(tmp_path / 'x'), '--fine-gain-schedule', '--realtime-control',
              '--grasp-model-dir', str(tmp_path), '--stage-model-dir', str(tmp_path)])
    assert not (tmp_path / 'x').exists()
