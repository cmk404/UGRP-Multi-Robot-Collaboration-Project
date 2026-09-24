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


@pytest.mark.parametrize('stage', ['yaw', 'lateral', 'forward'])
def test_near_target_ready_and_unsupported_decisions_are_unchanged(stage):
    near = 3 * TOLERANCES[stage]
    for decision in (_decision(near, .01), _decision(-near, -.01),
                     _decision(.2, 0., ready=True), _decision(.2, 0., ok=False),
                     _decision(.2, .04, precision='coarse')):
        out = schedule.schedule_decision(_model(stage), decision)
        assert out['command'] == decision['command']
        assert out['gain_schedule']['applied'] is False


def test_schedule_never_lowers_or_reverses_the_model_command():
    strong = schedule.schedule_decision(_model('lateral'), _decision(.02, .06))
    assert strong['command'] == .06
    opposed = schedule.schedule_decision(_model(), _decision(.2, -.02))
    assert opposed['command'] == -.02 and opposed['gain_schedule']['applied'] is False


def test_negative_forward_respects_reverse_bound_and_no_crossing_cap():
    reverse = schedule.schedule_decision(_model(), _decision(-.2, -.04))
    assert reverse['command'] == -.05
    tol = TOLERANCES['lateral']
    error = 3.2 * tol
    out = schedule.schedule_decision(_model('lateral'), _decision(error, .01))
    expected = min(schedule.STEP_FRACTION * error / schedule.NOMINAL_RESPONSE['lateral'],
                   (error - tol) / schedule.HIGH_RESPONSE['lateral'])
    assert out['command'] == pytest.approx(max(.01, expected))
    assert out['command'] * schedule.HIGH_RESPONSE['lateral'] <= error - tol + 1e-12


def test_legacy_models_are_rejected():
    with pytest.raises(ValueError):
        schedule.schedule_decision({'schema': 'other', 'stage': 'forward'}, _decision(.2, .04))


def _models():
    return {r: {s: {'schema': POSE_SCHEMA, 'stage': s, 'robot_id': r}
                for s in ('yaw', 'lateral', 'forward')} for r in ('r1', 'r3')}


def _run(enabled):
    far = _decision(.17, .034)
    ready = _decision(0., 0., ready=True)
    def predict(model, own, top):
        if model['stage'] != 'forward':
            return dict(ready)
        return dict(far) if predict.calls < 2 else dict(ready)
    predict.calls = 0
    def counted(model, own, top):
        value = predict(model, own, top)
        if model['stage'] == 'forward':
            predict.calls += 1
        return value
    scene = FakeScene()
    with patch('harness.camera_varied_start_student.predict_stage', side_effect=counted):
        result = run_approach(scene, _models(), fine_gain_schedule=enabled)
    return scene, result


def test_run_approach_applies_schedule_only_when_requested():
    scene, result = _run(False)
    forward = [d for d, _ in scene.drives if any(c['forward'] for c in d.values())]
    assert forward[0]['r1']['forward'] == .034
    assert 'fine_gain_schedule' not in result
    scene, result = _run(True)
    forward = [d for d, _ in scene.drives if any(c['forward'] for c in d.values())]
    assert forward[0]['r1']['forward'] == .15
    assert result['fine_gain_schedule']['schema'] == schedule.SCHEMA
    call = next(c for c in result['approach_calls'] if c['action']['forward'])
    assert call['decision']['model_command'] == .034


def test_run_approach_rejects_schedule_for_non_pose_models():
    with pytest.raises(ValueError, match='visual pose stage models'):
        run_approach(FakeScene(), {'r1': {'forward': {}}, 'r3': {'forward': {}}},
                     fine_gain_schedule=True)
