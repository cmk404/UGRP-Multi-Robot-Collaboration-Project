import copy
import math
import sys
from types import SimpleNamespace

import pytest

from harness.camera_varied_start_pose_student import (COMMAND_BOUNDS, GAINS,
    MIN_MOVING_COMMAND, SCHEMA, TOLERANCES, predict_pose_stage)


def _model(stage):
    return {"schema": SCHEMA, "robot_id": "r1", "stage": stage,
            "geometry": {"robot_id": "r1", "stage": stage},
            "settings": {"gain": GAINS[stage], "tolerance": TOLERANCES[stage],
                         "command_bounds": list(COMMAND_BOUNDS[stage]),
                         "minimum_moving_command": MIN_MOVING_COMMAND}}


def _inference(monkeypatch, result):
    seen = []
    def predict(model, image):
        seen.append(image)
        return result
    monkeypatch.setitem(sys.modules, "harness.camera_varied_start_geometry",
                        SimpleNamespace(predict_geometry=predict))
    monkeypatch.setitem(sys.modules, "harness.camera_varied_start_heading",
                        SimpleNamespace(predict_heading=predict))
    return seen


@pytest.mark.parametrize("stage", ["yaw", "lateral", "forward"])
def test_unready_visual_error_drives_signed_command_and_ready_stops(monkeypatch, stage):
    result = {"ok": True, "error": -TOLERANCES[stage] * 1.1, "diagnostics": {}}
    seen = _inference(monkeypatch, result)
    prediction = predict_pose_stage(_model(stage), b"own unused", b"top RGB")
    assert prediction["ok"] and not prediction["ready"]
    assert prediction["command"] == -MIN_MOVING_COMMAND
    assert seen == [b"top RGB"]
    result["error"] = TOLERANCES[stage] * .9
    prediction = predict_pose_stage(_model(stage), b"own", b"top RGB")
    assert prediction["ready"] and prediction["command"] == 0.


def test_unsupported_visual_state_cannot_move_or_enter_grasp(monkeypatch):
    _inference(monkeypatch, {"ok": False, "error": None, "diagnostics": {"reason": "no foreground"}})
    result = predict_pose_stage(_model("yaw"), b"own", b"top")
    assert not result["ok"] and not result["ready"] and result["command"] == 0.


def test_corrupt_calibration_and_nonfinite_prediction_are_rejected(monkeypatch):
    _inference(monkeypatch, {"ok": True, "error": math.nan, "diagnostics": {}})
    model = _model("forward")
    with pytest.raises(ValueError, match="nonfinite"):
        predict_pose_stage(model, b"own", b"top")
    changed = copy.deepcopy(model)
    changed["settings"]["tolerance"] = 1.
    with pytest.raises(ValueError, match="calibration"):
        predict_pose_stage(changed, b"own", b"top")
