import json
from pathlib import Path
import tempfile

import cv2
import numpy as np
import pytest

from harness.camera_approach_student import fit_approach_model, predict_approach
from scripts.train_camera_approach_student import _balanced, _labels, _validate_pair, train


def _views(state=0):
    own = np.zeros((144, 192, 3), np.uint8)
    top = np.zeros((192, 256, 3), np.uint8)
    cv2.rectangle(own, (60 + state, 45), (76 + state, 61), (220,)*3, -1)
    cv2.circle(top, (120 + 2 * state, 80 + state * state // 10), 7, (255,)*3, -1)
    result = []
    for image in (own, top):
        ok, encoded = cv2.imencode(".png", image);assert ok;result.append(encoded.tobytes())
    return tuple(result)


def _samples():
    rows = []
    for case in range(8):
        for state in (8, 6, 4, 2, 1, 0):
            own, top = _views(state)
            rows.append({"own_jpeg": own, "top_jpeg": top,
                         "forward": min(0.15, state * state * 0.004),
                         "stop": state == 0, "case_id": f"case-{case}"})
    return rows


def test_nonlinear_velocity_and_stop_model_is_json_serializable():
    goal = _views(0);model = fit_approach_model(*goal, _samples())
    json.dumps(model, allow_nan=False)
    assert model["diagnostics"]["hyperparameter_selection"] == "group_4fold"
    far = predict_approach(model, *_views(6))
    assert far["ok"] and far["forward"] > 0.003 and not far["ready"]
    assert 0 <= far["forward"] <= 0.15 and 0 <= far["stop_score"] <= 1


def test_goal_anchor_is_ready_with_zero_velocity():
    goal = _views(0);model = fit_approach_model(*goal, _samples())
    result = predict_approach(model, *goal)
    assert result["ok"] is True
    assert result["forward"] <= 0.003
    assert result["stop_score"] >= 0.65
    assert result["ready"] is True


def test_ood_rgb_stops_without_claiming_readiness():
    goal = _views(0);model = fit_approach_model(*goal, _samples())
    white = np.full((160, 220, 3), 255, np.uint8)
    ok, encoded = cv2.imencode(".png", white);assert ok
    result = predict_approach(model, encoded.tobytes(), encoded.tobytes())
    assert result["ok"] is False and result["forward"] == 0
    assert result["ready"] is False


def test_discarded_training_rgb_calibrates_domain_without_joining_regression():
    goal = _views(0)
    selected = _samples()
    own, top = _views(6)
    own_image = cv2.imdecode(np.frombuffer(own, np.uint8), cv2.IMREAD_COLOR)
    # A small appearance variation absent from the regression subset creates a
    # PCA residual while retaining the same supported approach coordinate.
    cv2.rectangle(own_image, (4, 4), (12, 12), (0, 80, 180), -1)
    ok, encoded = cv2.imencode(".png", own_image);assert ok
    discarded = {"own_jpeg": encoded.tobytes(), "top_jpeg": top,
                 "forward": 0.144, "stop": False, "case_id": "case-domain"}
    selected_only = fit_approach_model(*goal, selected)
    calibrated = fit_approach_model(
        *goal, selected, domain_samples=selected + [discarded])
    assert predict_approach(selected_only, discarded["own_jpeg"], top)["ok"] is False
    assert predict_approach(calibrated, discarded["own_jpeg"], top)["ok"] is True
    assert calibrated["diagnostics"]["support_count"] == selected_only["diagnostics"]["support_count"]
    assert calibrated["diagnostics"]["residual_domain_sample_count"] == len(selected) + 1


@pytest.mark.parametrize("forward,stop", [(-0.01, False), (0.151, False),
                                           (float("nan"), False), (0.1, 1)])
def test_malformed_or_out_of_range_labels_are_rejected(forward, stop):
    goal = _views(0);own, top = _views(2)
    rows = [{"own_jpeg": own, "top_jpeg": top, "forward": forward,
             "stop": stop, "case_id": f"case-{i}"} for i in range(4)]
    with pytest.raises(ValueError):
        fit_approach_model(*goal, rows)


def test_stop_false_with_zero_velocity_does_not_become_ready():
    goal = _views(0);rows = _samples()
    # An overshoot-like state may have zero forward while explicitly not stopped.
    own, top = _views(-2)
    for case in range(8):
        rows.append({"own_jpeg": own, "top_jpeg": top, "forward": 0.0,
                     "stop": False, "case_id": f"case-{case}"})
    model = fit_approach_model(*goal, rows)
    result = predict_approach(model, own, top)
    assert result["ok"] is True
    assert result["ready"] is False


def test_trainer_accepts_keyed_labels_and_balancing_preserves_near_stop_rows():
    keyed = {"sample-a": {"forward": 0.1, "stop": False}}
    assert _labels(keyed)["sample-a"]["forward"] == 0.1
    rows = []
    for case in range(4):
        for index in range(5):
            rows.append({"sample_id": f"{case}-{index}", "case_id": f"case-{case}",
                         "forward": 0.0 if index == 4 else 0.1, "stop": index == 4})
    selected = _balanced(rows, cap=12)
    assert {row["case_id"] for row in selected if row["stop"]} == {
        "case-0", "case-1", "case-2", "case-3"
    }
    selected_indices = {int(row["sample_id"].split("-")[1])
                        for row in selected if not row["stop"]}
    assert min(selected_indices) == 0
    assert max(selected_indices) >= 2


def test_duplicate_keyed_label_ids_are_not_silently_overwritten():
    with pytest.raises(ValueError):
        _labels([{"sample_id": "same", "forward": 0.1, "stop": False},
                 {"sample_id": "same", "forward": 0.0, "stop": True}])


def test_corrupt_excluded_actor_label_pair_is_rejected_before_filtering():
    actor = {"robot_id": "r3", "case_id": "excluded-case"}
    label = {"robot_id": "r1", "case_id": "excluded-case",
             "forward": 0.0, "stop": False}
    with pytest.raises(ValueError, match="robot mismatch"):
        _validate_pair(actor, label, "excluded-sample", {"excluded-case"})


def test_trainer_rejects_incomplete_teacher_collection():
    with tempfile.TemporaryDirectory() as value:
        teacher = Path(value) / "teacher"
        teacher.mkdir()
        (teacher / "report.json").write_text(json.dumps({"complete": False}))
        with pytest.raises(ValueError, match="must be complete"):
            train(teacher, Path(value) / "model")
