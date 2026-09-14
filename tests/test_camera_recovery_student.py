import json

import cv2
import numpy as np
import pytest

from harness.camera_recovery_student import fit_recovery_model, predict_recovery


def _views(state=(0, 0, 0)):
    x, y, z = state
    own = np.zeros((144, 192, 3), np.uint8)
    top = np.zeros((192, 256, 3), np.uint8)
    # Three independently observable image effects, with nonlinear placement.
    cv2.circle(own, (45 + int(3*x + .15*x*x), 45), 7, (220,)*3, -1)
    cv2.rectangle(own, (115, 80 + int(3*y)), (130, 95 + int(3*y)), (160,)*3, -1)
    cv2.circle(top, (120 + int(3*z), 80 + int(.2*z*z)), 6, (255,)*3, -1)
    encoded = []
    for image in (own, top):
        ok, value = cv2.imencode(".png", image);assert ok;encoded.append(value.tobytes())
    return tuple(encoded)


def _training():
    result = []
    # Correction is nonlinear in the observed state and spans all three axes.
    for state in ((-4,0,0),(-2,0,0),(2,0,0),(4,0,0),
                  (0,-4,0),(0,-2,0),(0,2,0),(0,4,0),
                  (0,0,-4),(0,0,-2),(0,0,2),(0,0,4),
                  (-2,2,-2),(2,-2,2)):
        own, top = _views(state)
        x, y, z = state
        correction = [int(round(-8*x - .5*x*abs(x))),
                      int(round(-8*y + .5*y*abs(y))), int(round(-8*z))]
        result.append({"own_jpeg": own, "top_jpeg": top,
                       "correction_pulses": correction})
    return result


def test_nonlinear_model_is_serializable_bounded_and_uses_goal_anchor():
    reference = _views()
    model = fit_recovery_model(*reference, _training())
    json.dumps(model, allow_nan=False)
    assert model["diagnostics"]["goal_anchor_included"] is True
    assert model["diagnostics"]["rank"] == 3
    assert "goal_pulses" not in json.dumps(model)
    result = predict_recovery(model, *_views((2, -2, 2)), max_step=25)
    assert result["observable"] is True
    assert result["delta_pulses"][0] < 0
    assert result["delta_pulses"][1] > 0
    assert result["delta_pulses"][2] < 0
    assert max(map(abs, result["delta_pulses"])) <= 25
    assert result["predicted_remaining_error"] == result["feature_error"]
    assert result["prediction_error_available"] is False


def test_goal_rgb_returns_zero_action():
    reference = _views()
    model = fit_recovery_model(*reference, _training())
    result = predict_recovery(model, *reference)
    assert result["observable"] is True
    assert result["delta_pulses"] == [0, 0, 0]
    assert result["reason"] == "at_learned_rgb_goal"


def test_far_rgb_state_fails_closed_instead_of_extrapolating():
    reference = _views()
    model = fit_recovery_model(*reference, _training())
    own = np.full((144, 192, 3), 255, np.uint8)
    top = np.full((192, 256, 3), 255, np.uint8)
    encoded = [cv2.imencode(".png", image)[1].tobytes() for image in (own, top)]
    result = predict_recovery(model, *encoded)
    assert result["observable"] is False
    assert result["delta_pulses"] == [0, 0, 0]


def test_rank_deficient_labels_and_nonfinite_models_are_rejected_or_closed():
    reference = _views()
    bad = []
    for state in ((-3,0,0),(-1,0,0),(1,0,0),(3,0,0),(0,-2,0),(0,2,0)):
        own, top = _views(state)
        bad.append({"own_jpeg": own, "top_jpeg": top,
                    "correction_pulses": [-state[0] * 10, 0, 0]})
    model = fit_recovery_model(*reference, bad)
    result = predict_recovery(model, *_views((1, 0, 0)))
    assert model["diagnostics"]["rank"] < 3
    assert result["observable"] is False
    assert result["delta_pulses"] == [0, 0, 0]
    model = fit_recovery_model(*reference, _training())
    model["kernel_alpha"][0][0] = float("nan")
    with pytest.raises(ValueError):
        predict_recovery(model, *reference)


def test_invalid_sample_shape_is_rejected():
    reference = _views()
    with pytest.raises(ValueError):
        fit_recovery_model(*reference, [{"own_jpeg": reference[0],
                                         "top_jpeg": reference[1],
                                         "correction_pulses": [1, 2]}])


def test_case_ids_select_grouped_four_fold_without_changing_public_prediction():
    reference = _views()
    rows = _training()
    grouped = [{**row, "case_id": f"case-{index // 2}"} for index, row in enumerate(rows)]
    model = fit_recovery_model(*reference, grouped)
    assert model["diagnostics"]["hyperparameter_selection"] == "group_4fold"
    assert np.isfinite(model["diagnostics"]["cross_validation_mse"])
    assert predict_recovery(model, *reference)["delta_pulses"] == [0, 0, 0]


def test_partial_case_ids_are_rejected_instead_of_leaking_frames():
    reference = _views()
    rows = _training()
    rows[0]["case_id"] = "only-one"
    with pytest.raises(ValueError, match="all samples require"):
        fit_recovery_model(*reference, rows)
