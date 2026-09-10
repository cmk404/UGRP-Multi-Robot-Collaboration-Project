import json

import cv2
import numpy as np
import pytest

from harness.camera_teacher_student import (
    encode_views, fit_visual_jacobian, predict_correction,
)


def jpeg(image):
    ok, encoded = cv2.imencode(".png", image)
    assert ok
    return encoded.tobytes()


def views(delta=(0, 0, 0)):
    own = np.zeros((144, 192, 3), np.uint8)
    top = np.zeros((192, 256, 3), np.uint8)
    a, b, c = map(int, delta)
    cv2.rectangle(own, (35 + a, 50), (49 + a, 64), (220, 220, 220), -1)
    cv2.rectangle(own, (120, 90 + b), (136, 106 + b), (150, 150, 150), -1)
    cv2.rectangle(top, (110 + c, 75), (120 + c, 89), (255, 255, 255), -1)
    return jpeg(own), jpeg(top)


def samples(columns=((4, 0, 0), (0, 4, 0), (0, 0, 4))):
    result = []
    for channel, shift in enumerate(columns):
        for sign in (-1, 1):
            pulse = [0, 0, 0]
            pulse[channel] = sign * 50
            own, top = views(tuple(sign * x for x in shift))
            result.append({"own_jpeg": own, "top_jpeg": top, "delta_pulses": pulse})
    return result


def test_encode_views_is_deterministic_and_top_is_explicitly_weighted():
    own, top = views()
    first = encode_views(own, top)
    second = encode_views(own, top)
    assert first.shape == (2 * 96 * 72 + 2 * 128 * 96,)
    assert np.array_equal(first, second)
    assert np.max(first[2 * 96 * 72:]) > np.max(first[:2 * 96 * 72])


def test_fit_is_json_serializable_and_contains_no_teacher_pose():
    own, top = views()
    model = fit_visual_jacobian(own, top, samples())
    json.dumps(model, allow_nan=False)
    assert model["diagnostics"]["rank"] == 3
    assert set(model) == {"schema", "channels", "feature_spec", "reference_features",
                          "feature_derivative", "diagnostics"}
    assert "goal_pulses" not in json.dumps(model)


def test_heldout_signed_offset_produces_bounded_opposite_correction():
    own, top = views()
    model = fit_visual_jacobian(own, top, samples())
    shifted_own, shifted_top = views((2, -2, 2))
    result = predict_correction(model, shifted_own, shifted_top, max_step=25)
    assert result["observable"] is True
    assert result["rank"] == 3
    assert result["delta_pulses"][0] < 0
    assert result["delta_pulses"][1] > 0
    assert result["delta_pulses"][2] < 0
    assert max(map(abs, result["delta_pulses"])) <= 25
    assert result["predicted_remaining_error"] < result["feature_error"]


def test_rank_deficiency_fails_closed():
    own, top = views()
    model = fit_visual_jacobian(own, top, samples(((4, 0, 0), (4, 0, 0), (0, 0, 4))))
    result = predict_correction(model, own, top)
    assert model["diagnostics"]["rank"] < 3
    assert result["observable"] is False
    assert result["delta_pulses"] == [0, 0, 0]


def test_invalid_images_shapes_and_nonfinite_models_are_rejected():
    own, top = views()
    with pytest.raises(ValueError):
        encode_views(b"bad", top)
    with pytest.raises(ValueError):
        fit_visual_jacobian(own, top, [{"own_jpeg": own, "top_jpeg": top,
                                       "delta_pulses": [1, 2]}])
    model = fit_visual_jacobian(own, top, samples())
    model["feature_derivative"][0][0] = float("nan")
    with pytest.raises(ValueError):
        predict_correction(model, own, top)
