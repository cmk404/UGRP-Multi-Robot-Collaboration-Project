import json

import cv2
import numpy as np
import pytest

from harness.camera_varied_start_geometry import (fit_geometry_model,
                                                   predict_geometry)


def jpeg(image):
    ok, encoded = cv2.imencode(".jpg", image)
    assert ok
    return encoded.tobytes()


def scene(x, y, *, outside=False):
    image = np.full((720, 960, 3), 65, np.uint8)
    cv2.rectangle(image, (x - 22, y - 14), (x + 22, y + 14), (0, 220, 255), -1)
    if outside:
        cv2.rectangle(image, (20, 20), (220, 140), (255, 0, 255), -1)
    return jpeg(image)


def samples():
    result = []
    for index in range(20):
        x = 290 + index * 11
        result.append({"top_jpeg": scene(x, 430), "error": (455 - x) / 1000,
                       "case_id": f"case-{index}"})
    return result


@pytest.fixture(scope="module")
def model():
    return fit_geometry_model(scene(455, 430), samples(), "r1", "forward")


def test_prediction_is_local_to_fixed_robot_roi(model):
    plain = predict_geometry(model, scene(390, 430))
    changed_elsewhere = predict_geometry(model, scene(390, 430, outside=True))
    assert plain["ok"] and changed_elsewhere["ok"]
    assert plain["error"] == pytest.approx(changed_elsewhere["error"], abs=1e-12)
    assert plain["error"] > predict_geometry(model, scene(500, 430))["error"]


def test_json_roundtrip_preserves_standalone_prediction(model):
    restored = json.loads(json.dumps(model, allow_nan=False))
    first = predict_geometry(model, scene(420, 430))
    second = predict_geometry(restored, scene(420, 430))
    assert second == first
    assert np.isfinite(second["error"])
    assert restored["diagnostics"]["case_count"] == 20


def test_training_is_deterministic():
    first = fit_geometry_model(scene(455, 430), samples(), "r1", "lateral")
    second = fit_geometry_model(scene(455, 430), list(reversed(samples())), "r1", "lateral")
    assert json.dumps(first, sort_keys=True) == json.dumps(second, sort_keys=True)


def test_malformed_inputs_and_models_are_rejected(model):
    with pytest.raises(ValueError):
        fit_geometry_model(b"", samples(), "r1", "forward")
    with pytest.raises(ValueError):
        fit_geometry_model(scene(455, 430), [], "r1", "forward")
    with pytest.raises(ValueError):
        fit_geometry_model(scene(455, 430), samples(), "r1", "yaw")
    with pytest.raises(ValueError):
        predict_geometry(model, b"not an image")
    broken = json.loads(json.dumps(model))
    broken["trees"][0]["left"] = []
    with pytest.raises(ValueError):
        predict_geometry(broken, scene(420, 430))

    cycle = json.loads(json.dumps(model))
    cycle["trees"][0]["left"][0] = 0
    cycle["trees"][0]["right"][0] = 0
    with pytest.raises(ValueError, match="cycle"):
        predict_geometry(cycle, scene(420, 430))

    nonfinite = json.loads(json.dumps(model))
    nonfinite["ood_nearest_limit"] = float("nan")
    with pytest.raises(ValueError):
        predict_geometry(nonfinite, scene(420, 430))


def test_far_foreground_is_rejected_by_learned_support(model):
    result = predict_geometry(model, scene(570, 500))
    assert not result["ok"]
    assert result["error"] == 0.0
