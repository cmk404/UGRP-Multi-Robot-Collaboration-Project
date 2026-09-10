import json

import cv2
import numpy as np
import pytest

from harness.camera_varied_start_student import (STAGES, TOP_ROIS,
    fit_stage_model, predict_stage)
from scripts.train_camera_varied_start_student import _balanced, _validate_pair


def _views(rid="r1", state=0, peer=0):
    own = np.zeros((180, 240, 3), np.uint8)
    top = np.zeros((720, 960, 3), np.uint8)
    cv2.rectangle(own, (80 + state, 55), (105 + state, 85), (180, 220, 240), -1)
    x1, y1, x2, y2 = TOP_ROIS[rid]
    cv2.circle(top, ((x1 + x2) // 2 + state * 3, (y1 + y2) // 2 + state),
               12, (240, 240, 240), -1)
    other = "r3" if rid == "r1" else "r1"
    ox1, oy1, ox2, oy2 = TOP_ROIS[other]
    cv2.rectangle(top, (ox1 + 20 + peer, oy1 + 20),
                  (ox1 + 45 + peer, oy1 + 45), (255, 120, 30), -1)
    encoded = []
    for image in (own, top):
        ok, value = cv2.imencode(".png", image); assert ok
        encoded.append(value.tobytes())
    return tuple(encoded)


def _samples(rid="r1", stage="yaw"):
    rows = []
    for case in range(5):
        for state in (-3, -2, -1, 0, 1, 2, 3):
            own, top = _views(rid, state, peer=case * 2)
            rows.append({"own_jpeg": own, "top_jpeg": top,
                "command": 0.0 if state == 0 else state * .01,
                "ready": state == 0, "case_id": f"case-{case}"})
    return rows


@pytest.mark.parametrize("stage", STAGES)
def test_stage_model_is_signed_bounded_and_goal_ready(stage):
    goal = _views("r1", 0)
    model = fit_stage_model(*goal, _samples(stage=stage), "r1", stage)
    json.dumps(model, allow_nan=False)
    left = predict_stage(model, *_views("r1", -2))
    right = predict_stage(model, *_views("r1", 2))
    assert left["ok"] and right["ok"]
    assert left["command"] < 0 < right["command"]
    ready = predict_stage(model, *goal)
    assert ready["ready"] and ready["command"] == 0 and ready["ready_score"] == 1


def test_robot_roi_ignores_pixels_in_peer_lane():
    goal = _views("r1", 0)
    model = fit_stage_model(*goal, _samples(), "r1", "yaw")
    own, top_a = _views("r1", 2, peer=0)
    _, top_b = _views("r1", 2, peer=90)
    a, b = predict_stage(model, own, top_a), predict_stage(model, own, top_b)
    assert a["command"] == pytest.approx(b["command"], abs=1e-12)
    assert a["diagnostics"] == pytest.approx(b["diagnostics"], abs=1e-12)


def test_unseen_rgb_fails_closed():
    goal = _views("r3", 0)
    model = fit_stage_model(*goal, _samples("r3"), "r3", "lateral")
    white = np.full((720, 960, 3), 255, np.uint8)
    ok, encoded = cv2.imencode(".png", white); assert ok
    result = predict_stage(model, encoded.tobytes(), encoded.tobytes())
    assert result["ok"] is False
    assert result["command"] == 0.0 and result["ready"] is False


def test_full_domain_residual_accepts_discarded_valid_rgb_without_more_support():
    goal = _views("r1", 0)
    selected = _samples()
    own, top = _views("r1", 2)
    image = cv2.imdecode(np.frombuffer(own, np.uint8), cv2.IMREAD_COLOR)
    cv2.rectangle(image, (4, 4), (15, 15), (20, 100, 240), -1)
    ok, encoded = cv2.imencode(".png", image); assert ok
    discarded = {"own_jpeg": encoded.tobytes(), "top_jpeg": top,
                 "command": .02, "ready": False, "case_id": "domain-extra"}
    narrow = fit_stage_model(*goal, selected, "r1", "yaw")
    wide = fit_stage_model(*goal, selected, "r1", "yaw",
                           domain_samples=selected + [discarded])
    assert predict_stage(narrow, discarded["own_jpeg"], top)["ok"] is False
    assert predict_stage(wide, discarded["own_jpeg"], top)["ok"] is True
    assert wide["diagnostics"]["support_count"] == narrow["diagnostics"]["support_count"]


def test_balancing_preserves_ready_nearzero_and_every_case():
    rows = []
    for case in range(4):
        for index in range(8):
            rows.append({"sample_id": f"{case}-{index}", "case_id": f"case-{case}",
                         "command": 0.0 if index == 7 else .02, "ready": index == 7})
    selected = _balanced(rows, cap=12)
    assert len(selected) == 12
    assert sum(row["ready"] for row in selected) == 4
    assert {row["case_id"] for row in selected} == {f"case-{i}" for i in range(4)}


def test_all_declared_rows_are_validated_even_if_excluded():
    actor = {"case_id": "excluded", "robot_id": "r1", "stage": "yaw"}
    label = {"case_id": "excluded", "robot_id": "r3", "stage": "yaw",
             "command": 0.0, "ready": True}
    with pytest.raises(ValueError, match="robot_id mismatch"):
        _validate_pair(actor, label, {"excluded"})
