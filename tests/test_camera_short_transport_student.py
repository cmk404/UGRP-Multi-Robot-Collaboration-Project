import hashlib

import cv2
import numpy as np

from harness.camera_short_transport_student import extract_transport_features, predict_transport
from scripts.train_camera_short_transport_student import checked_image


def jpeg(top=False, payload_x=160, own_fraction=.72):
    image = np.zeros((240, 320, 3), np.uint8)
    image[:] = (45, 45, 45)
    if top:
        cv2.rectangle(image, (payload_x - 35, 112), (payload_x + 35, 128), (0, 110, 240), -1)
        cv2.rectangle(image, (70, 170), (95, 205), (0, 220, 240), -1)
        cv2.rectangle(image, (230, 35), (255, 70), (0, 220, 240), -1)
    else:
        height = int(240 * own_fraction)
        cv2.rectangle(image, (0, 0), (319, height - 1), (0, 110, 240), -1)
    return cv2.imencode(".jpg", image)[1].tobytes()


def model():
    support = {name: [-1, 2] for name in ("top_payload_x", "top_payload_y",
        "top_payload_area", "top_own_x", "top_own_y", "own_orange_fraction", "own_orange_y")}
    return {"schema": "ugrp.camera_short_transport_model.v1", "robot_id": "r1",
            "progress_coefficients": [0, .8, 0],
            "feature_support": support, "held_own_orange_min": .6,
            "ready_progress_m": .19, "slow_progress_m": .16}


def test_features_are_image_only_and_robot_specific():
    result = extract_transport_features(jpeg(False), jpeg(True), "r1")
    assert result["ok"]
    assert set(result["features"]) == {"top_payload_x", "top_payload_y", "top_payload_area",
        "top_own_x", "top_own_y", "own_orange_fraction", "own_orange_y"}
    assert not extract_transport_features(b"bad", jpeg(True), "r1")["ok"]
    assert not extract_transport_features(jpeg(False), jpeg(True), "r2")["ok"]


def test_predict_is_bounded_and_ready_from_anchor_displacement():
    anchor_top = jpeg(True, 140)
    moving = predict_transport(model(), jpeg(False), jpeg(True, 180), jpeg(False), anchor_top)
    assert moving["ok"] and moving["held_estimate"] and not moving["ready"]
    assert 0 < moving["forward"] <= .10
    ready = predict_transport(model(), jpeg(False), jpeg(True, 220), jpeg(False), anchor_top)
    assert ready["ready"] and ready["forward"] == 0


def test_missing_evidence_domain_and_history_fail_closed():
    bad = predict_transport(model(), b"", jpeg(True), jpeg(False), jpeg(True))
    assert not bad["ok"] and bad["forward"] == 0 and not bad["held_estimate"]
    bounded = model(); bounded["feature_support"]["top_payload_x"] = [.4, .6]
    out = predict_transport(bounded, jpeg(False), jpeg(True, 250), jpeg(False), jpeg(True, 140))
    assert not out["ok"] and out["forward"] == 0
    invalid_history = predict_transport(model(), jpeg(False), jpeg(True), jpeg(False), jpeg(True),
                                        [{"kind": "drive", "forward": .1}])
    assert not invalid_history["ok"] and invalid_history["forward"] == 0


def test_low_own_orange_is_not_claimed_held():
    result = predict_transport(model(), jpeg(False, own_fraction=.3), jpeg(True),
                               jpeg(False), jpeg(True))
    assert result["ok"] and not result["held_estimate"] and result["forward"] == 0


def test_issued_commands_cannot_advance_unchanged_rgb():
    own, top = jpeg(False), jpeg(True, 140)
    actions = [{"kind": "drive", "forward": .1, "duration_s": .2}] * 20
    result = predict_transport(model(), own, top, own, top, actions)
    assert result["ok"] and result["diagnostics"]["progress_m"] == 0
    assert not result["ready"] and result["forward"] == .10


def test_malformed_models_fail_closed():
    own, top = jpeg(False), jpeg(True)
    malformed = [None, {**model(), "progress_coefficients": [0, float("nan"), 0]},
        {**model(), "feature_support": {**model()["feature_support"],
                                         "top_payload_x": [0, float("nan")]}}]
    for bad_model in malformed:
        result = predict_transport(bad_model, own, top, own, top)
        assert not result["ok"] and not result["ready"] and result["forward"] == 0


def test_training_image_provenance_rejects_sha_and_symlink_escape(tmp_path):
    image = tmp_path / "frame.jpg"; image.write_bytes(b"jpeg-record")
    digest = hashlib.sha256(image.read_bytes()).hexdigest()
    dataset = {}
    assert checked_image(tmp_path, {"path": "frame.jpg", "sha256": digest}, dataset) == b"jpeg-record"
    try:
        checked_image(tmp_path, {"path": "frame.jpg", "sha256": "0" * 64}, dataset)
        assert False, "mismatched SHA accepted"
    except ValueError:
        pass
    outside = tmp_path.parent / "outside-teacher.jpg"; outside.write_bytes(b"outside")
    link = tmp_path / "escape.jpg"; link.symlink_to(outside)
    try:
        checked_image(tmp_path, {"path": "escape.jpg",
            "sha256": hashlib.sha256(b"outside").hexdigest()}, dataset)
        assert False, "escaping symlink accepted"
    except ValueError:
        pass
