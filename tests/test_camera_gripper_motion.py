import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
import pytest

from harness.camera_gripper_motion import GripperMotionTracker


SATELLITE_FIXTURE = Path(__file__).parent / "fixtures" / "gripper_motion_satellite"


def _jpeg(image):
    ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 96])
    assert ok
    return encoded.tobytes()


def _scene(closed=False):
    image = np.full((240, 320, 3), 35, np.uint8)
    rng = np.random.default_rng(7)
    image[:] += rng.integers(0, 18, image.shape, dtype=np.uint8)
    centers = ((126, 120), (194, 120)) if not closed else ((145, 120), (175, 120))
    for x, y in centers:
        patch = rng.integers(60, 235, (20, 14, 3), dtype=np.uint8)
        image[y - 10:y + 10, x - 7:x + 7] = patch
    return image


def _calibrated(max_track_age=4):
    tracker = GripperMotionTracker(max_track_age=max_track_age)
    assert not tracker.update(_jpeg(_scene(False)), None)["valid"]
    result = tracker.update(
        _jpeg(_scene(True)), {"kind": "arm", "servo_id": 1, "pulse": 1500}
    )
    assert result["valid"], result
    return tracker, result


def test_isolated_gripper_change_initializes_without_coordinates_or_simulator_state():
    _, result = _calibrated()
    assert result["source"] == "isolated_gripper_motion"
    assert result["tracked_points"] >= 3
    assert result["center"] == pytest.approx([0.5, 120 / 240], abs=0.04)
    assert abs(result["opening_axis"][0]) > 0.9
    assert result["span_px"] >= 20
    assert 0 < result["confidence"] <= 1


def test_reinitializes_consistently_on_reverse_open_motion():
    tracker, closed = _calibrated()
    opened = tracker.update(
        _jpeg(_scene(False)), {"kind": "arm", "servo_id": 1, "pulse": 2000}
    )
    assert opened["valid"], opened
    assert opened["center"] == pytest.approx(closed["center"], abs=0.02)
    assert abs(np.dot(opened["opening_axis"], closed["opening_axis"])) > 0.95


def test_tracks_translation_and_rotation_from_only_calibrated_motion_pixels():
    tracker, calibrated = _calibrated()
    matrix = cv2.getRotationMatrix2D((160, 120), 7.0, 1.0)
    matrix[:, 2] += (9, -6)
    transformed = cv2.warpAffine(_scene(True), matrix, (320, 240), borderValue=(35, 35, 35))
    result = tracker.update(_jpeg(transformed), {"kind": "look", "pan_pulse": 1550})
    assert result["valid"], result
    original = np.array([calibrated["center"][0] * 320, calibrated["center"][1] * 240, 1.0])
    expected = matrix @ original
    actual = np.array([result["center"][0] * 320, result["center"][1] * 240])
    assert actual == pytest.approx(expected, abs=2.0)
    angle = np.deg2rad(-7.0)
    expected_axis = np.array([[np.cos(angle), -np.sin(angle)],
                              [np.sin(angle), np.cos(angle)]]) @ np.array(calibrated["opening_axis"])
    assert abs(float(np.dot(expected_axis, result["opening_axis"]))) > 0.97
    assert result["tracked_points"] >= 3
    assert result["confidence"] < calibrated["confidence"]


def test_no_pixel_change_cannot_fake_calibration():
    tracker = GripperMotionTracker()
    frame = _jpeg(_scene(False))
    tracker.update(frame, None)
    result = tracker.update(frame, {"kind": "arm", "servo_id": 1, "pulse": 1500})
    assert not result["valid"]
    assert result["confidence"] == 0
    assert "no measurable" in result["reason"]


def test_low_contrast_jpeg_bridge_is_split_by_corresponding_high_contrast_lobes():
    rng = np.random.default_rng(18)
    before = np.full((240, 320, 3), 45, np.uint8)
    before += rng.integers(0, 5, before.shape, dtype=np.uint8)
    after = before.copy()
    # Two textured vertical motion lobes with a weaker JPEG-like connecting
    # trace: threshold 15 is one component, stronger support separates them.
    after[91:101, 157:164] = np.clip(
        after[91:101, 157:164].astype(np.int16) + 42, 0, 255
    ).astype(np.uint8)
    after[111:121, 157:164] = np.clip(
        after[111:121, 157:164].astype(np.int16) + 42, 0, 255
    ).astype(np.uint8)
    after[101:111, 160:161] = np.clip(
        after[101:111, 160:161].astype(np.int16) + 18, 0, 255
    ).astype(np.uint8)
    tracker = GripperMotionTracker()
    tracker.update(_jpeg(before), None)
    result = tracker.update(
        _jpeg(after), {"kind": "arm", "servo_id": 1, "pulse": 1500}
    )
    assert result["valid"], result
    assert abs(result["opening_axis"][1]) > 0.9
    assert result["span_px"] >= 15
    assert result["tracked_points"] >= 3


def test_low_contrast_off_axis_clutter_cannot_override_rotated_lobe_consensus():
    rng = np.random.default_rng(28)
    before = np.full((240, 320, 3), 45, np.uint8)
    before += rng.integers(0, 5, before.shape, dtype=np.uint8)
    after = before.copy()
    for y in (92, 114):
        after[y:y + 9, 157:165] = np.clip(
            after[y:y + 9, 157:165].astype(np.int16) + 45, 0, 255
        ).astype(np.uint8)
    # A meaningful-size off-axis component is present at the base threshold,
    # but lacks repeated stronger-contrast support across the image pair.
    after[102:109, 136:146] = np.clip(
        after[102:109, 136:146].astype(np.int16) + 18, 0, 255
    ).astype(np.uint8)
    matrix = cv2.getRotationMatrix2D((161, 106), -11.0, 1.0)
    rotated_before = cv2.warpAffine(before, matrix, (320, 240), borderValue=(45, 45, 45))
    rotated_after = cv2.warpAffine(after, matrix, (320, 240), borderValue=(45, 45, 45))
    tracker = GripperMotionTracker()
    tracker.update(_jpeg(rotated_before), None)
    result = tracker.update(
        _jpeg(rotated_after), {"kind": "arm", "servo_id": 1, "pulse": 1500}
    )
    assert result["valid"], result
    expected_axis = np.array([-np.sin(np.deg2rad(11)), np.cos(np.deg2rad(11))])
    assert abs(float(np.dot(expected_axis, result["opening_axis"]))) > 0.95
    assert result["span_px"] >= 15


def test_meaningful_base_threshold_satellite_cannot_rotate_consensus_axis():
    rng = np.random.default_rng(38)
    before = np.full((240, 320, 3), 42, np.uint8)
    before += rng.integers(0, 6, before.shape, dtype=np.uint8)
    after = before.copy()
    for x in (145, 174):
        after[116:125, x:x + 10] = np.clip(
            after[116:125, x:x + 10].astype(np.int16) + 48, 0, 255
        ).astype(np.uint8)
    # This off-axis patch is meaningful at threshold 15, but disappears at
    # stronger thresholds while both real horizontal lobes persist.
    after[137:143, 145:151] = np.clip(
        after[137:143, 145:151].astype(np.int16) + 18, 0, 255
    ).astype(np.uint8)
    tracker = GripperMotionTracker()
    tracker.update(_jpeg(before), None)
    result = tracker.update(
        _jpeg(after), {"kind": "arm", "servo_id": 1, "pulse": 1500}
    )
    assert result["valid"], result
    assert abs(result["opening_axis"][0]) > 0.97
    assert abs(result["opening_axis"][1]) < 0.2


def test_actual_satellite_frames_retain_horizontal_close_and_open_axes():
    provenance = json.loads((SATELLITE_FIXTURE / "provenance.json").read_text())
    frames = {}
    for item in provenance["files"]:
        raw = (SATELLITE_FIXTURE / item["file"]).read_bytes()
        assert hashlib.sha256(raw).hexdigest() == item["sha256"]
        frames[item["file"]] = raw
    tracker = GripperMotionTracker(initial_gripper_pulse=2000)
    assert not tracker.update(frames["194-top.jpg"], None)["valid"]
    closed = tracker.update(frames["195-top.jpg"], provenance["actions_between_frames"][0])
    opened = tracker.update(frames["196-top.jpg"], provenance["actions_between_frames"][1])
    assert closed["valid"], closed
    assert opened["valid"], opened
    assert abs(closed["opening_axis"][0]) > 0.95
    assert abs(opened["opening_axis"][0]) > 0.95


def test_broad_whole_image_motion_is_rejected():
    tracker = GripperMotionTracker()
    first = _scene(False)
    second = np.roll(first, 18, axis=1)
    tracker.update(_jpeg(first), None)
    result = tracker.update(
        _jpeg(second), {"kind": "arm", "servo_id": 1, "pulse": 1500}
    )
    assert not result["valid"]
    assert result["confidence"] == 0


def test_tracking_expires_and_blank_occlusion_is_rejected_instead_of_guessed():
    tracker, calibrated = _calibrated(max_track_age=2)
    same = _jpeg(_scene(True))
    first = tracker.update(same, None)
    second = tracker.update(same, None)
    assert first["valid"] and second["valid"]
    assert first["confidence"] == pytest.approx(calibrated["confidence"] - 0.07)
    assert second["confidence"] == pytest.approx(calibrated["confidence"] - 0.14)
    expired = tracker.update(same, None)
    assert not expired["valid"]
    assert "expired" in expired["reason"]

    tracker, _ = _calibrated()
    blank = tracker.update(_jpeg(np.zeros((240, 320, 3), np.uint8)), None)
    assert not blank["valid"]
    assert blank["center"] is None
    assert blank["tracked_points"] == 0


def test_valid_outputs_are_finite_and_bounded():
    _, result = _calibrated()
    assert all(math_value == math_value and 0 <= math_value <= 1 for math_value in result["center"])
    assert all(math_value == math_value and abs(math_value) <= 1 for math_value in result["opening_axis"])
    assert result["span_px"] == result["span_px"] and result["span_px"] > 0
