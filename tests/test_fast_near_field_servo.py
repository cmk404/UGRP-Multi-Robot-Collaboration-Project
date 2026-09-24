"""The opt-in near-field speedup needs fresh, consistent own-RGB evidence."""

import base64
import hashlib
from pathlib import Path

import cv2
import numpy as np
import pytest

from harness import visual_box_skill as module
from harness.solo_box_transport import SoloBoxTransport


_SAVED_APPROACH = Path(__file__).parent / "fixtures/fast_near_field_servo"


def _box(error, *, confidence=.9, x=.28, **overrides):
    result = {
        "target_id": "small_box_01",
        "visible": True,
        "reason": "FLOOR_CUBOID_HYPOTHESIS_VALIDATED",
        "ambiguity_reason": None,
        "identity_source": "task_catalog_reference_only_not_visually_decoded",
        "provenance": module.GROUND_BOX_PROVENANCE,
        "confidence": confidence,
        "estimated_box_center_base_m": [x, 0., .016],
        "pixel_centroid": [320., 218.7 + error],
    }
    result.update(overrides)
    return result


def _own(frame, *, pulse=740, pan=1500, image_id=None):
    image = np.zeros((16, 16, 3), np.uint8)
    image[:, :, 0] = (30 * (frame if image_id is None else image_id)) % 256
    encoded = cv2.imencode(".jpg", image)[1].tobytes()
    return {
        "robot_id": "r2", "frame_id": frame, "sim_time": float(frame),
        "camera": "robot_cam", "image": base64.b64encode(encoded).decode(),
        "sha256": hashlib.sha256(encoded).hexdigest(),
        "actuator_state": {"servo_pulses": {
            "1": 2000, "3": pulse, "4": 2320, "5": 1320, "6": pan}},
    }


def _step(skill, monkeypatch, frame, box, *, pulse=740, pan=1500, image_id=None):
    monkeypatch.setattr(module, "observe_ground_box", lambda *args, **kwargs: box)
    return skill.decide(_own(frame, pulse=pulse, pan=pan, image_id=image_id))


def _skill(enabled=True):
    skill = module.VisualBoxSkill(robot_id="r2", fast_near_field_servo=enabled)
    skill._face_aligner = None  # The vertical servo branch precedes face alignment.
    return skill


def test_opt_in_uses_two_fresh_geometry_supported_views_and_audits_command(monkeypatch):
    skill = _skill()
    first = _step(skill, monkeypatch, 1, _box(80))
    assert first == {"kind": "pose", "pulses": {3: 732}}
    assert skill.last_approach_adjustment["cap_reason"] == "first_supported_frame"
    second = _step(skill, monkeypatch, 2, _box(74), pulse=732)
    assert second == {"kind": "pose", "pulses": {3: 708}}
    audit = skill.last_approach_adjustment
    assert audit["selected_cap_pwm"] == 24
    assert audit["observed_vertical_error_px"] == pytest.approx(74)
    assert audit["previous_observed_vertical_error_px"] == pytest.approx(80)
    assert audit["proposed_delta_pwm"] == -24
    assert audit["command_state_only_not_joint_measurement"] is True
    assert skill.history[-1]["approach_adjustment"] == audit


def test_default_and_solo_constructor_keep_old_eight_pwm_limit(monkeypatch):
    default = _skill(False)
    assert _step(default, monkeypatch, 1, _box(80)) == {"kind": "pose", "pulses": {3: 732}}
    assert _step(default, monkeypatch, 2, _box(74), pulse=732) == {"kind": "pose", "pulses": {3: 724}}
    assert SoloBoxTransport("near_magenta").box.fast_near_field_servo is False
    assert SoloBoxTransport("near_magenta", fast_near_field_servo=True).box.fast_near_field_servo is True
    with pytest.raises(ValueError, match="fast_near_field_servo"):
        module.VisualBoxSkill(fast_near_field_servo=1)


@pytest.mark.parametrize("changed,reason", [
    ({"confidence": .74}, "weak_or_ambiguous_own_rgb"),
    ({"provenance": "unverified_top_rgb"}, "weak_or_ambiguous_own_rgb"),
    ({"target_id": "another_box"}, "weak_or_ambiguous_own_rgb"),
    ({"ambiguity_reason": "TWO_CANDIDATES"}, "weak_or_ambiguous_own_rgb"),
    ({"reason": "UNVALIDATED_COLOR"}, "weak_or_ambiguous_own_rgb"),
    ({"pixel_centroid": [320., 218.7 + 44]}, "near_vertical_tolerance"),
    ({"estimated_box_center_base_m": [.17, 0., .016]}, "final_standoff"),
    ({"estimated_box_center_base_m": [.34, 0., .016]}, "own_rgb_target_discontinuous"),
])
def test_weak_or_close_view_falls_back_to_eight_pwm(monkeypatch, changed, reason):
    skill = _skill()
    _step(skill, monkeypatch, 1, _box(80))
    action = _step(skill, monkeypatch, 2, _box(74, **changed), pulse=732)
    assert action["pulses"][3] == 724
    assert skill.last_approach_adjustment["selected_cap_pwm"] == 8
    assert skill.last_approach_adjustment["cap_reason"] == reason


@pytest.mark.parametrize("error,pulse,image_id,reason", [
    (74, 740, None, "previous_command_state_unconfirmed"),
    (74, 732, 1, "not_a_fresh_rgb_view"),
    (84, 732, None, "vertical_error_grew"),
    (-74, 732, None, "vertical_error_reversed"),
    (79.5, 732, None, "vertical_error_not_improving"),
])
def test_command_or_feedback_disagreement_falls_back(monkeypatch, error, pulse, image_id, reason):
    skill = _skill()
    _step(skill, monkeypatch, 1, _box(80))
    action = _step(skill, monkeypatch, 2, _box(error), pulse=pulse, image_id=image_id)
    assert skill.last_approach_adjustment["selected_cap_pwm"] == 8
    assert skill.last_approach_adjustment["cap_reason"] == reason
    assert abs(action["pulses"][3] - pulse) == 8


def test_target_loss_and_pan_reorientation_break_fast_sequence(monkeypatch):
    skill = _skill()
    _step(skill, monkeypatch, 1, _box(80))
    assert _step(skill, monkeypatch, 2, {"visible": False}, pulse=732)["kind"] == "pose"
    action = _step(skill, monkeypatch, 3, _box(70), pulse=732)
    assert action["pulses"][3] == 724
    assert skill.last_approach_adjustment["cap_reason"] == "first_supported_frame"
    assert _step(skill, monkeypatch, 4, _box(65), pulse=724, pan=1400) == {
        "kind": "pose", "pulses": {6: 1500}}
    action = _step(skill, monkeypatch, 5, _box(60), pulse=724)
    assert action["pulses"][3] == 716


def test_error_reversal_requires_reestablishing_progress(monkeypatch):
    skill = _skill()
    _step(skill, monkeypatch, 1, _box(80))
    _step(skill, monkeypatch, 2, _box(-75), pulse=732)
    third = _step(skill, monkeypatch, 3, _box(-70), pulse=740)
    assert third["pulses"][3] == 748
    assert skill.last_approach_adjustment["cap_reason"] == "reestablishing_visual_progress"
    fourth = _step(skill, monkeypatch, 4, _box(-65), pulse=748)
    assert fourth["pulses"][3] == 772


def test_fiducial_or_direct_scalar_approach_cannot_enter_fast_path():
    skill = module.VisualBoxSkill(perception_mode="fiducial", fast_near_field_servo=True)
    action = skill._approach({"pixel_centroid": [320., 298.7], "confidence": 1.0},
                             np.asarray([.28, 0., .016]),
                             {"1": 2000, "3": 740, "4": 2320, "5": 1320, "6": 1500})
    assert action == {"kind": "pose", "pulses": {3: 732}}
    assert skill.last_approach_adjustment["selected_cap_pwm"] == 8


def test_saved_v26_own_rgb_pair_reaches_opt_in_gate():
    """Two real normalized frames retain the original detector and RGB hash."""
    skill = _skill()
    for index, frame, when, pulse, expected_sha, expected_target in (
        (28, 287, 33.55825000006922, 652,
         "ebff1a1e044a79817f42f4c2bbd6fa05bbc192598f6fdbcab11fb84a35d49b14", 644),
        (29, 297, 34.57150000007406, 644,
         "c1e66df272f359ee9673e075e7b3974d2fa1c8f9ea69d65e77f80c7c8780b618", 620),
    ):
        raw = (_SAVED_APPROACH / f"v26-solo-{index}-own.jpg").read_bytes()
        assert hashlib.sha256(raw).hexdigest() == expected_sha
        observation = {
            "robot_id": "r2", "frame_id": frame, "sim_time": when,
            "camera": "robot_cam", "image": base64.b64encode(raw).decode(),
            "sha256": expected_sha,
            "actuator_state": {"servo_pulses": {
                "1": 2000, "3": pulse, "4": 2320, "5": 1320, "6": 1479}},
        }
        assert skill.decide(observation) == {"kind": "pose", "pulses": {3: expected_target}}
    audit = skill.last_approach_adjustment
    assert audit["cap_reason"] == "two_fresh_consistent_own_rgb_views"
    assert audit["box_confidence"] >= .75
    assert audit["previous_observed_vertical_error_px"] > audit["observed_vertical_error_px"] >= 45
