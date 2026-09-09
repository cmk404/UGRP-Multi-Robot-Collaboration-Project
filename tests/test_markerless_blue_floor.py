"""Regression coverage from the seed-45 blue-floor release observation."""

import base64
import hashlib
import json
from pathlib import Path

import pytest

from harness.markerless_box import observe_ground_box
from harness import visual_box_skill as skill_module


FIXTURE_DIR = Path(__file__).parent / "fixtures" / "markerless_box" / "blue_floor_release"
METADATA = json.loads((FIXTURE_DIR / "metadata.json").read_text())


def _payload(frame):
    raw = (FIXTURE_DIR / frame["file"]).read_bytes()
    assert hashlib.sha256(raw).hexdigest() == frame["source_sha256"]
    return raw, base64.b64encode(raw).decode()


@pytest.mark.parametrize("frame", METADATA["frames"], ids=lambda frame: f"step-{frame['step']}")
def test_released_cuboid_remains_single_floor_candidate_over_blue_background(frame):
    """The floor must not multiply or merge the one projection-valid cargo candidate."""
    _, encoded = _payload(frame)

    result = observe_ground_box(encoded, frame["own_pwm"])

    assert result["visible"] is True
    assert result["reason"] == "FLOOR_CUBOID_HYPOTHESIS_VALIDATED"
    assert result["floor_hypothesis_projection_iou"] >= 0.58
    assert result.get("candidate_count") is None


def test_actual_valid_release_frame_can_start_controlled_ground_sweep():
    """One saved fit starts verification; it does not itself prove the three-view sweep."""
    frame = METADATA["frames"][0]
    raw, encoded = _payload(frame)
    skill = skill_module.VisualBoxSkill()
    skill.phase = "verify_release"
    skill._grasp = {int(key): value for key, value in METADATA["release_grasp_own_pwm"].items()}
    skill.held = True
    observation = {
        "robot_id": "r1",
        "frame_id": frame["step"],
        "sim_time": frame["time_s"],
        "camera": "robot_cam",
        "image": encoded,
        "sha256": hashlib.sha256(raw).hexdigest(),
        "actuator_state": {"servo_pulses": frame["own_pwm"]},
    }

    action = skill.decide(observation)

    assert skill.phase == "release_ground_left"
    assert action == {"kind": "pose", "pulses": {6: frame["own_pwm"]["6"] + 60}}
    assert skill.reason == "RUNNING"
