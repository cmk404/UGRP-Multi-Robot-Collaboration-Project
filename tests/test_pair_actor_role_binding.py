"""Bind pair model slots to each actor's own-motion TOP evidence."""

import base64
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
import pytest

from harness.rgb_skill_execution import PairActorSkill, RGBSkillUnsupported
from sim.research_dispatch_arena import authored_map


FIXTURES = Path(__file__).parent / "fixtures"
REFERENCE = (FIXTURES / "camera_goal_transport/reference-top.jpg").read_bytes()
SOURCE = FIXTURES / "pair_role_binding/v5-first-coarse-top.jpg"
PROVENANCE = json.loads((SOURCE.parent / "provenance.json").read_text())[
    SOURCE.name]
TOP = SOURCE.read_bytes()
SPEC = {"object_id": "beam", "route": "north", "dock": "dock_a"}


def _actor(robot, role, *, center=None, reference=REFERENCE):
    saved = {"initialization_replay": [{"targets": {}}]}
    assets = (saved, {"r1": {}, "r3": {}}, {"r1": {}, "r3": {}}, reference)
    actor = PairActorSkill(robot, role, authored_map("open"), SPEC, assets)
    actor.index = actor.phases.index("coarse")
    actor.identity_claim = {"valid": True, "fresh": True,
                            "center": center or PROVENANCE["own_motion_claims"][robot]}
    return actor


def _own_stub():
    # Coarse uses only TOP pixels and the previously isolated own-motion claim.
    return {"image": base64.b64encode(REFERENCE).decode(),
            "actuator_state": {"servo_pulses": {}}}


@pytest.mark.parametrize("robot,role,expected_side,observed_side", [
    ("r1", "lower", "lower", "upper"),
    ("r3", "upper", "upper", "lower"),
])
def test_v5_declared_roles_conflict_with_own_rgb_and_stop_before_motion(
        robot, role, expected_side, observed_side):
    assert hashlib.sha256(TOP).hexdigest() == PROVENANCE["sha256"]
    actor = _actor(robot, role)
    with pytest.raises(RGBSkillUnsupported, match="pair_role_image_side_mismatch") as raised:
        actor.decide(_own_stub(), TOP)
    assert raised.value.phase == "coarse"
    support = raised.value.diagnostics["role_support"]
    assert support["expected_side"] == expected_side
    assert support["own_claim_side"] == support["own_wheel_side"] == observed_side
    assert min(raised.value.diagnostics["prediction"]["heading"]["corner_pixels"]) >= 5
    assert actor.coarse is None


@pytest.mark.parametrize("robot,role,side", [
    ("r1", "upper", "upper"),
    ("r3", "lower", "lower"),
])
def test_v5_motion_bound_roles_match_the_saved_model_sides(robot, role, side):
    decision = _actor(robot, role).decide(_own_stub(), TOP)
    prediction = decision["evidence"]["prediction"]
    support = prediction["role_image_support"]
    assert decision["phase"] == "coarse"
    assert prediction["ok"] is True
    assert support["expected_side"] == support["own_claim_side"] == support["own_wheel_side"] == side
    assert min(prediction["heading"]["corner_pixels"]) >= 5
    if robot == "r3":
        assert decision["action"]["forward"] > 0
        assert decision["action"]["left"] == 0
    else:
        assert decision["action"]["forward"] == 0
        assert decision["action"]["left"] < 0


def test_role_binding_depends_on_own_pixels_instead_of_physical_robot_id():
    upper_claim = PROVENANCE["own_motion_claims"]["r1"]
    lower_claim = PROVENANCE["own_motion_claims"]["r3"]
    for robot, role, center in (("renamed-upper", "upper", upper_claim),
                                ("renamed-lower", "lower", lower_claim)):
        decision = _actor(robot, role, center=center).decide(_own_stub(), TOP)
        assert decision["evidence"]["prediction"]["ok"] is True


def test_wheel_rectangle_straddling_visible_beam_is_ambiguous_even_with_four_corners():
    frame = cv2.imdecode(np.frombuffer(TOP, np.uint8), cv2.IMREAD_COLOR)
    wheel_patch = frame[105:190, 65:160].copy()
    frame[105:190, 65:160] = 0
    frame[223:308, 65:160] = wheel_patch
    moved = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 95])[1].tobytes()
    claim = PROVENANCE["own_motion_claims"]["r1"]
    actor = _actor("r1", "upper", center=[claim[0], claim[1] + 118 / 720])
    with pytest.raises(RGBSkillUnsupported, match="pair_role_image_side_mismatch") as raised:
        actor.decide(_own_stub(), moved)
    assert raised.value.diagnostics["role_support"]["own_wheel_side"] == "ambiguous"
    assert min(raised.value.diagnostics["prediction"]["heading"]["corner_pixels"]) >= 5


def test_missing_wheel_corner_cannot_become_role_evidence():
    frame = cv2.imdecode(np.frombuffer(TOP, np.uint8), cv2.IMREAD_COLOR)
    frame[115:144, 75:112] = 0
    damaged = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 95])[1].tobytes()
    with pytest.raises(RGBSkillUnsupported, match="own_wheel_heading_unresolved") as raised:
        _actor("r1", "upper").decide(_own_stub(), damaged)
    assert raised.value.diagnostics["prediction"]["ok"] is False


def test_missing_reference_lane_has_its_own_failure_reason():
    frame = cv2.imdecode(np.frombuffer(REFERENCE, np.uint8), cv2.IMREAD_COLOR)
    frame[220:295, 450:530] = 0
    missing_upper_lane = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 95])[1].tobytes()
    with pytest.raises(RGBSkillUnsupported, match="reference_lane_unresolved") as raised:
        _actor("r1", "upper", reference=missing_upper_lane).decide(_own_stub(), TOP)
    assert raised.value.reason == "reference_lane_unresolved"
    assert raised.value.diagnostics["prediction"]["ok"] is False
