"""Recorded actor RGB at the v7 coarse-to-yaw boundary."""

import base64
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from harness.camera_varied_start_heading import predict_heading
from harness.camera_varied_start_student import predict_stage
from harness.dispatch_skill_binding import PairCoarsePixels, canonical_pair_top
from harness.rgb_skill_execution import PairActorSkill, RGBSkillUnsupported
from sim.research_dispatch_arena import authored_map


FIXTURE = Path(__file__).parent / "fixtures/pair_yaw_beam_overlap"
PROVENANCE = json.loads((FIXTURE / "provenance.json").read_text())
TOP = (FIXTURE / "r1-000368-top_rgb.jpg").read_bytes()
PREVIOUS_TOP = (FIXTURE / "r1-000367-top_rgb.jpg").read_bytes()
REFERENCE = (Path(__file__).parent / "fixtures/camera_goal_transport/reference-top.jpg").read_bytes()
MODELS = {slot: json.loads((FIXTURE / f"model-{slot}-yaw.json").read_text())
          for slot in ("r1", "r3")}
# These are approximate centers read from this TOP image, only to seed the
# same own-motion/continuity selector used by PairActorSkill. The selector
# must supply fresh, complete four-corner wheel evidence before heading runs.
CLAIM_CENTERS = {"r1": (219, 358), "r3": (216, 166)}


def _selector(slot):
    x, y = CLAIM_CENTERS[slot]
    identity = {slot: {"claim": {"valid": True, "fresh": True,
                                 "center": [x / 960, y / 720]}}}
    return PairCoarsePixels(identity, SimpleNamespace(pair={slot: slot}), REFERENCE)


def _actor(slot):
    saved = {"initialization_replay": [{"targets": {}}]}
    assets = (saved, {"r1": {}, "r3": {}},
              {s: {"yaw": MODELS[s]} for s in ("r1", "r3")}, REFERENCE)
    actor = PairActorSkill(slot, {"r1": "lower", "r3": "upper"}[slot],
                           authored_map("open"),
                           {"object_id": "beam", "route": "north", "dock": "dock_a"}, assets)
    actor.index = actor.phases.index("yaw")
    x, y = CLAIM_CENTERS[slot]
    actor.identity_claim = {"valid": True, "fresh": True,
                            "center": [x / 960, y / 720]}
    actor.coarse = _selector(slot)
    assert actor.coarse.decide(PREVIOUS_TOP, slot)["ok"]
    return actor


def _own():
    return {"image": base64.b64encode(REFERENCE).decode(),
            "actuator_state": {"servo_pulses": {}}}


@pytest.mark.parametrize("slot", ["r1", "r3"])
def test_recorded_yaw_boundary_uses_fresh_own_wheel_anchor(slot):
    for name, item in PROVENANCE["files"].items():
        assert hashlib.sha256((FIXTURE / name).read_bytes()).hexdigest() == item["sha256"]
    canonical, transform = canonical_pair_top(TOP, REFERENCE)
    model = MODELS[slot]
    assert not predict_heading(model["geometry"], canonical)["ok"]
    wheels = _selector(slot).decide(TOP, slot)
    assert wheels["ok"] and min(wheels["heading"]["corner_pixels"]) >= 5
    center = [a + b for a, b in zip(wheels["wheel_center_px"],
                                     transform["translation_px"])]
    prediction = predict_stage(model, b"own RGB unused by yaw", canonical,
                               wheel_center_px=center)
    assert prediction["ok"] and prediction["precision"] == "fine"
    assert prediction["diagnostics"]["method"] == "soft_yellow"
    assert prediction["diagnostics"]["ecc_score"] >= .70
    decision = _actor(slot).decide(_own(), TOP)
    support = decision["evidence"]["wheel_anchor"]["role_image_support"]
    assert support["expected_side"] == support["own_claim_side"] == support["own_wheel_side"]
    assert decision["evidence"]["prediction"]["ok"]
    assert decision["evidence"]["prediction"]["precision"] == "fine"


@pytest.mark.parametrize("slot", ["r1", "r3"])
def test_fine_heading_rejects_lost_wheel_corner(slot):
    frame = cv2.imdecode(np.frombuffer(TOP, np.uint8), cv2.IMREAD_COLOR)
    if slot == "r1":
        frame[335:354, 190:211] = 0
    else:
        frame[140:159, 182:205] = 0
    damaged = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 95])[1].tobytes()
    actor = _actor(slot)
    with pytest.raises(RGBSkillUnsupported, match="own_wheel_heading_unresolved"):
        actor.decide(_own(), damaged)


def test_heading_anchor_outside_calibrated_roi_is_rejected():
    canonical, _ = canonical_pair_top(TOP, REFERENCE)
    with pytest.raises(ValueError, match="outside heading ROI"):
        predict_heading(MODELS["r1"]["geometry"], canonical,
                        wheel_center_px=[0, 0])


def test_fine_heading_rechecks_saved_role_side_before_command():
    actor = _actor("r1")
    actor.identity_claim["center"][1] = 160 / 720
    with pytest.raises(RGBSkillUnsupported, match="pair_role_image_side_mismatch") as raised:
        actor.decide(_own(), TOP)
    support = raised.value.diagnostics["role_support"]
    assert support["expected_side"] == support["own_wheel_side"] == "lower"
    assert support["own_claim_side"] == "upper"
