"""Actor-only v6 coarse evidence and bounded v7 continuation checks."""

import base64
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
import pytest

from harness.rgb_skill_execution import (PairActorSkill, PairCoarseProgressBudget,
    RGBSkillUnsupported, PAIR_COARSE_BASE_DECISIONS, PAIR_COARSE_MAX_DECISIONS)
from sim.research_dispatch_arena import authored_map


FIXTURES = Path(__file__).parent / "fixtures"
ROOT = FIXTURES / "pair_coarse_progress"
TRACE = json.loads((ROOT / "v6-actor-coarse-predictions.json").read_text())
PROVENANCE = json.loads((ROOT / "provenance.json").read_text())
TOP = (ROOT / "r1-000270-top_rgb.jpg").read_bytes()
REFERENCE = (FIXTURES / "camera_goal_transport/reference-top.jpg").read_bytes()


def _prediction(image_error, ready=False):
    return {"image_error": image_error, "ready": ready}


def _replay_budget(robot):
    budget = PairCoarseProgressBudget()
    for entry in TRACE[robot]:
        budget.observe(entry)
    assert len(budget.errors_px) == PAIR_COARSE_BASE_DECISIONS
    return budget


def _actor(role="upper"):
    assets = ({"initialization_replay": [{"targets": {}}]},
              {"r1": {}, "r3": {}}, {"r1": {}, "r3": {}}, REFERENCE)
    actor = PairActorSkill("r1", role, authored_map("open"),
        {"object_id": "beam", "route": "north", "dock": "dock_a"}, assets)
    actor.index = actor.phases.index("coarse")
    actor.count = PAIR_COARSE_BASE_DECISIONS
    actor.identity_claim = {"valid": True, "fresh": True,
                            "center": PROVENANCE["own_motion_claim_center"]["r1"]}
    actor.coarse_budget = _replay_budget("r1")
    return actor


def _own():
    return {"image": base64.b64encode(REFERENCE).decode(),
            "actuator_state": {"servo_pulses": {}}}


def test_v6_trace_extends_only_when_upper_image_error_is_falling_or_lower_is_ready():
    assert hashlib.sha256(TOP).hexdigest() == PROVENANCE["top_image"]["sha256"]
    assert hashlib.sha256((ROOT / "v6-actor-coarse-predictions.json").read_bytes()).hexdigest() == PROVENANCE["trace_sha256"]
    assert {robot: len(entries) for robot, entries in TRACE.items()} == {"r1": 170, "r3": 170}
    upper, lower = _replay_budget("r1"), _replay_budget("r3")
    upper_support = upper.observe(TRACE["r1"][-1])
    lower_support = lower.observe(TRACE["r3"][-1])
    assert upper_support["observations"] == lower_support["observations"] == 171
    assert upper_support["window_progress_px"] > 2
    assert "window_progress_px" not in lower_support  # Ready at the barrier.


def test_recorded_next_top_frame_passes_full_role_and_four_corner_gates():
    actor = _actor()
    decision = actor.decide(_own(), TOP)
    prediction = decision["evidence"]["prediction"]
    assert decision["phase"] == "coarse" and prediction["ok"]
    assert prediction["role_image_support"]["expected_side"] == "upper"
    assert min(prediction["heading"]["corner_pixels"]) >= 5
    assert prediction["coarse_budget"]["window_progress_px"] > .5
    assert decision["action"]["kind"] == "mecanum"
    assert decision["action"]["forward"] == 0


def test_synthetic_continuation_can_finish_lateral_then_forward_within_finite_budget():
    # This continuation is a budget test, not an observed physical outcome.
    upper = _replay_budget("r1")
    lower = _replay_budget("r3")
    for step in range(1, 81):
        lateral = max(.0025, .004522678960152904 - step*.0002)
        forward = max(.064, .16508553220405286 - max(0, step-10)*.0016)
        ready = lateral <= .003 and forward <= .065
        upper.observe(_prediction([forward, lateral], ready))
        lower.observe(TRACE["r3"][-1])
    assert ready
    assert len(upper.errors_px) == len(lower.errors_px) == 250
    for _ in range(PAIR_COARSE_MAX_DECISIONS - 250):
        lower.observe(TRACE["r3"][-1])
    with pytest.raises(RGBSkillUnsupported, match="coarse_budget_exhausted"):
        lower.observe(TRACE["r3"][-1])


def test_static_and_one_frame_noise_cannot_extend_a_stalled_actor():
    stable = _prediction([.165, .005])
    for sequence in ([stable] * 171,
                     [stable] * 170 + [_prediction([.16, .005])],
                     [_prediction([.165 + (i % 2)*.0004, .005]) for i in range(171)]):
        budget = PairCoarseProgressBudget()
        with pytest.raises(RGBSkillUnsupported, match="coarse_rgb_stalled"):
            for prediction in sequence:
                budget.observe(prediction)
        assert len(budget.errors_px) == 171


def test_v6_progress_followed_by_stall_stops_at_next_observed_window():
    budget = _replay_budget("r1")
    held = TRACE["r1"][-1]
    with pytest.raises(RGBSkillUnsupported, match="coarse_rgb_stalled"):
        for _ in range(50):
            budget.observe(held)
    assert 171 < len(budget.errors_px) <= 219


def test_extra_budget_does_not_bypass_role_or_missing_wheel_rejection():
    wrong_role = _actor("lower")
    with pytest.raises(RGBSkillUnsupported, match="pair_role_image_side_mismatch"):
        wrong_role.decide(_own(), TOP)
    assert len(wrong_role.coarse_budget.errors_px) == 170

    damaged = cv2.imdecode(np.frombuffer(TOP, np.uint8), cv2.IMREAD_COLOR)
    damaged[115:200, 65:160] = 0
    encoded = cv2.imencode(".jpg", damaged, [cv2.IMWRITE_JPEG_QUALITY, 95])[1].tobytes()
    actor = _actor()
    with pytest.raises(RGBSkillUnsupported, match="own_wheel_heading_unresolved"):
        actor.decide(_own(), encoded)
    assert len(actor.coarse_budget.errors_px) == 170
