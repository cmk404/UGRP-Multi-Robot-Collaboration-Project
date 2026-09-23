"""Recorded TOP RGB checks for pair wheel identity near another robot."""

import hashlib
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from harness.dispatch_skill_binding import PairCoarsePixels


ROOT = Path(__file__).parent / "fixtures"
FRAMES = ROOT / "pair_heading_overlap"
REFERENCE = ROOT / "camera_goal_transport/reference-top.jpg"
CASES = [
    ("6089f80", [152, 155], 156,
     [114.2406876790831 / 960, 282.27793696275074 / 720], 381),
    ("bf476e2", [151], 158,
     [112.90634441087613 / 960, 282.9274924471299 / 720], 343),
    ("7c3e4aa", [151, 158], 159,
     [112.90634441087613 / 960, 282.9274924471299 / 720], 337),
]
HASHES = {
    156: "0bba4db4b15d7180bbe3201c4d8b42608401e47a83b050b86e2ee7e17ffdbe07",
    158: "2f6ab101d0974445772a74c8547150736bc5f1aa3b8820f128e95f3f64a156e0",
    159: "fbb56c4449f6201019b4b6e3999ce14254c01b890efb1c3590a895001d55d47f",
    175: "8cbd5d32aade7bbde81bd1911d807f8bb4689be63e43ccaeb3c233e774e5e7e4",
    176: "b6c59831a7ec23f2a9cb9d33905e9a96316646ad336215ff9af1fc23e52abf23",
}
NO_PRIOR_CENTERS = {
    156: [114.13569321533923 / 960, 279.5427728613569 / 720],
    158: [112.66470588235295 / 960, 280.79117647058825 / 720],
    159: [112.83381924198251 / 960, 279.9008746355685 / 720],
}


def _frame(number):
    return (FRAMES / f"r3-{number:06d}-top_rgb.jpg").read_bytes()


def _coarse(center):
    return PairCoarsePixels(
        {"r3": {"claim": {"valid": True, "center": center}}},
        SimpleNamespace(pair={"r3": "r3"}), REFERENCE.read_bytes())


def _with_prior(case):
    _, previous, _, center, _ = case
    coarse = _coarse(center)
    for number in previous:
        assert coarse.decide(_frame(number), "r3")["ok"] is True
    assert coarse.wheel_bounds["r3"]
    return coarse


def _v3_with_prior():
    # Actor's accepted r3-000174 TOP decision; never a referee position.
    coarse = _coarse([113.2724795640327 / 960, 270.03269754768394 / 720])
    coarse.wheel_bounds["r3"] = [81, 144, 254, 298]
    assert hashlib.sha256(_frame(175)).hexdigest() == HASHES[175]
    assert coarse.decide(_frame(175), "r3")["ok"] is True
    assert coarse.wheel_bounds["r3"] == [80, 143, 246, 297]
    return coarse


@pytest.mark.parametrize("case", CASES, ids=[case[0] for case in CASES])
def test_recorded_peer_wheel_recovered_from_prior_own_rgb_bounds(case):
    _, _, number, _, expected_pixels = case
    raw = _frame(number)
    assert hashlib.sha256(raw).hexdigest() == HASHES[number]
    prediction = _with_prior(case).decide(raw, "r3")
    assert prediction["ok"] is True
    assert prediction["ready"] is False
    assert prediction["mask"]["selection"] == "prior_own_rgb_bounds"
    assert prediction["mask"]["bounds_margin_px"] == 8
    assert prediction["mask"]["local_wheel_pixels"] == expected_pixels
    assert min(prediction["heading"]["corner_pixels"]) >= 5
    assert prediction["mask"]["center_step_px"] <= 8
    assert prediction["mask"]["bounds_step_px"] <= 8


@pytest.mark.parametrize("case", CASES, ids=[case[0] for case in CASES])
def test_ambiguous_frame_without_prior_wheel_bounds_stops(case):
    _, _, number, _, _ = case
    prediction = _coarse(NO_PRIOR_CENTERS[number]).decide(_frame(number), "r3")
    assert prediction["ok"] is False
    assert prediction["reason"] == "own_wheel_heading_unresolved"
    assert prediction["mask"]["prior_wheel_bounds_px"] is None
    assert prediction["forward"] == prediction["left"] == prediction["turn"] == 0.


@pytest.mark.parametrize("case", CASES, ids=[case[0] for case in CASES])
@pytest.mark.parametrize("erased", [(112, 285, 151, 318), (75, 295, 151, 319)],
                         ids=["missing_corner", "occluded_lower_wheels"])
def test_prior_bounds_cannot_approve_missing_corner_or_wheel_band(case, erased):
    _, _, number, _, _ = case
    coarse = _with_prior(case)
    prior_center = coarse.centers["r3"].copy()
    prior_bounds = list(coarse.wheel_bounds["r3"])
    frame = cv2.imdecode(np.frombuffer(_frame(number), np.uint8), cv2.IMREAD_COLOR)
    x1, y1, x2, y2 = erased
    frame[y1:y2, x1:x2] = 0
    damaged = cv2.imencode(".jpg", frame)[1].tobytes()
    prediction = coarse.decide(damaged, "r3")
    assert prediction["ok"] is False
    assert prediction["reason"] in {"own_wheel_heading_unresolved", "own_wheel_identity_discontinuous"}
    assert prediction["forward"] == prediction["left"] == prediction["turn"] == 0.
    np.testing.assert_array_equal(coarse.centers["r3"], prior_center)
    assert coarse.wheel_bounds["r3"] == prior_bounds


def test_large_image_displacement_does_not_rebind_wheel_identity():
    coarse = _with_prior(CASES[1])
    prior_center = coarse.centers["r3"].copy()
    prior_bounds = list(coarse.wheel_bounds["r3"])
    frame = cv2.imdecode(np.frombuffer(_frame(151), np.uint8), cv2.IMREAD_COLOR)
    moved = cv2.warpAffine(frame, np.float32([[1, 0, 9], [0, 1, 0]]),
                           (960, 720), flags=cv2.INTER_NEAREST)
    raw = cv2.imencode(".jpg", moved, [cv2.IMWRITE_JPEG_QUALITY, 95])[1].tobytes()
    prediction = coarse.decide(raw, "r3")
    assert prediction["ok"] is False
    assert prediction["reason"] == "own_wheel_identity_discontinuous"
    assert prediction["mask"]["center_step_px"] > 8
    assert prediction["forward"] == prediction["left"] == prediction["turn"] == 0.
    np.testing.assert_array_equal(coarse.centers["r3"], prior_center)
    assert coarse.wheel_bounds["r3"] == prior_bounds


def test_v3_peer_component_is_excluded_without_relaxing_four_corners():
    raw = _frame(176)
    assert hashlib.sha256(raw).hexdigest() == HASHES[176]
    prediction = _v3_with_prior().decide(raw, "r3")
    assert prediction["ok"] is True
    assert prediction["mask"]["selection"] == "prior_own_rgb_components"
    assert prediction["mask"]["bounds_margin_px"] == 8
    assert prediction["mask"]["selection_touches_bounds"] is False
    assert prediction["mask"]["local_wheel_pixels"] == 351
    assert prediction["heading"]["corner_pixels"] == [79, 33, 95, 77]
    assert prediction["mask"]["center_step_px"] < 1
    assert prediction["mask"]["bounds_step_px"] == 1


def test_v3_ambiguous_frame_without_prior_bounds_stops():
    coarse = _coarse([113.61994609164421 / 960, 270.33962264150944 / 720])
    prediction = coarse.decide(_frame(176), "r3")
    assert prediction["ok"] is False
    assert prediction["reason"] == "own_wheel_heading_unresolved"
    assert prediction["mask"]["prior_wheel_bounds_px"] is None


@pytest.mark.parametrize("damage", ["missing_corner", "occluded_lower_wheels", "shifted_nine_px"])
def test_v3_component_fallback_rejects_damaged_or_clipped_wheels(damage):
    coarse = _v3_with_prior()
    prior_center = coarse.centers["r3"].copy()
    prior_bounds = list(coarse.wheel_bounds["r3"])
    frame = cv2.imdecode(np.frombuffer(_frame(176), np.uint8), cv2.IMREAD_COLOR)
    if damage == "missing_corner":
        frame[280:310, 112:151] = 0
    elif damage == "occluded_lower_wheels":
        frame[280:319, 75:151] = 0
    else:
        frame = cv2.warpAffine(frame, np.float32([[1, 0, 9], [0, 1, 0]]),
                               (960, 720), flags=cv2.INTER_NEAREST)
    raw = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 95])[1].tobytes()
    prediction = coarse.decide(raw, "r3")
    assert prediction["ok"] is False
    assert prediction["reason"] in {"own_wheel_heading_unresolved", "own_wheel_identity_discontinuous"}
    assert prediction["forward"] == prediction["left"] == prediction["turn"] == 0.
    np.testing.assert_array_equal(coarse.centers["r3"], prior_center)
    assert coarse.wheel_bounds["r3"] == prior_bounds
