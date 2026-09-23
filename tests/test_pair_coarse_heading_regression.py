"""Recorded RGB regression for pair wheel selection near another robot."""

import hashlib
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from harness.dispatch_skill_binding import PairCoarsePixels


ROOT = Path(__file__).parent / "fixtures"
TOP = ROOT / "pair_heading_overlap/r3-000156-top_rgb.jpg"
NEW_TOP = ROOT / "pair_heading_overlap/r3-000158-top_rgb.jpg"
REFERENCE = ROOT / "camera_goal_transport/reference-top.jpg"
RECORDED_CROP_CENTER = [114.13569321533923 / 960, 279.5427728613569 / 720]
NEW_CROP_CENTER = [112.665 / 960, 280.791 / 720]


def _coarse(center=RECORDED_CROP_CENTER):
    return PairCoarsePixels(
        {"r3": {"claim": {"valid": True, "center": center}}},
        SimpleNamespace(pair={"r3": "r3"}), REFERENCE.read_bytes())


def test_recorded_peer_wheel_at_crop_edge_preserves_four_corner_heading():
    raw = TOP.read_bytes()
    assert hashlib.sha256(raw).hexdigest() == "0bba4db4b15d7180bbe3201c4d8b42608401e47a83b050b86e2ee7e17ffdbe07"
    prediction = _coarse().decide(raw, "r3")
    assert prediction["ok"] is True
    assert prediction["ready"] is False
    assert prediction["mask"]["initial_local_wheel_pixels"] == 422
    assert prediction["mask"]["local_wheel_pixels"] == 381
    assert prediction["mask"]["crop_half_size_px"] == [55, 44]
    assert min(prediction["heading"]["corner_pixels"]) >= 5


def test_second_crop_recovers_recorded_peer_wheel_on_44px_boundary():
    raw = NEW_TOP.read_bytes()
    assert hashlib.sha256(raw).hexdigest() == "2f6ab101d0974445772a74c8547150736bc5f1aa3b8820f128e95f3f64a156e0"
    prediction = _coarse(NEW_CROP_CENTER).decide(raw, "r3")
    assert prediction["ok"] is True
    assert prediction["ready"] is False
    assert prediction["mask"]["initial_local_wheel_pixels"] == 444
    assert prediction["mask"]["intermediate_local_wheel_pixels"] == 349
    assert prediction["mask"]["local_wheel_pixels"] == 343
    assert prediction["mask"]["crop_half_size_px"] == [55, 42]
    assert prediction["mask"]["second_crop_retry"] is True
    assert min(prediction["heading"]["corner_pixels"]) >= 5


@pytest.mark.parametrize("top,center", [(TOP, RECORDED_CROP_CENTER), (NEW_TOP, NEW_CROP_CENTER)])
@pytest.mark.parametrize("erased", [(112, 285, 151, 318), (75, 295, 151, 319)])
def test_inner_crops_cannot_approve_missing_corner_or_occluded_wheel_band(top, center, erased):
    frame = cv2.imdecode(np.frombuffer(top.read_bytes(), np.uint8), cv2.IMREAD_COLOR)
    # Leave the peer visible while hiding either one corner or the lower row.
    x1, y1, x2, y2 = erased
    frame[y1:y2, x1:x2] = 0
    damaged = cv2.imencode(".jpg", frame)[1].tobytes()
    prediction = _coarse(center).decide(damaged, "r3")
    assert prediction["ok"] is False
    assert prediction["reason"] == "own_wheel_heading_unresolved"
    assert prediction["mask"]["inner_crop_retry"] is True
    assert prediction["mask"]["second_crop_retry"] is True
    assert prediction["forward"] == prediction["left"] == prediction["turn"] == 0.
