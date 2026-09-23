"""Recorded RGB regression for pair wheel selection near another robot."""

import hashlib
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np

from harness.dispatch_skill_binding import PairCoarsePixels


ROOT = Path(__file__).parent / "fixtures"
TOP = ROOT / "pair_heading_overlap/r3-000156-top_rgb.jpg"
REFERENCE = ROOT / "camera_goal_transport/reference-top.jpg"
RECORDED_CROP_CENTER = [114.13569321533923 / 960, 279.5427728613569 / 720]


def _coarse():
    return PairCoarsePixels(
        {"r3": {"claim": {"valid": True, "center": RECORDED_CROP_CENTER}}},
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


def test_inner_crop_cannot_approve_missing_own_wheel_corner():
    frame = cv2.imdecode(np.frombuffer(TOP.read_bytes(), np.uint8), cv2.IMREAD_COLOR)
    # Erase one of the target's four wheel corners, retaining the peer above.
    frame[285:318, 112:151] = 0
    damaged = cv2.imencode(".jpg", frame)[1].tobytes()
    prediction = _coarse().decide(damaged, "r3")
    assert prediction["ok"] is False
    assert prediction["reason"] == "own_wheel_heading_unresolved"
    assert prediction["mask"]["inner_crop_retry"] is True
