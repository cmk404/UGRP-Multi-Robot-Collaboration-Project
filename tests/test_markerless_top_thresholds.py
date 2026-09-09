"""Portable regressions for bounded bright-top threshold extraction."""
import base64
import hashlib
import json
from pathlib import Path

import numpy as np

from harness.markerless_box import observe_ground_box
from tests.test_markerless_box import (
    POSE, _blank, _draw_box_with_bright_top, _jpeg,
)

ROOT = Path(__file__).parent / "fixtures/markerless_box/top_thresholds"
MANIFEST = json.loads((ROOT / "manifest.json").read_text())


def _saved(name):
    raw = (ROOT / name).read_bytes()
    metadata = MANIFEST["files"][name]
    assert hashlib.sha256(raw).hexdigest() == metadata["sha256"]
    assert metadata["sha256"] == metadata["source_wrist_sha256"]
    return base64.b64encode(raw).decode(), metadata["own_pose_commands"]


def test_shadowed_real_top_is_valid_in_all_required_available_views():
    results = []
    for name in ("origin.jpg", "left.jpg", "right.jpg"):
        image, pose = _saved(name)
        result = observe_ground_box(image, pose)
        assert result["visible"] is True, result
        assert result["reason"] in {
            "FLOOR_CUBOID_HYPOTHESIS_VALIDATED",
            "MEASURED_TOP_FACE_FLOOR_HYPOTHESIS_VALIDATED",
        }
        assert result["floor_hypothesis_projection_iou"] >= .70
        if result["reason"].startswith("MEASURED_TOP_FACE"):
            assert result["floor_top_height_residual_m"] < .008
        results.append(np.asarray(result["estimated_box_center_base_m"][:2]))
    assert max(np.linalg.norm(a-b) for a in results for b in results) <= .010


def test_threshold_duplicates_do_not_turn_one_synthetic_top_ambiguous():
    frame = _blank()
    _draw_box_with_bright_top(frame, center=(.29, 0.), reflection=True)
    result = observe_ground_box(_jpeg(frame), POSE)
    assert result["visible"] is True, result
    assert result["reason"] == "MEASURED_TOP_FACE_FLOOR_HYPOTHESIS_VALIDATED"


def test_two_spatially_distinct_synthetic_tops_remain_ambiguous():
    frame = _blank()
    _draw_box_with_bright_top(frame, center=(.34, -.07), reflection=True)
    _draw_box_with_bright_top(frame, center=(.34, .07), reflection=True)
    result = observe_ground_box(_jpeg(frame), POSE)
    assert result["visible"] is False, result
    assert result["reason"] == "MULTIPLE_INDISTINGUISHABLE_CYAN_CANDIDATES"
    assert result["candidate_count"] >= 2


def test_elevated_synthetic_top_stays_rejected():
    frame = _blank()
    _draw_box_with_bright_top(frame, center=(.35, 0.), bottom_z=.08, reflection=True)
    result = observe_ground_box(_jpeg(frame), POSE)
    assert result["visible"] is False, result
