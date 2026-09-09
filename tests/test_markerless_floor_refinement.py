"""Portable acceptance tests for bounded floor-cuboid refinement."""
import base64
import hashlib
import json
from pathlib import Path
import unittest

import numpy as np

from harness.markerless_box import observe_ground_box

ROOT = Path(__file__).resolve().parents[1]
FIXTURES = ROOT / "tests/fixtures/markerless_box/floor_refinement"
MANIFEST = json.loads((FIXTURES / "manifest.json").read_text())
def payload(name):
    raw = (FIXTURES / name).read_bytes()
    assert hashlib.sha256(raw).hexdigest() == MANIFEST["files"][name]["sha256"]
    return base64.b64encode(raw).decode(), MANIFEST["files"][name]["own_pose_commands"]


class FloorRefinementTests(unittest.TestCase):
    def test_candidate_makes_stationary_saved_pair_agree_without_changing_gate(self):
        results = []
        for name in ("origin.jpg", "left.jpg"):
            image, pose = payload(name)
            result = observe_ground_box(image, pose)
            self.assertTrue(result["visible"], result)
            results.append(result)
        separation = np.linalg.norm(
            np.asarray(results[1]["estimated_box_center_base_m"][:2])
            - np.asarray(results[0]["estimated_box_center_base_m"][:2])
        )
        self.assertLessEqual(separation, .010)
        self.assertGreaterEqual(results[0]["floor_hypothesis_projection_iou"], .90)
        self.assertGreaterEqual(results[1]["floor_hypothesis_projection_iou"], .70)

    def test_real_held_view_remains_rejected_as_ground_box(self):
        image, pose = payload("held.jpg")
        result = observe_ground_box(image, pose)
        self.assertFalse(result["visible"], result)

    def test_seed46_reflection_edge_pair_stays_visible_and_stationary(self):
        results = []
        for name in ("seed46-origin.jpg", "seed46-left.jpg"):
            image, pose = payload(name)
            result = observe_ground_box(image, pose)
            self.assertTrue(result["visible"], result)
            self.assertGreaterEqual(result["floor_hypothesis_projection_iou"], .70)
            results.append(result)
        separation = np.linalg.norm(
            np.asarray(results[1]["estimated_box_center_base_m"][:2])
            - np.asarray(results[0]["estimated_box_center_base_m"][:2])
        )
        self.assertLessEqual(separation, .010)

    def test_existing_blue_floor_release_frames_remain_valid_single_candidates(self):
        root = ROOT / "tests/fixtures/markerless_box/blue_floor_release"
        metadata = json.loads((root / "metadata.json").read_text())
        for frame in metadata["frames"]:
            with self.subTest(step=frame["step"]):
                image = base64.b64encode((root / frame["file"]).read_bytes()).decode()
                candidate = observe_ground_box(image, frame["own_pwm"])
                self.assertTrue(candidate["visible"], candidate)
                self.assertEqual(candidate.get("candidate_count"), None)
                self.assertGreaterEqual(candidate["floor_hypothesis_projection_iou"], .58)


if __name__ == "__main__":
    unittest.main()
