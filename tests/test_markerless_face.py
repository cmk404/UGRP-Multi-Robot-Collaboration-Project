from __future__ import annotations

import math
import unittest
from harness.markerless_face import MarkerlessFaceAligner


def box(yaw_deg: float, iou: float = .82) -> dict:
    return {"visible": True, "estimated_yaw_mod_pi_rad": math.radians(yaw_deg),
            "floor_hypothesis_projection_iou": iou}


class MarkerlessFaceAlignerTests(unittest.TestCase):
    def test_ninety_degree_equivalent_yaws_form_stable_evidence(self):
        aligner = MarkerlessFaceAligner()
        self.assertFalse(aligner.observe(box(2), (1.0, 0.0))["ready"])
        self.assertFalse(aligner.observe(box(92), (1.0, 0.0))["ready"])
        result = aligner.observe(box(182), (1.0, 0.0))
        self.assertTrue(result["ready"], result)
        self.assertLess(result["normal_xy"][0], -.99)

    def test_exact_ten_degree_spread_is_accepted(self):
        aligner = MarkerlessFaceAligner()
        for yaw in (45, 50, 55):
            result = aligner.observe(box(yaw), (1.0, 0.0))
        self.assertTrue(result["ready"], result)

    def test_three_unstable_yaws_are_rejected(self):
        aligner = MarkerlessFaceAligner()
        aligner.observe(box(0), (1.0, 0.0))
        aligner.observe(box(4), (1.0, 0.0))
        result = aligner.observe(box(18), (1.0, 0.0))
        self.assertFalse(result["ready"])
        self.assertEqual(result["reason"], "UNSTABLE_YAW_FITS")

    def test_initial_selection_chooses_nearest_outward_face(self):
        aligner = MarkerlessFaceAligner()
        for yaw in (43, 45, 47):
            result = aligner.observe(box(yaw), (1.0, 1.0))
        self.assertTrue(result["ready"], result)
        expected = -math.sqrt(.5)
        self.assertAlmostEqual(result["normal_xy"][0], expected, delta=.04)
        self.assertAlmostEqual(result["normal_xy"][1], expected, delta=.04)
        self.assertEqual(result["evidence"]["selection"], "outward_face_toward_chassis")

    def test_gradual_own_frame_heading_changes_recompute_nearest_candidate(self):
        aligner = MarkerlessFaceAligner()
        for yaw in (0, 2, 4):
            result = aligner.observe(box(yaw), (1.0, 0.0))
        previous = result["normal_xy"]
        for yaw in (7, 10, 13):
            result = aligner.observe(box(yaw), (0.9, -0.2))
            self.assertTrue(result["ready"], result)
            self.assertGreater(sum(a*b for a, b in zip(previous, result["normal_xy"])), .98)
            previous = result["normal_xy"]
        self.assertEqual(result["evidence"]["selection"],
                         "current_own_frame_candidate_nearest_previous_normal")
        self.assertGreater(abs(result["normal_xy"][1]), .15)

    def test_low_iou_does_not_enter_stability_window(self):
        aligner = MarkerlessFaceAligner()
        aligner.observe(box(0), (1.0, 0.0))
        aligner.observe(box(1), (1.0, 0.0))
        rejected = aligner.observe(box(0, .69), (1.0, 0.0))
        self.assertFalse(rejected["ready"])
        self.assertEqual(rejected["evidence"]["accepted_fit_count"], 0)
        self.assertFalse(aligner.observe(box(2), (1.0, 0.0))["ready"])


if __name__ == "__main__":
    unittest.main()
