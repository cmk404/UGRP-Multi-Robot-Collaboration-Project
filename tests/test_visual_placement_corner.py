import hashlib
import json
import unittest
from pathlib import Path

import cv2
import numpy as np

from harness.visual_placement_corner import (
    _orthogonal_axes, reconstruct_zone_from_corner, signed_footprint_margins,
)


FIXTURES = Path(__file__).parent / "fixtures" / "ci_recorded"


class RecordedCornerReconstructionTests(unittest.TestCase):
    def test_recovers_late_redelivery_frames_rejected_by_three_edge_fit(self):
        folder = FIXTURES / "corner_recovery"
        recovered = []
        for index in range(10, 21):
            frame = cv2.imread(str(folder / f"{index:04d}-nav.jpg"))
            fit = reconstruct_zone_from_corner(frame, "B")
            recovered.append(fit is not None)
            if fit is not None:
                self.assertGreaterEqual(min(fit["visible_edge_support_counts"]), 20)
                self.assertGreaterEqual(fit["perpendicular_angle_deg"], 78)
                self.assertLessEqual(fit["orthogonality_error_deg"], 3)
                self.assertGreaterEqual(fit["interior_color_fraction"], .78)
        self.assertGreaterEqual(sum(recovered), 8)

    def test_checkpoint_outside_frame_remains_geometrically_reconstructable(self):
        path = FIXTURES / "corner_recovery" / "0001-nav.jpg"
        fit = reconstruct_zone_from_corner(cv2.imread(str(path)), "B")
        self.assertIsNotNone(fit)
        margins = signed_footprint_margins(fit, (0.16749944, 0.00028043),
                                           (0.036, 0.038, 0.034))
        self.assertLess(min(margins), -.10)

    def test_historical_inside_frames_remain_inside_with_conservative_margin(self):
        manifest = json.loads((FIXTURES / "manifest.json").read_text())
        for call in (111, 112):
            evidence = manifest["corner_inside"][f"call-{call:04d}"]
            frame = cv2.imread(str(FIXTURES / evidence["file"]))
            fit = reconstruct_zone_from_corner(frame, "B")
            self.assertIsNotNone(fit)
            center = (evidence.get("calibrated_lowering_center_m")
                      or evidence.get("calibrated_released_box_center_m"))
            margins = signed_footprint_margins(fit, tuple(center),
                                               tuple(evidence["box_dimensions_m"]),
                                               evidence["placement_margin_m"])
            self.assertGreaterEqual(min(margins), 0.0)

    def test_rejects_frame_without_zone_color(self):
        path = FIXTURES / "corner_recovery" / "0020-nav.jpg"
        frame = cv2.imread(str(path))
        frame[:] = 0
        self.assertIsNone(reconstruct_zone_from_corner(frame, "B"))

    def test_does_not_fit_requested_zone_from_a_different_color(self):
        path = FIXTURES / "corner_recovery" / "0020-nav.jpg"
        frame = cv2.imread(str(path))
        # Preserve only pixels from the green target mask; asking for blue must
        # not reinterpret the same geometry as zone A.
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        green = cv2.inRange(hsv, np.array((40, 85, 30), np.uint8),
                            np.array((84, 255, 255), np.uint8))
        isolated = np.zeros_like(frame)
        isolated[green > 0] = frame[green > 0]
        self.assertIsNone(reconstruct_zone_from_corner(isolated, "A"))

    def test_rejects_when_corner_is_occluded(self):
        path = FIXTURES / "corner_recovery" / "0020-nav.jpg"
        frame = cv2.imread(str(path))
        # Remove the central image region containing the meeting points of the
        # projected floor boundaries; a single remaining edge is insufficient.
        frame[:, 180:500] = 0
        self.assertIsNone(reconstruct_zone_from_corner(frame, "B"))

    def test_rejects_color_region_clipped_to_image_edges(self):
        frame = np.zeros((480, 640, 3), np.uint8)
        frame[:, :420] = (0, 180, 0)
        self.assertIsNone(reconstruct_zone_from_corner(frame, "B"))

    def test_square_fit_is_exactly_orthogonal_and_rejects_large_angle_error(self):
        first = np.asarray((1.0, 0.0), np.float32)
        second = np.asarray((np.cos(np.deg2rad(88)), np.sin(np.deg2rad(88))), np.float32)
        fitted = _orthogonal_axes(first, second)
        self.assertIsNotNone(fitted)
        a, b, error = fitted
        self.assertAlmostEqual(float(np.dot(a, b)), 0.0, places=6)
        self.assertAlmostEqual(float(np.linalg.norm(a)), 1.0, places=6)
        self.assertAlmostEqual(float(np.linalg.norm(b)), 1.0, places=6)
        self.assertAlmostEqual(error, 2.0, places=4)
        ambiguous = np.asarray((np.cos(np.deg2rad(84)), np.sin(np.deg2rad(84))), np.float32)
        self.assertIsNone(_orthogonal_axes(first, ambiguous))

    def test_recorded_fixture_hashes_match_manifest(self):
        manifest = json.loads((FIXTURES / "manifest.json").read_text())
        for relative, metadata in manifest["files"].items():
            with self.subTest(file=relative):
                payload = (FIXTURES / relative).read_bytes()
                self.assertEqual(hashlib.sha256(payload).hexdigest(), metadata["sha256"])


if __name__ == "__main__":
    unittest.main()
