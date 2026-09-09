import base64
import hashlib
import json
from pathlib import Path
import unittest

import cv2
import numpy as np

from harness.visual_attachment import _cyan_object_mask, compare_box_comotion


def encode(frame):
    ok, payload = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 98])
    assert ok
    return base64.b64encode(payload.tobytes()).decode()


def cyan_scene(offset=(0, 0), scale=1.0):
    frame = np.full((480, 640, 3), 25, np.uint8)
    cx, cy = 320+offset[0], 280+offset[1]
    half_w, half_h = int(95*scale), int(55*scale)
    cv2.rectangle(frame, (cx-half_w, cy-half_h), (cx+half_w, cy+half_h),
                  (175, 155, 55), -1)
    # A brighter top face remains within the broad cyan mask.
    cv2.fillConvexPoly(frame, np.asarray(((cx-half_w, cy-half_h),
        (cx-half_w+18, cy-half_h-18), (cx+half_w-18, cy-half_h-18),
        (cx+half_w, cy-half_h))), (205, 185, 75))
    return frame


class VisualAttachmentTests(unittest.TestCase):
    def _saved_images(self, root, names):
        paths = [root / name for name in names]
        self.assertTrue(all(path.is_file() for path in paths), paths)
        return [base64.b64encode(path.read_bytes()).decode() for path in paths]

    def test_saved_fixture_manifest_hashes_match(self):
        root = Path(__file__).parent / "fixtures/visual_attachment"
        manifest = json.loads((root / "manifest.json").read_text())
        for relative, metadata in manifest["files"].items():
            with self.subTest(file=relative):
                payload = (root / relative).read_bytes()
                self.assertEqual(hashlib.sha256(payload).hexdigest(), metadata["sha256"])

    def test_fixed_close_mask_supports_visual_attachment(self):
        result = compare_box_comotion(encode(cyan_scene()), encode(cyan_scene(offset=(3, -2), scale=1.01)))
        self.assertTrue(result["attached"])
        self.assertGreaterEqual(result["mask_iou"], .88)
        self.assertLessEqual(result["centroid_delta_px"], 8)
        self.assertEqual(result["evidence"], "visual_attachment")
        self.assertFalse(result["color_is_identity_evidence"])

    def test_shifted_floor_mask_fails_comotion(self):
        result = compare_box_comotion(encode(cyan_scene()), encode(cyan_scene(offset=(38, 0))))
        self.assertFalse(result["attached"])
        self.assertEqual(result["reason"], "CYAN_OBJECT_DID_NOT_COMOVE_WITH_CAMERA")
        self.assertGreater(result["centroid_delta_px"], 8)

    def test_own_pan_calibration_accepts_saved_held_box_frames(self):
        root = Path(__file__).resolve().parents[1] / "outputs/warehouse_research/coela-gemini38-verified-01"
        pairs = (
            (root / "team-41/inputs/r2/0120-wrist.jpg", root / "team-41/inputs/r2/0121-wrist.jpg"),
            (root / "team-73/inputs/r2/0233-wrist.jpg", root / "team-73/inputs/r2/0234-wrist.jpg"),
        )
        if not all(path.exists() for pair in pairs for path in pair):
            self.skipTest("saved physical held-box regression frames are unavailable")
        for before, after in pairs:
            with self.subTest(after=after.name):
                result = compare_box_comotion(
                    base64.b64encode(before.read_bytes()).decode(),
                    base64.b64encode(after.read_bytes()).decode(),
                    camera_pan_delta_pwm=60,
                )
                self.assertTrue(result["attached"])
                # The former >8 px displacement was blue-floor contamination,
                # not held-cargo camera parallax.  The cargo-only mask remains
                # stable and inside the unchanged calibrated limit.
                self.assertGreaterEqual(result["mask_iou"], .88)
                self.assertLessEqual(result["centroid_delta_px"], 14.5)

    def test_pan_allowance_accepts_bounded_cargo_motion_rejected_as_stationary(self):
        before = encode(cyan_scene())
        shifted = encode(cyan_scene(offset=(12, 0)))
        self.assertFalse(compare_box_comotion(before, shifted)["attached"])
        result = compare_box_comotion(before, shifted, camera_pan_delta_pwm=60)
        self.assertTrue(result["attached"], result)
        self.assertGreater(result["centroid_delta_px"], 8.0)
        self.assertLessEqual(result["centroid_delta_px"], 14.5)

    def test_pan_allowance_still_rejects_detached_scale_motion(self):
        result = compare_box_comotion(
            encode(cyan_scene()), encode(cyan_scene(offset=(38, 0))),
            camera_pan_delta_pwm=60)
        self.assertFalse(result["attached"])

    def test_saved_physical_detached_probe_cannot_pass_both_sides_and_home(self):
        root = Path(__file__).parent / "fixtures/visual_attachment/detached"
        names = ("anchor", "left", "right", "home")
        encoded = dict(zip(names, self._saved_images(root, [f"{name}.jpg" for name in names])))
        results = [
            compare_box_comotion(encoded["left"], encoded["right"], camera_pan_delta_pwm=-120),
            compare_box_comotion(encoded["anchor"], encoded["home"]),
        ]
        self.assertFalse(all(item["attached"] for item in results))

    def test_saved_straight_drive_full_sweep_excludes_blue_floor(self):
        root = Path(__file__).parent / "fixtures/visual_attachment/straight46"
        anchor, left, right, home = self._saved_images(
            root, ["anchor.jpg", "left.jpg", "right.jpg", "home.jpg"])
        side = compare_box_comotion(left, right, camera_pan_delta_pwm=-120)
        returned = compare_box_comotion(anchor, home)
        self.assertTrue(side["attached"], side)
        self.assertTrue(returned["attached"], returned)
        self.assertGreater(side["mask_iou"], .99)
        self.assertAlmostEqual(side["area_ratio"], 1.0, delta=.01)

    def test_saved_m4_first_drive_monitor_is_not_a_hue_boundary_false_loss(self):
        root = Path(__file__).parent / "fixtures/visual_attachment/m4_first_drive"
        before, first_slice, shifted = self._saved_images(
            root, ["before.jpg", "first_slice.jpg", "shifted.jpg"])
        self.assertTrue(compare_box_comotion(before, first_slice)["attached"])
        result = compare_box_comotion(first_slice, shifted)
        self.assertTrue(result["attached"], result)
        self.assertGreater(result["mask_iou"], .99)
        self.assertLess(result["centroid_delta_px"], 1.0)
        self.assertAlmostEqual(result["area_ratio"], 1.0, delta=.01)

    def test_saved_combined_blue_to_gray_floor_keeps_cyan_box_mask_stable(self):
        root = Path(__file__).parent / "fixtures/visual_attachment/m4_first_drive"
        blue, gray = self._saved_images(
            root, ["combined-blue-floor.jpg", "combined-gray-floor.jpg"])
        result = compare_box_comotion(blue, gray)
        self.assertTrue(result["attached"], result)
        self.assertGreater(result["mask_iou"], .99)
        self.assertLess(result["centroid_delta_px"], 1.0)
        self.assertAlmostEqual(result["area_ratio"], 1.0, delta=.01)

    def test_saved_blue_floor_roi_is_excluded_while_lower_cyan_box_is_owned(self):
        root = Path(__file__).parent / "fixtures/visual_attachment/straight46"
        frame = cv2.imread(str(root / "left.jpg"))
        owned, area, _ = _cyan_object_mask(frame)
        self.assertEqual(int(np.count_nonzero(owned[30:125, 50:590])), 0)
        self.assertGreater(int(np.count_nonzero(owned[175:430, 50:590])), 137000)
        self.assertGreater(area, 180000)
        self.assertLess(area, 185000)

    def test_unrelated_hue_108_to_110_blue_background_is_excluded(self):
        for hue in (108, 109, 110):
            with self.subTest(hue=hue):
                hsv = np.full((480, 640, 3), (hue, 220, 180), np.uint8)
                high_hue_blue = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
                result = compare_box_comotion(encode(cyan_scene()), encode(high_hue_blue))
                self.assertFalse(result["attached"])
                self.assertEqual(
                    result["reason"], "CLOSE_CYAN_OBJECT_NOT_VISIBLE_IN_BOTH_FRAMES")

    def test_saved_compliant_held_box_passes_full_sweep_geometry(self):
        cohort = (Path(__file__).resolve().parents[1] /
                  "outputs/warehouse_research/coela-camera-repaired-01")
        sweeps = ((cohort / "team-41-natural/inputs/r2", (192, 193, 194, 195)),
                  (cohort / "team-41-status/inputs/r2", (221, 222, 223, 224)))
        if not all((root / f"{step:04d}-wrist.jpg").exists()
                   for root, steps in sweeps for step in steps):
            self.skipTest("saved compliant held-box sweep is unavailable")
        for root, steps in sweeps:
            anchor, left, right, home = [base64.b64encode(
                (root / f"{step:04d}-wrist.jpg").read_bytes()).decode() for step in steps]
            with self.subTest(run=root.parents[1].name):
                self.assertTrue(compare_box_comotion(left, right, camera_pan_delta_pwm=-120)["attached"])
                self.assertTrue(compare_box_comotion(anchor, home)["attached"])

    def test_blank_or_too_small_object_fails_visibility_gate(self):
        blank = np.zeros((480, 640, 3), np.uint8)
        small = blank.copy()
        cv2.rectangle(small, (300, 250), (340, 290), (175, 155, 55), -1)
        for after in (blank, small):
            with self.subTest(nonzero=int(np.count_nonzero(after))):
                result = compare_box_comotion(encode(cyan_scene()), encode(after))
                self.assertFalse(result["attached"])
                self.assertEqual(result["reason"], "CLOSE_CYAN_OBJECT_NOT_VISIBLE_IN_BOTH_FRAMES")

    def test_frame_size_change_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "CAMERA_FRAME_SIZE_CHANGED"):
            compare_box_comotion(encode(cyan_scene()), encode(np.zeros((240, 320, 3), np.uint8)))


if __name__ == "__main__":
    unittest.main()
