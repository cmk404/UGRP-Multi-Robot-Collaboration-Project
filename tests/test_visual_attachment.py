import base64
from pathlib import Path
import unittest

import cv2
import numpy as np

from harness.visual_attachment import compare_box_comotion


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
                self.assertGreater(result["centroid_delta_px"], 8)

    def test_pan_allowance_still_rejects_detached_scale_motion(self):
        result = compare_box_comotion(
            encode(cyan_scene()), encode(cyan_scene(offset=(38, 0))),
            camera_pan_delta_pwm=60)
        self.assertFalse(result["attached"])

    def test_saved_physical_detached_probe_cannot_pass_both_sides_and_home(self):
        root = Path(__file__).resolve().parents[1] / "outputs/grip-calibration-01"
        paths = {name: root / f"{name}.jpg" for name in ("anchor", "left", "right", "home")}
        if not all(path.exists() for path in paths.values()):
            self.skipTest("saved detached physical probe is unavailable")
        encoded = {name: base64.b64encode(path.read_bytes()).decode() for name, path in paths.items()}
        results = [
            compare_box_comotion(encoded["left"], encoded["right"], camera_pan_delta_pwm=-120),
            compare_box_comotion(encoded["anchor"], encoded["home"]),
        ]
        self.assertFalse(all(item["attached"] for item in results))

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
