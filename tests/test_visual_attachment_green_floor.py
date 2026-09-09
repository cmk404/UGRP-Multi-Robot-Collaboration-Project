"""Real own-camera regression for green floor merging into held cargo."""
from __future__ import annotations

import base64
import hashlib
import json
from pathlib import Path
import unittest

import cv2
import numpy as np

from harness.visual_attachment import _cyan_object_mask, compare_box_comotion

FIXTURES = Path(__file__).parent / "fixtures/visual_attachment/green_floor"


def image(name):
    return base64.b64encode((FIXTURES / name).read_bytes()).decode()


class GreenFloorAttachmentTests(unittest.TestCase):
    def test_fixture_provenance(self):
        manifest = json.loads((FIXTURES / "manifest.json").read_text())
        self.assertEqual(manifest["camera"], "r1 own wrist RGB")
        for name, metadata in manifest["files"].items():
            with self.subTest(file=name):
                digest = hashlib.sha256((FIXTURES / name).read_bytes()).hexdigest()
                self.assertEqual(digest, metadata["sha256"])
                self.assertEqual(digest, metadata["source_wrist_sha256"])
                self.assertEqual(set(metadata["own_pose_commands"]), {"1", "3", "4", "5", "6"})

    def test_green_floor_entry_keeps_real_held_box_supported(self):
        result = compare_box_comotion(image("monitor-before.jpg"), image("monitor-green-joined.jpg"))
        self.assertTrue(result["attached"], result)
        self.assertLess(result["centroid_delta_px"], 1.0)
        self.assertEqual(result["effective_centroid_limit_px"], 8.0)

    def test_owned_mask_excludes_visible_green_floor_and_keeps_cargo(self):
        frame = cv2.imread(str(FIXTURES / "monitor-green-joined.jpg"))
        mask, _, _ = _cyan_object_mask(frame)
        # Independent visible ROIs: upper green floor, lower cyan cargo.
        self.assertEqual(np.count_nonzero(mask[40:95, 170:540]), 0)
        self.assertEqual(np.count_nonzero(mask[200:400, 100:550]), 200 * 450)

    def test_real_initial_and_destination_grip_sweeps_still_pass(self):
        metadata = json.loads((FIXTURES / "manifest.json").read_text())["files"]
        for prefix in ("initial", "strict"):
            for before, after in (("anchor", "left"), ("anchor", "right"), ("left", "right"), ("anchor", "home")):
                first, second = f"{prefix}-{before}.jpg", f"{prefix}-{after}.jpg"
                pan = metadata[second]["own_pose_commands"]["6"] - metadata[first]["own_pose_commands"]["6"]
                with self.subTest(sweep=prefix, pair=(before, after)):
                    result = compare_box_comotion(image(first), image(second), camera_pan_delta_pwm=pan)
                    self.assertTrue(result["attached"], result)


if __name__ == "__main__":
    unittest.main()
