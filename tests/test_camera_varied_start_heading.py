import json
import math
import unittest
from unittest import mock

import cv2
import numpy as np

from harness import camera_varied_start_heading as heading


def jpeg(image):
    ok, data = cv2.imencode(".jpg", image)
    assert ok
    return data.tobytes()


class HeadingModelTest(unittest.TestCase):
    def setUp(self):
        self.frame = jpeg(np.zeros((720, 960, 3), np.uint8))
        self.model = {
            "schema": heading.SCHEMA, "robot_id": "r1", "stage": "yaw",
            "roi": list(heading.TOP_ROIS["r1"]),
            "background_png": heading._encode_png(np.zeros((205, 330, 3), np.uint8)),
            "templates_png": {
                "soft_yellow": heading._encode_png(np.zeros((96, 96), np.uint8)),
                "full_delta": heading._encode_png(np.zeros((96, 96), np.uint8)),
            },
            "goal_center": [48, 48],
            "thresholds": {"soft_yellow": 0.70, "full_delta": 0.65},
            "coefficients": {"soft_yellow": [0, 0, 0, 0.01, 0, 0, 0, 0], "full_delta": [0, 0, 0, 0.02, 0, 0, 0, 0]},
            "domains": {mode: {"minimum": [-200, -100, -16], "maximum": [200, 100, 16]}
                        for mode in heading.THRESHOLDS},
        }

    def test_actual_rgb_training_roundtrip_and_signed_rotation(self):
        def frame(cx, cy, angle):
            image = np.zeros((720, 960, 3), np.uint8)
            for dx, dy in ((-16, -19), (16, -19), (-16, 19), (16, 19)):
                cv2.rectangle(image, (cx + dx - 7, cy + dy - 3),
                              (cx + dx + 7, cy + dy + 3), (0, 180, 230), -1)
            rotation = cv2.getRotationMatrix2D((cx, cy), angle, 1.)
            return jpeg(cv2.warpAffine(image, rotation, (960, 720)))
        rows = [{"top_jpeg": frame(340 + case * 22, 420 + case % 3 * 8, angle),
                 "case_id": f"pose-{case}", "error": math.radians(angle)}
                for case in range(8) for angle in (-8, -4, 0, 4, 8)]
        model = heading.fit_heading_model(frame(430, 428, 0), rows, "r1")
        restored = json.loads(json.dumps(model, allow_nan=False))
        for angle in (-3, 3):
            result = heading.predict_heading(restored, frame(440, 425, angle))
            self.assertTrue(result["ok"])
            self.assertEqual(result["precision"], "fine")
            self.assertAlmostEqual(result["error"], math.radians(angle), delta=math.radians(.5))

    def test_fine_measurement_is_preferred(self):
        feature = np.array([1, 0, 0, 2, 0, 0, 0, 0], float)
        with mock.patch.object(heading, "_measure", return_value=(feature, 0.91)) as measure:
            result = heading.predict_heading(self.model, self.frame)
        self.assertTrue(result["ok"])
        self.assertAlmostEqual(result["error"], 0.02)
        self.assertEqual(result["precision"], "fine")
        self.assertEqual(result["diagnostics"]["precision"], "fine")
        self.assertEqual(measure.call_count, 1)

    def test_full_delta_is_tagged_coarse(self):
        feature = np.array([1, 0, 0, 2, 0, 0, 0, 0], float)
        with mock.patch.object(heading, "_measure", side_effect=[None, (feature, 0.72)]):
            result = heading.predict_heading(self.model, self.frame)
        self.assertTrue(result["ok"])
        self.assertAlmostEqual(result["error"], 0.04)
        self.assertEqual(result["precision"], "coarse")
        self.assertEqual(result["diagnostics"]["precision"], "coarse")
        self.assertEqual(result["diagnostics"]["method"], "full_delta")

    def test_unsupported_frame_fails_closed(self):
        with mock.patch.object(heading, "_measure", return_value=None):
            result = heading.predict_heading(self.model, self.frame)
        self.assertFalse(result["ok"])
        self.assertIsNone(result["error"])
        self.assertEqual(result["precision"], "unavailable")
        self.assertEqual(result["diagnostics"]["precision"], "unavailable")


if __name__ == "__main__":
    unittest.main()
