import base64
import json
from pathlib import Path
import unittest

import cv2
import numpy as np

from harness.visual_arm import camera_extrinsics
from harness.visual_box_surface import observe_known_box_top
from sim.masterpi_camera_profile import CAMERA_FISHEYE_D, scaled_camera_matrix


POSE = {1: 1500, 3: 675, 4: 1679, 5: 2240, 6: 1444}


def encode(frame):
    ok, data = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 98])
    assert ok
    return base64.b64encode(data.tobytes()).decode()


def synthetic_top(center=(.22, -.014, .060), dims=(.034, .040), pose=POSE):
    a, b = dims
    points = np.asarray([(center[0]-a/2, center[1]-b/2, center[2]),
                         (center[0]+a/2, center[1]-b/2, center[2]),
                         (center[0]+a/2, center[1]+b/2, center[2]),
                         (center[0]-a/2, center[1]+b/2, center[2])])
    origin, axes = camera_extrinsics(pose)
    camera_points = (points-np.asarray(origin)) @ np.asarray(axes).T
    projected, _ = cv2.fisheye.projectPoints(camera_points.reshape(1, -1, 3),
        np.zeros((3, 1)), np.zeros((3, 1)), scaled_camera_matrix(640, 480),
        np.asarray(CAMERA_FISHEYE_D).reshape(4, 1))
    quad = np.rint(projected.reshape(4, 2)).astype(np.int32)
    frame = np.full((480, 640, 3), (20, 20, 20), np.uint8)
    cv2.fillConvexPoly(frame, quad, (190, 185, 90))
    return frame


class VisualBoxSurfaceTests(unittest.TestCase):
    def test_synthetic_top_recovers_surface_patch_and_center_height(self):
        result = observe_known_box_top(encode(synthetic_top()), POSE)
        self.assertTrue(result["visible"])
        np.testing.assert_allclose(result["estimated_surface_patch_base_m"], [.22, -.014, .060], atol=.008)
        self.assertAlmostEqual(result["estimated_box_center_height_m"], .044, delta=.008)
        self.assertNotIn("estimated_base_center_m", result)
        self.assertLess(result["edge_fit_rmse_m"], .004)
        self.assertGreater(result["confidence"], .35)

    def test_no_cyan_surface_is_invisible(self):
        result = observe_known_box_top(encode(np.zeros((480, 640, 3), np.uint8)), POSE)
        self.assertFalse(result["visible"])
        self.assertEqual(result["reason"], "CYAN_TOP_QUAD_NOT_VISIBLE")

    def test_clipped_top_is_rejected(self):
        frame = np.zeros((480, 640, 3), np.uint8)
        cv2.fillConvexPoly(frame, np.asarray(((0, 220), (300, 230), (290, 310), (0, 300))), (190, 185, 90))
        self.assertFalse(observe_known_box_top(encode(frame), POSE)["visible"])

    def test_recorded_rgb_distinguishes_floor_box_from_lifted_box(self):
        root = Path(__file__).parent / "fixtures" / "visual_box_surface"
        poses = json.loads((root / "metadata.json").read_text())
        estimates = {}
        for name in ("ground", "lifted"):
            jpeg = base64.b64encode((root / f"{name}.jpg").read_bytes()).decode()
            result = observe_known_box_top(jpeg, poses[name]["servo_pulses"])
            self.assertTrue(result["visible"])
            estimates[name] = result["estimated_box_center_height_m"]
        self.assertLess(estimates["ground"], .035)
        self.assertGreater(estimates["lifted"], .055)
        self.assertGreater(estimates["lifted"] - estimates["ground"], .04)


if __name__ == "__main__":
    unittest.main()
