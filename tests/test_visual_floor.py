import base64
import unittest

import cv2
import numpy as np

from harness.visual_floor import floor_point, observe_zone
from sim.masterpi_camera_profile import (
    CAMERA_CX_PX, CAMERA_CY_PX, CAMERA_FISHEYE_D, CAMERA_FX_PX, CAMERA_FY_PX,
)


K = np.asarray(((CAMERA_FX_PX, 0, CAMERA_CX_PX),
                (0, CAMERA_FY_PX, CAMERA_CY_PX), (0, 0, 1)), dtype=np.float64)
D = np.asarray(CAMERA_FISHEYE_D, dtype=np.float64).reshape(4, 1)


def encode(frame):
    ok, payload = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 98])
    assert ok
    return base64.b64encode(payload.tobytes()).decode()


def known_camera_to_base(point, pose):
    """Camera at base (0,0,1): optical z=base x, image down=-base z."""
    point = np.asarray(point, dtype=float)
    rotation = np.asarray(((0, 0, 1), (-1, 0, 0), (0, -1, 0)), dtype=float)
    return (np.asarray((0., 0., 1.)) + rotation @ point).tolist()


def raw_pixel_for_ideal_ray(x, y):
    point = np.asarray((x, y), dtype=np.float64).reshape(1, 1, 2)
    return cv2.fisheye.distortPoints(point, K, D).reshape(2)


class VisualFloorTests(unittest.TestCase):
    def test_green_floor_patch_projects_to_known_base_point(self):
        # Ideal camera ray (x=.06 right, y=.30 down, z=1) intersects the floor
        # from 1 m height at base [3.333 forward, -.2 lateral, 0].
        pixel = raw_pixel_for_ideal_ray(.06, .30)
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        center = tuple(np.rint(pixel).astype(int))
        cv2.rectangle(frame, (center[0]-12, center[1]-9),
                      (center[0]+12, center[1]+9), (0, 180, 0), -1)
        result = observe_zone(encode(frame), "B", {}, known_camera_to_base)
        self.assertTrue(result["visible"])
        np.testing.assert_allclose(result["estimated_base_m"], [10/3, -.2, 0.], atol=.055)
        np.testing.assert_allclose(result["pixel"], pixel, atol=1.5)
        self.assertEqual(result["source"], "own_rgb_color_floor_projection")

    def test_no_matching_zone_color_is_invisible(self):
        frame = np.full((480, 640, 3), (80, 80, 80), dtype=np.uint8)
        result = observe_zone(encode(frame), "A", {}, known_camera_to_base)
        self.assertEqual(result, {"visible": False, "source": "own_rgb_floor_color"})

    def test_horizon_ray_is_rejected(self):
        point = floor_point([CAMERA_CX_PX, CAMERA_CY_PX], {}, known_camera_to_base)
        self.assertIsNone(point)

    def test_ray_above_horizon_points_behind_floor_and_is_rejected(self):
        pixel = raw_pixel_for_ideal_ray(0., -.2)
        self.assertIsNone(floor_point(pixel, {}, known_camera_to_base))

    def test_projection_uses_only_pixel_pose_and_supplied_extrinsic(self):
        calls = []

        def audited_transform(point, pose):
            calls.append((list(point), pose))
            return known_camera_to_base(point, pose)

        pixel = raw_pixel_for_ideal_ray(0., .5)
        result = floor_point(pixel, {"own_servo_pose": [3, 4, 5, 6]}, audited_transform)
        np.testing.assert_allclose(result, [2., 0., 0.], atol=1e-6)
        self.assertEqual(len(calls), 2)
        self.assertTrue(all(call[1] == {"own_servo_pose": [3, 4, 5, 6]} for call in calls))


if __name__ == "__main__":
    unittest.main()
