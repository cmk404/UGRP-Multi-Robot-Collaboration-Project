import base64
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from harness.monocular_box import (
    CameraBoxTracker, PRINTED_MARKER_SIDE_M, camera_point_to_robot_recommendation,
    observe_box,
)
from sim.masterpi_camera_profile import CAMERA_FISHEYE_D, scaled_camera_matrix


def encode(frame):
    ok, payload = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 98])
    assert ok
    return base64.b64encode(payload.tobytes()).decode()


def synthetic_marker(marker_id=14, tvec=(0.025, -0.015, 0.55), size=(640, 480),
                     in_plane_rotation_rad=0.0):
    width, height = size
    frame = np.full((height, width, 3), 255, dtype=np.uint8)
    marker = cv2.aruco.generateImageMarker(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), marker_id, 300)
    half = PRINTED_MARKER_SIDE_M / 2
    obj = np.asarray(((-half, half, 0), (half, half, 0),
                      (half, -half, 0), (-half, -half, 0)), dtype=np.float64)
    base_rotation, _ = cv2.Rodrigues(np.asarray((np.pi, 0, 0), dtype=np.float64))
    spin_rotation, _ = cv2.Rodrigues(np.asarray((0, 0, in_plane_rotation_rad), dtype=np.float64))
    rvec, _ = cv2.Rodrigues(base_rotation @ spin_rotation)
    projected, _ = cv2.fisheye.projectPoints(
        obj.reshape(1, -1, 3), rvec,
        np.asarray(tvec, dtype=np.float64).reshape(3, 1),
        scaled_camera_matrix(width, height), np.asarray(CAMERA_FISHEYE_D).reshape(4, 1))
    dst = projected.reshape(4, 2).astype(np.float32)
    src = np.asarray(((0, 0), (299, 0), (299, 299), (0, 299)), dtype=np.float32)
    transform = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(marker, transform, (width, height), borderValue=255)
    mask = cv2.warpPerspective(np.full_like(marker, 255), transform, (width, height), borderValue=0)
    for channel in range(3):
        frame[..., channel][mask > 0] = warped[mask > 0]
    return frame


def steep_oblique_marker(marker_id=14):
    """Synthetic equivalent of probe-07 frame 0043's complete thin tag."""
    frame = np.full((480, 640, 3), 210, dtype=np.uint8)
    marker = cv2.aruco.generateImageMarker(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), marker_id, 300)
    src = np.asarray(((0, 0), (299, 0), (299, 299), (0, 299)), np.float32)
    dst = np.asarray(((223, 265), (214, 108), (251, 195), (258, 382)), np.float32)
    transform = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(marker, transform, (640, 480), borderValue=255)
    mask = cv2.warpPerspective(np.full_like(marker, 255), transform, (640, 480), borderValue=0)
    for channel in range(3):
        frame[..., channel][mask > 0] = warped[mask > 0]
    return frame


class MonocularBoxTests(unittest.TestCase):
    def test_detects_marker_14_and_recovers_metric_camera_pose(self):
        expected = np.asarray((.025, -.015, .55))
        result = observe_box(encode(synthetic_marker(tvec=expected)), "small_box_01")
        self.assertTrue(result["visible"])
        actual = np.asarray(result["marker_pose_camera"]["translation_m"])
        np.testing.assert_allclose(actual, expected, atol=.025)
        self.assertLess(result["reprojection_rmse_px"], 2.5)
        self.assertEqual(result["known_marker_geometry"]["marker_id"], 14)
        self.assertEqual(result["known_marker_geometry"]["printed_marker_side_m"], .075)
        self.assertIn("marker_size_pnp", result["provenance"])

    def test_rotated_print_does_not_apply_height_as_camera_horizontal_offset(self):
        result = observe_box(encode(synthetic_marker(in_plane_rotation_rad=np.pi / 2)))
        marker = np.asarray(result["marker_pose_camera"]["translation_m"])
        inset = np.asarray(result["box_face_inset_camera_m"])
        # Only the 19 mm face depth may be applied before a gravity-aligned
        # camera-to-base transform; the 60 mm height must not rotate sideways.
        self.assertAlmostEqual(float(np.linalg.norm(inset - marker)), .019, delta=.003)
        self.assertNotIn("box_center_camera_estimate_m", result)
        conversion = result["upright_box_center_conversion"]
        self.assertFalse(conversion["available_in_camera_frame"])
        self.assertEqual(conversion["base_z_offset_m"], -.060)

    def test_target_identity_filters_other_valid_box_marker(self):
        image = encode(synthetic_marker(marker_id=15))
        self.assertFalse(observe_box(image, "small_box_01")["visible"])
        self.assertTrue(observe_box(image, "small_box_02")["visible"])

    def test_complete_steep_oblique_marker_is_not_intermittently_rejected(self):
        result = observe_box(encode(steep_oblique_marker()), "small_box_01")
        self.assertTrue(result["visible"])
        self.assertEqual(result["known_marker_geometry"]["marker_id"], 14)
        self.assertTrue(np.isfinite(result["reprojection_rmse_px"]))

    def test_small_far_default_detector_target_remains_visible(self):
        result = observe_box(
            encode(synthetic_marker(marker_id=15, tvec=(-.08, .02, 1.15))),
            "small_box_02")
        self.assertTrue(result["visible"])
        self.assertEqual(result["known_marker_geometry"]["marker_id"], 15)

    def test_blank_image_is_honestly_not_visible(self):
        result = observe_box(encode(np.full((480, 640, 3), 180, np.uint8)))
        self.assertFalse(result["visible"])
        self.assertNotIn("marker_pose_camera", result)
        self.assertEqual(result["image_size_px"], [640, 480])

    def test_rejects_unknown_target_and_non_jpeg(self):
        with self.assertRaisesRegex(ValueError, "UNKNOWN_BOX_ID"):
            observe_box(encode(synthetic_marker()), "oak_plank_01")
        with self.assertRaisesRegex(ValueError, "INVALID_BASE64_JPEG"):
            observe_box(base64.b64encode(b"not jpeg").decode())

    def test_robot_transform_is_not_claimed_from_provisional_static_mount(self):
        recommendation = camera_point_to_robot_recommendation()
        self.assertFalse(recommendation["available"])
        self.assertIn("servos 3, 4, 5", recommendation["required_inputs"][0])
        self.assertNotIn("translation_m", recommendation)

    def test_tracker_uses_bounded_lk_after_confirmed_id(self):
        tracker = CameraBoxTracker(max_tracking_frames=3)
        first_image = encode(synthetic_marker(tvec=(0., 0., .45)))
        moved_image = encode(synthetic_marker(tvec=(.008, .004, .45)))
        first = tracker.observe(first_image)
        self.assertTrue(first["visible"])
        with patch("harness.monocular_box._detect_target_corners", return_value=[]):
            moved = tracker.observe(moved_image)
        self.assertTrue(moved["visible"])
        self.assertEqual(moved["frames_since_id_confirmed"], 1)
        self.assertIn("lk_tracked_from_confirmed_aruco", moved["provenance"])
        self.assertGreater(moved["pixel_centroid"][0], first["pixel_centroid"][0])

    def test_tracker_drops_blank_frame_instead_of_coasting(self):
        tracker = CameraBoxTracker()
        self.assertTrue(tracker.observe(encode(synthetic_marker(tvec=(0., 0., .45))))["visible"])
        blank = encode(np.full((480, 640, 3), 220, np.uint8))
        self.assertFalse(tracker.observe(blank)["visible"])
        self.assertFalse(tracker.observe(blank)["visible"])

    def test_tracker_expires_at_configured_frame_budget(self):
        tracker = CameraBoxTracker(max_tracking_frames=2)
        image = encode(synthetic_marker(tvec=(0., 0., .45)))
        self.assertTrue(tracker.observe(image)["visible"])
        with patch("harness.monocular_box._detect_target_corners", return_value=[]):
            self.assertEqual(tracker.observe(image)["frames_since_id_confirmed"], 1)
            self.assertEqual(tracker.observe(image)["frames_since_id_confirmed"], 2)
            self.assertFalse(tracker.observe(image)["visible"])


if __name__ == "__main__":
    unittest.main()
