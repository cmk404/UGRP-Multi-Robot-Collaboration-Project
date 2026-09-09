from __future__ import annotations

import base64
import hashlib
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from harness.visual_placement import _project_ground, inspect_placement


POSE = {"3": 705, "4": 1742, "5": 2207, "6": 1554}


def obs(frame: np.ndarray, camera: str) -> dict:
    ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 98])
    assert ok
    payload = encoded.tobytes()
    return {"camera": camera, "image": base64.b64encode(payload).decode(),
            "sha256": hashlib.sha256(payload).hexdigest(),
            "actuator_state": {"servo_pulses": POSE}}


def wrist() -> np.ndarray:
    return np.full((480, 640, 3), (180, 180, 180), np.uint8)


def nav_zone(center=(0.36, 0.0)) -> np.ndarray:
    frame = np.full((480, 640, 3), 40, np.uint8)
    cx, cy = center
    half = .41
    metric = ((cx-half, cy-half), (cx+half, cy-half),
              (cx+half, cy+half), (cx-half, cy+half))
    pixels = np.asarray([_project_ground(point, 640, 480) for point in metric], np.int32)
    cv2.fillConvexPoly(frame, pixels, (35, 145, 35))
    return frame


def observed(center=(0.36, 0.0)) -> dict:
    return {
        "target_id": "small_box_01",
        "visible": True,
        "reason": "FLOOR_CUBOID_HYPOTHESIS_VALIDATED",
        "pixel_centroid": [320.0, 260.0],
        "estimated_box_center_base_m": [center[0], center[1], 0.016],
        "confidence": 0.84,
        "provenance": "own_rgb+own_camera_fk+known_floor_cuboid_projection",
        "identity_source": "task_catalog_reference_only_not_visually_decoded",
    }


class MarkerlessPlacementTests(unittest.TestCase):
    @patch("harness.visual_placement.observe_box", side_effect=AssertionError("ArUco called"))
    def test_default_before_release_never_calls_fiducial_detector(self, _observe_box):
        result = inspect_placement(obs(wrist(), "robot_cam"), obs(nav_zone(), "nav_cam"),
                                   cargo_id="small_box_01", destination_zone="B",
                                   held_identity_confirmed=True)
        self.assertEqual(result["perception_mode"], "markerless")
        self.assertEqual(result["status"], "inside", result)
        self.assertEqual(result["identity"]["identity_source"],
                         "caller_selected_target_continuity_not_visually_decoded")

    @patch("harness.visual_placement.observe_ground_box")
    def test_missing_or_ambiguous_released_observation_fails_closed(self, detector):
        for reason in ("CYAN_SILHOUETTE_NOT_VISIBLE",
                       "MULTIPLE_INDISTINGUISHABLE_CYAN_CANDIDATES"):
            with self.subTest(reason=reason):
                detector.return_value = {"visible": False, "reason": reason,
                                         "provenance": "own_rgb_shape_fit"}
                result = inspect_placement(obs(wrist(), "robot_cam"),
                                           obs(nav_zone(), "nav_cam"),
                                           cargo_id="small_box_01", destination_zone="B",
                                           stage="released", held_identity_confirmed=True,
                                           release_commanded=True)
                self.assertEqual(result["status"], "uncertain", result)
                self.assertEqual(result["reason"], reason)

    @patch("harness.visual_placement.forward_grip",
           return_value=(4.0, 4.0, 0.0))
    @patch("harness.visual_placement.observe_ground_box")
    def test_released_center_comes_from_fresh_rgb_fit_not_lowering_fk_or_truth(
            self, detector, _forward_grip):
        detector.return_value = observed((0.36, 0.0))
        result = inspect_placement(obs(wrist(), "robot_cam"), obs(nav_zone(), "nav_cam"),
                                   cargo_id="small_box_01", destination_zone="B",
                                   stage="released", held_identity_confirmed=True,
                                   release_commanded=True)
        self.assertEqual(result["status"], "inside", result)
        self.assertEqual(result["calibrated_released_box_center_m"], (0.36, 0.0))
        self.assertEqual(result["calibration_estimate_source"],
                         "fresh_markerless_ground_box_rgb_shape_fit_plus_owned_pwm_fk")
        self.assertNotIn("truth", str(result).lower())
        self.assertIn("not_visually_decoded", result["identity"]["identity_source"])
        _forward_grip.assert_not_called()

    @patch("harness.visual_placement.observe_ground_box", return_value=observed())
    def test_released_requires_commanded_release_and_selected_target_continuity(self, _detector):
        for held, commanded in ((False, True), (True, False)):
            with self.subTest(held=held, commanded=commanded):
                result = inspect_placement(obs(wrist(), "robot_cam"),
                                           obs(nav_zone(), "nav_cam"),
                                           cargo_id="small_box_01", destination_zone="B",
                                           stage="released", held_identity_confirmed=held,
                                           release_commanded=commanded)
                self.assertEqual(result["status"], "uncertain", result)
                self.assertEqual(result["reason"],
                                 "RELEASE_OR_SELECTED_TARGET_CONTINUITY_UNCONFIRMED")


if __name__ == "__main__":
    unittest.main()
