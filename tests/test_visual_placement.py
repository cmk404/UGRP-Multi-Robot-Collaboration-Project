from __future__ import annotations

import base64
import hashlib
import json
from pathlib import Path
import unittest

import cv2
import numpy as np

from harness.visual_placement import _project_ground, inspect_placement


POSE = {"3": 705, "4": 1742, "5": 2207, "6": 1554}


def obs(frame: np.ndarray, camera: str) -> dict:
    ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 98])
    assert ok
    data = encoded.tobytes()
    return {"camera": camera, "image": base64.b64encode(data).decode(),
            "sha256": hashlib.sha256(data).hexdigest(),
            "actuator_state": {"servo_pulses": POSE}}


def wrist_with_marker(marker_id: int = 15) -> np.ndarray:
    frame = np.full((480, 640, 3), 210, np.uint8)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker = cv2.aruco.generateImageMarker(dictionary, marker_id, 150)
    marker = cv2.copyMakeBorder(marker, 24, 24, 24, 24, cv2.BORDER_CONSTANT, value=255)
    frame[135:333, 221:419] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
    return frame


def nav_green_everywhere() -> np.ndarray:
    return np.full((480, 640, 3), (35, 145, 35), np.uint8)


def nav_zone(center=(0.36, 0.0)) -> np.ndarray:
    frame = np.full((480, 640, 3), 40, np.uint8)
    cx, cy = center; half = .41
    metric = ((cx-half, cy-half), (cx+half, cy-half),
              (cx+half, cy+half), (cx-half, cy+half))
    pixels = np.asarray([_project_ground(point, 640, 480) for point in metric], np.int32)
    cv2.fillConvexPoly(frame, pixels, (35, 145, 35))
    return frame


class VisualPlacementTests(unittest.TestCase):
    def test_before_release_reconstructs_occluded_near_edge_for_true_inside(self):
        result = inspect_placement(obs(wrist_with_marker(), "robot_cam"),
                                   obs(nav_zone(), "nav_cam"),
                                   perception_mode="fiducial", cargo_id="small_box_02", destination_zone="B",
                                   stage="before_release")
        self.assertEqual(result["status"], "inside", result)
        self.assertEqual(result["reason"], "TARGET_FOOTPRINT_INSIDE_VISIBLE_ZONE_WITH_MARGIN")
        self.assertEqual(result["status"], "inside")
        self.assertTrue(result["identity"]["confirmed"])
        self.assertIn("calibrated_lowering_center_m", result)
        json.dumps(result)

    def test_released_visible_target_inside_zone_has_positive_evidence(self):
        result = inspect_placement(obs(wrist_with_marker(), "robot_cam"),
                                   obs(nav_zone(), "nav_cam"),
                                   perception_mode="fiducial", cargo_id="small_box_02", destination_zone="B",
                                   stage="released")
        self.assertEqual(result["status"], "inside", result)
        self.assertGreaterEqual(min(result["calibrated_footprint_signed_margin_m"]), 0.0)
        self.assertIn("calibrated_released_box_center_m", result)

    def test_visible_marker_and_clearly_wrong_floor_is_outside(self):
        nav = nav_zone((.80, .40))
        result = inspect_placement(obs(wrist_with_marker(), "robot_cam"), obs(nav, "nav_cam"),
                                   perception_mode="fiducial", cargo_id="small_box_02", destination_zone="B", stage="released")
        self.assertEqual(result["status"], "outside", result)
        self.assertEqual(result["reason"], "TARGET_FOOTPRINT_OUTSIDE_VISIBLE_ZONE_MARGIN")

    def test_wrong_marker_identity_is_conservatively_uncertain(self):
        result = inspect_placement(obs(wrist_with_marker(14), "robot_cam"),
                                   obs(nav_green_everywhere(), "nav_cam"),
                                   perception_mode="fiducial", cargo_id="small_box_02", destination_zone="B")
        self.assertEqual(result["status"], "uncertain")
        self.assertEqual(result["reason"], "TARGET_MARKER_OCCLUDED_OR_UNCONFIRMED")

    def test_confirmed_held_identity_allows_occluded_marker_square_fit(self):
        blank = np.full((480, 640, 3), (255, 255, 0), np.uint8)
        result = inspect_placement(obs(blank, "robot_cam"), obs(nav_zone(), "nav_cam"),
                                   perception_mode="fiducial", cargo_id="small_box_02", destination_zone="B",
                                   held_identity_confirmed=True)
        self.assertEqual(result["status"], "inside", result)
        self.assertEqual(result["identity"]["provenance"],
                         "caller_confirmed_tracked_id_grasp_plus_continuous_visual_grip")

    def test_released_without_observable_target_position_is_uncertain(self):
        blank = np.full((480, 640, 3), 80, np.uint8)
        result = inspect_placement(obs(blank, "robot_cam"), obs(nav_green_everywhere(), "nav_cam"),
                                   perception_mode="fiducial", cargo_id="small_box_02", destination_zone="B", stage="released")
        self.assertEqual((result["status"], result["reason"]),
                         ("uncertain", "TARGET_POSITION_UNOBSERVABLE"))

    def test_recorded_r2_call68_is_not_false_inside(self):
        root = Path(__file__).resolve().parents[1]
        folder = root / "outputs/warehouse_research/gemini38-team-dev-01/inputs/r2"
        if not folder.exists():
            self.skipTest("recorded Gemini development frames unavailable")
        def recorded(path: Path, camera: str) -> dict:
            data = path.read_bytes()
            return {"camera": camera, "image": base64.b64encode(data).decode(),
                    "sha256": hashlib.sha256(data).hexdigest(),
                    "actuator_state": {"servo_pulses": POSE}}
        result = inspect_placement(recorded(folder / "0181-wrist.jpg", "robot_cam"),
                                   recorded(folder / "0181-nav.jpg", "nav_cam"),
                                   perception_mode="fiducial", cargo_id="small_box_02", destination_zone="B",
                                   stage="before_release", held_identity_confirmed=True)
        self.assertNotEqual(result["status"], "inside", result)
        self.assertEqual(result["status"], "outside")
        self.assertEqual(result["reason"], "TARGET_FOOTPRINT_OUTSIDE_VISIBLE_ZONE_MARGIN")
        self.assertEqual(result["wrist_sha256"],
                         "189356a6c569c46b1a8188a489299b34f95b8303fce84c2a4a7727d2791747f7")
        self.assertEqual(result["nav_sha256"],
                         "367561b6c250b18165725b8f27aeea43eed1fd003bedbc88a3caa24d99eb9e30")

    def test_recorded_r2_post_release_marker_confirms_outside(self):
        root = Path(__file__).resolve().parents[1]
        folder = root / "outputs/warehouse_research/gemini38-team-dev-01/inputs/r2"
        if not folder.exists():
            self.skipTest("recorded Gemini development frames unavailable")
        released_pose = {"1": 2000, "3": 500, "4": 2392, "5": 1320, "6": 1500}
        def recorded(path: Path, camera: str) -> dict:
            data = path.read_bytes()
            return {"camera": camera, "image": base64.b64encode(data).decode(),
                    "sha256": hashlib.sha256(data).hexdigest(),
                    "actuator_state": {"servo_pulses": released_pose}}
        result = inspect_placement(recorded(folder / "0200-wrist.jpg", "robot_cam"),
                                   recorded(folder / "0200-nav.jpg", "nav_cam"),
                                   perception_mode="fiducial", cargo_id="small_box_02", destination_zone="B", stage="released")
        self.assertEqual(result["status"], "outside", result)
        self.assertTrue(result["identity"]["confirmed"])
        self.assertAlmostEqual(result["identity"]["reprojection_rmse_px"], .4434, places=3)


if __name__ == "__main__":
    unittest.main()
