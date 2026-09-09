from __future__ import annotations

import base64
import hashlib
import json
import unittest

import cv2
import numpy as np

from harness.navigation_evidence import navigation_evidence
from harness.visual_drive_guard import validate_visual_drive


def observation(frame: np.ndarray, **extra) -> dict:
    ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 98])
    assert ok
    payload = encoded.tobytes()
    return {"camera": "nav_cam", "image": base64.b64encode(payload).decode(),
            "sha256": hashlib.sha256(payload).hexdigest(), **extra}


def scene() -> np.ndarray:
    return np.full((480, 640, 3), 40, np.uint8)


class NavigationEvidenceTests(unittest.TestCase):
    def test_only_rgb_bytes_affect_evidence(self):
        frame = scene()
        cv2.rectangle(frame, (80, 250), (180, 400), (0, 150, 255), -1)
        first = navigation_evidence(observation(frame, robot_id="r1", actuator_state={"x": 1}), "A")
        second = navigation_evidence(observation(frame, robot_id="other", hidden_pose=[99, 99]), "A")
        self.assertEqual(first, second)
        self.assertNotIn("hidden_pose", json.dumps(first))

    def test_forward_status_exactly_matches_existing_guard(self):
        frame = scene()
        cv2.rectangle(frame, (250, 375), (390, 479), (0, 150, 255), -1)
        obs = observation(frame)
        expected = validate_visual_drive(obs, {"kind": "drive", "fwd": .01, "turn": 0.0})
        actual = navigation_evidence(obs, "B")["forward_stop_check"]
        self.assertEqual((actual["vetoed"], actual["reason"]),
                         (not expected["allowed"], expected["reason"]))
        self.assertIn("does not establish a safe route", actual["meaning"])

    def test_requested_target_is_visible_and_other_zone_is_absent(self):
        frame = scene()
        cv2.rectangle(frame, (160, 280), (480, 470), (255, 30, 20), -1)
        blue = navigation_evidence(observation(frame), "A")["target_floor"]
        green = navigation_evidence(observation(frame), "B")["target_floor"]
        self.assertTrue(blue["visible"])
        self.assertGreater(blue["region"]["area_fraction"], .1)
        self.assertFalse(green["visible"])
        self.assertIsNone(green["region"])

    def test_growing_obstacle_reports_positive_screen_area_change(self):
        before = scene()
        after = scene()
        cv2.rectangle(before, (280, 330), (360, 410), (0, 150, 255), -1)
        cv2.rectangle(after, (220, 290), (420, 479), (0, 150, 255), -1)
        result = navigation_evidence(observation(after), "C", observation(before))
        self.assertGreater(result["screen_change"]["largest_obstacle_area_fraction_delta"], .09)
        self.assertIn("no object identity tracking", result["screen_change"]["meaning"])
        self.assertIn("not physical distance", result["screen_change"]["meaning"])

    def test_previous_rgb_from_named_other_robot_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "PREVIOUS_NAVIGATION_ROBOT_MISMATCH"):
            navigation_evidence(observation(scene(), robot_id="r1"), "A",
                                observation(scene(), robot_id="r2"))


if __name__ == "__main__":
    unittest.main()
