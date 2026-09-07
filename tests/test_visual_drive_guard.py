from __future__ import annotations

import base64
import hashlib
import unittest

import cv2
import numpy as np

from harness.visual_drive_guard import validate_visual_drive


def observation(frame: np.ndarray) -> dict:
    ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 98])
    assert ok
    payload = encoded.tobytes()
    return {"robot_id": "r1", "frame_id": 1, "sim_time": 0.1,
            "image": base64.b64encode(payload).decode(),
            "sha256": hashlib.sha256(payload).hexdigest(), "camera": "nav_cam",
            "actuator_state": {}}


def scene() -> np.ndarray:
    return np.full((480, 640, 3), 40, np.uint8)


class VisualDriveGuardTests(unittest.TestCase):
    def test_close_orange_in_inflated_corridor_vetoes_llm_forward(self):
        frame = scene()
        cv2.rectangle(frame, (250, 375), (390, 479), (0, 150, 255), -1)
        result = validate_visual_drive(observation(frame),
                                       {"kind": "drive", "forward": .1, "turn": 0})
        self.assertFalse(result["allowed"])
        self.assertEqual(result["reason"], "ORANGE_OBSTACLE_IN_FORWARD_FOOTPRINT")
        self.assertEqual(result["evidence"]["image_sha256"], observation(frame)["sha256"])
        self.assertLess(result["evidence"]["blocking_region"]["estimated_forward_m"], .45)

    def test_far_or_lateral_orange_does_not_replace_llm_steering(self):
        frame = scene()
        cv2.rectangle(frame, (250, 230), (390, 300), (0, 150, 255), -1)
        cv2.rectangle(frame, (0, 390), (35, 479), (0, 150, 255), -1)
        action = {"kind": "drive", "fwd": .12, "turn": -.08}
        result = validate_visual_drive(observation(frame), action)
        self.assertTrue(result["allowed"])
        self.assertEqual(result["reason"], "VISUAL_FORWARD_CLEAR")
        self.assertNotIn("action", result["evidence"])

    def test_close_magenta_peer_vetoes_forward_with_height_profile(self):
        frame = scene()
        cv2.rectangle(frame, (270, 320), (370, 450), (255, 0, 255), -1)
        result = validate_visual_drive(observation(frame),
                                       {"kind": "drive", "fwd": .06, "turn": .1})
        self.assertFalse(result["allowed"])
        self.assertEqual(result["reason"], "MAGENTA_PEER_IN_FORWARD_FOOTPRINT")
        self.assertIn("peer-height", result["evidence"]["blocking_region"]["range_model"])

    def test_turn_in_place_is_allowed_with_nonoverlapping_visible_obstacle(self):
        frame = scene()
        cv2.rectangle(frame, (245, 260), (395, 340), (0, 150, 255), -1)
        result = validate_visual_drive(observation(frame),
                                       {"kind": "drive", "forward": 0, "turn": .2})
        self.assertEqual((result["allowed"], result["reason"]),
                         (True, "IN_PLACE_TURN_VISUALLY_CLEAR"))

    def test_missing_image_fails_drive_without_fallback(self):
        result = validate_visual_drive({}, {"kind": "drive", "fwd": .1, "turn": 0})
        self.assertFalse(result["allowed"])
        self.assertEqual(result["evidence"]["inspection"], "no fallback source")

    def test_non_drive_is_allowed_without_camera_and_colors_stay_separate(self):
        self.assertTrue(validate_visual_drive({}, {"kind": "wait", "duration": .2})["allowed"])
        frame = scene()
        cv2.rectangle(frame, (220, 360), (310, 479), (0, 255, 255), -1)  # yellow goal
        cv2.rectangle(frame, (330, 360), (420, 479), (255, 255, 0), -1)  # cyan load
        result = validate_visual_drive(observation(frame), {"kind": "drive", "fwd": .1, "turn": 0})
        self.assertTrue(result["allowed"])
        self.assertFalse(result["evidence"]["orange_regions"])
        self.assertFalse(result["evidence"]["magenta_peer_regions"])


if __name__ == "__main__":
    unittest.main()
