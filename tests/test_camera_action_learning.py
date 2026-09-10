import unittest

import cv2
import numpy as np

from harness.camera_action_learning import CameraActionLearner, visual_features
from harness.camera_pair_policy import CameraPairPlanner


def solid_jpeg(rgb):
    bgr = np.full((24, 24, 3), rgb[::-1], dtype=np.uint8)
    ok, encoded = cv2.imencode(".jpg", bgr)
    assert ok
    return encoded.tobytes()


class Completer:
    def complete(self, messages, *, images):
        return '{"reason":"visible pixels","action":{"kind":"wait"}}'


class CameraActionLearningTests(unittest.TestCase):
    def test_visual_features_are_deterministic_pixel_only_values(self):
        red, green = solid_jpeg((255, 0, 0)), solid_jpeg((0, 255, 0))
        first = visual_features(red, green)
        second = visual_features(red, green)
        np.testing.assert_array_equal(first, second)
        self.assertEqual(first.shape, (24,))
        self.assertGreater(first[0], 0.95)
        self.assertGreater(first[13], 0.95)

    def test_spatial_features_detect_same_color_moving_between_regions(self):
        left = np.zeros((24, 24, 3), dtype=np.uint8)
        right = np.zeros_like(left)
        left[2:8, 2:8] = (0, 0, 255)
        right[2:8, 16:22] = (0, 0, 255)
        _, left_jpeg = cv2.imencode(".jpg", left)
        _, right_jpeg = cv2.imencode(".jpg", right)
        top = solid_jpeg((0, 0, 0))
        self.assertFalse(np.allclose(visual_features(left_jpeg.tobytes(), top), visual_features(right_jpeg.tobytes(), top)))

    def test_ridge_learns_synthetic_command_associated_visual_delta(self):
        learner = CameraActionLearner()
        before = np.zeros(24)
        wait = {"kind": "wait"}
        drive = {"kind": "drive", "forward": 0.05, "turn": 0.0, "duration_s": 0.3}
        for _ in range(8):
            learner.observe(wait, before, before)
            learner.observe(drive, before, np.full(24, 0.2))
        summary = learner.summarize([wait, drive])
        self.assertEqual(summary["sample_count"], 16)
        wait_delta = summary["predictions"][0]["predicted_pixel_feature_delta"]
        drive_delta = summary["predictions"][1]["predicted_pixel_feature_delta"]
        self.assertGreater(np.mean(drive_delta), np.mean(wait_delta) + 0.05)
        self.assertIn("confounded", summary["warning"])
        self.assertIn("do not prove", summary["warning"])

    def test_unseen_action_channel_is_withheld_instead_of_extrapolated(self):
        learner = CameraActionLearner()
        pixels = np.zeros(24)
        drive = {"kind": "drive", "forward": 0.05, "turn": 0.0, "duration_s": 0.3}
        arm = {"kind": "arm", "servo_id": 1, "pulse": 1500}
        for _ in range(4):
            learner.observe(drive, pixels, pixels)
        summary = learner.summarize([drive, arm])
        self.assertIsNotNone(summary["predictions"][0]["predicted_pixel_feature_delta"])
        self.assertIsNone(summary["predictions"][1]["predicted_pixel_feature_delta"])
        self.assertEqual(summary["predictions"][1]["sample_count"], 0)
        self.assertEqual(summary["supported_candidate_count"], 1)

    def test_out_of_range_command_is_withheld_even_with_channel_support(self):
        learner = CameraActionLearner()
        pixels = np.zeros(24)
        observed = {"kind": "drive", "forward": 0.05, "turn": 0.0, "duration_s": 0.3}
        outside = {"kind": "drive", "forward": 0.1, "turn": 0.0, "duration_s": 0.3}
        for _ in range(4):
            learner.observe(observed, pixels, pixels)
        prediction = learner.summarize([outside])["predictions"][0]
        self.assertIsNone(prediction["predicted_pixel_feature_delta"])
        self.assertIn("outside", prediction["unsupported_reason"])

    def test_learned_replay_uses_previous_valid_action_and_current_pixels(self):
        planner = CameraPairPlanner("r1", Completer(), mode="learned")
        planner.prepare_request(solid_jpeg((0, 0, 0)), solid_jpeg((0, 0, 0)))
        planner.record_action({"kind": "drive", "forward": 0.05, "turn": 0.0, "duration_s": 0.3})
        request = planner.prepare_request(solid_jpeg((80, 0, 0)), solid_jpeg((0, 0, 0)))
        self.assertEqual(planner.last_learning_summary["sample_count"], 1)
        self.assertEqual(len(planner._learner.transitions), 1)
        self.assertIn("LEARNED_VISUAL_EFFECT", request["messages"][1]["content"])
        self.assertIn("confounded", request["messages"][1]["content"])

    def test_invalid_or_missing_action_never_creates_transition(self):
        planner = CameraPairPlanner("r1", Completer(), mode="learned")
        planner.prepare_request(solid_jpeg((0, 0, 0)), solid_jpeg((0, 0, 0)))
        with self.assertRaises(ValueError):
            planner.record_action({"kind": "drive", "forward": 9, "turn": 0, "duration_s": 1})
        planner.prepare_request(solid_jpeg((50, 0, 0)), solid_jpeg((0, 0, 0)))
        self.assertEqual(planner.last_learning_summary["sample_count"], 0)

    def test_transition_storage_is_bounded(self):
        learner = CameraActionLearner(max_transitions=3)
        pixels = np.zeros(24)
        for _ in range(5):
            learner.observe({"kind": "wait"}, pixels, pixels)
        self.assertEqual(len(learner.transitions), 3)


if __name__ == "__main__":
    unittest.main()
