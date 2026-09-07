import base64
import hashlib
import unittest

import cv2
import numpy as np

from harness.visual_navigation import VisualNavigator


def observation(frame, *, rid="r1", seq=1):
    ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 96])
    assert ok
    data = encoded.tobytes()
    return {"robot_id": rid, "frame_id": seq, "sim_time": float(seq),
            "image": base64.b64encode(data).decode(),
            "sha256": hashlib.sha256(data).hexdigest(), "camera": "nav_cam",
            "actuator_state": {"motor_commands": [0, 0, 0, 0], "servo_pulses": {}}}


def scene():
    return np.full((480, 640, 3), (45, 45, 45), np.uint8)


class VisualNavigatorTest(unittest.TestCase):
    def test_centered_blue_goal_steers_forward_from_pixels(self):
        image = scene()
        cv2.rectangle(image, (220, 300), (420, 430), (255, 0, 0), -1)
        nav = VisualNavigator("A", "r1")
        action = nav.decide(observation(image))
        self.assertEqual(action["kind"], "drive")
        self.assertGreater(action["fwd"], 0)
        self.assertAlmostEqual(action["turn"], 0.0, delta=.03)
        self.assertTrue(nav.last_observation["goal"]["regions"])

    def test_yellow_goal_is_not_confused_with_orange_obstacle(self):
        image = scene()
        cv2.rectangle(image, (30, 260), (250, 470), (0, 150, 255), -1)  # orange BGR
        cv2.rectangle(image, (360, 285), (620, 455), (0, 255, 255), -1)  # yellow
        nav = VisualNavigator("C", "r1")
        nav.decide(observation(image))
        seen = nav.last_observation
        self.assertTrue(seen["goal"]["regions"])
        self.assertTrue(seen["obstacles"])
        self.assertGreater(seen["goal"]["regions"][0]["pixel_bbox"][0], 300)

    def test_cyan_carried_load_is_not_goal_or_obstacle(self):
        image = scene()
        cv2.rectangle(image, (170, 280), (470, 479), (255, 255, 0), -1)
        nav = VisualNavigator("C", "r1")
        nav.decide(observation(image))
        self.assertFalse(nav.last_observation["goal"]["regions"])
        self.assertFalse(nav.last_observation["obstacles"])
        self.assertFalse(nav.last_observation["peers"])

    def test_close_orange_obstacle_causes_persistent_same_side_detour(self):
        image = scene()
        cv2.rectangle(image, (245, 335), (420, 479), (0, 150, 255), -1)
        nav = VisualNavigator("B", "r1")
        first = nav.decide(observation(image, seq=1))
        second = nav.decide(observation(image, seq=2))
        self.assertEqual(first["kind"], "drive")
        self.assertNotEqual(first["turn"], 0)
        self.assertEqual(np.sign(first["turn"]), np.sign(second["turn"]))
        self.assertLessEqual(first["fwd"], .03)

    def test_detour_persists_while_obstacle_is_laterally_clear_but_still_ahead(self):
        nav = VisualNavigator("A", "r1")
        blocked = scene()
        cv2.rectangle(blocked, (245, 335), (420, 479), (0, 150, 255), -1)
        first = nav.decide(observation(blocked, seq=1))
        lateral = scene()
        cv2.rectangle(lateral, (0, 230), (30, 355), (0, 150, 255), -1)
        second = nav.decide(observation(lateral, seq=2))
        third = nav.decide(observation(lateral, seq=3))
        self.assertEqual(np.sign(first["turn"]), np.sign(second["turn"]))
        self.assertEqual(np.sign(second["turn"]), np.sign(third["turn"]))
        self.assertEqual(second["fwd"], .08)

    def test_detour_requires_three_clear_frames_before_goal_steering(self):
        nav = VisualNavigator("A", "r1")
        blocked = scene()
        cv2.rectangle(blocked, (245, 335), (420, 479), (0, 150, 255), -1)
        nav.decide(observation(blocked, seq=1))
        goal = scene()
        cv2.rectangle(goal, (220, 300), (420, 430), (255, 0, 0), -1)
        a = nav.decide(observation(goal, seq=2))
        b = nav.decide(observation(goal, seq=3))
        c = nav.decide(observation(goal, seq=4))
        self.assertEqual((a["fwd"], b["fwd"]), (.06, .06))
        self.assertEqual(c["kind"], "drive")
        self.assertAlmostEqual(c["turn"], 0.0, delta=.002)

    def test_visible_close_magenta_peer_yields_then_has_finite_escape(self):
        image = scene()
        cv2.rectangle(image, (270, 330), (370, 470), (255, 0, 255), -1)
        nav = VisualNavigator("A", "r1")
        actions = [nav.decide(observation(image, seq=i)) for i in range(1, 5)]
        self.assertEqual([a["kind"] for a in actions[:3]], ["wait"] * 3)
        self.assertTrue(all(a["duration"] == .5 for a in actions[:3]))
        self.assertEqual(actions[3]["kind"], "drive")
        self.assertNotEqual(actions[3]["turn"], 0)
        self.assertIsNotNone(nav.last_observation["peers"][0]["conservative_range_m"])

    def test_large_near_goal_finishes_with_exact_reason(self):
        image = scene()
        cv2.rectangle(image, (105, 260), (535, 479), (255, 0, 0), -1)
        nav = VisualNavigator("A", "r1")
        self.assertEqual(nav.decide(observation(image)),
                         {"kind": "finish", "reason": "NAVIGATION_ARRIVED"})

    def test_bottom_touch_alone_does_not_arrive_without_zone_center_depth(self):
        image = scene()
        cv2.rectangle(image, (105, 350), (535, 479), (255, 0, 0), -1)
        nav = VisualNavigator("A", "r1")
        action = nav.decide(observation(image))
        self.assertEqual(action["kind"], "drive")
        center = nav.last_observation["goal"]["regions"][0]["estimated_zone_center_m"]
        self.assertLess(center[0], .18)

    def test_independent_instances_do_not_share_search_or_history(self):
        blank = scene()
        one = VisualNavigator("A", "r1")
        two = VisualNavigator("B", "r2")
        a1 = one.decide(observation(blank, rid="r1", seq=1))
        b1 = two.decide(observation(blank, rid="r2", seq=1))
        self.assertEqual(a1, b1)
        one.decide(observation(blank, rid="r1", seq=2))
        self.assertEqual(len(one.history), 2)
        self.assertEqual(len(two.history), 1)

    def test_missing_goal_scans_in_place_then_finishes_without_blind_drive(self):
        nav = VisualNavigator("A", "r1")
        actions = [nav.decide(observation(scene(), seq=i)) for i in range(1, 18)]
        scans = actions[:-1]
        self.assertTrue(all(a == {"kind": "drive", "fwd": 0.0,
                                        "turn": .2, "duration": 1.0} for a in scans))
        self.assertEqual(actions[-1],
                         {"kind": "finish", "reason": "NAVIGATION_SEARCH_EXHAUSTED"})

    def test_rejects_wrong_camera_and_hidden_fields(self):
        nav = VisualNavigator("A", "r1")
        obs = observation(scene())
        obs["camera"] = "robot_cam"
        with self.assertRaisesRegex(ValueError, "INVALID_NAVIGATION_CAMERA"):
            nav.decide(obs)
        obs = observation(scene())
        obs["position"] = [0, 0]
        with self.assertRaisesRegex(ValueError, "FIELDS"):
            nav.decide(obs)

    def test_history_is_bounded_and_older_numeric_frames_are_stale(self):
        nav = VisualNavigator("A", "r1")
        for seq in range(1, 28):
            nav.decide(observation(scene(), seq=seq))
        self.assertEqual(len(nav.history), 24)
        self.assertEqual(len(nav.last_observation["history"]), 24)
        with self.assertRaisesRegex(ValueError, "STALE"):
            nav.decide(observation(scene(), seq=26))


if __name__ == "__main__":
    unittest.main()
