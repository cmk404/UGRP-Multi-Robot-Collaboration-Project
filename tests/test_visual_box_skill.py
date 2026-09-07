import base64
import hashlib
import unittest
from unittest import mock

import cv2
import numpy as np

from harness.visual_box_skill import VisualBoxSkill


def cyan_jpeg(*, x=220, y=170, width=120, height=80):
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.rectangle(frame, (x, y), (x + width, y + height), (200, 200, 0), -1)
    ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 100])
    if not ok:
        raise RuntimeError("synthetic JPEG encoding failed")
    return encoded.tobytes()


JPEG = cyan_jpeg()


def observation(frame=1, now=0.0, pose=None, image=JPEG):
    return {
        "robot_id": "r1",
        "frame_id": frame,
        "sim_time": now,
        "image": base64.b64encode(image).decode(),
        "sha256": hashlib.sha256(image).hexdigest(),
        "camera": "robot_cam",
        "actuator_state": {
            "motor_commands": [0.0] * 4,
            "servo_pulses": pose or {"1": 2000, "3": 740, "4": 2320, "5": 1320, "6": 1500},
        },
    }


class _Tracker:
    def __init__(self, result):
        self.result = result

    def observe(self, image):
        return dict(self.result)


class VisualBoxSkillTests(unittest.TestCase):
    def test_explicit_near_field_reacquisition_bypasses_behind_chassis_standoff(self):
        skill = VisualBoxSkill(near_field_reacquisition=True)
        box = {"pixel_centroid": [320, 218.7],
               "marker_pose_camera": {"rotation_rvec_rad": [0.0, 0.0, 0.0]}}
        target = np.asarray([.1675, 0.0, .0195])
        pose = {"1": 2000, "3": 500, "4": 2392, "5": 1320, "6": 1500}
        # Optical marker +Z maps to robot -X: a front-facing box whose normal
        # standoff would be 18 cm behind the chassis.
        axes = ((0., 1., 0.), (0., 0., 1.), (-1., 0., 0.))
        with mock.patch("harness.visual_box_skill.camera_extrinsics", return_value=((0., 0., 0.), axes)):
            action = skill._approach(box, target, pose)
        self.assertEqual(action, {"kind": "wait", "duration": .05})
        self.assertTrue(skill._face_approach)

    def test_default_near_field_target_keeps_standard_face_route(self):
        skill = VisualBoxSkill()
        box = {"pixel_centroid": [320, 218.7],
               "marker_pose_camera": {"rotation_rvec_rad": [0.0, 0.0, 0.0]}}
        target = np.asarray([.1675, 0.0, .0195])
        pose = {"1": 2000, "3": 500, "4": 2392, "5": 1320, "6": 1500}
        axes = ((0., 1., 0.), (0., 0., 1.), (-1., 0., 0.))
        with mock.patch("harness.visual_box_skill.camera_extrinsics", return_value=((0., 0., 0.), axes)):
            action = skill._approach(box, target, pose)
        self.assertEqual(action["kind"], "drive")
        self.assertNotEqual(action["turn"], 0.0)
        self.assertFalse(skill._face_approach)

    def test_missing_target_returns_bounded_search_and_finishes(self):
        skill = VisualBoxSkill()
        skill.tracker = _Tracker({"visible": False})
        first = skill.decide(observation())
        self.assertEqual(first, {"kind": "drive", "fwd": 0.0, "turn": 0.12, "duration": 0.4})
        self.assertEqual(skill.history[0]["phase"], "approach")
        for frame in range(2, 22):
            result = skill.decide(observation(frame, frame * 0.1))
        self.assertEqual(result, {"kind": "finish", "reason": "TARGET_NOT_VISIBLE"})
        self.assertEqual(skill.phase, "finished")

    def test_state_owns_no_simulator_or_actuator_capability(self):
        skill = VisualBoxSkill()
        skill.tracker = _Tracker({"visible": False})
        action = skill.decide(observation())
        self.assertEqual(action["kind"], "drive")
        self.assertLessEqual(action["duration"], 1.0)
        self.assertFalse(hasattr(skill, "world"))
        self.assertFalse(hasattr(skill, "robot"))
        self.assertFalse(hasattr(skill, "model"))
        self.assertFalse(hasattr(skill, "data"))

    def test_attachment_must_survive_left_right_home_before_carry(self):
        skill = VisualBoxSkill()
        skill.phase = "verify_lift"
        pose = {"1": 1500, "3": 757, "4": 1746, "5": 2221, "6": 1500}
        skill.tracker = _Tracker({"visible": False})
        left = skill.decide(observation(1, 0.0, pose))
        right = skill.decide(observation(2, 0.1, {**pose, "6": 1560}))
        home = skill.decide(observation(3, 0.2, {**pose, "6": 1440}))
        ready = skill.decide(observation(4, 0.3, pose))
        self.assertEqual(left, {"kind": "pose", "pulses": {6: 1560, 1: 1500}})
        self.assertEqual(right, {"kind": "pose", "pulses": {6: 1440, 1: 1500}})
        self.assertEqual(home, {"kind": "pose", "pulses": {6: 1500, 1: 1500}})
        self.assertEqual(ready["kind"], "wait")
        self.assertTrue(skill.held)
        drive = skill.decide(observation(5, 0.4, pose))
        self.assertEqual(drive, {"kind": "drive", "fwd": 0.12, "turn": 0.0, "duration": 1.0})

    def test_height_or_visibility_alone_cannot_pass_changed_attachment_mask(self):
        skill = VisualBoxSkill()
        skill.phase = "verify_lift"
        pose = {"1": 1500, "3": 757, "4": 1746, "5": 2221, "6": 1500}
        # Even a high metric target is insufficient: the controlled pan probe
        # must preserve the close cyan object's image-relative mask.
        skill.tracker = _Tracker({
            "visible": True,
            "reprojection_rmse_px": 1.0,
            "box_face_inset_camera_m": [0.0, 0.0, 0.2],
        })
        first = skill.decide(observation(1, 0.0, pose))
        self.assertEqual(first["kind"], "pose")
        moved = cyan_jpeg(x=270, y=170)
        right = skill.decide(observation(2, 0.1, {**pose, "6": 1560}, moved))
        self.assertEqual(right, {"kind": "pose", "pulses": {6: 1440, 1: 1500}})
        opposite = cyan_jpeg(x=170, y=170)
        home = skill.decide(observation(3, 0.2, {**pose, "6": 1440}, opposite))
        self.assertEqual(home, {"kind": "pose", "pulses": {6: 1500, 1: 1500}})
        rejected = skill.decide(observation(4, 0.3, pose))
        self.assertEqual(rejected, {"kind": "finish", "reason": "VISUAL_ATTACHMENT_UNCONFIRMED"})
        self.assertFalse(skill.held)

    def test_carry_monitor_uses_explicit_probe_before_declaring_drop(self):
        skill = VisualBoxSkill(task="short_transfer")
        skill.phase = "carry"
        skill.held = True
        skill.tracker = _Tracker({"visible": False})
        pose = {"1": 1500, "3": 757, "4": 1746, "5": 2221, "6": 1500}
        skill._attachment_image = observation()["image"]
        skill._carry_previous_image = observation()["image"]
        shifted = cyan_jpeg(x=250, y=170)
        probe = skill.decide(observation(1, 0.0, pose, shifted))
        self.assertEqual(probe, {"kind": "pose", "pulses": {6: 1560, 1: 1500}})
        # A world-fixed cyan patch fails both pan probes even though returning
        # home would make it resemble its anchor again.
        skill.decide(observation(2, .1, {**pose, "6": 1560}, cyan_jpeg(x=280, y=170)))
        skill.decide(observation(3, .2, {**pose, "6": 1440}, cyan_jpeg(x=220, y=170)))
        rejected = skill.decide(observation(4, .3, pose, shifted))
        self.assertEqual(rejected, {"kind": "finish", "reason": "VISUAL_LOAD_DROPPED_OR_OCCLUDED"})

    def test_known_surface_fallback_can_confirm_release_without_marker(self):
        skill = VisualBoxSkill()
        skill.phase = "verify_release"
        skill._grasp = {3: 1100, 4: 1900, 5: 2400, 6: 1500}
        skill.tracker = _Tracker({"visible": False})
        surface = {
            "visible": True,
            "confidence": 0.8,
            "estimated_surface_patch_base_m": [0.16, 0.0, 0.031],
            "estimated_box_center_height_m": 0.015,
        }
        with mock.patch("harness.visual_box_skill.observe_known_box_top", return_value=surface):
            result = skill.decide(observation())
        self.assertEqual(result, {"kind": "finish", "reason": "VISUAL_RELEASE_CONFIRMED"})
        self.assertEqual(skill.last_target, (0.16, 0.0, 0.015))

    def test_rejects_stale_malformed_or_foreign_port_observations(self):
        skill = VisualBoxSkill()
        skill.tracker = _Tracker({"visible": False})
        skill.decide(observation())
        with self.assertRaisesRegex(ValueError, "stale"):
            skill.decide(observation())
        bad = observation(2, 0.1)
        bad["sha256"] = "0" * 64
        with self.assertRaisesRegex(ValueError, "hash"):
            skill.decide(bad)
        foreign = observation(2, 0.1)
        foreign["robot_id"] = "r2"
        with self.assertRaisesRegex(ValueError, "another sensor"):
            skill.decide(foreign)
        extra = observation(2, 0.1)
        extra["world_state"] = {}
        with self.assertRaisesRegex(ValueError, "schema"):
            skill.decide(extra)


if __name__ == "__main__":
    unittest.main()
