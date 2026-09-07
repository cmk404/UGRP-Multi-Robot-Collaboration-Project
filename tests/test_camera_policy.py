import base64
import hashlib
import json
import unittest

from harness.camera_policy import CameraPlanner
from harness.coela_modules import PlanningError
from harness.groq import _to_groq_messages


JPEG = b"\xff\xd8camera pixels\xff\xd9"


def observation(robot_id="r1", **updates):
    value = {"robot_id": robot_id, "frame_id": "f-7", "sim_time": 2.5,
             "image": base64.b64encode(JPEG).decode(),
             "sha256": hashlib.sha256(JPEG).hexdigest(), "camera": "robot_cam",
             "actuator_state": {"servo1_pulse": 2000, "moving": False}}
    value.update(updates)
    return value


class Completer:
    model_name = "fixture"
    last_usage = {"prompt_tokens": 12}

    def __init__(self, reply=None):
        self.reply = reply or {"action": {"kind": "drive", "forward": .1, "turn": 0., "duration_s": .4},
                               "private_reason": "표지 구역을 향해 짧게 전진", "message": None}
        self.calls = []

    def complete(self, messages, *, image):
        self.calls.append((messages, image))
        return json.dumps(self.reply, ensure_ascii=False)


class CameraPlannerTests(unittest.TestCase):
    def test_forwards_image_as_multimodal_payload_and_strips_it_from_text(self):
        completer = Completer()
        result = CameraPlanner("r1", completer).decide(observation(), {"notes": ["방금 본 주장"]}, [], destination="B")
        messages, image = completer.calls[0]
        encoded = base64.b64encode(JPEG).decode()
        self.assertEqual(image, "data:image/jpeg;base64," + encoded)
        self.assertNotIn(encoded, json.dumps(messages))
        self.assertNotIn("camera pixels", json.dumps(messages))
        self.assertEqual(result["input_frame"]["sha256"], hashlib.sha256(JPEG).hexdigest())
        self.assertNotIn("image", result["input_frame"])

    def test_data_uri_reaches_real_groq_multimodal_message_branch(self):
        # Long payload guards against the original failure mode where raw
        # base64 was interpreted as an overlong filesystem path.
        jpeg = b"\xff\xd8" + (b"real-camera-pixels" * 400) + b"\xff\xd9"
        encoded = base64.b64encode(jpeg).decode()
        uri = "data:image/jpeg;base64," + encoded
        converted = _to_groq_messages(
            [{"role": "system", "content": "visual only"},
             {"role": "user", "content": "inspect frame"}], uri)
        image_url = converted[-1]["content"][1]["image_url"]["url"]
        self.assertEqual(image_url, uri)

    def test_rejects_foreign_invalid_or_unknown_frame_fields(self):
        planner = CameraPlanner("r1", Completer())
        with self.assertRaisesRegex(ValueError, "FOREIGN_FRAME"):
            planner.decide(observation("r2"), {}, [])
        with self.assertRaisesRegex(ValueError, "IMAGE_HASH_MISMATCH"):
            planner.decide(observation(sha256="0" * 64), {}, [])
        with self.assertRaisesRegex(ValueError, "INVALID_OBSERVATION_FIELDS"):
            planner.decide(observation(world_state={}), {}, [])

    def test_rejects_bad_action_bounds_nan_and_extra_fields(self):
        invalid = [
            {"kind": "drive", "forward": .151, "turn": 0, "duration_s": .2},
            {"kind": "drive", "forward": float("nan"), "turn": 0, "duration_s": .2},
            {"kind": "look", "pan_pulse": 499},
            {"kind": "look", "pan_pulse": 1500, "tilt_pulse": 1500},
            {"kind": "arm", "servo_id": 2, "pulse": 1500},
            {"kind": "arm", "servo_id": 6, "pulse": 1500},
            {"kind": "wait", "target": "B"},
        ]
        for action in invalid:
            with self.subTest(action=action), self.assertRaises(PlanningError):
                CameraPlanner("r1", Completer({"action": action, "private_reason": "사진 근거", "message": None})).decide(observation(), {}, [])

    def test_rejects_spatial_oracle_context_before_model_call(self):
        completer = Completer()
        planner = CameraPlanner("r1", completer)
        for memory, messages in (({"peer_positions": {"r2": [1, 2]}}, []), ({}, [{"cargo": [{"x": 1}]}])):
            with self.assertRaisesRegex(ValueError, "SPATIAL_ORACLE_FORBIDDEN"):
                planner.decide(observation(), memory, messages)
        self.assertEqual(completer.calls, [])

    def test_natural_message_is_korean_and_excludes_sender(self):
        good = {"action": {"kind": "wait"}, "private_reason": "응답 대기",
                "message": {"recipients": ["r2"], "content": "r2, 현재 사진을 확인해 주세요", "kind": "help_request", "observed_ids": []}}
        self.assertEqual(CameraPlanner("r1", Completer(good)).decide(observation(), {}, [])["message"]["recipients"], ["r2"])
        for content, recipients in (("please look", ["r2"]), ("확인해 주세요", ["r1"])):
            bad = {**good, "message": {**good["message"], "content": content, "recipients": recipients}}
            with self.assertRaises(PlanningError):
                CameraPlanner("r1", Completer(bad)).decide(observation(), {}, [])

    def test_structured_message_reuses_communication_contract(self):
        good = {"action": {"kind": "wait"}, "private_reason": "로컬 사진 판단",
                "message": {"recipients": ["r2"], "content": {"kind": "intent",
                    "cargo_id": "small_box_01", "participants": ["r1"]},
                    "observed_ids": ["small_box_01"]}}
        result = CameraPlanner("r1", Completer(good)).decide(
            observation(), {}, [], mode="structured")
        self.assertEqual(result["message"]["content"]["cargo_id"], "small_box_01")
        bad = {**good, "message": {**good["message"],
            "content": {"kind": "intent", "text": "free text backdoor"}}}
        with self.assertRaises(PlanningError):
            CameraPlanner("r1", Completer(bad)).decide(
                observation(), {}, [], mode="structured")

    def test_none_mode_requires_message_silence(self):
        silent = {"action": {"kind": "wait"}, "private_reason": "사진 대기", "message": None}
        self.assertIsNone(CameraPlanner("r1", Completer(silent)).decide(
            observation(), {}, [], mode="none")["message"])
        speaking = {**silent, "message": {"recipients": ["r2"],
            "content": "r2에게 보고합니다", "kind": "intent", "observed_ids": []}}
        with self.assertRaises(PlanningError):
            CameraPlanner("r1", Completer(speaking)).decide(
                observation(), {}, [], mode="none")

    def test_servo_one_gripper_action_is_accepted(self):
        reply = {"action": {"kind": "arm", "servo_id": 1, "pulse": 1500},
                 "private_reason": "사진을 보고 그리퍼를 조정", "message": None}
        self.assertEqual(CameraPlanner("r1", Completer(reply)).decide(observation(), {}, [])["action"]["servo_id"], 1)


if __name__ == "__main__":
    unittest.main()
