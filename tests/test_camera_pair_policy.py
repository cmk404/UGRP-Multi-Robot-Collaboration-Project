import base64
import json
import math
import unittest

from harness.camera_pair_policy import CameraPairPlanner


def jpeg(payload: bytes) -> bytes:
    return b"\xff\xd8" + payload + b"\xff\xd9"


class Completer:
    def __init__(self, response=None):
        self.response = response or {
            "reason": "orange beam is visible ahead",
            "action": {"kind": "drive", "forward": 0.1, "turn": 0, "duration_s": 0.4},
        }
        self.calls = []

    def complete(self, messages, *, images):
        self.calls.append({"messages": messages, "images": images})
        if isinstance(self.response, str):
            return self.response
        return json.dumps(self.response, allow_nan=True)


class CameraPairPlannerTests(unittest.TestCase):
    def test_grasp_task_changes_only_static_instruction(self):
        completer = Completer()
        CameraPairPlanner('r1', completer, task='grasp').decide(jpeg(b'own'), jpeg(b'top'))
        request = completer.calls[0]
        self.assertIn('grasp and lift', request['messages'][0]['content'])
        self.assertEqual(len(request['images']), 2)
        self.assertNotIn('world_state', json.dumps(request))
        with self.assertRaises(ValueError):
            CameraPairPlanner('r1', completer, task='hidden_target')

    def test_request_contains_only_static_text_and_exact_two_images(self):
        own, overhead = jpeg(b"r1-private-pixels"), jpeg(b"shared-top-pixels")
        completer = Completer()
        planner = CameraPairPlanner("r1", completer)
        action = planner.decide(own, overhead)

        self.assertEqual(action["kind"], "drive")
        self.assertEqual(planner.last_request, completer.calls[0])
        request = planner.last_request
        self.assertEqual([item["label"] for item in request["images"]], ["OWN_VIEW", "OVERHEAD"])
        decoded = [base64.b64decode(item["image"].split(",", 1)[1]) for item in request["images"]]
        self.assertEqual(decoded, [own, overhead])
        text = json.dumps(request["messages"])
        self.assertNotIn(base64.b64encode(own).decode(), text)
        other = Completer()
        CameraPairPlanner("r1", other).decide(jpeg(b"different-own"), jpeg(b"different-top"))
        self.assertEqual(request["messages"], other.calls[0]["messages"])
        self.assertEqual(set(request), {"messages", "images"})

    def test_each_planner_receives_only_its_constructed_own_frame(self):
        top = jpeg(b"top")
        fixtures = {"r1": jpeg(b"only-r1"), "r3": jpeg(b"only-r3")}
        calls = {}
        for robot_id in ("r1", "r3"):
            completer = Completer()
            CameraPairPlanner(robot_id, completer).decide(fixtures[robot_id], top)
            calls[robot_id] = completer.calls[0]["images"]
        for robot_id, other in (("r1", "r3"), ("r3", "r1")):
            own_uri = calls[robot_id][0]["image"]
            self.assertEqual(base64.b64decode(own_uri.split(",", 1)[1]), fixtures[robot_id])
            self.assertNotIn(base64.b64encode(fixtures[other]).decode(), json.dumps(calls[robot_id]))

    def test_rejects_non_jpeg_or_non_bytes_before_call(self):
        completer = Completer()
        planner = CameraPairPlanner("r1", completer)
        for own, top in ((b"not jpeg", jpeg(b"top")), (jpeg(b"own"), bytearray(jpeg(b"top")))):
            with self.subTest(own=own), self.assertRaises(ValueError):
                planner.decide(own, top)
        self.assertEqual(completer.calls, [])

    def test_rejects_extra_response_keys_including_coordinates(self):
        responses = [
            {"reason": "x", "action": {"kind": "wait"}, "coords": [1, 2]},
            {"reason": "x", "action": {"kind": "wait", "target_pose": [1, 2]}},
        ]
        for response in responses:
            with self.subTest(response=response), self.assertRaises(ValueError):
                CameraPairPlanner("r1", Completer(response)).decide(jpeg(b"own"), jpeg(b"top"))

    def test_action_bounds_nonfinite_and_types_are_strict(self):
        invalid = [
            {"kind": "drive", "forward": 0.151, "turn": 0, "duration_s": 1},
            {"kind": "drive", "forward": 0.1, "turn": math.inf, "duration_s": 1},
            {"kind": "drive", "forward": 0.1, "turn": 0, "duration_s": math.nan},
            {"kind": "look", "pan_pulse": 499},
            {"kind": "arm", "servo_id": 2, "pulse": 1500},
            {"kind": "arm", "servo_id": 1, "pulse": 2501},
            {"kind": "wait", "world": {}},
        ]
        for action in invalid:
            response = {"reason": "pixels", "action": action}
            with self.subTest(action=action), self.assertRaises(ValueError):
                CameraPairPlanner("r3", Completer(response)).decide(jpeg(b"own"), jpeg(b"top"))

    def test_saves_raw_response_and_returns_action_only(self):
        response = {"reason": "visible pixels", "action": {"kind": "arm", "servo_id": 1, "pulse": 1500}}
        planner = CameraPairPlanner("r3", Completer(response))
        self.assertEqual(planner.decide(jpeg(b"own"), jpeg(b"top")), response["action"])
        self.assertEqual(json.loads(planner.last_response), response)

    def test_accepts_only_optional_standard_json_fence(self):
        body = '{"reason":"pixels","action":{"kind":"wait"}}'
        for raw in (f"```json\n{body}\n```", f"```\n{body}\n```"):
            with self.subTest(raw=raw):
                action = CameraPairPlanner("r1", Completer(raw)).decide(jpeg(b"own"), jpeg(b"top"))
                self.assertEqual(action, {"kind": "wait"})
        for raw in (f"result:\n```json\n{body}\n```", f"```json\n{body}\n```\nextra"):
            with self.subTest(raw=raw), self.assertRaises(ValueError):
                CameraPairPlanner("r1", Completer(raw)).decide(jpeg(b"own"), jpeg(b"top"))


if __name__ == "__main__":
    unittest.main()
