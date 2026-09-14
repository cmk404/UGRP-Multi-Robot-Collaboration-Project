import json
import unittest

import cv2
import numpy as np

from harness.semantic_pick_policy import PickMatchPlanner, SemanticPickInterpreter, PROFILE_HASH
from harness.visual_arm import tool_pose


def jpeg(color):
    ok, encoded = cv2.imencode(".jpg", np.full((30, 40, 3), color, np.uint8))
    assert ok
    return encoded.tobytes()


class Completer:
    def __init__(self, reply):
        self.reply = reply
        self.calls = []

    def complete(self, messages, *, images):
        self.calls.append((messages, images))
        return json.dumps(self.reply)


class PickMatchPlannerTest(unittest.TestCase):
    def test_request_contains_only_fixed_task_images_and_own_selected_history(self):
        c = Completer({"reason": "visible", "action": {"kind": "approach"}})
        planner = PickMatchPlanner(c, "skill")
        planner.record_issued({"kind": "wait", "duration": .3})
        result = planner.decide(jpeg((0, 0, 255)), jpeg((255, 0, 0)))
        self.assertEqual(result["action"], {"kind": "approach"})
        context = json.loads(c.calls[0][0][1]["content"])
        self.assertEqual(set(context), {"task", "mode", "own_model_selected_actions"})
        self.assertEqual(context["own_model_selected_actions"], [{"kind": "wait", "duration": .3}])
        self.assertEqual([image["label"] for image in c.calls[0][1]], ["OWN_CAMERA", "SHARED_TOP_CAMERA"])
        self.assertEqual(planner.last_request["profile_hash"], PROFILE_HASH)

    def test_history_is_last_four_model_selected_actions(self):
        planner = PickMatchPlanner(Completer({}), "semantic")
        units = ["STILL", "MV_UP", "GRASP", "MV_LEFT", "PITCH_DOWN"]
        for unit in units:
            planner.record_issued({"kind": "semantic", "unit": unit})
        self.assertEqual([a["unit"] for a in planner.issued_actions], units[-4:])

    def test_parser_rejects_extra_state_image_and_coordinates(self):
        bad = [
            {"reason": "x", "action": {"kind": "approach"}, "state": "ready"},
            {"reason": "x", "action": {"kind": "approach", "image": "hidden"}},
            {"reason": "x", "action": {"kind": "semantic", "unit": "MV_UP", "x": .1}},
        ]
        for reply in bad:
            mode = "semantic" if reply["action"]["kind"] == "semantic" else "skill"
            with self.subTest(reply=reply), self.assertRaises(ValueError):
                PickMatchPlanner(Completer(reply), mode).decide(jpeg(0), jpeg(1))


class SemanticPickInterpreterTest(unittest.TestCase):
    def setUp(self):
        # Existing visual-box search pose, with room for all tested increments.
        self.initial = {1: 2000, 3: 740, 4: 2320, 5: 1320, 6: 1500}
        self.interpreter = SemanticPickInterpreter(self.initial)

    def test_cartesian_increment_is_command_grounded_and_preserves_pitch(self):
        before = tool_pose(self.initial)
        macro = self.interpreter.compile({"kind": "semantic", "unit": "MV_UP"})
        after = tool_pose(macro["pulses"])
        self.assertLess(abs(after.x_m - before.x_m), .002)
        self.assertLess(abs(after.y_m - before.y_m), .002)
        self.assertLess(abs(after.z_m - before.z_m - .01), .002)
        self.assertLess(abs(after.pitch_deg - before.pitch_deg), 1.0)
        self.assertEqual(self.interpreter.issued_targets, {str(k): v for k, v in self.initial.items()})

    def test_compile_does_not_mutate_until_each_raw_servo_action_is_issued(self):
        macro = self.interpreter.compile({"kind": "semantic", "unit": "MV_FWD"})
        self.assertEqual(self.interpreter.issued_targets["3"], self.initial[3])
        for servo, pulse in macro["pulses"].items():
            raw = ({"kind": "look", "pan_pulse": pulse} if servo == 6 else
                   {"kind": "arm", "servo_id": servo, "pulse": pulse})
            self.interpreter.record_issued(raw)
        self.assertEqual({int(k): v for k, v in self.interpreter.issued_targets.items() if int(k) in macro["pulses"]},
                         macro["pulses"])

    def test_each_translation_and_pitch_unit_meets_fk_contract(self):
        before = tool_pose(self.initial)
        expected = {
            "MV_FWD": (.01, 0, 0), "MV_BACK": (-.01, 0, 0),
            "MV_LEFT": (0, .01, 0), "MV_RIGHT": (0, -.01, 0),
            "MV_UP": (0, 0, .01), "MV_DOWN": (0, 0, -.01),
        }
        for unit, delta in expected.items():
            with self.subTest(unit=unit):
                macro = SemanticPickInterpreter(self.initial).compile({"kind": "semantic", "unit": unit})
                after = tool_pose(macro["pulses"])
                actual = np.array([after.x_m-before.x_m, after.y_m-before.y_m, after.z_m-before.z_m])
                self.assertLess(np.linalg.norm(actual-np.array(delta)), .002)
                self.assertLess(abs(after.pitch_deg-before.pitch_deg), 1.0)
        for unit, delta in (("PITCH_UP", 5), ("PITCH_DOWN", -5)):
            with self.subTest(unit=unit):
                macro = SemanticPickInterpreter(self.initial).compile({"kind": "semantic", "unit": unit})
                after = tool_pose(macro["pulses"])
                self.assertLess(np.linalg.norm(np.array([after.x_m-before.x_m,
                    after.y_m-before.y_m, after.z_m-before.z_m])), .002)
                self.assertLess(abs((after.pitch_deg-before.pitch_deg)-delta), 1.0)

    def test_unreachable_target_raises_without_mutation(self):
        extreme = SemanticPickInterpreter({1: 2000, 3: 500, 4: 500, 5: 500, 6: 500})
        before = extreme.issued_targets
        with self.assertRaises(ValueError):
            extreme.compile({"kind": "semantic", "unit": "MV_RIGHT"})
        self.assertEqual(extreme.issued_targets, before)

    def test_non_arm_units_compile_to_exact_existing_macros(self):
        cases = {
            "GRASP": {"kind": "pose", "pulses": {1: 1500}},
            "RELEASE": {"kind": "pose", "pulses": {1: 2000}},
            "STILL": {"kind": "wait", "duration": .3},
            "BASE_FWD": {"kind": "drive", "fwd": .1, "turn": 0., "duration": .6},
            "BASE_LEFT": {"kind": "drive", "fwd": 0., "turn": .12, "duration": .4},
            "BASE_RIGHT": {"kind": "drive", "fwd": 0., "turn": -.12, "duration": .4},
        }
        for unit, expected in cases.items():
            with self.subTest(unit=unit):
                self.assertEqual(self.interpreter.compile({"kind": "semantic", "unit": unit}), expected)


if __name__ == "__main__":
    unittest.main()
