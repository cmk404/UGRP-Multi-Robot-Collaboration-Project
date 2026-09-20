import json
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

from harness.gemini_proxy import _to_gemini_multi_image_messages
from harness.semantic_pick_policy import PickMatchPlanner
from scripts.audit_pick_match import audit_paths


def jpeg(value):
    ok, encoded = cv2.imencode(".jpg", np.full((16, 20, 3), value, np.uint8))
    assert ok
    return encoded.tobytes()


def make_episode(root, condition, *, seed=7, capture=False, tamper=False):
    root.mkdir(); (root / "inputs").mkdir(); (root / "wire").mkdir()
    config = {"condition": condition, "seed": seed, "model": "test-model", "seconds": 10,
              "max_calls": 2, "max_input_tokens": 1000, "timeout": 4, "capture_only": capture,
              "git_sha": "abc", "physics_paused_during_inference": True,
              "physics": {"impratio": 10, "noslip_iterations": 0}, "seconds_is_sim_budget": True}
    fixture = {"seed": seed, "initial_qpos_sha256": "q", "camera_parameters_sha256": "c",
               "geometry_sha256": "g", "initial_evaluation_state": {
                   "cargo": {"box": {"constraints_active": {"left": False, "right": False}}}}}
    action = ({"kind": "wait", "duration": .3} if condition == "skill"
              else {"kind": "semantic", "unit": "STILL"})
    calls = [] if capture else [{"call": 1, "own_sha256": "", "top_sha256": "", "action": action}]
    result = {"condition": condition, "seed": seed, "git_sha": "abc",
              "reason": "CAPTURE_ONLY" if capture else "SIM_BUDGET",
              "wire_requests": len(calls)}
    for name, value in (("config.json", config), ("fixture-evaluation-only.json", fixture),
                        ("result.json", result), ("calls.json", calls)):
        (root / name).write_text(json.dumps(value))
    (root / "final-fixture-evaluation-only.json").write_text(json.dumps({
        "camera_parameters_sha256": "c", "geometry_sha256": "g"}))
    (root / "evaluation-only.jsonl").write_text(json.dumps({
        "constraints_active": {"left": False, "right": False}}) + "\n")
    if calls:
        own, top = jpeg(10), jpeg(20)
        (root / "inputs/call-001-own.jpg").write_bytes(own)
        (root / "inputs/call-001-top.jpg").write_bytes(top)
        calls[0]["own_sha256"] = __import__("hashlib").sha256(own).hexdigest()
        calls[0]["top_sha256"] = __import__("hashlib").sha256(top).hexdigest()
        (root / "calls.json").write_text(json.dumps(calls))
        request = PickMatchPlanner(object(), condition).prepare_request(own, top)
        wire = {"model": "test-model", "messages": _to_gemini_multi_image_messages(
                    request["messages"], request["images"]), "temperature": .2,
                "max_tokens": 650, "reasoning_effort": "none"}
        if tamper:
            wire["messages"][-1]["content"][0]["text"] = json.dumps({"task": "raise_and_hold_box", "mode": condition,
                "own_model_selected_actions": [], "actor_state": "hidden"})
        (root / "wire/001.json").write_text(json.dumps(wire))


class PickMatchAuditTest(unittest.TestCase):
    def test_valid_paired_live_cohort(self):
        with tempfile.TemporaryDirectory() as tmp:
            base = Path(tmp); skill = base / "skill"; semantic = base / "semantic"
            make_episode(skill, "skill"); make_episode(semantic, "semantic")
            report = audit_paths([skill, semantic])
            self.assertTrue(report["ok"], report)
            self.assertTrue(report["pair_checks"][0]["ok"])

    def test_wire_extra_state_and_active_constraint_fail(self):
        with tempfile.TemporaryDirectory() as tmp:
            episode = Path(tmp) / "episode"
            make_episode(episode, "skill", tamper=True)
            (episode / "evaluation-only.jsonl").write_text(json.dumps({
                "constraints_active": {"weld": True}}) + "\n")
            report = audit_paths([episode])
            codes = {item["code"] for item in report["episodes"][0]["issues"]}
            self.assertIn("WIRE_REQUEST_MISMATCH", codes)
            self.assertIn("EXTRA_ACTOR_STATE_IN_WIRE", codes)
            self.assertIn("CONSTRAINT_ACTIVE", codes)

    def test_single_episode_does_not_require_a_pair(self):
        with tempfile.TemporaryDirectory() as tmp:
            episode = Path(tmp) / "episode"
            make_episode(episode, "semantic")
            report = audit_paths([episode])
            self.assertTrue(report["ok"], report)
            self.assertEqual(report["pair_checks"], [])

    def test_capture_is_not_accepted_as_live_pair(self):
        with tempfile.TemporaryDirectory() as tmp:
            base = Path(tmp); skill = base / "skill"; semantic = base / "semantic"
            make_episode(skill, "skill", capture=True); make_episode(semantic, "semantic", capture=True)
            (skill / "calls.json").unlink(); (semantic / "calls.json").unlink()
            report = audit_paths([skill, semantic])
            self.assertFalse(report["ok"])
            self.assertEqual(report["episodes"][0]["classification"], "capture_only")
            self.assertIn("not a live cohort", report["pair_checks"][0]["issues"][0])


if __name__ == "__main__":
    unittest.main()
