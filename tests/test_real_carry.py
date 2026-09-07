from __future__ import annotations

import json
import subprocess
import unittest

from harness.real_carry import parse_carry_evidence, read_carry_evidence
from harness.state import StateEstimator


def payload(*, created_at=1000.0, color="red", pulse=1500):
    return json.dumps({
        "version": 1,
        "source": "precision_pick_floor_clear",
        "grasp_state": "PROBABLE_HELD",
        "target_color": color,
        "gripper_pulse": pulse,
        "created_at": created_at,
    })


class RealCarryEvidenceTests(unittest.TestCase):
    def test_fresh_handoff_is_valid(self):
        e = parse_carry_evidence(payload(), now=1100.0)
        self.assertTrue(e.known)
        self.assertTrue(e.valid)
        self.assertEqual(e.target_color, "red")
        self.assertAlmostEqual(e.age_s, 100.0)

    def test_stale_handoff_is_known_but_invalid(self):
        e = parse_carry_evidence(payload(), now=1701.0)
        self.assertTrue(e.known)
        self.assertFalse(e.valid)
        self.assertEqual(e.reason, "stale")

    def test_missing_handoff_is_known_absence(self):
        e = parse_carry_evidence("", now=1000.0)
        self.assertTrue(e.known)
        self.assertFalse(e.valid)
        self.assertEqual(e.reason, "missing")

    def test_transport_failure_is_unknown_not_negative_evidence(self):
        def boom(*args, **kwargs):
            raise subprocess.TimeoutExpired(args[0], 5)
        e = read_carry_evidence(ssh_run=boom, ssh_args=("ugrp1", []))
        self.assertFalse(e.known)
        self.assertFalse(e.valid)
        self.assertEqual(e.reason, "transport_unavailable")

    def test_stale_known_evidence_clears_only_probable_hold(self):
        est = StateEstimator()
        est.state.grasp.state = "PROBABLE_HELD"
        est.state.grasp.held_object_color = "red"
        est.state.grasp.confidence = .65
        self.assertTrue(est.reconcile_probable_carry({
            "known": True, "valid": False, "reason": "stale",
            "target_color": "red", "age_s": 701.0,
        }))
        self.assertEqual(est.state.grasp.state, "UNKNOWN")
        self.assertIsNone(est.state.grasp.held_object_color)

        est.state.grasp.state = "HELD"
        est.state.grasp.held_object_color = "blue"
        self.assertFalse(est.reconcile_probable_carry({
            "known": True, "valid": False, "reason": "missing",
        }))
        self.assertEqual(est.state.grasp.state, "HELD")
        self.assertEqual(est.state.grasp.held_object_color, "blue")

    def test_transport_unknown_never_clears_probable_hold(self):
        est = StateEstimator()
        est.state.grasp.state = "PROBABLE_HELD"
        est.state.grasp.held_object_color = "red"
        self.assertFalse(est.reconcile_probable_carry({
            "known": False, "valid": False, "reason": "transport_failed",
        }))
        self.assertEqual(est.state.grasp.state, "PROBABLE_HELD")
        self.assertEqual(est.state.grasp.held_object_color, "red")

    def test_observe_scene_can_reconcile_stale_probable_hold(self):
        est = StateEstimator()
        est.state.grasp.state = "PROBABLE_HELD"
        est.state.grasp.held_object_color = "red"
        est.update_tool_result({"result": {
            "skill": "observe_scene",
            "command_status": "ACCEPTED",
            "execution_status": "COMPLETED",
            "outcome_status": "ACHIEVED",
            "carry_evidence": {
                "known": True, "valid": False, "reason": "stale",
                "target_color": "red", "age_s": 900.0,
            },
            "detections": {},
        }})
        self.assertEqual(est.state.grasp.state, "UNKNOWN")
        self.assertIsNone(est.state.grasp.held_object_color)


if __name__ == "__main__":
    unittest.main()
