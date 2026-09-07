import unittest
from scripts.evaluate_coela import score_scenario


class EvaluationTests(unittest.TestCase):
    def test_deadline_cleanup_does_not_replace_timeout_with_physical_failure(self):
        event = {"event": "failed", "reason": "MIXED_EXECUTION_CANCELLED"}
        r = score_scenario({"success": False, "reason": "TIMEOUT", "status": {"history": [event]}},
                           [{"event_id": "e"}], [], "private_obstacle")
        self.assertIsNone(r["first_execution_failure"])
        self.assertEqual(r["primary_failure"], "TIMEOUT")
        self.assertEqual(r["cleanup_failures"], [event])
    def test_unrelated_or_wrong_action_recovery_does_not_pass(self):
        base = {"success": True, "reason": "SUCCESS", "status": {"history": []}}
        for events in ([{"event": "recovered", "event_id": "other", "action": "replan"}],
                       [{"event": "recovered", "event_id": "expected", "action": "resume"}]):
            scored = score_scenario(base, [{"event_id": "expected"}], events, "private_obstacle")
            self.assertFalse(scored["success"])
            self.assertTrue(scored["execution_success"])
            self.assertEqual(scored["scenario_verdict"], "NOT_RECOVERED")

    def test_physical_failure_is_preserved_before_recovery_failure(self):
        failure = {"event": "failed", "reason": "SOLO_TIMEOUT:APPROACH", "sim_time": 97.7}
        base = {"success": False, "reason": "INCOMPLETE", "status": {"history": [failure]}}
        scored = score_scenario(base, [{"event_id": "later"}], [], "private_obstacle")
        self.assertEqual(scored["reason"], "INCOMPLETE")
        self.assertEqual(scored["primary_failure"], "SOLO_TIMEOUT:APPROACH")
        self.assertEqual(scored["scenario_verdict"], "NOT_RECOVERED")
        self.assertEqual(scored["first_execution_failure"], failure)

    def test_matching_event_replan_and_delivery_both_required(self):
        events = [{"event": "recovered", "event_id": "expected", "action": "replan"}]
        for success in (False, True):
            scored = score_scenario({"success": success, "status": {}}, [{"event_id": "expected"}], events, "private_obstacle")
            self.assertEqual(scored["success"], success)
            self.assertTrue(scored["injected_event_recovered"])


if __name__ == "__main__": unittest.main()
