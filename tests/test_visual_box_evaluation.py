import unittest

from harness.visual_box_evaluation import evaluate_visual_box_transfer


def state(position, *, stable=False, constraint=False, destination=False):
    return {
        "cargo": {
            "box": {
                "position": list(position),
                "stable": stable,
                "constraints_active": {"r1": constraint},
                "evaluation": {"success": destination},
            }
        }
    }


class VisualBoxEvaluationTests(unittest.TestCase):
    def test_short_transfer_passes_only_all_required_outcome_gates(self):
        result = evaluate_visual_box_transfer(
            state((0.0, 0.0, 0.015)),
            state((0.30, 0.0, 0.015), stable=True),
            [
                {"warehouse_state": state((0.0, 0.0, 0.060))},
                {"warehouse_state": state((0.18, 0.0, 0.058))},
            ],
        )
        self.assertTrue(result["success"])
        self.assertAlmostEqual(result["max_box_height_m"], 0.060)
        self.assertAlmostEqual(result["max_lift_above_initial_m"], 0.045)
        self.assertAlmostEqual(result["planar_displacement_m"], 0.30)
        self.assertEqual(result["destination_evaluation_success"], None)
        self.assertIn("not_cooperation_proof", result["claim_scope"])

    def test_constraint_at_any_sample_fails_even_when_motion_passes(self):
        result = evaluate_visual_box_transfer(
            state((0.0, 0.0, 0.015)),
            state((0.30, 0.0, 0.015), stable=True),
            [state((0.1, 0.0, 0.070), constraint=True)],
        )
        self.assertFalse(result["success"])
        self.assertTrue(result["active_constraints_during_episode"])
        self.assertFalse(result["gates"]["constraint_free"])

    def test_final_height_does_not_replace_peak_lift_and_stability(self):
        result = evaluate_visual_box_transfer(
            state((0.0, 0.0, 0.015)),
            state((0.30, 0.0, 0.070), stable=False),
            [],
        )
        self.assertFalse(result["success"])
        self.assertTrue(result["gates"]["lift"])
        self.assertFalse(result["gates"]["final_stable"])

    def test_destination_mode_adds_exact_goal_gate(self):
        initial = state((0.0, 0.0, 0.015))
        log = [state((0.1, 0.0, 0.060))]
        missed = evaluate_visual_box_transfer(
            initial,
            state((0.30, 0.0, 0.015), stable=True, destination=False),
            log,
            mode="destination_zone",
        )
        reached = evaluate_visual_box_transfer(
            initial,
            state((0.30, 0.0, 0.015), stable=True, destination=True),
            log,
            mode="destination_zone",
        )
        self.assertFalse(missed["success"])
        self.assertFalse(missed["gates"]["destination_zone"])
        self.assertTrue(reached["success"])

    def test_rejects_ambiguous_or_malformed_evaluation_data(self):
        two = state((0, 0, 0.015))
        two["cargo"]["other"] = dict(two["cargo"]["box"])
        with self.assertRaisesRegex(ValueError, "cargo_id"):
            evaluate_visual_box_transfer(two, two, [])
        with self.assertRaisesRegex(ValueError, "finite"):
            evaluate_visual_box_transfer(
                state((0, 0, 0.015)), state((float("nan"), 0, 0.015)), []
            )
        with self.assertRaisesRegex(ValueError, "mode"):
            evaluate_visual_box_transfer(
                state((0, 0, 0.015)), state((0.3, 0, 0.015)), [], mode="cooperation"
            )


if __name__ == "__main__":
    unittest.main()
