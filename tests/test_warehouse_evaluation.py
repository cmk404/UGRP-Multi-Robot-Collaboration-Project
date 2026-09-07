import unittest

from harness.warehouse_evaluation import EpisodeResult, EvaluationLedger


class EvaluationLedgerTests(unittest.TestCase):
    def test_matched_summary_includes_missing_as_failures(self):
        ledger = EvaluationLedger(("no_comm", "peer_comm"), (1, 2), ("A-B",), budget=10)
        ledger.record(EpisodeResult("no_comm", 1, "A-B", True, 2.0, input_tokens=None,
                                   output_tokens=None, budget=10))
        ledger.record(EpisodeResult("peer_comm", 1, "A-B", None, error="timeout", budget=10))
        summary = ledger.summary()["conditions"]
        self.assertEqual(summary["no_comm"]["denominator"], 2)
        self.assertEqual(summary["no_comm"]["failures"], 1)
        self.assertIsNone(summary["no_comm"]["total_tokens_mean"])
        self.assertTrue(summary["peer_comm"]["failures"], 1)

    def test_duplicate_unknown_and_mismatched_records_rejected(self):
        ledger = EvaluationLedger(("a",), (1,), ("s",), budget={"max": 3})
        row = EpisodeResult("a", 1, "s", False, budget={"max": 3})
        ledger.append(row)
        with self.assertRaises(ValueError):
            ledger.append(row)
        with self.assertRaises(ValueError):
            ledger.append(EpisodeResult("unknown", 1, "s", True, budget={"max": 3}))
        with self.assertRaises(ValueError):
            ledger.append(EpisodeResult("a", 1, "s", True, budget={"max": 4}))

    def test_paired_comparison_uses_same_seed_and_scenario(self):
        ledger = EvaluationLedger(("a", "b"), (1, 2), ("x", "y"))
        for seed in (1, 2):
            for scenario in ("x", "y"):
                ledger.record(EpisodeResult("a", seed, scenario, False, 5.0))
                ledger.record(EpisodeResult("b", seed, scenario, True, 3.0))
        paired = ledger.paired("a", "b")
        self.assertEqual(paired["paired_recorded"], 4)
        self.assertEqual(paired["success_delta_mean"], 1.0)
        self.assertEqual(paired["b_win_fraction"], 1.0)
        self.assertEqual(len(paired["pairs"]), 4)

    def test_success_is_strict_and_provenance_cannot_be_mixed(self):
        ledger = EvaluationLedger(("a",), (1, 2), ("s",))
        with self.assertRaises(ValueError):
            ledger.record(EpisodeResult("a", 1, "s", "false"))
        ledger.record(EpisodeResult("a", 1, "s", True, provenance="live"))
        with self.assertRaises(ValueError):
            ledger.record(EpisodeResult("a", 2, "s", True, provenance="fixture"))

    def test_duplicate_scenarios_are_rejected(self):
        with self.assertRaises(ValueError):
            EvaluationLedger(("a",), (1,), ("s", "s"))


if __name__ == "__main__":
    unittest.main()
