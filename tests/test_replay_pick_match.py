import json
from pathlib import Path
import tempfile
import unittest

from scripts import replay_pick_match as replay


class ReplayHelpersTest(unittest.TestCase):
    def test_loads_only_ordered_raw_actions(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "raw.jsonl"
            path.write_text("\n".join(json.dumps(row) for row in [
                {"event": "macro_submitted", "time": 0.1},
                {"event": "raw_action", "time": 0.2, "raw_action": {"kind": "wait"}},
                {"event": "raw_action", "time": 0.3, "raw_action": {"kind": "arm", "servo_id": 1, "pulse": 1500}},
            ]))
            self.assertEqual(len(replay.load_raw_actions(path)), 2)

    def test_rejects_nonmonotonic_commands(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "raw.jsonl"
            path.write_text("\n".join(json.dumps({"event": "raw_action", "time": when,
                "raw_action": {"kind": "wait"}}) for when in (0.2, 0.1)))
            with self.assertRaises(ValueError):
                replay.load_raw_actions(path)

    def test_close_trajectory_is_required(self):
        reference = [{"elapsed_sim_s": 0.0, "position": [0, 0, 0], "lift_m": 0.0},
                     {"elapsed_sim_s": 0.1, "position": [0, 0, .05], "lift_m": .05}]
        replay_rows = [{"elapsed_sim_s": 0.0, "position": [0, 0, 0], "lift_m": 0.0},
                       {"elapsed_sim_s": 0.1, "position": [0, 0, .05001], "lift_m": .05001}]
        self.assertTrue(replay.compare_trajectories(reference, replay_rows, 2e-4, 2e-4)["accepted"])
        replay_rows[1]["position"][2] = .06
        self.assertFalse(replay.compare_trajectories(reference, replay_rows, 2e-4, 2e-4)["accepted"])


if __name__ == "__main__":
    unittest.main()
