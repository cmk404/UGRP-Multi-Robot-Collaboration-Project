from __future__ import annotations

import unittest
from unittest.mock import patch

from sim.warehouse_observation import observe_robots_scan

SEARCH_POSE = {1: 2000, 3: 740, 4: 2320, 5: 1320, 6: 1500}


class FakeWorld:
    def __init__(self):
        self.servo_batches = []

    def _team_joint_move_servos(self, poses, duration_s, *, settle_s=0.0):
        self.servo_batches.append((dict(poses), float(duration_s), float(settle_s)))


class ParallelSensorScanTests(unittest.TestCase):
    def test_active_heads_move_together_and_finish_independently(self):
        world = FakeWorld()
        calls = []

        def fake_observe(_world, rid, *, observation_id, sensor_seed):
            calls.append((rid, observation_id, sensor_seed))
            if rid == "r1":
                cargo = [{"cargo_id": "oak_plank_01", "label_color": "red"}]
            else:
                cargo = [{"cargo_id": "steel_pipe_01", "label_color": "blue"}]
            return {"cargo": cargo}

        with patch("sim.warehouse_observation.observe_robot", side_effect=fake_observe):
            result = observe_robots_scan(
                world,
                ["r1", "r2"],
                pending_ids={"r1": {"oak_plank_01"}, "r2": {"steel_pipe_01"}},
                observation_ids={"r1": "o1", "r2": "o2"},
                sensor_seed=41,
                pans=(1500, 1300),
            )

        # Initial stage includes both heads; second stage only the unfinished
        # head would be present if its first frame had not satisfied its goal.
        self.assertGreaterEqual(len(world.servo_batches), 2)
        self.assertEqual(set(world.servo_batches[0][0]), {"r1", "r2"})
        self.assertEqual(set(world.servo_batches[-1][0]), {"r1", "r2"})
        self.assertEqual([rid for rid, _, _ in calls], ["r1", "r2"])
        self.assertEqual(result["r1"]["scan_pans"], [1500])
        self.assertEqual(result["r2"]["scan_pans"], [1500])
        self.assertEqual(result["r1"]["observation_id"], "o1")
        self.assertEqual(result["r2"]["observation_id"], "o2")
        self.assertEqual(result["r1"]["cargo"][0]["cargo_id"], "oak_plank_01")
        self.assertEqual(result["r2"]["cargo"][0]["cargo_id"], "steel_pipe_01")
        self.assertEqual(result["r1"]["provenance"]["privileged_state_used"], False)

    def test_restore_is_collective_when_scan_raises(self):
        world = FakeWorld()

        with patch(
            "sim.warehouse_observation.observe_robot",
            side_effect=RuntimeError("render failed"),
        ), self.assertRaisesRegex(RuntimeError, "render failed"):
            observe_robots_scan(world, ["r1", "r2"], pans=(1500,))
        self.assertEqual(set(world.servo_batches[-1][0]), {"r1", "r2"})
        self.assertEqual(world.servo_batches[-1][0]["r1"], dict(SEARCH_POSE))
        self.assertEqual(world.servo_batches[-1][0]["r2"], dict(SEARCH_POSE))


if __name__ == "__main__":
    unittest.main()
