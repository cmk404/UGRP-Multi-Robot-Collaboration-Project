from __future__ import annotations

from io import StringIO
import math
import unittest

from sim.crew_motion_metrics import CrewMotionRecorder


def pose(x: float, y: float = 0.0, yaw: float = 0.0) -> dict[str, object]:
    return {"position": [x, y, 0.0], "yaw": yaw}


class CrewMotionMetricsTests(unittest.TestCase):
    def test_three_robots_moving_in_same_window_is_distinct_from_sequential_motion(self):
        concurrent = CrewMotionRecorder()
        concurrent.observe(0.0, {rid: pose(0.0) for rid in ("r1", "r2", "r3")})
        concurrent.observe(1.0, {rid: pose(1.0) for rid in ("r1", "r2", "r3")})
        actual = concurrent.summary()["concurrency"]
        self.assertAlmostEqual(actual["overlap_3way_seconds"], 1.0)
        self.assertAlmostEqual(actual["overlap_2way_seconds"], 1.0)

        sequential = CrewMotionRecorder()
        sequential.observe(0.0, {rid: pose(0.0) for rid in ("r1", "r2", "r3")})
        sequential.observe(1.0, {"r1": pose(1.0), "r2": pose(0.0), "r3": pose(0.0)})
        sequential.observe(2.0, {"r1": pose(1.0), "r2": pose(1.0), "r3": pose(0.0)})
        sequential.observe(3.0, {rid: pose(1.0) for rid in ("r1", "r2", "r3")})
        self.assertEqual(sequential.summary()["concurrency"]["overlap_3way_seconds"], 0.0)
        self.assertEqual(sequential.summary()["concurrency"]["overlap_2way_seconds"], 0.0)

    def test_heading_projection_distinguishes_forward_and_backward(self):
        recorder = CrewMotionRecorder()
        recorder.observe(0.0, {"forward": pose(0.0, yaw=0.0), "backward": pose(0.0, yaw=0.0), "turned": pose(0.0, yaw=math.pi)})
        recorder.observe(1.0, {"forward": pose(1.0, yaw=0.0), "backward": pose(-1.0, yaw=0.0), "turned": pose(-1.0, yaw=math.pi)})
        result = recorder.summary()["robots"]
        self.assertAlmostEqual(result["forward"]["forward_travel_m"], 1.0)
        self.assertAlmostEqual(result["backward"]["reverse_travel_m"], 1.0)
        self.assertAlmostEqual(result["turned"]["forward_travel_m"], 1.0)

    def test_zero_time_jump_and_nonmonotonic_timestamp_are_preserved_as_errors(self):
        recorder = CrewMotionRecorder()
        recorder.observe(0.0, {"r1": pose(0.0)})
        recorder.observe(0.0, {"r1": pose(5.0)})
        recorder.observe(-1.0, {"r1": pose(8.0)})
        summary = recorder.summary()
        self.assertEqual(len(summary["diagnostics"]["zero_time_pose_jumps"]), 1)
        self.assertEqual(len(summary["diagnostics"]["invalid_samples"]), 2)
        self.assertEqual(summary["robots"]["r1"]["distance_m"], 0.0)

    def test_rotation_and_tiny_grasp_correction_are_separate_from_translation(self):
        recorder = CrewMotionRecorder()
        recorder.observe(0.0, {"r1": pose(0.0, yaw=0.0)})
        recorder.observe(1.0, {"r1": pose(0.01, yaw=math.pi / 2)})
        metrics = recorder.summary()["robots"]["r1"]
        self.assertEqual(metrics["moving_seconds"], 0.0)
        self.assertEqual(metrics["rotation_seconds"], 1.0)
        self.assertAlmostEqual(metrics["small_rotational_translation_m"], 0.01)

    def test_idle_jitter_does_not_create_motion_or_overlap(self):
        recorder = CrewMotionRecorder()
        recorder.observe(0.0, {"r1": pose(0.0), "r2": pose(0.0)})
        recorder.observe(0.1, {"r1": pose(0.001), "r2": pose(-0.001)})
        recorder.observe(0.2, {"r1": pose(0.0), "r2": pose(0.0)})
        result = recorder.summary()
        self.assertEqual(result["concurrency"]["overlap_2way_seconds"], 0.0)
        self.assertEqual(result["robots"]["r1"]["moving_seconds"], 0.0)

    def test_motion_totals_are_sampling_rate_invariant(self):
        fast = CrewMotionRecorder()
        slow = CrewMotionRecorder()
        fast.observe(0.0, {"r1": pose(0.0)})
        slow.observe(0.0, {"r1": pose(0.0)})
        for index in range(1, 11):
            fast.observe(index * 0.1, {"r1": pose(index * 0.1)})
        for index in range(1, 3):
            slow.observe(index * 0.5, {"r1": pose(index * 0.5)})
        fast_metrics = fast.summary()["robots"]["r1"]
        slow_metrics = slow.summary()["robots"]["r1"]
        self.assertAlmostEqual(fast_metrics["distance_m"], slow_metrics["distance_m"])
        self.assertAlmostEqual(fast_metrics["moving_seconds"], slow_metrics["moving_seconds"])
        self.assertAlmostEqual(fast_metrics["forward_travel_m"], slow_metrics["forward_travel_m"])

    def test_longest_idle_while_one_peer_moves(self):
        recorder = CrewMotionRecorder()
        recorder.observe(0.0, {"idle": pose(0.0), "peer": pose(0.0)})
        recorder.observe(2.0, {"idle": pose(0.0), "peer": pose(2.0)})
        metrics = recorder.summary()["robots"]
        self.assertAlmostEqual(metrics["idle"]["longest_idle_while_peers_moving_seconds"], 2.0)

    def test_idle_duration_accumulates_across_dense_peer_samples(self):
        recorder = CrewMotionRecorder()
        for index in range(11):
            timestamp = index * 0.1
            recorder.observe(timestamp, {"idle": pose(0.0), "peer": pose(timestamp)})
        metrics = recorder.summary()["robots"]
        self.assertAlmostEqual(metrics["idle"]["longest_idle_while_peers_moving_seconds"], 1.0)

    def test_jsonl_sink_contains_raw_samples(self):
        sink = StringIO()
        recorder = CrewMotionRecorder(sink)
        recorder.observe(0.0, {"r1": pose(0.0)})
        recorder.observe(1.0, {"r1": pose(1.0)})
        lines = sink.getvalue().splitlines()
        self.assertEqual(len(lines), 2)
        self.assertEqual(lines[0].count("simtime"), 1)
        self.assertTrue('"valid":true' in lines[1])


if __name__ == "__main__":
    unittest.main()
