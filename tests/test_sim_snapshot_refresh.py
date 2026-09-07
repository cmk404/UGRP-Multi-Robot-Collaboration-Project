from __future__ import annotations

import threading
import time
import unittest

from sim.bridge import BridgeState


class SnapshotRefreshTests(unittest.TestCase):
    def test_robot_refresh_waits_for_new_frame_and_releases_subscription(self):
        bridge = BridgeState("test-token")
        bridge.push_frame(b"old", {}, state_seq=1, frame_meta={"generated_wall_s": time.time() - 10}, robot_id="r1")

        def publish():
            time.sleep(0.03)
            bridge.push_frame(b"fresh", {}, state_seq=2, frame_meta={"generated_wall_s": time.time()}, robot_id="r1")

        thread = threading.Thread(target=publish)
        thread.start()
        result = bridge.fresh_snapshot("robot:r1", lambda: bridge.robot_frames.get("r1"), lambda: bridge.robot_frame_meta.get("r1"), timeout_s=0.2, max_age_s=0.75)
        thread.join()
        self.assertEqual(result, b"fresh")
        self.assertEqual(bridge.active_streams, 0)
        self.assertEqual(bridge.stream_subscriptions, {})

    def test_timeout_does_not_return_stale_frame_and_releases_subscription(self):
        bridge = BridgeState("test-token")
        bridge.push_frame(b"stale", {}, state_seq=1, frame_meta={"generated_wall_s": time.time() - 10}, robot_id="r1")
        result = bridge.fresh_snapshot("robot:r1", lambda: bridge.robot_frames.get("r1"), lambda: bridge.robot_frame_meta.get("r1"), timeout_s=0.03, max_age_s=0.75)
        self.assertIsNone(result)
        self.assertEqual(bridge.active_streams, 0)

    def test_fresh_cached_frame_is_fast_and_does_not_subscribe(self):
        bridge = BridgeState("test-token")
        bridge.push_frame(b"current", {}, state_seq=1, frame_meta={"generated_wall_s": time.time()}, robot_id="r1")
        started = time.monotonic()
        result = bridge.fresh_snapshot("robot:r1", lambda: bridge.robot_frames.get("r1"), lambda: bridge.robot_frame_meta.get("r1"), timeout_s=0.2, max_age_s=0.75)
        self.assertEqual(result, b"current")
        self.assertLess(time.monotonic() - started, 0.05)
        self.assertEqual(bridge.active_streams, 0)

    def test_team_refresh_uses_observer_publication(self):
        bridge = BridgeState("test-token")
        bridge.push_observer_frame("team_overview", b"old", {"generated_wall_s": time.time() - 10})

        def publish():
            time.sleep(0.02)
            bridge.push_observer_frame("team_overview", b"fresh", {"generated_wall_s": time.time()})

        thread = threading.Thread(target=publish)
        thread.start()
        result = bridge.fresh_snapshot("team_overview", lambda: bridge.observer_frames.get("team_overview"), lambda: bridge.observer_frame_meta.get("team_overview"), timeout_s=0.2, max_age_s=0.75)
        thread.join()
        self.assertEqual(result, b"fresh")
        self.assertEqual(bridge.active_streams, 0)


if __name__ == "__main__":
    unittest.main()
