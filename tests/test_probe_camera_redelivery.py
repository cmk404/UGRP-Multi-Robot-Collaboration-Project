import unittest

from scripts.probe_camera_redelivery import FreshFrameGuard, inside_destination, prefix_events


def obs(camera, frame, digest, when):
    return {"camera": camera, "frame_id": frame, "sha256": digest, "sim_time": when}


class FreshFrameGuardTests(unittest.TestCase):
    def test_requires_both_frames_to_advance_between_decisions(self):
        guard = FreshFrameGuard()
        guard.accept(obs("robot_cam", 1, "a", 232.0), obs("nav_cam", 2, "b", 232.0))
        guard.accept(obs("robot_cam", 3, "c", 233.0), obs("nav_cam", 4, "d", 233.0))
        with self.assertRaisesRegex(ValueError, "STALE_CAMERA_FRAME"):
            guard.accept(obs("robot_cam", 3, "e", 234.0), obs("nav_cam", 5, "f", 234.0))

    def test_allows_same_pixels_from_a_new_capture(self):
        guard = FreshFrameGuard()
        guard.accept(obs("robot_cam", 1, "a", 232.0), obs("nav_cam", 2, "b", 232.0))
        guard.accept(obs("robot_cam", 3, "a", 233.0), obs("nav_cam", 4, "b", 233.0))


class PrefixReplayTests(unittest.TestCase):
    def test_raw_prefix_is_bounded_and_missing_finish_stop_is_inferred(self):
        commands = [
            {"event": "raw_action", "time": 10, "robot_id": "r2", "raw_action": {"kind": "stop"}},
            {"event": "raw_action", "time": 233, "robot_id": "r2", "raw_action": {"kind": "drive"}},
        ]
        decisions = [{"disposition": "accepted", "time": 20, "robot_id": "r2",
                      "decision": {"action": {"kind": "finish"}}}]
        events = prefix_events(commands, [], decisions, [])
        self.assertEqual([event["time"] for event in events], [10, 20])
        self.assertEqual(events[-1]["event"], "inferred_stop")


class OfflineDestinationTests(unittest.TestCase):
    def test_rotated_footprint_not_axis_aligned_half_dimensions(self):
        class Spec:
            dimensions_m = (0.4, 0.1, 0.1)
        class Zone:
            center_xy = (0.0, 0.0)
            half_extents_xy = (0.3, 0.3)
        class World:
            warehouse_spec_by_id = {"small_box_02": Spec()}
            warehouse_zones = {"B": Zone()}

        state = {"cargo": {"small_box_02": {
            "position": (0.15, 0.0, 0.05), "yaw": 0.7853981633974483,
        }}}
        self.assertFalse(inside_destination(World(), state))


if __name__ == "__main__":
    unittest.main()
