#!/usr/bin/env python3
import importlib.util
import pathlib
import sys
import types
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "red_block" / "track.py"
SPEC = importlib.util.spec_from_file_location("track_red_block", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def blob(nx: float, ny: float, area: int = 4000):
    return types.SimpleNamespace(nx=nx, ny=ny, area=area)


class TrackRedBlockTests(unittest.TestCase):
    def test_center_blob_holds_gaze(self):
        gaze = MODULE.Gaze()
        updated = MODULE.tracking_step(blob(0.5, 0.5), gaze, dt=0.1)
        self.assertEqual(updated.pan, MODULE.PAN_START)
        self.assertEqual(updated.tilt, MODULE.TILT_START)

    def test_right_blob_decreases_pan(self):
        gaze = MODULE.Gaze()
        updated = MODULE.tracking_step(blob(0.75, 0.5), gaze, dt=0.1)
        self.assertLess(updated.pan, MODULE.PAN_START)

    def test_left_blob_increases_pan(self):
        gaze = MODULE.Gaze()
        updated = MODULE.tracking_step(blob(0.25, 0.5), gaze, dt=0.1)
        self.assertGreater(updated.pan, MODULE.PAN_START)

    def test_low_blob_decreases_tilt(self):
        gaze = MODULE.Gaze()
        updated = MODULE.tracking_step(blob(0.5, 0.75), gaze, dt=0.1)
        self.assertLess(updated.tilt, MODULE.TILT_START)

    def test_missing_blob_holds_pose(self):
        gaze = MODULE.Gaze(pan=1400, tilt=800)
        updated = MODULE.tracking_step(None, gaze, dt=0.1)
        self.assertEqual(updated.pan, 1400)
        self.assertEqual(updated.tilt, 800)

    def test_flip_x_inverts_pan(self):
        right = MODULE.tracking_step(blob(0.75, 0.5), MODULE.Gaze(), dt=0.1)
        flipped = MODULE.tracking_step(
            blob(0.75, 0.5), MODULE.Gaze(), dt=0.1, flip_x=True
        )
        self.assertLess(right.pan, MODULE.PAN_START)
        self.assertGreater(flipped.pan, MODULE.PAN_START)

    def test_tracker_accepts_search_peripheral_pan_envelope(self):
        self.assertEqual(MODULE.PAN_MIN, 1050)
        self.assertEqual(MODULE.PAN_MAX, 1950)
        gaze = MODULE.Gaze(pan=1950, tilt=620)
        blob = types.SimpleNamespace(nx=.50, ny=.50)
        updated = MODULE.tracking_step(blob, gaze, dt=.1)
        self.assertEqual(updated.pan, 1950)

    def test_track_uses_small_target_threshold_and_bounded_miss_tolerance(self):
        self.assertLessEqual(MODULE.TRACK_MIN_AREA, 500)
        self.assertGreaterEqual(MODULE.MAX_CONSECUTIVE_MISSES, 3)

    def test_track_requires_multiple_center_confirmations(self):
        self.assertGreaterEqual(MODULE.CENTER_CONFIRMATIONS, 2)


    def test_near_floor_target_is_centered_at_safe_tilt_limit(self):
        gaze = MODULE.Gaze(pan=1525, tilt=MODULE.TILT_MIN)
        self.assertTrue(MODULE.centered_at_gaze(0.534, 0.873, gaze))

    def test_same_low_target_is_not_centered_when_camera_can_still_tilt(self):
        gaze = MODULE.Gaze(pan=1525, tilt=800)
        self.assertFalse(MODULE.centered_at_gaze(0.534, 0.873, gaze))

    def test_centered_helper(self):
        self.assertTrue(MODULE.centered(0.51, 0.49))
        self.assertFalse(MODULE.centered(0.7, 0.5))


if __name__ == "__main__":
    unittest.main(verbosity=2)
