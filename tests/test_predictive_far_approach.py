from __future__ import annotations

import importlib.util
import pathlib
import sys
import types
import unittest
from unittest import mock

ROOT = pathlib.Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "scripts" / "red_block"


def load_reference():
    if str(PACKAGE) not in sys.path:
        sys.path.insert(0, str(PACKAGE))
    path = PACKAGE / "physical_state_machine_reference.py"
    spec = importlib.util.spec_from_file_location("predictive_far_approach_test", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class PredictiveFarApproachTests(unittest.TestCase):
    @staticmethod
    def block(ref, radius: float):
        return ref.BlockEstimate(
            radius_cm=radius,
            lateral_left_cm=0.0,
            forward_cm=radius,
            yaw_left_deg=0.0,
            camera_radius_cm=7.0,
            camera_height_cm=10.0,
            ray_pitch_deg=-60.0,
            block_height_cm=3.0,
            nx=0.5,
            ny=0.4,
        )

    @staticmethod
    def robot():
        class Motors:
            def __init__(self):
                self.writes = []
            def write_motor(self, motor, speed):
                self.writes.append((motor, speed))
        class Robot:
            def __init__(self):
                self.pose = {3: 500, 4: 2320, 5: 1320, 6: 1500}
                self.motors = Motors()
                self.stops = 0
            def stop(self):
                self.stops += 1
        return Robot()

    def test_prediction_stops_before_coarse_boundary(self):
        ref = load_reference()
        robot = self.robot()
        gaze = ref.Gaze(1500, 700)
        blob = types.SimpleNamespace(nx=.50, ny=.35)
        recentered = ref.Gaze(1500, 700)
        # Actual range is still outside the 36 cm stop boundary, but the
        # short-horizon forecast crosses the guard, so STOP must win now.
        with mock.patch.object(ref, "read_locked", return_value=(object(), blob)), \
             mock.patch.object(ref, "estimate_block", return_value=self.block(ref, 38.5)), \
             mock.patch.object(ref, "tracking_update") as track, \
             mock.patch.object(ref, "centre_gaze", return_value=recentered), \
             mock.patch.object(ref.time, "sleep", return_value=None), \
             mock.patch.object(ref.time, "monotonic", side_effect=[0.0, .05, .10]):
            result = ref.continuous_far_approach(
                robot, object(), object(), gaze,
                target_radius_cm=25.5, flip_x=False, flip_y=False,
            )
        self.assertEqual(result.stop_reason, "predicted-boundary")
        self.assertEqual(result.frames, 1)
        self.assertGreaterEqual(robot.stops, 2)  # pre-start + safety STOP
        track.assert_not_called()

    def test_prediction_rate_anomaly_stops_without_third_motion_cycle(self):
        ref = load_reference()
        robot = self.robot()
        gaze = ref.Gaze(1500, 700)
        blobs = [types.SimpleNamespace(nx=.50, ny=.35), types.SimpleNamespace(nx=.50, ny=.36)]
        estimates = [self.block(ref, 46.0), self.block(ref, 41.0)]
        recentered = ref.Gaze(1500, 700)
        with mock.patch.object(ref, "read_locked", side_effect=[(object(), b) for b in blobs]), \
             mock.patch.object(ref, "estimate_block", side_effect=estimates), \
             mock.patch.object(ref, "tracking_update", return_value=(gaze, .05, False)) as track, \
             mock.patch.object(ref, "centre_gaze", return_value=recentered), \
             mock.patch.object(ref.time, "sleep", return_value=None), \
             mock.patch.object(ref.time, "monotonic", side_effect=[0.0, .05, .10, .15]):
            result = ref.continuous_far_approach(
                robot, object(), object(), gaze,
                target_radius_cm=25.5, flip_x=False, flip_y=False,
            )
        self.assertEqual(result.stop_reason, "prediction-rate")
        self.assertEqual(result.frames, 2)
        self.assertEqual(track.call_count, 1)
        self.assertGreaterEqual(robot.stops, 2)

    def test_offcentre_pan_does_not_chirp_stop_far_approach(self):
        ref = load_reference()
        robot = self.robot()
        gaze = ref.Gaze(1261, 700)  # previous REAL failure: far from centre, not physical limit
        blob = types.SimpleNamespace(nx=.50, ny=.35)
        estimates = [self.block(ref, 49.2), self.block(ref, 48.0)]
        tracked = ref.Gaze(1261, 700)
        recentered = ref.Gaze(1261, 700)
        with mock.patch.object(ref, "read_locked", side_effect=[(object(), blob), (object(), blob)]), \
             mock.patch.object(ref, "estimate_block", side_effect=estimates), \
             mock.patch.object(ref, "tracking_update", return_value=(tracked, .20, False)) as track, \
             mock.patch.object(ref, "centre_gaze", return_value=recentered), \
             mock.patch.object(ref.time, "sleep", return_value=None), \
             mock.patch.object(ref.time, "monotonic", side_effect=[0.0, .10, .20, .46, .50]):
            result = ref.continuous_far_approach(
                robot, object(), object(), gaze,
                target_radius_cm=25.5, flip_x=False, flip_y=False,
            )
        self.assertNotEqual(result.stop_reason, "pan-boundary")
        self.assertNotEqual(result.stop_reason, "pan-physical-limit")
        self.assertGreaterEqual(track.call_count, 1)

    def test_tracking_limits_are_coarse_to_fine(self):
        ref = load_reference()
        self.assertEqual(ref.tracking_axis_step_limit(.13, pan=True), ref.TRACK_PAN_COARSE_STEP)
        self.assertEqual(ref.tracking_axis_step_limit(.08, pan=True), ref.TRACK_PAN_MEDIUM_STEP)
        self.assertEqual(ref.tracking_axis_step_limit(.03, pan=True), ref.TRACK_PAN_STEP)
        self.assertGreater(ref.TRACK_PAN_COARSE_STEP, ref.TRACK_PAN_MEDIUM_STEP)
        self.assertGreater(ref.TRACK_PAN_MEDIUM_STEP, ref.TRACK_PAN_STEP)

    def test_body_yaw_prediction_is_bounded_and_tapers(self):
        ref = load_reference()
        coarse = ref.body_align_pulse_duration(25.0, ref.BODY_ALIGN_RATE_PRIOR_DEG_S)
        medium = ref.body_align_pulse_duration(8.0, ref.BODY_ALIGN_RATE_PRIOR_DEG_S)
        fine = ref.body_align_pulse_duration(3.0, ref.BODY_ALIGN_RATE_PRIOR_DEG_S)
        self.assertLessEqual(coarse, ref.BODY_ALIGN_MAX_PULSE_SECONDS)
        self.assertGreater(coarse, medium)
        self.assertGreater(medium, fine)
        self.assertGreaterEqual(fine, ref.TURN_FINE_PULSE_SECONDS)
        self.assertAlmostEqual(ref.BODY_BEARING_DEADBAND_DEG, 0.5)
        self.assertGreaterEqual(ref.body_align_pulse_duration(1.0, ref.BODY_ALIGN_RATE_PRIOR_DEG_S), 0.10)

    def test_precision_zone_never_uses_predictive_stream(self):
        ref = load_reference()
        self.assertTrue(ref.should_use_continuous_far_approach(45.0, 25.5))
        self.assertFalse(ref.should_use_continuous_far_approach(30.0, 25.5))
        self.assertGreater(ref.COARSE_STREAM_EXIT_RADIUS_CM, ref.FACE_ALIGN_START_RADIUS_CM)


if __name__ == "__main__":
    unittest.main(verbosity=2)
