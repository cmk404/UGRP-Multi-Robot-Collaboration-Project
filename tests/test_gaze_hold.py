#!/usr/bin/env python3
import importlib.util
import pathlib
import sys
import types
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "scripts" / "red_block"


def load(name: str):
    path = PACKAGE / f"{name}.py"
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[name] = module
    sys.path.insert(0, str(PACKAGE))
    spec.loader.exec_module(module)
    return module


TRACK = load("track")
GAZE = load("gaze_hold")


class FakeRobot:
    def __init__(self):
        self.nudges = []

    def nudge_servos(self, mapping, duration=0.10):
        self.nudges.append(dict(mapping))

    def stop(self):
        pass


class PursueTrimStepTests(unittest.TestCase):
    def _load_precision(self):
        try:
            import physical_state_machine_reference as precision
        except ImportError:
            precision = load("physical_state_machine_reference")
        return precision

    KEY = ("forward", 35, 0.1)

    def _run_trim(self, precision, blobs, gaze_pan=1500, init_corr=0,
                  target_nx=0.5, deadband=0.025, key=None, state=None):
        robot = FakeRobot()
        gaze = TRACK.Gaze(pan=gaze_pan, tilt=800)
        if state is None:
            state = GAZE.PursueState(gaze=gaze)
        if init_corr:
            state.corr_by_motion[key or self.KEY] = init_corr
        outcome = None
        for b in blobs:
            # One helper call per observed median frame, like the loops do.
            gaze, outcome = precision.pursue_trim_step(
                robot, gaze, state,
                blob=b, flip_x=False, target_nx=target_nx,
                deadband=deadband, motion_key=key,
            )
        return gaze, outcome, robot, state

    def test_centered_costs_no_move(self):
        precision = self._load_precision()
        _gaze, outcome, robot, _state = self._run_trim(
            precision, [blob(0.5, 0.5)], init_corr=0)
        self.assertEqual(outcome, "centered")
        self.assertEqual(robot.nudges, [])

    def test_single_trim_records_memory(self):
        precision = self._load_precision()
        gaze, outcome, robot, state = self._run_trim(
            precision, [blob(0.7, 0.5)], init_corr=0, key=self.KEY)
        self.assertEqual(outcome, "trimmed")
        self.assertEqual(len(robot.nudges), 1)
        self.assertNotEqual(state.corr_by_motion.get(self.KEY, 0), 0)
        self.assertLess(gaze.pan, 1500)  # target right -> eye right

    def test_feedforward_prenudge_hits(self):
        precision = self._load_precision()
        # pulse 1: drift right, trim corrects by C under its motion key
        _g, _o, _r, state = self._run_trim(
            precision, [blob(0.7, 0.5)], key=self.KEY)
        corr = state.corr_by_motion.get(self.KEY, 0)
        self.assertNotEqual(corr, 0)
        # pulse 2: same motion repeats; pre-nudge anticipates, verify frame
        # already centred -> only the pre-nudge servo move happens.
        _g2, outcome2, robot2, _s2 = self._run_trim(
            precision, [blob(0.5, 0.5)], key=self.KEY, state=state)
        self.assertEqual(outcome2, "centered")
        self.assertEqual(len(robot2.nudges), 1)

    def test_trim_applies_damping_gain(self):
        precision = self._load_precision()
        gaze, outcome, robot, _state = self._run_trim(
            precision, [blob(0.7, 0.5)], init_corr=0, key=self.KEY)
        self.assertEqual(outcome, "trimmed")
        import math
        hfov = math.degrees(2.0 * math.atan(math.tan(math.radians(24.0)) * 4.0 / 3.0))
        expect = 1500 + round((0.5 - 0.7) * hfov * (2000.0 / 180.0) * GAZE.PURSUE_TRIM_GAIN)
        self.assertEqual(gaze.pan, expect)
        self.assertLess(GAZE.PURSUE_TRIM_GAIN, 1.0)

    def test_memory_isolation_across_motions(self):
        precision = self._load_precision()
        # correction learned behind a big pulse must not replay before a
        # small one (SIM: 0.28 s staging corr replayed at 0.10 s creep
        # oscillated the eye and fooled the ny progress gate).
        _g, _o, _r, state = self._run_trim(
            precision, [blob(0.7, 0.5)], key=("forward", 35, 0.28))
        self.assertNotEqual(
            state.corr_by_motion.get(("forward", 35, 0.28), 0), 0)
        _g2, outcome2, robot2, _s2 = self._run_trim(
            precision, [blob(0.5, 0.5)], key=("forward", 35, 0.10),
            state=state)
        self.assertEqual(outcome2, "centered")
        self.assertEqual(robot2.nudges, [])

    def test_lost_falls_back(self):
        precision = self._load_precision()
        _gaze, outcome, _robot, state = self._run_trim(
            precision, [None], init_corr=50, key=self.KEY)
        self.assertEqual(outcome, "lost")
        self.assertEqual(state.corr_by_motion.get(self.KEY, 0), 0)

    def test_saturated_falls_back(self):
        precision = self._load_precision()
        # target LEFT of centre (nx<0.5) drives pan UP into the hold ceiling.
        _gaze, outcome, _robot, _state = self._run_trim(
            precision, [blob(0.02, 0.5)],
            gaze_pan=GAZE.GAZE_HOLD_PAN_MAX - 2, init_corr=0)
        self.assertEqual(outcome, "saturated")


def far_blob(nx=0.5, ny=0.3, w=16, h=20, area=320):
    return types.SimpleNamespace(nx=nx, ny=ny, width=w, height=h, area=area)


class SizeRangeTests(unittest.TestCase):
    def _load_precision(self):
        try:
            import physical_state_machine_reference as precision
        except ImportError:
            precision = load("physical_state_machine_reference")
        return precision

    def test_known_geometry(self):
        precision = self._load_precision()
        d = precision.size_range_estimate(far_blob(nx=0.5, w=15, h=15))
        self.assertIsNotNone(d)
        self.assertGreater(d, 100.0)
        self.assertLess(d, 130.0)

    def test_rejects_tiny_and_edge(self):
        precision = self._load_precision()
        self.assertIsNone(precision.size_range_estimate(far_blob(w=2, h=2)))
        self.assertIsNone(precision.size_range_estimate(far_blob(nx=0.02)))
        self.assertIsNone(precision.size_range_estimate(far_blob(nx=0.99)))
        # bottom-truncated: cut-off blob would read far, so reject it
        self.assertIsNone(precision.size_range_estimate(far_blob(nx=0.5, ny=0.95, w=30, h=40)))

    def test_fusion_vetoes_far_floor(self):
        # Live refusal fixture: floor triangulation reads 69.0cm against
        # 90.9cm truth while the cube fit reads 87.5cm.  Far floor must lose.
        precision = self._load_precision()
        pose = {1: 2000, 3: 855, 4: 2320, 5: 1320, 6: 1520}
        blob = types.SimpleNamespace(
            nx=0.5264, ny=0.4941, width=23.0, height=25.0,
            cx=0.5264 * 640.0, cy=0.4941 * 480.0, area=575,
        )
        est = precision.estimate_block(pose, blob)
        self.assertIsNotNone(est)
        self.assertEqual(est.source, "size_apparent")
        self.assertAlmostEqual(est.radius_cm, 87.5, delta=4.0)

    def test_conservative_min_side(self):
        precision = self._load_precision()
        d = precision.size_range_estimate(far_blob(w=30, h=15))
        d_h = precision.size_range_estimate(far_blob(w=15, h=15))
        # wider pixel extent -> nearer (conservative) estimate
        self.assertLess(d, d_h)

    def test_floor_ray_still_wins_near(self):
        precision = self._load_precision()
        from unittest import mock
        cam = types.SimpleNamespace(height_cm=12.0, radius_cm=5.0, pitch_deg=-10.0)
        pose = {3: 700, 4: 2200, 5: 780, 6: 1500}
        blob = far_blob(nx=0.5, ny=0.8, w=40, h=24, area=960)
        with mock.patch.object(precision, "forward_kinematics", return_value=cam):
            est = precision.estimate_block(pose, blob)
        self.assertIsNotNone(est)
        self.assertEqual(est.source, "floor_ray")
        self.assertGreater(est.radius_cm, 20.0)
        self.assertLess(est.radius_cm, 50.0)

    def test_size_fallback_far(self):
        precision = self._load_precision()
        from unittest import mock
        # Grounded fixture: recorded SIM far frame (block truth 95.4 cm).
        # Level-camera mock keeps the floor ray unavailable so the
        # cube-model size fit answers: ~94 cm, not the old pinhole ~70.
        cam = types.SimpleNamespace(height_cm=18.0, radius_cm=8.0, pitch_deg=0.0)
        pose = {3: 740, 4: 2320, 5: 1320, 6: 1500}
        blob = far_blob(nx=0.7986, ny=0.2594, w=25, h=26, area=650)
        with mock.patch.object(precision, "forward_kinematics", return_value=cam):
            est = precision.estimate_block(pose, blob)
        self.assertIsNotNone(est)
        self.assertEqual(est.source, "size_apparent")
        self.assertAlmostEqual(est.radius_cm, 94.0, delta=4.0)
        self.assertAlmostEqual(est.yaw_left_deg, -18.3, delta=2.0)


def blob(nx: float, ny: float, area: int = 2000):
    return types.SimpleNamespace(nx=nx, ny=ny, area=area)


class GazeHoldTests(unittest.TestCase):
    def test_hold_envelope_is_inside_face_margin(self):
        self.assertLess(GAZE.GAZE_HOLD_MAX_DEG, 34.0)
        self.assertGreater(GAZE.GAZE_HOLD_PAN_MIN, TRACK.PAN_MIN)
        self.assertLess(GAZE.GAZE_HOLD_PAN_MAX, TRACK.PAN_MAX)

    def test_tracking_converges(self):
        st = GAZE.PursueState(gaze=TRACK.Gaze(pan=1500, tilt=800))
        for _ in range(30):
            status = GAZE.pursue_step(st, blob(0.7, 0.5), dt=0.1)
            self.assertIn(status, ("TRACKING", "SATURATED"))
        self.assertNotEqual(st.gaze.pan, 1500)

    def test_saturation_reports_body_transfer(self):
        st = GAZE.PursueState(
            gaze=TRACK.Gaze(pan=GAZE.GAZE_HOLD_PAN_MAX - 5, tilt=800)
        )
        statuses = {
            GAZE.pursue_step(st, blob(0.95, 0.5), dt=0.1) for _ in range(20)
        }
        self.assertIn("SATURATED", statuses)
        self.assertLessEqual(st.gaze.pan, GAZE.GAZE_HOLD_PAN_MAX)

    def test_feedforward_is_inverse_of_chassis_yaw(self):
        after = GAZE.feedforward_pan(1500, 6.0)
        self.assertLess(after, 1500)
        self.assertEqual(GAZE.feedforward_pan(after, -6.0), 1500)

    def test_blink_grace_then_search(self):
        st = GAZE.PursueState(gaze=TRACK.Gaze(pan=1500, tilt=800))
        self.assertEqual(GAZE.pursue_step(st, None, dt=0.1), "LOST_BLINK")
        self.assertEqual(GAZE.pursue_step(st, None, dt=0.1), "LOST_BLINK")
        self.assertEqual(st.gaze.pan, 1500)
        self.assertEqual(GAZE.pursue_step(st, None, dt=0.1), "LOST_SEARCH")

    def test_saccade_goes_to_memory_bearing(self):
        st = GAZE.PursueState(gaze=TRACK.Gaze(pan=1500, tilt=800))
        GAZE.pursue_step(st, None, dt=0.1, memory_bearing_deg=13.55)
        GAZE.pursue_step(st, None, dt=0.1)
        self.assertEqual(GAZE.pursue_step(st, None, dt=0.1), "LOST_SACCADE")
        self.assertGreater(st.gaze.pan, 1500)
        self.assertAlmostEqual(st.gaze.pan - 1500, 150, delta=3)

    def test_saccade_clips_and_reports(self):
        pan, clipped = GAZE.saccade_pan(1500, 60.0)
        self.assertTrue(clipped)
        self.assertEqual(pan, GAZE.GAZE_HOLD_PAN_MAX)

    def test_recovery_after_reacquire(self):
        st = GAZE.PursueState(gaze=TRACK.Gaze(pan=1500, tilt=800))
        GAZE.pursue_step(st, None, dt=0.1)
        self.assertEqual(
            GAZE.pursue_step(st, blob(0.5, 0.5), dt=0.1), "TRACKING"
        )
        self.assertEqual(st.misses, 0)

    def test_normalize_rejects_unknown_mode(self):
        self.assertEqual(GAZE.normalize_gaze_mode(None), "fixate")
        self.assertEqual(GAZE.normalize_gaze_mode("pursue"), "pursue")
        with self.assertRaises(ValueError):
            GAZE.normalize_gaze_mode("stare")


if __name__ == "__main__":
    unittest.main()
