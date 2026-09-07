#!/usr/bin/env python3
import importlib.util
import pathlib
import sys
import types
import unittest
from unittest import mock


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


PLAN = load("plan")
CAMERA = load("camera")
POSES = load("poses")
TRACK = load("track")
APPROACH = load("approach")


def blob(nx: float, ny: float, area: int):
    return types.SimpleNamespace(nx=nx, ny=ny, area=area)


class ApproachPlannerTests(unittest.TestCase):
    def test_track_envelope_matches_precision_controller(self):
        self.assertEqual((TRACK.PAN_MIN, TRACK.PAN_MAX), (1050, 1950))
        self.assertEqual((TRACK.TILT_MIN, TRACK.TILT_MAX), (500, 1200))


    def test_run_on_robot_retries_one_implausible_range_jump(self):
        fake_robot = types.SimpleNamespace(dry_run=False, stop=mock.Mock())
        with mock.patch.object(
            APPROACH, "run_coarse_approach",
            side_effect=[RuntimeError("range changed implausibly from 60 to 22"), 0],
        ) as run, mock.patch.object(APPROACH, "invalidate_pick_plan") as invalidate,              mock.patch.object(APPROACH.time, "sleep", return_value=None):
            self.assertEqual(APPROACH.run_approach(fake_robot, precision=object()), 0)
        self.assertEqual(run.call_count, 2)
        self.assertGreaterEqual(fake_robot.stop.call_count, 1)
        invalidate.assert_called_once()

    def test_run_on_robot_does_not_retry_nonrecoverable_safety_failure(self):
        fake_robot = types.SimpleNamespace(dry_run=False, stop=mock.Mock(), probe=mock.Mock(return_value={"ok": True, "battery_mv": 7400}))
        args = types.SimpleNamespace(dry_run=False, flip_x=False, flip_y=False)
        with mock.patch.object(APPROACH, "Robot", return_value=fake_robot), \
             mock.patch.object(APPROACH.signal, "signal"), \
             mock.patch.object(APPROACH, "run_approach", side_effect=RuntimeError("chassis safety radius violated")) as run:
            with self.assertRaisesRegex(RuntimeError, "chassis safety radius"):
                APPROACH.run_on_robot(args)
        self.assertEqual(run.call_count, 1)

    def test_far_centered_blob_drives_forward(self):
        command = PLAN.approach_command(blob(0.50, 0.30, 1200))
        self.assertEqual(command.kind, "forward")
        self.assertGreater(command.speed, 0)

    def test_right_blob_rotates_right(self):
        command = PLAN.approach_command(blob(0.80, 0.50, 2000))
        self.assertEqual(command.kind, "rotate-right")

    def test_left_blob_rotates_left(self):
        command = PLAN.approach_command(blob(0.20, 0.50, 2000))
        self.assertEqual(command.kind, "rotate-left")

    def test_flip_x_inverts_turn(self):
        command = PLAN.approach_command(blob(0.80, 0.50, 2000), flip_x=True)
        self.assertEqual(command.kind, "rotate-left")


    def test_stale_search_pose_is_reasserted_before_video_geometry(self):
        calls = []

        class StopAfterTrust(RuntimeError):
            pass

        class Robot:
            dry_run = False
            def __init__(self):
                self.pose = {3:740,4:2320,5:1320,6:1500}
                self.fresh = False
            def pose_state_fresh(self):
                calls.append(("fresh", self.fresh))
                return self.fresh
            def reassert_pose_together(self, pose, duration):
                calls.append(("reassert", dict(pose), duration))
                self.fresh = True

        fake = types.SimpleNamespace(
            STREAM_URL="unused", CAPTURE_ARM_POSE={3:600,4:2200,5:1900},
            LiveVideo=mock.Mock(side_effect=StopAfterTrust("video-opened")),
        )
        robot = Robot()
        with mock.patch.object(APPROACH, "invalidate_pick_plan"), \
             mock.patch.object(APPROACH.time, "sleep", return_value=None):
            with self.assertRaisesRegex(StopAfterTrust, "video-opened"):
                APPROACH.run_approach(robot, precision=fake)
        reassert = next(c for c in calls if c[0] == "reassert")
        self.assertEqual(reassert[1], {3:740,4:2320,5:1320,6:1500})
        self.assertEqual(reassert[2], APPROACH.POSE_REASSERT_SECONDS)
        self.assertEqual(calls[-1], ("fresh", True))

    def test_non_search_camera_pose_is_rejected_before_video(self):
        class Robot:
            dry_run = False
            def __init__(self):
                self.pose = {3:600,4:2200,5:1900,6:1540}
                self.fresh = False
            def pose_state_fresh(self):
                return self.fresh
            def reassert_pose_together(self, pose, duration):
                raise AssertionError("non-search pose must fail before reassert")

        fake = types.SimpleNamespace(
            STREAM_URL="unused", CAPTURE_ARM_POSE={3:600,4:2200,5:1900},
            LiveVideo=mock.Mock(side_effect=AssertionError("video must not open")),
        )
        robot = Robot()
        with mock.patch.object(APPROACH, "invalidate_pick_plan"):
            with self.assertRaisesRegex(RuntimeError, "requires search/track camera geometry"):
                APPROACH.run_approach(robot, precision=fake)
        fake.LiveVideo.assert_not_called()

    def test_run_approach_uses_coarse_metric_then_stopped_capture_handoff(self):
        calls = []
        target = types.SimpleNamespace(nx=.50, ny=.72, area=5000)

        class FakeVideo:
            def __init__(self, url): calls.append(("video", url))
            def settle(self, delay): calls.append(("settle", delay))
            def read(self): return object()
            def close(self): calls.append(("close",))

        class FakeLock:
            def __init__(self, item): self.last = item

        class FakeRobot:
            def __init__(self):
                self.pose = {6:1610, 3:980, 4:2320, 5:1320}
                self.dry_run = False
            def pose_state_fresh(self): return True
            def stop(self): calls.append(("stop",))

        fake = types.SimpleNamespace(
            STREAM_URL="stream", CAMERA_DELAY_SECONDS=.12, TRACK_MIN_AREA=800,
            CENTER_CONFIRMATIONS=3,
            CAPTURE_ARM_POSE={3:600,4:2200,5:1900},
            LiveVideo=FakeVideo, TargetLock=FakeLock, LockedTargetLost=RuntimeError,
        )
        fake.detect_red_blob = lambda *a, **k: target
        fake.confirmed_candidate = lambda video, candidate: candidate
        fake.centre_gaze = lambda robot, video, lock, gaze, **kwargs: gaze
        fake.align_body_to_gaze = lambda robot, video, lock, gaze, **kwargs: (
            calls.append(("align_body", gaze.pan)) or types.SimpleNamespace(pan=1500, tilt=gaze.tilt)
        )
        fake.estimate_block = lambda pose, blob: types.SimpleNamespace(radius_cm=40.0, yaw_left_deg=20.0)
        staging = types.SimpleNamespace(
            gaze=types.SimpleNamespace(pan=1500, tilt=700),
            target_still_visible=True,
            block=types.SimpleNamespace(radius_cm=26.0),
        )
        def coarse(*a, **kw):
            calls.append((
                "coarse_metric", kw["target_radius_cm"], kw["allow_blind_arrival"],
                kw["allow_continuous_far"], kw["max_motion_steps"],
            ))
            return staging
        fake.approach_with_locked_gaze = coarse
        fake.visual_precapture_approach = mock.Mock(side_effect=AssertionError("pick owns precapture"))
        fake.establish_capture_pose = mock.Mock(side_effect=AssertionError("pick owns capture pose"))
        fake.visual_capture_approach = mock.Mock(side_effect=AssertionError("pick owns capture depth"))
        fake.measure_arm_face_alignment = mock.Mock(side_effect=AssertionError("pick owns face alignment"))

        r = FakeRobot()
        with mock.patch.object(APPROACH, "invalidate_pick_plan") as invalidate,              mock.patch.object(APPROACH, "save_coarse_handoff") as save,              mock.patch.object(APPROACH.time, "sleep", return_value=None):
            self.assertEqual(
                APPROACH.run_approach(
                    r, precision=fake, fast_camera_control=True,
                ),
                0,
            )
        invalidate.assert_called_once()
        self.assertEqual(
            [c for c in calls if c[0] == "coarse_metric"],
            [("coarse_metric", 26.0, False, True, 24)],
        )
        save.assert_called_once()
        self.assertEqual(save.call_args.kwargs["target_color"], "red")
        self.assertEqual(save.call_args.kwargs["coarse_radius_cm"], 26.0)
        fake.visual_precapture_approach.assert_not_called()
        fake.establish_capture_pose.assert_not_called()
        fake.visual_capture_approach.assert_not_called()
        fake.measure_arm_face_alignment.assert_not_called()

    def test_far_visible_target_without_metric_range_defers_to_guarded_staging(self):
        calls = []
        target = types.SimpleNamespace(nx=.53, ny=.46, area=450, height=22)

        class StopAfterStaging(RuntimeError):
            pass

        class FakeVideo:
            def __init__(self, url): pass
            def settle(self, delay): calls.append(("settle", delay))
            def read(self): return object()
            def close(self): calls.append(("close",))

        class FakeLock:
            def __init__(self, item): self.last = item

        class FakeRobot:
            dry_run = True
            def __init__(self): self.pose = {3:855, 4:2320, 5:1320, 6:1950}
            def pose_state_fresh(self): return True
            def stop(self): calls.append(("stop",))

        fake = types.SimpleNamespace(
            STREAM_URL="stream", CAMERA_DELAY_SECONDS=.12, TRACK_MIN_AREA=100,
            CENTER_CONFIRMATIONS=3,
            PRECAPTURE_ARM_POSE={3:500,4:2320,5:1320},
            CAPTURE_ARM_POSE={3:600,4:2200,5:1900},
            LiveVideo=FakeVideo, TargetLock=FakeLock, LockedTargetLost=RuntimeError,
        )
        fake.detect_red_blob = lambda *a, **k: target
        fake.confirmed_candidate = lambda video, candidate: candidate
        fake.centre_gaze = lambda robot, video, lock, gaze, **kwargs: gaze
        fake.target_bearing_left_deg = lambda gaze, blob, **kwargs: 45.0
        fake.align_body_to_gaze = lambda robot, video, lock, gaze, **kwargs: (
            calls.append(("align_body",))
            or types.SimpleNamespace(pan=1500, tilt=gaze.tilt)
        )
        fake.estimate_block = mock.Mock(return_value=None)
        def staging(*a, **kw):
            calls.append(("staging", kw["allow_blind_arrival"], kw["allow_body_realign"]))
            raise StopAfterStaging("staging-called")
        fake.approach_with_locked_gaze = staging

        with mock.patch.object(APPROACH, "invalidate_pick_plan"), \
             mock.patch.object(APPROACH.time, "sleep", return_value=None):
            with self.assertRaisesRegex(StopAfterStaging, "staging-called"):
                APPROACH.run_approach(FakeRobot(), precision=fake)

        self.assertIn(("align_body",), calls)
        self.assertIn(("staging", False, False), calls)
        self.assertGreaterEqual(fake.estimate_block.call_count, 2)

    def test_diagonal_face_alignment_is_deferred_to_pick(self):
        target = types.SimpleNamespace(nx=.50, ny=.50, area=5000)

        class FakeVideo:
            def __init__(self, url): pass
            def settle(self, delay): pass
            def read(self): return object()
            def close(self): pass

        class FakeLock:
            def __init__(self, item): self.last = item

        class FakeRobot:
            dry_run = True
            def __init__(self): self.pose={3:980,4:2320,5:1320,6:1500}
            def pose_state_fresh(self): return True
            def stop(self): pass

        fake = types.SimpleNamespace(
            STREAM_URL="stream", CAMERA_DELAY_SECONDS=.12, TRACK_MIN_AREA=800,
            CENTER_CONFIRMATIONS=3, CAPTURE_TARGET_NX=.50,
            CAPTURE_ARM_POSE={3:600,4:2200,5:1900},
            LiveVideo=FakeVideo, TargetLock=FakeLock, LockedTargetLost=RuntimeError,
        )
        fake.detect_red_blob=lambda *a,**k: target
        fake.confirmed_candidate=lambda video,candidate:candidate
        fake.centre_gaze=lambda *a,**k:a[3]
        fake.align_body_to_gaze=lambda *a,**k:a[3]
        fake.estimate_block=lambda pose,blob:types.SimpleNamespace(radius_cm=26.0,yaw_left_deg=0.0)
        fake.approach_with_locked_gaze=lambda *a,**k: types.SimpleNamespace(
            gaze=a[3], target_still_visible=True,
            block=types.SimpleNamespace(radius_cm=26.0),
        )
        fake.measure_arm_face_alignment=mock.Mock(side_effect=AssertionError("pick owns face alignment"))
        fake.face_reposition_dogleg=mock.Mock(side_effect=AssertionError("pick owns dog-leg"))

        with mock.patch.object(APPROACH,"invalidate_pick_plan"),              mock.patch.object(APPROACH,"save_coarse_handoff") as save,              mock.patch.object(APPROACH.time,"sleep",return_value=None):
            self.assertEqual(APPROACH.run_approach(FakeRobot(), precision=fake), 0)
        save.assert_called_once()
        fake.measure_arm_face_alignment.assert_not_called()
        fake.face_reposition_dogleg.assert_not_called()

    def test_live_close_pose_must_return_to_search_track_geometry_before_approach(self):
        class Robot:
            dry_run=False
            pose={3:600,4:2200,5:1900,6:1500}
            def pose_state_fresh(self): return True
        fake=types.SimpleNamespace(
            STREAM_URL='unused', CAPTURE_ARM_POSE={3:600,4:2200,5:1900},
            LiveVideo=mock.Mock(side_effect=AssertionError('video must not open')),
        )
        with mock.patch.object(APPROACH,'invalidate_pick_plan'):
            with self.assertRaisesRegex(RuntimeError,'requires search/track camera geometry'):
                APPROACH.run_approach(Robot(),precision=fake)
        fake.LiveVideo.assert_not_called()

    def test_face_restage_uses_bounded_dogleg_then_straight_depth_recovery(self):
        calls=[]
        gaze=types.SimpleNamespace(pan=1500,tilt=700)
        target=types.SimpleNamespace(nx=.50,ny=.72,area=9000)
        lock=types.SimpleNamespace(last=target)
        robot=object()
        samples=iter([
            (types.SimpleNamespace(error_deg=31.0),4.0,31.0,target),
            (types.SimpleNamespace(error_deg=6.0),6.0,6.0,target),
        ])
        fake=types.SimpleNamespace(CAPTURE_TARGET_NX=.50)
        fake.measure_arm_face_alignment=lambda *a,**k:next(samples)
        fake.face_reposition_dogleg=lambda *a,**k:calls.append(("dogleg",k["side"]))
        fake.fine_align_horizontal=lambda *a,**k:(calls.append(("fine_x",)) or a[3])
        fake.approach_with_locked_gaze=lambda *a,**k:(
            calls.append((
                "depth",k["target_radius_cm"],k["allow_continuous_far"],
                k["allow_body_realign"],k["max_motion_steps"],
            ))
            or types.SimpleNamespace(gaze=a[3])
        )

        result=APPROACH._bounded_face_direction_restage(
            robot,object(),lock,gaze,fake,flip_x=False,flip_y=False
        )
        self.assertIs(result,gaze)
        self.assertEqual(calls[0],("dogleg","right"))
        self.assertEqual(calls[1],("fine_x",))
        self.assertEqual(calls[2],("depth",26.0,False,False,6))

    def test_face_restage_tolerates_one_noisy_face_sample_when_bearing_moves_sideways(self):
        calls=[]
        gaze=types.SimpleNamespace(pan=1500,tilt=700)
        target=types.SimpleNamespace(nx=.50,ny=.72,area=9000)
        lock=types.SimpleNamespace(last=target)
        robot=object()
        samples=iter([
            (types.SimpleNamespace(error_deg=12.0),-9.0,+12.0,target),
            (types.SimpleNamespace(error_deg=12.5),-6.5,+12.5,target),
            (types.SimpleNamespace(error_deg=6.0),-4.0,+6.0,target),
        ])
        fake=types.SimpleNamespace(CAPTURE_TARGET_NX=.50)
        fake.measure_arm_face_alignment=lambda *a,**k:next(samples)
        fake.face_reposition_dogleg=lambda *a,**k:calls.append(("dogleg",k["side"]))
        fake.fine_align_horizontal=lambda *a,**k:a[3]
        fake.approach_with_locked_gaze=lambda *a,**k:types.SimpleNamespace(gaze=a[3])

        result=APPROACH._bounded_face_direction_restage(
            robot,object(),lock,gaze,fake,flip_x=False,flip_y=False
        )
        self.assertIs(result,gaze)
        self.assertEqual(calls,[("dogleg","right"),("dogleg","right")])

    def test_face_restage_keeps_using_measured_slow_dogleg_progress_up_to_six(self):
        calls=[]
        gaze=types.SimpleNamespace(pan=1500,tilt=700)
        target=types.SimpleNamespace(nx=.50,ny=.72,area=9000)
        lock=types.SimpleNamespace(last=target)
        robot=object()
        samples=iter([
            (types.SimpleNamespace(error_deg=16.6),+19.6,-16.6,target),
            (types.SimpleNamespace(error_deg=12.8),+16.2,-12.8,target),
            (types.SimpleNamespace(error_deg=13.4),+14.4,-13.4,target),
            (types.SimpleNamespace(error_deg=10.9),+12.2,-10.9,target),
            (types.SimpleNamespace(error_deg=8.3),+9.3,-8.3,target),
            (types.SimpleNamespace(error_deg=6.4),+6.5,-6.4,target),
        ])
        fake=types.SimpleNamespace(
            CAPTURE_TARGET_NX=.50,
            FACE_REPOSITION_MAX_PULSES=6,
            FACE_REPOSITION_MAX_STALLED_PULSES=2,
        )
        fake.measure_arm_face_alignment=lambda *a,**k:next(samples)
        fake.face_reposition_dogleg=lambda *a,**k:calls.append(("dogleg",k["side"]))
        fake.fine_align_horizontal=lambda *a,**k:a[3]
        fake.approach_with_locked_gaze=lambda *a,**k:types.SimpleNamespace(gaze=a[3])

        result=APPROACH._bounded_face_direction_restage(
            robot,object(),lock,gaze,fake,flip_x=False,flip_y=False
        )
        self.assertIs(result,gaze)
        self.assertEqual(calls,[("dogleg","left")]*5)

    def test_face_restage_allows_seventh_dogleg_while_measurements_keep_improving(self):
        calls=[]
        gaze=types.SimpleNamespace(pan=1500,tilt=700)
        target=types.SimpleNamespace(nx=.50,ny=.72,area=9000)
        lock=types.SimpleNamespace(last=target)
        robot=object()
        samples=iter([
            (types.SimpleNamespace(error_deg=16.6),+19.6,-16.6,target),
            (types.SimpleNamespace(error_deg=14.8),+17.8,-14.8,target),
            (types.SimpleNamespace(error_deg=13.2),+16.0,-13.2,target),
            (types.SimpleNamespace(error_deg=11.9),+14.4,-11.9,target),
            (types.SimpleNamespace(error_deg=10.5),+12.8,-10.5,target),
            (types.SimpleNamespace(error_deg=9.0),+11.0,-9.0,target),
            (types.SimpleNamespace(error_deg=7.8),+9.4,-7.8,target),
            (types.SimpleNamespace(error_deg=6.4),+7.8,-6.4,target),
        ])
        fake=types.SimpleNamespace(CAPTURE_TARGET_NX=.50)
        fake.measure_arm_face_alignment=lambda *a,**k:next(samples)
        fake.face_reposition_dogleg=lambda *a,**k:calls.append(("dogleg",k["side"]))
        fake.fine_align_horizontal=lambda *a,**k:a[3]
        fake.approach_with_locked_gaze=lambda *a,**k:types.SimpleNamespace(gaze=a[3])

        result=APPROACH._bounded_face_direction_restage(
            robot,object(),lock,gaze,fake,flip_x=False,flip_y=False
        )
        self.assertIs(result,gaze)
        self.assertEqual(calls,[("dogleg","left")]*7)

    def test_pick_ready_zone_is_tighter_than_coarse_arrival(self):
        coarse = blob(0.59, 0.75, 5000)
        tight = blob(0.54, 0.75, 5000)
        self.assertEqual(PLAN.approach_command(coarse).kind, "arrived")
        self.assertFalse(PLAN.in_pick_ready_zone(coarse))
        self.assertTrue(PLAN.in_pick_ready_zone(tight))
        self.assertFalse(PLAN.in_pick_ready_zone(tight, pan=1620))
        self.assertTrue(PLAN.in_pick_ready_zone(tight, pan=1570))

    def test_near_centered_blob_is_arrived(self):
        command = PLAN.approach_command(blob(0.50, 0.75, 5000))
        self.assertEqual(command.kind, "arrived")
        self.assertTrue(PLAN.in_grasp_zone(blob(0.50, 0.75, 5000)))

    def test_too_close_backs_up(self):
        command = PLAN.approach_command(blob(0.50, 0.95, 12000))
        self.assertEqual(command.kind, "backward")

    def test_missing_blob_is_lost(self):
        self.assertEqual(PLAN.approach_command(None).kind, "lost")

    def test_turn_is_stronger_than_the_stuck_spin(self):
        command = PLAN.approach_command(blob(0.20, 0.50, 4000))
        self.assertEqual(command.kind, "rotate-left")
        self.assertGreaterEqual(command.speed, 31)
        self.assertGreaterEqual(command.duration, 0.4)

    def test_same_view_detects_the_frozen_left_blob(self):
        first = blob(0.134, 0.633, 1661)
        second = blob(0.134, 0.633, 1622)
        self.assertTrue(PLAN.same_view(first, second))
        self.assertFalse(PLAN.same_view(first, blob(0.28, 0.633, 1661)))

    def test_tiny_left_edge_blob_is_ignored(self):
        self.assertTrue(CAMERA.is_small_edge_blob(0.134, 1661))
        self.assertFalse(CAMERA.is_small_edge_blob(0.40, 1661))
        self.assertFalse(CAMERA.is_small_edge_blob(0.134, 4000))

    def test_search_pose_looks_at_the_floor(self):
        self.assertLess(POSES.POSE_SEARCH[3], POSES.POSE_OBSERVE[3])
        self.assertGreater(POSES.POSE_SEARCH[5], POSES.POSE_OBSERVE[5])

    def test_search_look_sweeps_the_head(self):
        first_pan, first_tilt = PLAN.search_look(0, tilt_home=740)
        side_pan, _tilt = PLAN.search_look(1, tilt_home=740)
        self.assertEqual(first_pan, 1500)
        self.assertNotEqual(side_pan, 1500)
        self.assertEqual(first_tilt, 740)

    def test_search_drive_waits_for_a_head_sweep(self):
        self.assertIsNone(PLAN.search_drive(0))
        later = PLAN.search_drive(len(PLAN.SEARCH_PANS))
        self.assertIsNotNone(later)
        self.assertIn(later.kind, {"rotate-left", "rotate-right"})

    def test_yawed_neck_uses_small_alignment_pulse(self):
        command = PLAN.approach_command(blob(0.50, 0.55, 3000), pan=1318)
        self.assertEqual(command.kind, "rotate-right")
        self.assertGreaterEqual(command.speed, 31)
        self.assertGreaterEqual(command.duration, 0.12)
        self.assertLessEqual(command.duration, 0.28)

    def test_yawed_neck_turns_the_body(self):
        centered = blob(0.50, 0.40, 1200)
        self.assertEqual(PLAN.approach_command(centered, pan=1500).kind, "forward")
        self.assertEqual(PLAN.approach_command(centered, pan=1800).kind, "rotate-left")
        self.assertEqual(PLAN.approach_command(centered, pan=1200).kind, "rotate-right")

    def test_precision_approach_rejects_incomplete_metric_pose_before_video(self):
        class FakeRobot:
            pose = {3: 620, 6: 1500}

        precision = mock.Mock()
        with self.assertRaisesRegex(RuntimeError, "trusted full arm pose"):
            APPROACH.run_approach(FakeRobot(), precision=precision)
        precision.LiveVideo.assert_not_called()



if __name__ == "__main__":
    unittest.main(verbosity=2)
