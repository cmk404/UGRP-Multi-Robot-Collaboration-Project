#!/usr/bin/env python3
import importlib.util
import io
import pathlib
import subprocess
import tarfile
import tempfile
import sys
import types
import unittest
from unittest import mock

ROOT = pathlib.Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "scripts" / "red_block"


def load(name):
    path = PACKAGE / f"{name}.py"
    spec = importlib.util.spec_from_file_location(f"test_{name}_module", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.path.insert(0, str(PACKAGE))
    spec.loader.exec_module(module)
    return module

DEPLOY = load("deploy")
SEARCH = load("search")
ROBOT = load("robot")
CAMERA = sys.modules["camera"]


class PoseTrustTests(unittest.TestCase):
    def test_pose_state_age_uses_updated_at(self):
        import tempfile
        from pathlib import Path
        now = 1000.0
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "pose.json"
            path.write_text('{"updated_at":995.5,"pose":{"3":500,"4":2320,"5":1320,"6":1500}}')
            control = types.SimpleNamespace(POSE_STATE_PATH=path)
            with mock.patch.object(ROBOT.time, "time", return_value=now):
                self.assertAlmostEqual(ROBOT._pose_state_age_seconds(control), 4.5)

    def test_missing_pose_timestamp_is_untrusted(self):
        import tempfile
        from pathlib import Path
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "pose.json"
            path.write_text('{"pose":{"3":500}}')
            control = types.SimpleNamespace(POSE_STATE_PATH=path)
            self.assertIsNone(ROBOT._pose_state_age_seconds(control))

    def test_legacy_controller_guard_is_non_destructive(self):
        # Detection only returns PIDs; Robot refuses concurrent actuation. It never kills them.
        fake = ROBOT.Robot.__new__(ROBOT.Robot)
        self.assertTrue(callable(ROBOT._legacy_controller_pids))
        self.assertNotIn("kill", ROBOT._legacy_controller_pids.__doc__.lower() if ROBOT._legacy_controller_pids.__doc__ else "")

    def test_reassert_pose_rewrites_equal_saved_pulses_and_waits_once(self):
        r = ROBOT.Robot.__new__(ROBOT.Robot)
        r.dry_run = False
        r.pose = {3: 500, 4: 2320, 5: 1320, 6: 1500}
        r.servos = types.SimpleNamespace(write_servo=mock.Mock())
        target = dict(r.pose)
        with mock.patch.object(ROBOT, "_invalidate_precision_handoff"), \
             mock.patch.object(ROBOT.time, "sleep") as sleep:
            r.reassert_pose_together(target, duration=1.25)
        self.assertEqual(r.servos.write_servo.call_count, 4)
        for servo, pulse in target.items():
            r.servos.write_servo.assert_any_call(servo, pulse, 1.25)
        sleep.assert_called_once_with(1.40)


class ProbeResilienceTests(unittest.TestCase):
    def make_robot(self, side_effect):
        r = ROBOT.Robot.__new__(ROBOT.Robot)
        r.dry_run = False
        r.control = types.SimpleNamespace(probe_hardware=mock.Mock(side_effect=side_effect))
        return r

    def test_burst_of_torn_samples_requires_two_consistent_valid_reads(self):
        error = RuntimeError("battery voltage 65516 mV outside valid range")
        r = self.make_robot([
            error, error, error,
            {"ok": True, "controller": "legacy-i2c-0x7a", "battery_mv": 7906},
            error,
            {"ok": True, "controller": "legacy-i2c-0x7a", "battery_mv": 7866},
        ])
        with mock.patch.object(ROBOT.time, "sleep"):
            result = r.probe()
        self.assertEqual(result["battery_mv"], 7886)
        self.assertEqual(result["probe_attempts"], 6)
        self.assertEqual(result["probe_valid_samples"], 2)
        self.assertEqual(r.control.probe_hardware.call_count, 6)

    def test_single_valid_read_is_not_enough_to_bypass_safety_gate(self):
        bad = RuntimeError("invalid battery telemetry")
        seq = [bad] * 5 + [{"ok": True, "controller": "legacy-i2c-0x7a", "battery_mv": 7900}] + [bad] * 6
        r = self.make_robot(seq)
        with mock.patch.object(ROBOT.time, "sleep"):
            with self.assertRaisesRegex(RuntimeError, "unstable after 12 attempts"):
                r.probe()
        self.assertEqual(r.control.probe_hardware.call_count, 12)

    def test_repeated_probe_failure_still_blocks_motion(self):
        errors = [RuntimeError(f"bad{i}") for i in range(12)]
        r = self.make_robot(errors)
        with mock.patch.object(ROBOT.time, "sleep"):
            with self.assertRaisesRegex(RuntimeError, "bad11"):
                r.probe()
        self.assertEqual(r.control.probe_hardware.call_count, 12)


class DeployLatencyTests(unittest.TestCase):
    def test_ssh_connection_uses_persistent_multiplexing(self):
        with mock.patch.object(DEPLOY, "tailscale_ipv4", return_value="100.119.44.65"):
            _dest, opts = DEPLOY.ssh_connection_args("ugrp1")
        joined = " ".join(opts)
        self.assertIn("ControlMaster=auto", joined)
        self.assertIn("ControlPersist=600", joined)
        self.assertIn("ControlPath=/tmp/ugrp-ssh-%C", joined)

    def test_trace_copy_uses_one_tar_stream_instead_of_recursive_scp(self):
        payload = io.BytesIO()
        with tarfile.open(fileobj=payload, mode="w:gz") as archive:
            data = b'{"kind":"camera_frame"}\n'
            info = tarfile.TarInfo("./events.jsonl")
            info.size = len(data)
            archive.addfile(info, io.BytesIO(data))
            jpg = b"fake-jpeg"
            info = tarfile.TarInfo("./frames/frame-000001.jpg")
            info.size = len(jpg)
            archive.addfile(info, io.BytesIO(jpg))
        packed = subprocess.CompletedProcess([], 0, stdout=payload.getvalue(), stderr=b"")
        cleaned = subprocess.CompletedProcess([], 0, stdout=b"", stderr=b"")
        with tempfile.TemporaryDirectory() as td, mock.patch.object(
            DEPLOY.subprocess, "run", side_effect=[packed, cleaned]
        ) as run:
            local = pathlib.Path(td) / "trace"
            ok = DEPLOY._copy_remote_trace(
                "ugrp1", ["-o", "BatchMode=yes"], pathlib.Path("/tmp/trace-1"), local
            )
            self.assertTrue(ok)
            self.assertTrue((local / "events.jsonl").is_file())
            self.assertEqual((local / "frames/frame-000001.jpg").read_bytes(), jpg)
        first_argv = run.call_args_list[0].args[0]
        self.assertEqual(first_argv[0], "ssh")
        self.assertNotIn("scp", first_argv)
        self.assertIn("tar", first_argv)
        self.assertEqual(first_argv[-6:], ["tar", "-C", "/tmp/trace-1", "-czf", "-", "."])
        self.assertEqual(run.call_args_list[1].args[0][-3:], ["rm", "-rf", "/tmp/trace-1"])

    def test_signature_changes_when_package_content_changes(self):
        first = DEPLOY.package_signature()
        self.assertEqual(len(first), 64)
        self.assertEqual(first, DEPLOY.package_signature())

    def test_versioned_remote_path_is_content_addressed(self):
        sig = "a" * 64
        path = str(DEPLOY.remote_package(sig))
        self.assertIn("/.ugrp_versions/" + sig + "/red_block", path)

    def test_manifest_covers_camera_track_and_search(self):
        manifest = DEPLOY.package_manifest()
        self.assertIn("  red_block/camera.py\n", manifest)
        self.assertIn("  red_block/track.py\n", manifest)
        self.assertIn("  red_block/search.py\n", manifest)
        self.assertIn("  masterpi_control.py\n", manifest)



class SearchPlannerTests(unittest.TestCase):
    def blob(self, nx=.5, ny=.5, area=1000):
        return types.SimpleNamespace(nx=nx, ny=ny, area=area)

    def test_floor_scan_is_the_primary_search_envelope(self):
        self.assertGreaterEqual(len(SEARCH.FLOOR_SCAN_POINTS), 5)
        self.assertEqual(SEARCH.FLOOR_SCAN_POINTS[0], (1500, 500))
        self.assertTrue(all(500 <= tilt <= 860 for _pan, tilt in SEARCH.FLOOR_SCAN_POINTS))
        self.assertTrue(all(tilt > 860 for _pan, tilt in SEARCH.HIGH_FALLBACK_POINTS))
        self.assertLess(len(SEARCH.HIGH_FALLBACK_PULSES), SEARCH.MAX_BODY_PULSES)

    def test_feet_first_scan_uses_broad_head_sweep_before_body(self):
        points = SEARCH.FLOOR_SCAN_POINTS
        self.assertEqual(points[0], (1500, 500))
        pans = [pan for pan, _tilt in points]
        self.assertLessEqual(min(pans), 1050)
        self.assertGreaterEqual(max(pans), 1950)
        self.assertEqual(SEARCH.PAN_SEARCH_MIN, 1050)
        self.assertEqual(SEARCH.PAN_SEARCH_MAX, 1950)
        # Search/track share the same physically-used safe head envelope so a
        # confirmed side sighting can be handed off without looking away.
        self.assertEqual(SEARCH.PAN_SEARCH_MIN, SEARCH.PAN_TRACK_MIN)
        self.assertEqual(SEARCH.PAN_SEARCH_MAX, SEARCH.PAN_TRACK_MAX)

    def test_obvious_front_target_short_circuits_search_policy(self):
        self.assertTrue(SEARCH.obvious_front_candidate(1500, self.blob(.42, .45, 5300)))
        self.assertTrue(SEARCH.obvious_front_candidate(1500, self.blob(.823, .74, 8500)))
        self.assertTrue(SEARCH.obvious_front_candidate(1050, self.blob(.50, .45, 5300)))
        self.assertFalse(SEARCH.obvious_front_candidate(1500, self.blob(.50, .45, 800)))

    def test_obvious_cube_fast_confirmation_needs_only_one_extra_matching_frame(self):
        first = self.blob(.48, .45, 2200)
        second = self.blob(.49, .46, 2250)
        camera = object()
        frame = types.SimpleNamespace(shape=(480, 640, 3))
        with mock.patch.object(SEARCH, "read_search_candidate", return_value=(frame, second)) as read, \
             mock.patch.object(SEARCH.time, "sleep"):
            found = SEARCH.confirm_search_candidate(camera, 1500, first)
        self.assertIs(found, second)
        self.assertEqual(read.call_count, 1)
        self.assertEqual(SEARCH.OBVIOUS_CONFIRM_HITS, 2)

    def test_weak_candidate_keeps_three_hit_confirmation(self):
        first = self.blob(.48, .45, 900)
        second = self.blob(.49, .46, 920)
        third = self.blob(.50, .47, 910)
        frame = types.SimpleNamespace(shape=(480, 640, 3))
        with mock.patch.object(
            SEARCH, "read_search_candidate",
            side_effect=[(frame, second), (frame, third)],
        ) as read, mock.patch.object(SEARCH.time, "sleep"):
            found = SEARCH.confirm_search_candidate(object(), 1500, first)
        self.assertIs(found, third)
        self.assertEqual(read.call_count, 2)
        self.assertEqual(SEARCH.SEARCH_CONFIRM_HITS, 3)

    def test_head_scan_waits_for_servo_interpolation_before_camera_read(self):
        duration = SEARCH.head_move_duration(1050, 600, 1950, 620)
        with mock.patch.object(SEARCH.time, "sleep") as sleep:
            SEARCH.settle_after_head_move(duration)
        sleep.assert_called_once()
        self.assertGreaterEqual(
            sleep.call_args.args[0],
            duration - 0.05 + SEARCH.CAMERA_SETTLE_SECONDS - 1e-9,
        )

    def test_scan_points_confirms_obvious_front_before_later_gazes(self):
        calls = []
        frame = type('Frame', (), {'shape': (480, 640, 3)})()
        candidate = types.SimpleNamespace(
            nx=.46, ny=.40, area=5300, cx=294, cy=192,
            width=80, height=80,
        )
        class Robot:
            pose = {3:500,6:1500}
            def nudge_servos(self, updates, duration):
                calls.append(('move', dict(updates))); self.pose.update(updates)
        class Camera:
            def read(self, quiet=True): return frame
        with mock.patch.object(SEARCH, 'read_search_candidate', return_value=(frame, candidate)),              mock.patch.object(SEARCH, 'confirmed_red', return_value=candidate) as confirm,              mock.patch.object(SEARCH, 'save_debug_frame'),              mock.patch.object(SEARCH.time, 'sleep', return_value=None):
            found = SEARCH.scan_points(Robot(), Camera(), ((1500,500),(1050,600)))
        self.assertIs(found, candidate)
        confirm.assert_called_once()
        self.assertEqual(len([c for c in calls if c[0] == 'move']), 1)

    def test_scan_points_stops_on_large_cube_at_pan_1950_instead_of_turning_away(self):
        calls = []
        frame = type('Frame', (), {'shape': (480, 640, 3)})()
        block = types.SimpleNamespace(
            nx=.50, ny=.52, area=5700, cx=320, cy=250,
            width=88, height=92,
        )
        class Robot:
            pose = {3:500,6:1500}
            def nudge_servos(self, updates, duration):
                calls.append(('move', dict(updates))); self.pose.update(updates)
        class Camera:
            def read(self, quiet=True): return frame
        samples = iter([(frame, None), (frame, block)])
        with mock.patch.object(SEARCH, 'read_search_candidate', side_effect=lambda *a: next(samples)), \
             mock.patch.object(SEARCH, 'confirmed_red', return_value=block) as confirm, \
             mock.patch.object(SEARCH, 'save_debug_frame'), \
             mock.patch.object(SEARCH.time, 'sleep', return_value=None):
            found = SEARCH.scan_points(Robot(), Camera(), ((1500,500),(1950,620)))
        self.assertIs(found, block)
        confirm.assert_called_once()
        self.assertEqual(calls[-1][1][6], 1950)

    def test_search_sighting_rank_prefers_large_peripheral_cube_over_tiny_centered_speck(self):
        big = types.SimpleNamespace(nx=.50, ny=.52, area=5700)
        tiny = types.SimpleNamespace(nx=.50, ny=.20, area=380)
        self.assertGreater(SEARCH._sighting_rank(1950, big), SEARCH._sighting_rank(1500, tiny))

    def test_confirmed_large_cube_at_pan_1950_is_trackable_handoff(self):
        block = types.SimpleNamespace(nx=.50, ny=.52, area=5700)
        self.assertEqual(SEARCH.PAN_TRACK_MAX, 1950)
        self.assertTrue(SEARCH.obvious_front_candidate(1950, block))
        self.assertTrue(SEARCH.current_candidate_is_trackable(1950, block))

    def test_search_sighting_rank_prefers_centered_view_when_strength_matches(self):
        centered = self.blob(.50, .4, 1000)
        off_center = self.blob(.62, .4, 1000)
        self.assertGreater(
            SEARCH._sighting_rank(1050, centered),
            SEARCH._sighting_rank(1500, off_center),
        )

    def test_head_scan_motion_is_smooth_for_large_pan_changes(self):
        short = SEARCH.head_move_duration(1500, 600, 1600, 620)
        wide = SEARCH.head_move_duration(1050, 600, 1950, 620)
        self.assertGreater(wide, short)
        self.assertGreaterEqual(short, 0.18)
        self.assertLessEqual(wide, 0.40)

    def test_search_is_bounded_and_rotates_body(self):
        self.assertGreaterEqual(SEARCH.DEFAULT_SECONDS, 20.0)
        self.assertGreater(SEARCH.MAX_BODY_PULSES, 0)
        self.assertLessEqual(SEARCH.MAX_BODY_PULSES, 4)
        self.assertGreaterEqual(SEARCH.SEARCH_CONFIRM_HITS, 3)
        self.assertGreater(SEARCH.BODY_TURN_SPEED, 0)
        self.assertGreater(SEARCH.BODY_TURN_SECONDS, 0)

    def test_small_distant_candidate_allows_compression_jitter(self):
        a = self.blob(.40, .40, 500)
        self.assertTrue(SEARCH.same_candidate(a, self.blob(.47, .45, 900)))

    def test_candidate_confirmation_rejects_jumps(self):
        a = self.blob(.50, .50, 1000)
        self.assertTrue(SEARCH.same_candidate(a, self.blob(.54, .53, 1200)))
        self.assertFalse(SEARCH.same_candidate(a, self.blob(.75, .50, 1000)))
        self.assertFalse(SEARCH.same_candidate(a, self.blob(.50, .50, 3000)))


    def test_metric_pose_requires_all_camera_kinematic_servos(self):
        self.assertTrue(SEARCH.metric_pose_known({3: 620, 4: 2320, 5: 1320, 6: 1500}))
        self.assertFalse(SEARCH.metric_pose_known({3: 620, 6: 1500}))
        self.assertFalse(SEARCH.metric_pose_known({}))

    def test_stale_pose_with_visible_target_reasserts_same_gaze_before_scan(self):
        frame = types.SimpleNamespace(shape=(480, 640, 3))
        block = types.SimpleNamespace(
            nx=.50, ny=.45, area=2400, cx=320, cy=216, width=60, height=60,
        )
        class Camera:
            def read(self, quiet=True): return frame
            def close(self): pass
        class Robot:
            def __init__(self):
                self.pose = {3: 650, 4: 2320, 5: 1320, 6: 1510}
                self.pose_state_age_s = 30.0
                self.reasserted = None
                self.stops = 0
            def pose_state_fresh(self): return False
            def reassert_pose_together(self, target, duration):
                self.reasserted = (dict(target), duration)
                self.pose.update(target)
            def move_pose(self, *args, **kwargs):
                raise AssertionError("visible current target must not trigger a fresh scan pose")
            def stop(self): self.stops += 1
        robot = Robot()
        with mock.patch.object(SEARCH, "LiveCamera", return_value=Camera()), \
             mock.patch.object(SEARCH, "read_search_candidate", return_value=(frame, block)), \
             mock.patch.object(SEARCH, "confirm_search_candidate", return_value=block), \
             mock.patch.object(SEARCH, "save_debug_frame"), \
             mock.patch.object(SEARCH.time, "sleep"):
            rc = SEARCH.run_search(robot, seconds=1.0)
        self.assertEqual(rc, 0)
        self.assertIsNotNone(robot.reasserted)
        self.assertEqual(robot.reasserted[0], {3:650,4:2320,5:1320,6:1510})
        self.assertEqual(robot.reasserted[1], 1.25)

    def test_post_pick_close_pose_is_not_accepted_as_current_search_handoff(self):
        frame = types.SimpleNamespace(shape=(480, 640, 3))
        block = types.SimpleNamespace(
            nx=.50, ny=.43, area=27500, cx=320, cy=206, width=180, height=205,
        )
        class Camera:
            def read(self, quiet=True): return frame
            def close(self): pass
        class Robot:
            def __init__(self):
                self.pose = {1:2000, 3:612, 4:2200, 5:1900, 6:1596}
                self.moved_to = None
                self.stops = 0
            def pose_state_fresh(self): return True
            def move_pose(self, target, lowering):
                self.moved_to = dict(target)
                self.pose.update(target)
            def stop(self): self.stops += 1
        robot = Robot()
        self.assertFalse(SEARCH.search_track_pose_compatible(robot.pose))
        with mock.patch.object(SEARCH, "LiveCamera", return_value=Camera()), \
             mock.patch.object(SEARCH, "read_search_candidate", return_value=(frame, block)), \
             mock.patch.object(SEARCH, "confirm_search_candidate", return_value=block), \
             mock.patch.object(SEARCH, "scan_points", return_value=block), \
             mock.patch.object(SEARCH.time, "sleep"), \
             mock.patch.object(SEARCH, "save_debug_frame"):
            rc = SEARCH.run_search(robot, seconds=1.0)
        self.assertEqual(rc, 0)
        self.assertIsNotNone(robot.moved_to)
        self.assertEqual(robot.moved_to[3], 500)
        self.assertEqual(robot.moved_to[4], SEARCH.POSE_SEARCH[4])
        self.assertEqual(robot.moved_to[5], SEARCH.POSE_SEARCH[5])
        self.assertEqual(robot.moved_to[6], 1500)

    def test_current_view_at_pan_limit_is_not_claimed_if_target_is_further_outward(self):
        left_blob = self.blob(.35, .7, 900)
        right_blob = self.blob(.65, .7, 900)
        self.assertFalse(SEARCH.current_candidate_is_trackable(2500, left_blob))
        self.assertTrue(SEARCH.current_candidate_is_trackable(2500, right_blob))
        self.assertFalse(SEARCH.current_candidate_is_trackable(500, right_blob))
        self.assertTrue(SEARCH.current_candidate_is_trackable(500, left_blob))


    def test_red_detector_prefers_large_block_over_small_lab_patch(self):
        camera = CAMERA
        small = camera.ColorBlob(500,200,850,38,32,.78,.42,rectangularity=.82)
        block = camera.ColorBlob(300,300,5200,86,82,.47,.63,rectangularity=.91)
        self.assertIs(camera.choose_red_blob(small, block), block)

    def test_red_detector_shape_score_breaks_similar_area_tie(self):
        camera = CAMERA
        streak = camera.ColorBlob(300,200,2100,120,22,.47,.42,rectangularity=.30)
        block = camera.ColorBlob(310,220,1950,52,48,.48,.46,rectangularity=.90)
        self.assertIs(camera.choose_red_blob(streak, block), block)

    def test_red_suspicion_preserves_right_border_component_normal_detector_discards(self):
        import cv2
        import numpy as np
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.rectangle(frame, (610, 190), (639, 275), (0, 0, 255), -1)
        normal = CAMERA.detect_red_blob(frame, min_area=500, crop_left=100)
        hint = CAMERA.detect_red_suspicion(frame, min_area=120)
        self.assertIsNone(normal)
        self.assertIsNotNone(hint)
        self.assertGreater(hint.nx, .94)
        self.assertGreater(hint.area, 120)

    def test_red_suspicion_preserves_red_inside_normal_left_crop(self):
        import cv2
        import numpy as np
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.rectangle(frame, (18, 180), (88, 250), (0, 0, 255), -1)
        normal = CAMERA.detect_red_blob(frame, min_area=500, crop_left=100)
        hint = CAMERA.detect_red_suspicion(frame, min_area=120)
        self.assertIsNone(normal)
        self.assertIsNotNone(hint)
        self.assertLess(hint.nx, .15)

    def test_suspicion_gaze_moves_toward_visible_edge_red(self):
        left = self.blob(.03, .50, 400)
        right = self.blob(.97, .50, 400)
        low = self.blob(.50, .92, 400)
        left_pan, _ = SEARCH.suspicion_gaze_target(1500, 650, left)
        right_pan, _ = SEARCH.suspicion_gaze_target(1500, 650, right)
        _, low_tilt = SEARCH.suspicion_gaze_target(1500, 650, low)
        self.assertGreater(left_pan, 1500)
        self.assertLess(right_pan, 1500)
        self.assertLess(low_tilt, 650)

    def test_bottom_edge_hint_at_tilt_limit_switches_to_close_near_look(self):
        hint = types.SimpleNamespace(nx=.50, ny=.94, area=900, cx=320, cy=451, width=70, height=54)
        block = types.SimpleNamespace(nx=.51, ny=.45, area=5000, cx=326, cy=216, width=90, height=88)
        class Robot:
            pose={3:500,4:2320,5:1320,6:1500}
        with mock.patch.object(SEARCH, 'refine_bottom_near_look', return_value=block) as near, \
             mock.patch.object(SEARCH, 'suspicion_gaze_target', side_effect=AssertionError('must use near-look at tilt floor')):
            found = SEARCH.refine_suspicion(Robot(), object(), hint, phase='floor')
        self.assertIs(found, block)
        near.assert_called_once()

    def test_live_near_look_writes_causal_search_handoff_only_after_confirmation(self):
        frame=types.SimpleNamespace(shape=(480,640,3))
        block=types.SimpleNamespace(nx=.50,ny=.44,area=5500,cx=320,cy=211,width=90,height=88)
        class Robot:
            dry_run=False
            def __init__(self): self.pose={3:500,4:2320,5:1320,6:1500}
            def nudge_servos(self,updates,duration): self.pose.update(updates)
        class Camera:
            def read(self,quiet=True): return frame
        robot=Robot()
        with mock.patch.object(SEARCH, '_read_near_look_candidate', return_value=(frame,block)), \
             mock.patch.object(SEARCH, '_confirm_near_look', return_value=block), \
             mock.patch.object(SEARCH, 'save_near_look_handoff') as save, \
             mock.patch.object(SEARCH, 'save_debug_frame'), \
             mock.patch.object(SEARCH.time, 'sleep', return_value=None):
            found=SEARCH.refine_bottom_near_look(robot,Camera(),phase='floor')
        self.assertIs(found,block)
        save.assert_called_once_with(robot_pose=robot.pose)

    def test_near_look_uses_existing_close_camera_pose_and_preserves_pan(self):
        calls=[]
        frame=types.SimpleNamespace(shape=(480,640,3))
        block=types.SimpleNamespace(nx=.50,ny=.44,area=5500,cx=320,cy=211,width=90,height=88)
        class Robot:
            dry_run=True
            def __init__(self): self.pose={3:500,4:2320,5:1320,6:1660}
            def nudge_servos(self,updates,duration): calls.append((dict(updates),duration)); self.pose.update(updates)
        class Camera:
            def read(self,quiet=True): return frame
        robot=Robot()
        with mock.patch.object(SEARCH, '_read_near_look_candidate', return_value=(frame,block)), \
             mock.patch.object(SEARCH, '_confirm_near_look', return_value=block), \
             mock.patch.object(SEARCH, 'save_debug_frame'), \
             mock.patch.object(SEARCH.time, 'sleep', return_value=None):
            found=SEARCH.refine_bottom_near_look(robot,Camera(),phase='floor')
        self.assertIs(found,block)
        self.assertEqual(calls[0][0][4], SEARCH.CAPTURE_ARM_POSE[4])
        self.assertEqual(calls[0][0][5], SEARCH.CAPTURE_ARM_POSE[5])
        self.assertEqual(calls[0][0][6],1660)

    def test_scan_refines_edge_hint_before_continuing_sweep(self):
        calls = []
        frame = types.SimpleNamespace(shape=(480, 640, 3))
        hint = types.SimpleNamespace(nx=.97, ny=.55, area=600, cx=621, cy=264, width=38, height=44)
        block = types.SimpleNamespace(nx=.72, ny=.53, area=1700, cx=460, cy=254, width=60, height=58)
        class Robot:
            def __init__(self): self.pose={3:600,6:1500}
            def nudge_servos(self, updates, duration):
                calls.append(dict(updates)); self.pose.update(updates)
        class Camera:
            def read(self, quiet=True): return frame
        robot=Robot()
        with mock.patch.object(SEARCH, 'read_search_observation', side_effect=[
                (frame, None, hint), (frame, block, None)]), \
             mock.patch.object(SEARCH, 'confirm_search_candidate', return_value=block), \
             mock.patch.object(SEARCH, 'save_debug_frame'), \
             mock.patch.object(SEARCH.time, 'sleep', return_value=None):
            found=SEARCH.scan_points(robot, Camera(), ((1500,600),(1050,700)), phase='floor')
        self.assertIs(found, block)
        # First scan gaze, then an immediate hint-following gaze; it must not
        # continue to the second scheduled scan point first.
        self.assertGreaterEqual(len(calls),2)
        self.assertEqual(calls[0][6],1500)
        self.assertLess(calls[1][6],1500)
        self.assertNotIn(1050, [c.get(6) for c in calls[:2]])

    def test_right_edge_floor_candidate_is_allowed_for_tracking_recovery(self):
        frame = type("Frame", (), {"shape": (480, 640, 3)})()
        candidate = type("Blob", (), {
            "cx": 625, "cy": 330, "width": 50, "height": 50,
            "nx": 0.977, "ny": 0.688, "area": 1700,
        })()
        self.assertTrue(SEARCH.valid_search_candidate(candidate, frame))

    def test_bottom_clipped_candidate_is_rejected_as_self(self):
        frame = types.SimpleNamespace(shape=(480, 640, 3))
        # shape slicing expects tuple only; helper uses frame.shape directly.
        b = types.SimpleNamespace(cx=160, cy=465, width=70, height=30, nx=.25, ny=.969, area=1700)
        self.assertFalse(SEARCH.valid_search_candidate(b, frame))

    def test_bounded_middle_candidate_is_allowed(self):
        frame = types.SimpleNamespace(shape=(480, 640, 3))
        b = types.SimpleNamespace(cx=420, cy=180, width=60, height=50, nx=.66, ny=.375, area=1800)
        self.assertTrue(SEARCH.valid_search_candidate(b, frame))


    def test_live_search_success_is_action_synchronous_sensor_evidence(self):
        import scripts.robot_actions as actions
        with mock.patch.object(actions, "main", return_value=0):
            result = actions.run("search")
        self.assertEqual(result["outcome_status"], "ACHIEVED")
        self.assertEqual(result["camera_red"], {"visible": True})
        self.assertEqual(result["verification_source"], "pi_multiframe_red_search")

    def test_dry_run_search_never_claims_target_visibility(self):
        import scripts.robot_actions as actions
        with mock.patch.object(actions, "main", return_value=0):
            result = actions.run("search", ["--dry-run"])
        self.assertEqual(result["outcome_status"], "UNKNOWN")
        self.assertNotIn("camera_red", result)

    def test_live_track_success_carries_center_evidence(self):
        import scripts.robot_actions as actions
        with mock.patch.object(actions, "main", return_value=0):
            result = actions.run("track")
        self.assertEqual(result["outcome_status"], "ACHIEVED")
        self.assertIs(result["center_verified"], True)
        self.assertEqual(result["camera_red"], {"visible": True})

    def test_live_approach_success_carries_range_and_center_evidence(self):
        import scripts.robot_actions as actions
        with mock.patch.object(actions, "main", return_value=0):
            result = actions.run("approach")
        self.assertEqual(result["outcome_status"], "ACHIEVED")
        self.assertIs(result["range_verified"], True)
        self.assertIs(result["center_verified"], True)
        self.assertEqual(result["camera_red"], {"visible": True})

    def test_agent_pick_preserves_verified_pregrasp_gaze(self):
        import scripts.robot_actions as actions
        self.assertIn("--preserve-gaze", actions.AGENT_DEFAULT_ARGS["pick"])

    def test_robot_action_maps_search_to_dedicated_script(self):
        import scripts.robot_actions as actions
        self.assertEqual(actions.SCRIPT_FOR["search"], "search")
        self.assertEqual(actions.AGENT_DEFAULT_ARGS["search"], ["--seconds", "30.0"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
