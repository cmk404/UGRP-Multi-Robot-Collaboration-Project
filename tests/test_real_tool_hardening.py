from __future__ import annotations

import pathlib
import os
import subprocess
import sys
import tempfile
import time
import unittest
from types import SimpleNamespace
from unittest import mock

ROOT = pathlib.Path(__file__).resolve().parents[1]
RED_BLOCK = ROOT / "scripts" / "red_block"
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
if str(RED_BLOCK) not in sys.path:
    sys.path.insert(0, str(RED_BLOCK))

from harness.executive import TaskExecutive
from harness.state import StateEstimator
from scripts import robot_actions
import carry_handoff
import near_look_handoff
import approach
import carry
import deploy
import fetch
import pick
import place
import put_down
import remote_watchdog
import robot
import track


class PublicToolSurfaceTests(unittest.TestCase):
    def test_legacy_composite_grasp_tools_are_not_ai_facing(self):
        self.assertNotIn("fetch", robot_actions.ACTIONS)
        self.assertNotIn("carry", robot_actions.ACTIONS)
        # Public tool count is intentionally not frozen: peer/team tools may be
        # added without exposing the hidden deterministic acquisition executor.
        self.assertNotIn("run_program", robot_actions.ACTIONS)
        self.assertNotIn("program_runner", robot_actions.ACTIONS)
        for name in ("search", "track", "approach", "pick", "place", "put_down"):
            self.assertIn(name, robot_actions.ACTIONS)

    def test_fetch_compatibility_cli_uses_only_precision_path(self):
        calls = []

        class FakeRobot:
            dry_run = True
            def stop(self):
                calls.append("stop")
            def probe(self):
                return {"ok": True}

        args = SimpleNamespace(
            dry_run=True, flip_x=False, flip_y=False, no_hold=True,
            grasp_pulse=None,
        )
        with mock.patch.object(fetch, "Robot", return_value=FakeRobot()), \
             mock.patch.object(fetch.signal, "signal"), \
             mock.patch.object(fetch, "run_approach", side_effect=lambda *a, **k: calls.append("approach")), \
             mock.patch.object(fetch, "run_precision_pick", side_effect=lambda *a, **k: calls.append("precision_pick") or 0):
            self.assertEqual(fetch.run_on_robot(args), 0)
        self.assertIn("approach", calls)
        self.assertIn("precision_pick", calls)
        self.assertLess(calls.index("approach"), calls.index("precision_pick"))
        self.assertFalse(hasattr(fetch, "grasp_and_lift"))

    def test_standalone_carry_pick_is_disabled_before_actuation(self):
        with self.assertRaisesRegex(RuntimeError, "standalone carry pickup is disabled"):
            carry.run_on_robot(SimpleNamespace())

    def test_pick_legacy_fixed_pose_knobs_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "legacy --grasp-pulse is disabled"):
            pick.main(["--grasp-pulse", "1920"])
        with self.assertRaisesRegex(ValueError, "legacy --chassis-align is disabled"):
            pick.main(["--chassis-align"])
        self.assertFalse(hasattr(pick, "grasp_and_lift"))

    def test_direct_track_cli_is_bounded_by_default(self):
        args = track.build_parser().parse_args([])
        self.assertEqual(args.seconds, track.DEFAULT_TRACK_SECONDS)
        self.assertGreater(args.seconds, 0.0)

    def test_place_on_blue_alias_always_supplies_destination_color(self):
        with mock.patch.object(robot_actions, "main", return_value=0) as main:
            out = robot_actions.run("place_on_blue")
        argv = main.call_args.args[0]
        self.assertIn("--target-color", argv)
        self.assertEqual(argv[argv.index("--target-color") + 1], "red")
        self.assertIn("--destination-color", argv)
        self.assertEqual(argv[argv.index("--destination-color") + 1], "blue")
        self.assertTrue(out["ok"])


class CarryHandoffTests(unittest.TestCase):
    def test_carry_handoff_is_short_lived_and_self_invalidates_when_stale(self):
        with tempfile.TemporaryDirectory() as td:
            path = pathlib.Path(td) / "carry.json"
            with mock.patch.object(carry_handoff, "CARRY_PATH", path):
                carry_handoff.save_carry_handoff(
                    gripper_pulse=1500,
                    robot_pose={1: 1500, 3: 700, 4: 2200, 5: 780, 6: 1500},
                )
                payload = carry_handoff.load_carry_handoff()
                self.assertEqual(payload["source"], "precision_pick_floor_clear")
                self.assertEqual(payload["grasp_state"], "PROBABLE_HELD")
                created = float(payload["created_at"])
                with self.assertRaisesRegex(RuntimeError, "stale"):
                    carry_handoff.load_carry_handoff(
                        now=created + carry_handoff.CARRY_MAX_AGE_SECONDS + 1.0
                    )
                self.assertFalse(path.exists())

    def test_carry_handoff_rejects_arm_pose_drift_after_pick(self):
        with tempfile.TemporaryDirectory() as td:
            path = pathlib.Path(td) / "carry.json"
            pose = {1: 1500, 3: 700, 4: 2200, 5: 780, 6: 1500}
            with mock.patch.object(carry_handoff, "CARRY_PATH", path):
                carry_handoff.save_carry_handoff(gripper_pulse=1500, robot_pose=pose)
                changed = dict(pose)
                changed[4] = 2100
                with self.assertRaisesRegex(RuntimeError, "carry handoff pose changed at servo 4"):
                    carry_handoff.require_carry_handoff(
                        current_gripper_pulse=1500, current_robot_pose=changed
                    )
                self.assertFalse(path.exists())

    def test_live_place_rejects_closed_gripper_without_causal_pick_handoff(self):
        live_robot = SimpleNamespace(dry_run=False, pose={1: 1500})
        with mock.patch.object(
            place,
            "require_carry_handoff",
            side_effect=RuntimeError("recent precision-pick carry handoff is missing; refusing placement"),
        ) as require:
            with self.assertRaisesRegex(place.PlaceError, "carry handoff is missing"):
                place._require_closed_gripper(live_robot)
        require.assert_called_once_with(current_gripper_pulse=1500, current_robot_pose={1: 1500})

    def test_dry_run_place_does_not_require_physical_handoff(self):
        dry_robot = SimpleNamespace(dry_run=True, pose={1: 1500})
        with mock.patch.object(place, "require_carry_handoff") as require:
            place._require_closed_gripper(dry_robot)
        require.assert_not_called()

    def test_successful_precision_pick_creates_carry_handoff(self):
        events = []
        class FakeVideo:
            def __init__(self, url): pass
            def settle(self, delay): pass
            def close(self): pass
        class FakeRobot:
            dry_run = False
            def __init__(self): self.pose={1:1500,3:500,4:2320,5:1320,6:1500}
            def stop(self): pass
            def move_pose(self, pose, lowering): self.pose.update(pose)
            def move_servo(self, servo, pulse, duration): self.pose[servo]=pulse
        precision = SimpleNamespace(
            STREAM_URL="stream", CAMERA_DELAY_SECONDS=0.0,
            DELIVERY_CARRY_POSE={1:1500,3:600,4:2200,5:1500},
            GRIPPER_ID=1, GRIPPER_OPEN=2000, LiveVideo=FakeVideo,
            execute_pick_to_hover=lambda r,v,p: "reference",
            verify_grasp=lambda r,v,ref: (True,None),
            grasp_descent_waypoints=lambda p: [
                {3:810,4:2100,5:1800}, {3:820,4:2050,5:1900},
            ],
        )
        fake_plan=SimpleNamespace(base_pulse=1510, hover_pose={3:800,4:2150,5:1700})
        fake_robot=FakeRobot()
        coarse={"target_color":"red", "gaze":{"pan":1500,"tilt":500}}
        with mock.patch.object(pick,"load_coarse_handoff_payload",return_value=coarse),              mock.patch.object(pick,"require_coarse_handoff_color"),              mock.patch.object(pick,"validate_coarse_handoff_pose"),              mock.patch.object(pick,"_prepare_nearfield_pick_plan",return_value=fake_plan),              mock.patch.object(pick,"invalidate_pick_plan"),              mock.patch.object(pick,"invalidate_carry_handoff") as invalidate_carry,              mock.patch.object(pick,"save_carry_handoff") as save_carry:
            self.assertEqual(pick.run_precision_pick(fake_robot, precision=precision), 0)
        invalidate_carry.assert_called_once()
        save_carry.assert_called_once()
        self.assertEqual(save_carry.call_args.kwargs["gripper_pulse"],1500)
        self.assertEqual(save_carry.call_args.kwargs["return_base_pulse"],1510)
        self.assertEqual(save_carry.call_args.kwargs["return_hover_pose"],fake_plan.hover_pose)
        self.assertEqual(len(save_carry.call_args.kwargs["return_descent_poses"]),2)

    def test_chassis_motion_revokes_only_pickup_return_path(self):
        with tempfile.TemporaryDirectory() as td:
            path = pathlib.Path(td) / "carry.json"
            pose = {1: 1500, 3: 700, 4: 2200, 5: 780, 6: 1500}
            with mock.patch.object(carry_handoff, "CARRY_PATH", path):
                carry_handoff.save_carry_handoff(
                    gripper_pulse=1500, robot_pose=pose, target_color="red",
                    return_base_pulse=1500,
                    return_hover_pose={3: 800, 4: 2100, 5: 1700},
                    return_descent_poses=[{3: 820, 4: 2050, 5: 1900}],
                )
                carry_handoff.invalidate_return_path()
                payload = carry_handoff.load_carry_handoff()
                self.assertEqual(payload["target_color"], "red")
                self.assertNotIn("return_path", payload)
                with self.assertRaisesRegex(RuntimeError, "no pickup return path"):
                    carry_handoff.require_return_path(payload)

    def test_put_down_uses_saved_inverse_pick_path_before_opening(self):
        events = []
        class FakeRobot:
            dry_run = False
            def __init__(self):
                self.pose = {1: 1500, 3: 700, 4: 2200, 5: 780, 6: 1500}
            def stop(self): events.append(("stop",))
            def move_servo(self, servo, pulse, duration):
                events.append(("servo", servo, pulse))
                self.pose[servo] = pulse

        fake = FakeRobot()
        handoff = {"target_color": "red"}
        path = {
            "base_pulse": 1510,
            "hover_pose": {3: 800, 4: 2100, 5: 1700},
            "descent_poses": [
                {3: 810, 4: 2080, 5: 1800},
                {3: 820, 4: 2050, 5: 1900},
            ],
        }
        moved = []
        fake_precision = SimpleNamespace(
            move_arm_together=lambda r, pose, duration: moved.append(dict(pose)),
        )
        with mock.patch.object(put_down, "load_carry_handoff", return_value=handoff), \
             mock.patch.object(put_down, "require_carry_handoff"), \
             mock.patch.object(put_down, "require_return_path", return_value=path), \
             mock.patch.object(put_down, "invalidate_carry_handoff") as invalidate, \
             mock.patch.dict(sys.modules, {"physical_state_machine_reference": fake_precision}):
            self.assertEqual(put_down.execute_put_down(fake), "red")
        self.assertEqual(events[1], ("servo", 6, 1510))
        self.assertEqual(moved[:3], [path["hover_pose"], *path["descent_poses"]])
        open_event = next(i for i, e in enumerate(events) if e[:2] == ("servo", 1))
        self.assertGreater(open_event, 1)
        self.assertEqual(events[open_event][2], 2000)
        invalidate.assert_called_once()

    def test_opening_gripper_invalidates_carry_handoff(self):
        instance = object.__new__(robot.Robot)
        instance.pose = {1: 1500}
        instance.dry_run = True
        instance.servos = SimpleNamespace(write_servo=lambda *args: None)
        with mock.patch.object(robot, "_invalidate_carry_handoff") as invalidate, \
             mock.patch.object(robot, "record_pose"), \
             mock.patch.object(robot, "trace_event"):
            instance.move_servo(1, 2000, 0.1)
        invalidate.assert_called_once()


class NearLookHandoffTests(unittest.TestCase):
    def test_near_look_handoff_is_fresh_and_bound_to_fixed_arm_geometry(self):
        with tempfile.TemporaryDirectory() as td:
            path=pathlib.Path(td)/'near.json'
            with mock.patch.object(near_look_handoff,'PATH',path), \
                 mock.patch.object(near_look_handoff.time,'time',return_value=1000.0):
                near_look_handoff.save_near_look_handoff(
                    robot_pose={3:600,4:2200,5:1900,6:1500}
                )
                payload=near_look_handoff.require_near_look_handoff(
                    current_pose={3:700,4:2200,5:1900,6:1600}, now=1010.0
                )
                self.assertEqual(payload['source'],near_look_handoff.SOURCE)
                with self.assertRaisesRegex(RuntimeError,'arm geometry changed'):
                    near_look_handoff.require_near_look_handoff(
                        current_pose={3:700,4:2320,5:1900,6:1600}, now=1010.0
                    )
                self.assertFalse(path.exists())

    def test_near_look_handoff_expires(self):
        with tempfile.TemporaryDirectory() as td:
            path=pathlib.Path(td)/'near.json'
            with mock.patch.object(near_look_handoff,'PATH',path), \
                 mock.patch.object(near_look_handoff.time,'time',return_value=1000.0):
                near_look_handoff.save_near_look_handoff(
                    robot_pose={4:2200,5:1900}
                )
            with mock.patch.object(near_look_handoff,'PATH',path):
                with self.assertRaisesRegex(RuntimeError,'stale'):
                    near_look_handoff.require_near_look_handoff(
                        current_pose={4:2200,5:1900},
                        now=1000.0+near_look_handoff.MAX_AGE_SECONDS+1.0,
                    )
                self.assertFalse(path.exists())


class ExecutiveCarryIsolationTests(unittest.TestCase):
    def test_real_red_pipeline_is_blocked_while_object_may_be_carried(self):
        est = StateEstimator()
        est.state.grasp.state = "PROBABLE_HELD"
        executive = TaskExecutive(strict_pick_preconditions=True)
        for skill in ("search", "track", "approach", "pick"):
            decision = executive.check(skill, est.state)
            self.assertFalse(decision.allowed, skill)
            self.assertEqual(decision.failure_code, "OBJECT_ALREADY_CARRIED")
        self.assertTrue(executive.check("put_down", est.state).allowed)
        self.assertTrue(executive.check("place_on_blue", est.state).allowed)


class RemoteWatchdogTests(unittest.TestCase):
    def test_watchdog_terminates_hung_child(self):
        started = time.monotonic()
        code = remote_watchdog.run_guarded(
            [sys.executable, "-c", "import time; time.sleep(10)"],
            0.10,
        )
        elapsed = time.monotonic() - started
        self.assertEqual(code, 124)
        self.assertLess(elapsed, 2.0)

    def test_watchdog_sigterm_terminates_child_group(self):
        with tempfile.TemporaryDirectory() as td:
            pid_path = pathlib.Path(td) / "child.pid"
            child_code = (
                "import os,time,pathlib; "
                f"pathlib.Path({str(pid_path)!r}).write_text(str(os.getpid())); "
                "time.sleep(30)"
            )
            watchdog_path = RED_BLOCK / "remote_watchdog.py"
            proc = subprocess.Popen([
                sys.executable, str(watchdog_path), "--timeout", "30", "--",
                sys.executable, "-c", child_code,
            ])
            deadline = time.monotonic() + 2.0
            while not pid_path.exists() and time.monotonic() < deadline:
                time.sleep(0.02)
            self.assertTrue(pid_path.exists())
            child_pid = int(pid_path.read_text())
            proc.terminate()
            proc.wait(timeout=5.0)
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                try:
                    os.kill(child_pid, 0)
                except ProcessLookupError:
                    break
                time.sleep(0.02)
            with self.assertRaises(ProcessLookupError):
                os.kill(child_pid, 0)

    def test_deploy_wraps_remote_skill_with_watchdog_and_local_timeout(self):
        with tempfile.TemporaryDirectory() as td:
            last_error = pathlib.Path(td) / "last-error.txt"
            with mock.patch.object(deploy, "LAST_REMOTE_ERROR_PATH", last_error), \
                 mock.patch.object(deploy, "ssh_connection_args", return_value=("ugrp1", ["-o", "BatchMode=yes"])), \
                 mock.patch.object(deploy, "package_signature", return_value="abc"), \
                 mock.patch.object(deploy, "_ensure_deployed", return_value=True), \
                 mock.patch.object(deploy, "remote_package", return_value=pathlib.Path("/remote/red_block")), \
                 mock.patch.object(deploy, "_copy_remote_trace", return_value=False), \
                 mock.patch.object(deploy, "_run_tee", return_value=(0, "", "", False)) as run:
                self.assertEqual(deploy.deploy_and_run("search.py", ["--seconds", "30"]), 0)
            argv = run.call_args.args[0]
            self.assertIn("/remote/red_block/remote_watchdog.py", argv)
            self.assertIn("/remote/red_block/search.py", argv)
            self.assertEqual(
                run.call_args.args[1],
                deploy.REMOTE_SKILL_TIMEOUTS["search.py"] + deploy.SSH_TIMEOUT_MARGIN_SECONDS,
            )

    def test_approach_rejects_post_pick_pose_before_opening_video(self):
        class FakeRobot:
            pose = {1: 2000, 3: 612, 4: 2200, 5: 1900, 6: 1484}
            def pose_state_fresh(self): return True

        precision = SimpleNamespace(
            STREAM_URL="unused",
            LiveVideo=mock.Mock(side_effect=AssertionError("video must not open")),
        )
        with mock.patch.object(approach, "invalidate_pick_plan"):
            with self.assertRaisesRegex(RuntimeError, "requires search/track camera geometry"):
                approach.run_approach(FakeRobot(), precision=precision)

    def test_invalid_post_pick_approach_pose_requires_safe_search_reset(self):
        code, required, recovery = robot_actions.classify_failure(
            "approach",
            "precision approach requires search/track camera geometry; "
            "servo4=2200 expected=2320, servo5=1900 expected=1320. "
            "Run search first to reset the arm before metric ranging.",
        )
        self.assertEqual(code, "APPROACH_POSE_INVALID")
        self.assertEqual(required, "arm.pose=SEARCH_TRACK")
        self.assertEqual(recovery, "search")

    def test_approach_progress_stall_requires_search_reset(self):
        for reason in (
            "capture creep made insufficient visual progress (0.083->0.087)",
            "face-route chassis turn exhausted bounded bearing pulses",
            "face-route straight translation exhausted bounded pulses",
            "face-route passed safe radial envelope by 1.0cm before reaching 2-D endpoint (bearing-error=+12.2deg)",
            "face-route ended 26.8deg from face normal; refusing close approach",
            "face-route improved face error by only 1.3deg",
        ):
            code, required, recovery = robot_actions.classify_failure("approach", reason)
            self.assertEqual(code, "APPROACH_PROGRESS_STALLED")
            self.assertEqual(required, "arm.pose=SEARCH_TRACK")
            self.assertEqual(recovery, "search")

    def test_transport_and_watchdog_failures_are_not_misclassified_as_target_loss(self):
        unavailable = robot_actions.classify_failure(
            "approach", "cannot reach ugrp1: Tailscale has no IPv4"
        )
        timeout = robot_actions.classify_failure(
            "approach", "remote skill safety timeout after 120.0s"
        )
        carry_state = robot_actions.classify_failure(
            "place_on_blue", "recent precision-pick carry handoff is missing; refusing placement"
        )
        actuator = robot_actions.classify_failure(
            "move_forward", "motor 1 speed 0 write timed out after 0.50s"
        )
        camera = robot_actions.classify_failure(
            "approach", "MJPEG reader failed: MJPEG HTTP stream ended"
        )
        pose_stale = robot_actions.classify_failure(
            "approach", "pose telemetry remains stale after exact-pose reassertion"
        )
        self.assertEqual(pose_stale, ("APPROACH_POSE_STALE", "arm.pose_trust=FRESH", None))
        too_close = robot_actions.classify_failure(
            "approach", "could not retreat capture target into the requested hand-eye window"
        )
        self.assertEqual(too_close, ("TARGET_TOO_CLOSE", "target.capture_depth=SAFE", None))

        retreat_lost = robot_actions.classify_failure(
            "approach", "target lost during straight close-range retreat; chassis stopped"
        )
        self.assertEqual(unavailable[0], "ROBOT_UNAVAILABLE")
        self.assertIsNone(unavailable[2])
        self.assertEqual(timeout[0], "SKILL_TIMEOUT")
        self.assertIsNone(timeout[2])
        self.assertEqual(carry_state[0], "CARRY_STATE_UNVERIFIED")
        self.assertIsNone(carry_state[2])
        self.assertEqual(actuator[0], "ACTUATOR_IO_FAILED")
        self.assertIsNone(actuator[2])
        self.assertEqual(camera[0], "CAMERA_UNAVAILABLE")
        self.assertIsNone(camera[2])
        self.assertEqual(retreat_lost[0], "TARGET_NOT_VISIBLE")
        self.assertEqual(retreat_lost[2], "search")


if __name__ == "__main__":
    unittest.main(verbosity=2)
