from __future__ import annotations

import unittest

from harness.visual_macro_runtime import VisualMacroExecutor


class Port:
    def __init__(self, rid):
        self.robot_id = rid
        self.applied = []
        self.ticks = []
        self.stops = []

    def tick(self, now):
        self.ticks.append(now)

    def apply(self, action, now):
        self.applied.append((now, dict(action)))

    def stop(self):
        self.stops.append(True)


class MonotonicPort(Port):
    def __init__(self, rid):
        super().__init__(rid)
        self.clock = -float("inf")

    def tick(self, now):
        if now < self.clock:
            raise ValueError("sim_time must not move backwards")
        self.clock = now
        super().tick(now)

    def apply(self, action, now):
        if now < self.clock:
            raise ValueError("sim_time must not move backwards")
        self.clock = now
        super().apply(action, now)


def observation(**pulses):
    values = {"1": 2000, "3": 1000, "4": 1000, "5": 1000, "6": 1500}
    values.update({str(k): v for k, v in pulses.items()})
    return {"sha256": "frame-hash", "actuator_state": {"servo_pulses": values}}


class VisualMacroExecutorTests(unittest.TestCase):
    def test_continuous_carry_still_expires_without_fresh_submission(self):
        port = Port('r2')
        executor = VisualMacroExecutor(port, drive_settle_by_phase={'carry':0.0})
        action = {'kind':'drive','fwd':.1,'turn':0.,'duration':.25}
        executor.submit(action, observation(), 'carry', 0.)
        executor.tick(.25)
        self.assertTrue(executor.idle)
        self.assertEqual(len(port.stops), 1)
        executor.tick(.5)
        self.assertEqual(len(port.applied), 1)
        executor.submit(action, observation(), 'approach', .5)
        executor.tick(.75)
        self.assertFalse(executor.idle)
        executor.tick(.95)
        self.assertTrue(executor.idle)

    def test_guarded_four_second_drive_uses_fresh_guard_for_each_slice(self):
        port = Port("r1")
        checks = []

        def guard(action, now):
            checks.append((dict(action), now))
            return {"allowed": True, "reason": "clear", "evidence": {"frame": now}}

        executor = VisualMacroExecutor(port, drive_guard=guard)
        action = {"kind": "drive", "fwd": .1, "turn": .02, "duration": 4.0}
        executor.submit(action, observation(), "approach", 0.0)
        for step in range(1, 17):
            executor.tick(step * .25)
        self.assertEqual(len(port.applied), 16)
        self.assertEqual(len(checks), 16)
        self.assertTrue(all(item == action for item, _ in checks))
        self.assertTrue(all(raw["duration_s"] <= .25 for _, raw in port.applied))

    def test_guard_rejection_mid_macro_stops_and_records_evidence(self):
        port, events = Port("r1"), []

        def guard(_action, now):
            if now >= .5:
                return {"allowed": False, "reason": "OBSTACLE", "evidence": {"range": .1}}
            return {"allowed": True, "reason": "clear"}

        executor = VisualMacroExecutor(port, events.append, guard)
        executor.submit({"kind": "drive", "fwd": .1, "turn": 0., "duration": 2.0},
                        observation(), "approach", 0.0)
        executor.tick(.25)
        executor.tick(.5)
        self.assertTrue(executor.idle)
        self.assertEqual(len(port.applied), 2)
        self.assertEqual(len(port.stops), 1)
        self.assertEqual(executor.last_interruption["reason"], "OBSTACLE")
        self.assertEqual(executor.last_interruption["evidence"], {"range": .1})
        self.assertEqual(events[-1]["event"], "macro_interrupted")

    def test_late_tick_skips_expired_drive_slices_and_never_restarts_tail(self):
        port = Port("r1")
        executor = VisualMacroExecutor(
            port, drive_guard=lambda _action, _now: {"allowed": True, "reason": "clear"})
        executor.submit({"kind": "drive", "fwd": .1, "turn": 0., "duration": 1.0},
                        observation(), "approach", 0.0)
        executor.tick(1.0)
        executor.tick(1.2)
        self.assertTrue(executor.idle)
        self.assertEqual(len(port.applied), 1)
        self.assertEqual(len(port.stops), 1)

    def test_off_grid_ticks_use_only_remainder_of_each_active_slice(self):
        port = Port("r1")
        executor = VisualMacroExecutor(
            port, drive_guard=lambda _action, _now: {"allowed": True, "reason": "clear"})
        executor.submit({"kind": "drive", "fwd": .1, "turn": 0., "duration": 1.0},
                        observation(), "approach", 0.0)
        executor.tick(.252)
        executor.tick(.504)
        executor.tick(.756)
        self.assertEqual(len(port.applied), 4)
        self.assertAlmostEqual(port.applied[1][1]["duration_s"], .248)
        self.assertAlmostEqual(port.applied[2][1]["duration_s"], .246)
        self.assertAlmostEqual(port.applied[3][1]["duration_s"], .244)
        self.assertLessEqual(max(now + raw["duration_s"] for now, raw in port.applied), 1.0)

    def test_two_millisecond_float_clock_keeps_guarded_drive_alive_to_four_seconds(self):
        port = MonotonicPort("r1")
        checks = []
        executor = VisualMacroExecutor(
            port, drive_guard=lambda _action, now: checks.append(now) or
            {"allowed": True, "reason": "clear"})
        executor.submit({"kind": "drive", "fwd": .1, "turn": 0., "duration": 4.0},
                        observation(), "approach", 0.0)
        clock = 0.0
        while clock < 4.2:
            clock += .002
            executor.tick(clock)
        self.assertGreater(len(port.applied), 1)
        self.assertEqual(len(checks), len(port.applied))
        self.assertLessEqual(max(now + raw["duration_s"] for now, raw in port.applied),
                             4.0 + 1e-12)
        self.assertTrue(executor.idle)

    def test_cancel_clears_future_pose_events_and_submit_resets_interruption(self):
        port = Port("r1")
        executor = VisualMacroExecutor(port)
        executor.submit({"kind": "pose", "pulses": {3: 2200}}, observation(), "carry", 0.0)
        executor.cancel(.01, "RUNNER_STOP")
        executor.tick(3.0)
        self.assertTrue(executor.idle)
        self.assertFalse(port.applied)
        self.assertEqual(executor.last_interruption["reason"], "RUNNER_STOP")
        executor.submit({"kind": "wait", "duration": .05}, observation(), "carry", 3.0)
        self.assertIsNone(executor.last_interruption)

    def test_guarded_drive_limits_are_strict_and_unguarded_limit_is_preserved(self):
        guarded = VisualMacroExecutor(
            Port("r1"), drive_guard=lambda _a, _n: {"allowed": True, "reason": "clear"})
        for duration in (0.0, 4.0001, -0.1):
            with self.assertRaises(ValueError):
                guarded.submit({"kind": "drive", "fwd": .1, "turn": 0., "duration": duration},
                               observation(), "approach", 0.0)
        unguarded = VisualMacroExecutor(Port("r2"))
        with self.assertRaises(ValueError):
            unguarded.submit({"kind": "drive", "fwd": .1, "turn": 0., "duration": 1.01},
                             observation(), "approach", 0.0)

    def test_slow_arm_does_not_block_other_drive_or_its_next_action(self):
        arm_port, drive_port = Port("r1"), Port("r2")
        arm = VisualMacroExecutor(arm_port)
        drive = VisualMacroExecutor(drive_port)
        arm.submit({"kind": "pose", "pulses": {3: 2200}}, observation(), "carry", 0.0)
        drive.submit({"kind": "drive", "fwd": .1, "turn": 0., "duration": .1},
                     observation(), "approach", 0.0)
        for now in (0.05, 0.1, 0.2, 0.3):
            arm.tick(now)
            drive.tick(now)
        self.assertFalse(arm.idle)
        self.assertTrue(drive.idle)
        drive.submit({"kind": "wait", "duration": .05}, observation(), "approach", .3)
        drive.tick(.35)
        self.assertTrue(drive.idle)
        self.assertTrue(arm_port.applied)
        self.assertEqual(drive_port.applied[0][1]["kind"], "drive")

    def test_pose_uses_observed_start_cubic_samples_and_phase_settle(self):
        port = Port("r1")
        events = []
        executor = VisualMacroExecutor(port, events.append)
        executor.submit({"kind": "pose", "pulses": {3: 1600, 6: 1700}},
                        observation(**{"3": 1000, "6": 1500}), "approach", 2.0)
        # max delta 600 => 1.0 s, sampled every .05 s, then .30 s settle.
        self.assertFalse(port.applied)
        executor.tick(2.05)
        first_arm = next(action for _, action in port.applied if action["kind"] == "arm")
        self.assertEqual(first_arm["pulse"], 1004)
        executor.tick(3.0)
        self.assertEqual(port.applied[-2][1], {"kind": "arm", "servo_id": 3, "pulse": 1600})
        self.assertEqual(port.applied[-1][1], {"kind": "look", "pan_pulse": 1700})
        self.assertFalse(executor.idle)
        executor.tick(3.3)
        self.assertTrue(executor.idle)
        self.assertTrue(all(event["source_frame_sha256"] == "frame-hash" for event in events))

    def test_drive_lease_expires_then_executor_emits_explicit_stop(self):
        port = Port("r1")
        events = []
        executor = VisualMacroExecutor(port, events.append)
        executor.submit({"kind": "drive", "fwd": .1, "turn": .02, "duration": .2},
                        observation(), "approach", 1.0)
        self.assertEqual(port.applied[0][0], 1.0)
        executor.tick(1.2)  # CameraRobotPort owns lease expiry at this tick.
        self.assertFalse(executor.idle)
        executor.tick(1.4)
        self.assertTrue(executor.idle)
        self.assertEqual(len(port.stops), 1)
        self.assertIn("explicit_stop", [event["event"] for event in events])
        self.assertEqual(port.ticks, [1.2, 1.4])

    def test_finish_stops_immediately_and_invalid_macros_do_nothing(self):
        port = Port("r1")
        executor = VisualMacroExecutor(port)
        executor.submit({"kind": "finish", "reason": "DONE"}, observation(), "done", 0.0)
        self.assertTrue(executor.idle)
        self.assertEqual(len(port.stops), 1)
        invalid = (
            {"kind": "unknown"},
            {"kind": "drive", "fwd": .2, "turn": 0., "duration": .1},
            {"kind": "pose", "pulses": {2: 1500}},
            {"kind": "wait", "duration": -1},
        )
        for action in invalid:
            with self.assertRaises(ValueError):
                executor.submit(action, observation(), "approach", 1.0)
        self.assertEqual(port.applied, [])
        self.assertTrue(executor.idle)

    def test_late_shared_tick_never_backdates_non_grid_pose_sample(self):
        port = MonotonicPort("r1")
        events = []
        executor = VisualMacroExecutor(port, events.append)
        # 200 pulse delta gives .333... s duration and sample times that do not
        # land on the runner's 2 ms physics clock.
        executor.submit({"kind": "pose", "pulses": {3: 1200}}, observation(),
                        "carry", 0.0)
        for step in range(1, 210):
            executor.tick(step * .002)
        self.assertTrue(executor.idle)
        self.assertTrue(port.applied)
        self.assertTrue(all(abs(time / .002 - round(time / .002)) < 1e-9
                            for time, _ in port.applied))
        raw_events = [event for event in events if event["event"] == "raw_action"]
        self.assertTrue(all(event["time"] >= event["scheduled_time"] for event in raw_events))
        self.assertTrue(any(event["time"] > event["scheduled_time"] for event in raw_events))


if __name__ == "__main__":
    unittest.main()
