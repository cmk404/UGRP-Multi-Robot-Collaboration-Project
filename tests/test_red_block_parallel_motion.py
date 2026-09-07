#!/usr/bin/env python3
import importlib.util
import pathlib
import sys
import unittest
from unittest import mock

ROOT = pathlib.Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "scripts" / "red_block"
if str(PACKAGE) not in sys.path:
    sys.path.insert(0, str(PACKAGE))


def load(name: str):
    path = PACKAGE / f"{name}.py"
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


robot_mod = load("robot")
search_mod = load("search")


class FakeServos:
    def __init__(self, calls): self.calls = calls
    def write_servo(self, servo, pulse, duration):
        self.calls.append(("servo", servo, pulse, duration))


class FakeMotors:
    def __init__(self, calls): self.calls = calls
    def write_motor(self, motor, speed):
        self.calls.append(("motor", motor, speed))


class ParallelServoChassisTests(unittest.TestCase):
    def make_robot(self, *, dry_run=True):
        calls = []
        r = robot_mod.Robot.__new__(robot_mod.Robot)
        r.dry_run = dry_run
        r.pose = {3: 700, 5: 900, 6: 1600}
        r.servos = FakeServos(calls)
        r.motors = FakeMotors(calls)
        control = mock.Mock()
        control.drive_speeds.side_effect = lambda direction, speed: [
            (1, -speed), (2, speed), (3, speed), (4, -speed)
        ]
        r.control = control
        return r, calls, control

    def test_dry_run_issues_servo_targets_before_wheels_and_stops_wheels(self):
        r, calls, control = self.make_robot(dry_run=True)
        with mock.patch.object(robot_mod, "record_pose") as record, \
             mock.patch.object(robot_mod, "_invalidate_precision_handoff"), \
             mock.patch.object(robot_mod, "_invalidate_near_look_handoff"), \
             mock.patch.object(robot_mod, "_invalidate_carry_return_path"):
            r.move_servos_and_drive(
                {3: 800, 6: 1500}, servo_duration=.30,
                direction="rotate-left", speed=35, drive_duration=.18,
            )
        self.assertEqual(calls[0][:3], ("servo", 3, 800))
        self.assertEqual(calls[1][:3], ("servo", 6, 1500))
        first_motor = next(i for i, call in enumerate(calls) if call[0] == "motor")
        self.assertGreater(first_motor, 1)
        self.assertEqual(calls[-4:], [
            ("motor", 1, 0), ("motor", 2, 0),
            ("motor", 3, 0), ("motor", 4, 0),
        ])
        self.assertEqual(r.pose[3], 800)
        self.assertEqual(r.pose[6], 1500)
        record.assert_called_once_with(r.pose, reason="parallel_servo_chassis")
        control.drive_speeds.assert_called_once_with("rotate-left", 35)

    def test_live_path_stops_motors_even_if_motor_start_fails(self):
        r, calls, control = self.make_robot(dry_run=False)
        stop_contexts = []
        control.stop_all.side_effect = lambda motors, context: stop_contexts.append(context)
        count = {"n": 0}
        def fail_second_motor(motor, speed):
            calls.append(("motor", motor, speed))
            count["n"] += 1
            if count["n"] == 2:
                raise RuntimeError("i2c write failed")
        r.motors.write_motor = fail_second_motor
        with mock.patch.object(robot_mod, "_invalidate_precision_handoff"), \
             mock.patch.object(robot_mod, "_invalidate_near_look_handoff"), \
             mock.patch.object(robot_mod, "_invalidate_carry_return_path"), \
             self.assertRaises(RuntimeError):
            r.move_servos_and_drive(
                {3: 800}, servo_duration=.30,
                direction="rotate-left", speed=35, drive_duration=.18,
            )
        self.assertEqual(stop_contexts, ["parallel-pre-motion", "parallel-motion-finally"])

    def test_rejects_ineffective_wheel_speed(self):
        r, _calls, _control = self.make_robot()
        with self.assertRaises(ValueError):
            r.move_servos_and_drive(
                {3: 800}, servo_duration=.3,
                direction="rotate-left", speed=30, drive_duration=.18,
            )


class SearchParallelPolicyTests(unittest.TestCase):
    def test_source_uses_parallel_group_only_for_empty_sector_recenter(self):
        source = (PACKAGE / "search.py").read_text()
        needle = "robot.move_servos_and_drive("
        self.assertEqual(source.count(needle), 1)
        call_at = source.index(needle)
        empty_branch = source.index("full head sweep empty -> rotate body sector")
        self.assertGreater(call_at, empty_branch)
        # The peripheral-target body turn deliberately remains ordinary drive:
        # gaze must stay fixed there to preserve the acquired target bearing.
        self.assertIn('robot.drive(direction, BODY_TURN_SPEED, duration)', source)


if __name__ == "__main__":
    unittest.main(verbosity=2)
