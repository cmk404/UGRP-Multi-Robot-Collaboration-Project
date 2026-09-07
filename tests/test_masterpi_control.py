#!/usr/bin/env python3
import contextlib
import importlib.util
import io
import pathlib
import subprocess
import unittest
from unittest import mock


ROOT = pathlib.Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "masterpi_control.py"
SPEC = importlib.util.spec_from_file_location("masterpi_control", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class FakeTransport:
    def __init__(self, failures=0):
        self.calls = []
        self.failures = failures

    def write_motor(self, motor, speed):
        self.calls.append((motor, speed))
        if self.failures:
            self.failures -= 1
            raise RuntimeError("injected stop failure")


class MasterPiControlTests(unittest.TestCase):
    def test_legacy_sign_mapping_and_i2c_frame(self):
        self.assertEqual(MODULE.board_signed_speed(1, 25), -25)
        self.assertEqual(MODULE.board_signed_speed(2, 25), 25)
        self.assertEqual(MODULE.board_signed_speed(3, -25), 25)
        self.assertEqual(MODULE.board_signed_speed(4, -25), -25)
        self.assertEqual(
            MODULE.i2c_argv(1, 25),
            ["/usr/sbin/i2ctransfer", "-y", "-a", "1", "w2@0x7a", "0x1f", "0xe7"],
        )

    def test_drive_mapping(self):
        self.assertEqual(MODULE.drive_speeds("forward", 35), [(1, -35), (2, 35), (3, 35), (4, -35)])
        self.assertEqual(MODULE.drive_speeds("backward", 35), [(1, 35), (2, -35), (3, -35), (4, 35)])
        self.assertEqual(MODULE.drive_speeds("left", 35), [(1, -35), (2, -35), (3, -35), (4, -35)])
        self.assertEqual(MODULE.drive_speeds("right", 35), [(1, 35), (2, 35), (3, 35), (4, 35)])
        self.assertEqual(MODULE.drive_speeds("rotate-right", 35), [(1, 35), (2, -35), (3, 35), (4, -35)])
        with self.assertRaises(ValueError):
            MODULE.drive_speeds("forward", 30)

    def test_servo_i2c_frame_uses_little_endian_duration_and_pulse(self):
        self.assertEqual(
            MODULE.servo_i2c_argv(6, 1500, 1.5),
            [
                "/usr/sbin/i2ctransfer",
                "-y",
                "-a",
                "1",
                "w7@0x7a",
                "0x28",
                "0x01",
                "0xdc",
                "0x05",
                "0x06",
                "0xdc",
                "0x05",
            ],
        )

    def test_servo_validation_excludes_servo_two_and_checks_ranges(self):
        for servo in (2, 0, 7):
            with self.assertRaises(ValueError):
                MODULE.servo_i2c_argv(servo, 1500, 1.0)
        for pulse in (499, 2501):
            with self.assertRaises(ValueError):
                MODULE.servo_i2c_argv(1, pulse, 1.0)
        for duration in (0.09, 3.01):
            with self.assertRaises(ValueError):
                MODULE.servo_i2c_argv(1, 1500, duration)

    def test_servo_dry_run_does_not_call_subprocess(self):
        with mock.patch.object(MODULE.subprocess, "run") as run:
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                MODULE.ServoTransport(dry_run=True).write_servo(3, 695, 0.1)
        run.assert_not_called()
        self.assertIn("w7@0x7a", output.getvalue())

    def test_servo_main_does_not_stop_motors(self):
        with mock.patch.object(MODULE.subprocess, "run") as run, mock.patch.object(
            MODULE, "stop_all"
        ) as stop_all:
            self.assertEqual(MODULE.main(["servo", "1", "1500", "--duration", "1.0", "--dry-run"]), 0)
        run.assert_not_called()
        stop_all.assert_not_called()

    def test_strict_parser_rejects_invalid_ranges(self):
        parser = MODULE.build_parser()
        with self.assertRaises(SystemExit):
            parser.parse_args(["motor", "1", "101"])
        with self.assertRaises(SystemExit):
            parser.parse_args(["drive", "forward", "--speed", "34"])
        with self.assertRaises(SystemExit):
            parser.parse_args(["drive", "forward", "--speed", "35", "--duration", "2.01"])

    def test_dry_run_does_not_call_subprocess(self):
        transport = MODULE.MotorTransport(dry_run=True)
        output = io.StringIO()
        with self.assertRaises(ValueError):
            transport.write_motor(1, 34)
        with contextlib.redirect_stdout(output):
            transport.write_motor(1, 35)
        self.assertIn("DRY-RUN", output.getvalue())
        self.assertIn("-a 1", output.getvalue())


    def test_every_nonzero_motor_command_below_35_is_rejected(self):
        transport = MODULE.MotorTransport(dry_run=True)
        for speed in list(range(1, 35)) + list(range(-34, 0)):
            with self.assertRaises(ValueError, msg=f"speed={speed}"):
                transport.write_motor(1, speed)
        self.assertEqual(MODULE.MIN_EFFECTIVE_WHEEL_SPEED, 35)

    def test_zero_motor_speed_remains_valid_for_stop(self):
        transport = MODULE.MotorTransport(dry_run=True)
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            transport.write_motor(1, 0)
        self.assertIn("DRY-RUN", output.getvalue())

    def test_raw_motor_speed_is_capped_at_chassis_policy(self):
        transport = MODULE.MotorTransport(dry_run=True)
        with self.assertRaises(ValueError):
            transport.write_motor(1, MODULE.MAX_MOTOR_SPEED + 1)
        with self.assertRaises(ValueError):
            transport.write_motor(1, -100)
        with self.assertRaises(SystemExit):
            MODULE.build_parser().parse_args(["motor", "1", "100", "--duration", "0.5"])

    def test_lateral_motion_has_stronger_separate_speed_envelope(self):
        self.assertEqual(MODULE.MAX_CHASSIS_SPEED, 40)
        self.assertEqual(MODULE.MAX_LATERAL_SPEED, 70)
        self.assertEqual(MODULE.PREFERRED_LATERAL_SPEED, 65)
        self.assertEqual(MODULE.drive_speeds("right", 70), [(1, 70), (2, 70), (3, 70), (4, 70)])
        self.assertEqual(MODULE.drive_speeds("left", 70), [(1, -70), (2, -70), (3, -70), (4, -70)])
        with self.assertRaises(ValueError):
            MODULE.drive_speeds("forward", 41)
        with self.assertRaises(ValueError):
            MODULE.drive_speeds("rotate-left", 41)

    def test_motion_segment_duration_is_bounded(self):
        transport = FakeTransport()
        with self.assertRaises(ValueError):
            MODULE._run_motion(transport, [(1, 35)], MODULE.MAX_CHASSIS_SEGMENT_S + 0.01)
        with self.assertRaises(ValueError):
            MODULE._run_motion(transport, [(1, 35)], 0.0)
        # No motor write may happen before the duration is validated.
        self.assertEqual(transport.calls, [])

    def test_stop_retries_four_zero_writes(self):
        transport = FakeTransport(failures=1)
        MODULE.stop_all(transport, context="test")
        self.assertEqual(transport.calls[0], (1, 0))
        self.assertEqual(transport.calls[-4:], [(1, 0), (2, 0), (3, 0), (4, 0)])
        self.assertEqual(len(transport.calls), 5)

    def test_probe_i2c_argv(self):
        select, read = MODULE.probe_i2c_argv()
        self.assertEqual(select, ["/usr/sbin/i2ctransfer", "-y", "-a", "1", "w1@0x7a", "0x00"])
        self.assertEqual(read, ["/usr/sbin/i2ctransfer", "-y", "-a", "1", "r2@0x7a"])

    def test_parse_probe_output_little_endian_and_whitespace(self):
        self.assertEqual(MODULE.parse_probe_output("0x76 0x1c"), 7286)
        self.assertEqual(MODULE.parse_probe_output("0xe6 0x1c\n"), 7398)
        self.assertEqual(MODULE.parse_probe_output("0x00 0x20"), 8192)
        self.assertEqual(MODULE.parse_probe_output("  0X76 \t 0X1C \n"), 7286)

    def test_parse_probe_output_invalid_token_count_or_format(self):
        for invalid in ("", "0x76", "0x76 0x1c 0x00", "error read timeout", "0x100 0x1c"):
            with self.subTest(invalid=invalid):
                with self.assertRaises(MODULE.ControlError):
                    MODULE.parse_probe_output(invalid)

    def test_parse_probe_output_rejects_out_of_range(self):
        for invalid_raw in ("0x00 0x00", "0xff 0xff", "0x40 0x11", "0x50 0x25"):
            with self.subTest(invalid_raw=invalid_raw):
                with self.assertRaises(MODULE.ControlError):
                    MODULE.parse_probe_output(invalid_raw)

    def test_probe_hardware_success(self):
        select, read = MODULE.probe_i2c_argv()
        completed = subprocess.CompletedProcess(
            args=read,
            returncode=0,
            stdout="0x76 0x1c\n",
            stderr="",
        )
        with mock.patch.object(MODULE.subprocess, "run", return_value=completed) as mock_run:
            res = MODULE.probe_hardware()
        self.assertEqual(
            res,
            {"ok": True, "controller": "legacy-i2c-0x7a", "battery_mv": 7286},
        )
        self.assertEqual(mock_run.call_count, 2)
        mock_run.assert_any_call(
            select,
            check=True,
            shell=False,
            capture_output=True,
            text=True,
            timeout=MODULE.PROBE_TIMEOUT,
        )
        mock_run.assert_any_call(
            read,
            check=True,
            shell=False,
            capture_output=True,
            text=True,
            timeout=MODULE.PROBE_TIMEOUT,
        )

    def test_probe_hardware_dry_run_raises_control_error(self):
        with self.assertRaises(MODULE.ControlError):
            MODULE.probe_hardware(dry_run=True)

    def test_probe_hardware_timeout_and_error(self):
        with mock.patch.object(
            MODULE.subprocess,
            "run",
            side_effect=subprocess.TimeoutExpired(cmd=MODULE.probe_i2c_argv(), timeout=2.0),
        ):
            with self.assertRaises(MODULE.ControlError):
                MODULE.probe_hardware()

        with mock.patch.object(
            MODULE.subprocess,
            "run",
            side_effect=subprocess.CalledProcessError(returncode=1, cmd=MODULE.probe_i2c_argv(), stderr="Error"),
        ):
            with self.assertRaises(MODULE.ControlError):
                MODULE.probe_hardware()

        with mock.patch.object(
            MODULE.subprocess,
            "run",
            side_effect=FileNotFoundError("No such file"),
        ):
            with self.assertRaises(MODULE.ControlError):
                MODULE.probe_hardware()

    def test_probe_main_success(self):
        completed = subprocess.CompletedProcess(
            args=MODULE.probe_i2c_argv(),
            returncode=0,
            stdout="0x76 0x1c\n",
            stderr="",
        )
        with mock.patch.object(MODULE.subprocess, "run", return_value=completed) as run, mock.patch.object(
            MODULE, "stop_all"
        ) as stop_all:
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                exit_code = MODULE.main(["probe"])
        self.assertEqual(exit_code, 0)
        self.assertEqual(output.getvalue().strip(), '{"ok":true,"controller":"legacy-i2c-0x7a","battery_mv":7286}')
        self.assertEqual(run.call_count, 2)
        stop_all.assert_not_called()

    def test_probe_main_dry_run_rejected(self):
        with self.assertRaises(MODULE.ControlError):
            MODULE.main(["probe", "--dry-run"])
        with self.assertRaises(MODULE.ControlError):
            MODULE.main(["--dry-run", "probe"])

    def test_actuator_writes_are_individually_time_bounded(self):
        completed = subprocess.CompletedProcess(args=[], returncode=0)
        with mock.patch.object(MODULE.subprocess, "run", return_value=completed) as run:
            MODULE.MotorTransport(dry_run=False).write_motor(1, 35)
        run.assert_called_once_with(
            MODULE.i2c_argv(1, 35),
            check=True,
            shell=False,
            timeout=MODULE.I2C_WRITE_TIMEOUT,
        )

        with mock.patch.object(MODULE.subprocess, "run", return_value=completed) as run, mock.patch.object(
            MODULE, "record_servo_command"
        ):
            MODULE.ServoTransport(dry_run=False).write_servo(3, 1500, 0.5)
        run.assert_called_once_with(
            MODULE.servo_i2c_argv(3, 1500, 0.5),
            check=True,
            shell=False,
            timeout=MODULE.I2C_WRITE_TIMEOUT,
        )

    def test_actuator_write_timeout_becomes_control_error(self):
        with mock.patch.object(
            MODULE.subprocess,
            "run",
            side_effect=subprocess.TimeoutExpired(cmd=["i2ctransfer"], timeout=MODULE.I2C_WRITE_TIMEOUT),
        ):
            with self.assertRaisesRegex(MODULE.ControlError, "write timed out"):
                MODULE.MotorTransport(dry_run=False).write_motor(1, 35)

    def test_no_shell_execution(self):
        source = SCRIPT.read_text(encoding="utf-8")
        self.assertNotIn("shell=True", source)
        self.assertIn("timeout=I2C_WRITE_TIMEOUT", source)


if __name__ == "__main__":
    unittest.main(verbosity=2)
