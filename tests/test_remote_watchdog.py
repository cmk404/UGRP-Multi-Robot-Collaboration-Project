import importlib.util
import pathlib
import subprocess
import sys
import unittest
from unittest import mock

ROOT = pathlib.Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "red_block" / "remote_watchdog.py"
SPEC = importlib.util.spec_from_file_location("remote_watchdog", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class RemoteWatchdogTests(unittest.TestCase):
    def test_timeout_kills_group_and_then_forces_motor_stop(self):
        with mock.patch.object(MODULE, "_force_stop_actuators") as force_stop:
            code = MODULE.run_guarded([sys.executable, "-c", "import time; time.sleep(30)"], 0.2)
        self.assertEqual(code, 124)
        force_stop.assert_called_once()
        self.assertIn("safety timeout", force_stop.call_args.args[0])

    def test_force_stop_runs_control_script_stop_next_to_watchdog(self):
        with mock.patch.object(MODULE.os.path, "exists", return_value=True), \
             mock.patch.object(MODULE.subprocess, "run") as run:
            run.return_value = mock.Mock(returncode=0, stderr="")
            MODULE._force_stop_actuators("unit")
        argv = run.call_args.args[0]
        self.assertEqual(argv[1], MODULE.CONTROL_SCRIPT)
        self.assertEqual(argv[2], "stop")
        self.assertEqual(pathlib.Path(MODULE.CONTROL_SCRIPT).parent, SCRIPT.parent)

    def test_clean_exit_does_not_force_stop(self):
        with mock.patch.object(MODULE, "_force_stop_actuators") as force_stop:
            code = MODULE.run_guarded([sys.executable, "-c", "pass"], 10.0)
        self.assertEqual(code, 0)
        force_stop.assert_not_called()


if __name__ == "__main__":
    unittest.main()
