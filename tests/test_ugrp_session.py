import json
import fcntl
import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
SESSION = ROOT / "scripts" / "ugrp_session.py"
SESSION_SPEC = importlib.util.spec_from_file_location("ugrp_session_for_test", SESSION)
SESSION_MODULE = importlib.util.module_from_spec(SESSION_SPEC)
assert SESSION_SPEC.loader is not None
SESSION_SPEC.loader.exec_module(SESSION_MODULE)


class UgrpSessionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.env = {**os.environ, "UGRP_SESSION_DIR": self.temp.name}

    def tearDown(self):
        self.temp.cleanup()

    def wait_for(self, predicate, timeout=3.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.02)
        self.fail("condition did not become true")

    def test_wrapper_signal_cleans_up_child_process_group(self):
        child_pid_file = Path(self.temp.name) / "grandchild.pid"
        code = (
            "import pathlib,subprocess,sys,time; "
            "p=subprocess.Popen([sys.executable,'-c','import time; time.sleep(60)']); "
            f"pathlib.Path({str(child_pid_file)!r}).write_text(str(p.pid)); time.sleep(60)"
        )
        wrapper = subprocess.Popen(
            [sys.executable, str(SESSION), "run", "test", "--", sys.executable, "-c", code],
            env=self.env,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        record_file = Path(self.temp.name) / "test.json"
        self.wait_for(lambda: record_file.exists() and child_pid_file.exists())
        record = json.loads(record_file.read_text())
        grandchild_pid = int(child_pid_file.read_text())
        wrapper.send_signal(signal.SIGTERM)
        self.assertEqual(wrapper.wait(timeout=4), 128 + signal.SIGTERM)
        wrapper.communicate()
        self.wait_for(lambda: not record_file.exists())
        with self.assertRaises(ProcessLookupError):
            os.killpg(int(record["pgid"]), 0)
        with self.assertRaises(ProcessLookupError):
            os.kill(grandchild_pid, 0)

    def test_wrapper_allows_child_to_write_result_after_sigint(self):
        ready = Path(self.temp.name) / "ready"
        result = Path(self.temp.name) / "result.json"
        term = Path(self.temp.name) / "unexpected-term"
        code = (
            "import pathlib,signal,sys,time; "
            f"ready=pathlib.Path({str(ready)!r}); "
            f"result=pathlib.Path({str(result)!r}); "
            f"term=pathlib.Path({str(term)!r}); "
            "signal.signal(signal.SIGINT, lambda *_: (time.sleep(.3), result.write_text('done'), sys.exit(130))); "
            "signal.signal(signal.SIGTERM, lambda *_: (term.write_text('term'), sys.exit(143))); "
            "ready.touch(); time.sleep(60)"
        )
        wrapper = subprocess.Popen(
            [sys.executable, str(SESSION), "run", "graceful", "--", sys.executable, "-c", code],
            env=self.env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
        )
        record_file = Path(self.temp.name) / "graceful.json"
        self.wait_for(lambda: ready.exists() and record_file.exists())
        wrapper.send_signal(signal.SIGINT)
        out, err = wrapper.communicate(timeout=4)
        self.assertEqual(wrapper.returncode, 130, (out, err))
        self.assertEqual(result.read_text(), "done")
        self.assertFalse(term.exists(), err)
        self.assertFalse(record_file.exists())

    def test_stop_command_terminates_session(self):
        wrapper = subprocess.Popen(
            [sys.executable, str(SESSION), "run", "manual", "--", sys.executable, "-c", "import time; time.sleep(60)"],
            env=self.env,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        record_file = Path(self.temp.name) / "manual.json"
        self.wait_for(record_file.exists)
        stopped = subprocess.run(
            [sys.executable, str(SESSION), "stop", "manual", "--grace", "1"],
            env=self.env,
            capture_output=True,
            text=True,
            timeout=4,
        )
        self.assertEqual(stopped.returncode, 0, stopped.stderr)
        self.assertEqual(wrapper.wait(timeout=4), 128 + signal.SIGTERM)
        wrapper.communicate()
        self.assertFalse(record_file.exists())

    def test_term_ignoring_child_is_killed_after_grace(self):
        ready_file = Path(self.temp.name) / "stubborn.ready"
        child_code = (
            "import pathlib,signal,time; "
            "signal.signal(signal.SIGTERM, lambda *_: None); "
            f"pathlib.Path({str(ready_file)!r}).write_text('ready'); time.sleep(60)"
        )
        wrapper = subprocess.Popen(
            [sys.executable, str(SESSION), "run", "--grace", "0.1", "stubborn", "--", sys.executable, "-c", child_code],
            env=self.env,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        record_file = Path(self.temp.name) / "stubborn.json"
        self.wait_for(lambda: record_file.exists() and ready_file.exists())
        wrapper.send_signal(signal.SIGTERM)
        self.assertEqual(wrapper.wait(timeout=3), 128 + signal.SIGKILL)
        wrapper.communicate()
        self.assertFalse(record_file.exists())

    def test_stop_refuses_record_without_matching_start_identity(self):
        record_file = Path(self.temp.name) / "stale.json"
        record_file.write_text(json.dumps({
            "leader_pid": os.getpid(),
            "pgid": os.getpid(),
            "leader_start": "not the current process",
        }))
        stopped = subprocess.run(
            [sys.executable, str(SESSION), "stop", "stale"],
            env=self.env,
            capture_output=True,
            text=True,
        )
        self.assertEqual(stopped.returncode, 2)
        self.assertIn("refusing an unverified kill", stopped.stderr)

    def test_immediate_success_does_not_race_session_setup(self):
        completed = subprocess.run(
            [sys.executable, str(SESSION), "run", "quick", "--", sys.executable, "-c", "pass"],
            env=self.env,
            capture_output=True,
            text=True,
            timeout=3,
        )
        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertFalse((Path(self.temp.name) / "quick.json").exists())

    def test_short_leader_cleanup_terminates_spawned_descendant(self):
        descendant_pid_file = Path(self.temp.name) / "descendant.pid"
        code = (
            "import pathlib,subprocess,sys; "
            "p=subprocess.Popen([sys.executable,'-c','import time; time.sleep(60)']); "
            f"pathlib.Path({str(descendant_pid_file)!r}).write_text(str(p.pid))"
        )
        with mock.patch.dict(os.environ, self.env, clear=True), mock.patch.object(
            SESSION_MODULE, "process_start_identity", return_value=None
        ):
            self.assertEqual(
                SESSION_MODULE.run_session(
                    "short",
                    [sys.executable, "-c", code],
                    grace=0.2,
                ),
                0,
            )
        descendant_pid = int(descendant_pid_file.read_text())
        with self.assertRaises(ProcessLookupError):
            os.kill(descendant_pid, 0)
        self.assertFalse((Path(self.temp.name) / "short.json").exists())

    def test_same_name_lock_rejects_start_before_spawning_child(self):
        marker = Path(self.temp.name) / "spawned"
        lock_path = Path(self.temp.name) / "same.lock"
        lock_path.touch()
        with lock_path.open("a+") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            rejected = subprocess.run(
                [
                    sys.executable,
                    str(SESSION),
                    "run",
                    "same",
                    "--",
                    sys.executable,
                    "-c",
                    f"from pathlib import Path; Path({str(marker)!r}).touch()",
                ],
                env=self.env,
                capture_output=True,
                text=True,
                timeout=3,
            )
        self.assertEqual(rejected.returncode, 2)
        self.assertIn("already starting or running", rejected.stderr)
        self.assertFalse(marker.exists())
        self.assertFalse((Path(self.temp.name) / "same.json").exists())

    def test_stale_stop_cannot_remove_record_while_replacement_lock_is_held(self):
        record_path = Path(self.temp.name) / "replacement.json"
        record = {
            "leader_pid": os.getpid(),
            "pgid": os.getpid(),
            "leader_start": "stale identity",
            "owner": "replacement",
        }
        record_path.write_text(json.dumps(record) + "\n")
        lock_path = Path(self.temp.name) / "replacement.lock"
        lock_path.touch()
        with lock_path.open("a+") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            stopped = subprocess.run(
                [sys.executable, str(SESSION), "stop", "replacement"],
                env=self.env,
                capture_output=True,
                text=True,
                timeout=3,
            )
        self.assertEqual(stopped.returncode, 2)
        self.assertIn("refusing an unverified kill", stopped.stderr)
        self.assertEqual(json.loads(record_path.read_text()), record)


if __name__ == "__main__":
    unittest.main()
