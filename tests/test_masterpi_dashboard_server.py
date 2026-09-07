import json
import os
import shutil
import tempfile
import threading
import unittest
from http.client import HTTPConnection
from pathlib import Path
from unittest.mock import patch

from dashboard.server import (
    CommandResult,
    CommandRunner,
    ControlTransport,
    DashboardController,
    LocalTransport,
    MockTransport,
    SSHTransport,
    build_parser,
    create_transport,
    make_server,
    servo_argv,
    status_argv,
    stop_argv,
    validate_drive_payload,
    validate_servo_payload,
)
from dashboard.transport import AddressResolver


class FakeProcess:
    def __init__(self, running=True):
        self.running = running
        self.terminated = False
        self.killed = False
        self.wait_calls = []

    def poll(self):
        return None if self.running else 0

    def terminate(self):
        self.terminated = True
        self.running = False

    def kill(self):
        self.killed = True
        self.running = False

    def wait(self, timeout=None):
        self.wait_calls.append(timeout)
        self.running = False
        return 0


class FakeRunner:
    def __init__(self, status=None, stop=CommandResult(0), auth=CommandResult(0)):
        self.status = status or CommandResult(0, '{"ok": true, "controller": "MasterPi-v1", "battery_mv": 8100}')
        self.stop = stop
        self.auth = auth
        self.calls = []
        self.processes = []

    def run(self, argv, *, timeout):
        self.calls.append(("run", list(argv), timeout))
        if argv[-1] == "true":
            return self.auth
        if argv[-1] == "probe":
            return self.status
        return self.stop

    def start(self, argv):
        process = FakeProcess()
        self.calls.append(("start", list(argv)))
        self.processes.append(process)
        return process


class DashboardHTTPTests(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.mkdtemp()
        self.dist = Path(self.temp_dir) / "frontend"
        self.dist.mkdir()
        (self.dist / "index.html").write_text("<main>MasterPi</main>", encoding="utf-8")
        (self.dist / "app.js").write_text("console.log('ok')", encoding="utf-8")
        self.runner = FakeRunner()
        resolver = AddressResolver(
            mdns_resolver=lambda _host: None,
            port_prober=lambda _ip, _port, _timeout: False,
            local_ip_getter=lambda: {"127.0.0.1"},
        )
        self.controller = DashboardController(
            runner=self.runner,
            register_shutdown=False,
            transport=SSHTransport(resolver=resolver),
        )
        self.server = make_server("127.0.0.1", 0, controller=self.controller, dist_dir=self.dist)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.host, self.port = self.server.server_address

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)
        shutil.rmtree(self.temp_dir)

    def request(self, method, path, body=None):
        conn = HTTPConnection(self.host, self.port, timeout=2)
        headers = {}
        encoded = None
        if body is not None:
            encoded = json.dumps(body).encode()
            headers["Content-Type"] = "application/json"
            headers["Content-Length"] = str(len(encoded))
        conn.request(method, path, body=encoded, headers=headers)
        response = conn.getresponse()
        data = response.read()
        conn.close()
        return response.status, json.loads(data) if response.getheader("Content-Type", "").startswith("application/json") else data.decode()

    def test_status_uses_batch_mode_and_safe_detail(self):
        status, body = self.request("GET", "/api/status")
        self.assertEqual(status, 200)
        self.assertEqual(body["connected"], True)
        self.assertEqual(body["target"], "ugrp1")
        self.assertEqual(body["detail"], "ready")
        self.assertEqual(body["probe"], {"ok": True, "controller": "MasterPi-v1", "battery_mv": 8100})
        call = self.runner.calls[0]
        self.assertEqual(call[0], "run")
        self.assertIn("BatchMode=yes", call[1])
        self.assertNotIn("stderr", json.dumps(body))

    def test_status_failure_does_not_expose_command_output(self):
        self.runner.status = CommandResult(255, "", "secret-looking private detail")
        self.runner.auth = CommandResult(255, "", "secret-looking private detail")
        status, body = self.request("GET", "/api/status")
        self.assertEqual(status, 200)
        self.assertEqual(body["connected"], False)
        self.assertEqual(body["detail"], "ssh_unreachable")
        self.assertNotIn("secret", json.dumps(body))

    def test_status_controller_failure(self):
        self.runner.status = CommandResult(1, '{"ok": false}', "error")
        status, body = self.request("GET", "/api/status")
        self.assertEqual(status, 200)
        self.assertEqual(body["connected"], False)
        self.assertEqual(body["detail"], "controller_unreachable")

    def test_status_does_not_probe_i2c_while_motion_is_active(self):
        self.controller._last_probe = {"ok": True, "controller": "MasterPi-v1", "battery_mv": 8100}
        self.controller._active_process = FakeProcess(running=True)
        status, body = self.request("GET", "/api/status")
        self.assertEqual(status, 200)
        self.assertTrue(body["connected"])
        self.assertEqual(body["detail"], "motion_active")
        self.assertEqual(self.runner.calls, [])

    def test_drive_returns_promptly_and_uses_exact_argv(self):
        status, body = self.request("POST", "/api/drive", {"direction": "forward", "speed": 35, "duration": 0.5})
        self.assertEqual(status, 202)
        self.assertTrue(body["success"])
        motion_call = self.runner.calls[0]
        self.assertEqual(motion_call[0], "start")
        self.assertEqual(motion_call[1][-6:], ["drive", "forward", "--speed", "35", "--duration", "0.5"])
        self.assertEqual(motion_call[1][-7], "/home/ugrp1/MasterPi/tools/masterpi_control.py")

    def test_arm_returns_200_and_runs_synchronously(self):
        status, body = self.request("POST", "/api/arm", {"servo": 3, "pulse": 1500, "duration": 0.5})
        self.assertEqual(status, 200)
        self.assertTrue(body["success"])
        arm_call = self.runner.calls[0]
        self.assertEqual(arm_call[0], "run")
        self.assertEqual(arm_call[1][-5:], ["servo", "3", "1500", "--duration", "0.5"])
        self.assertEqual(arm_call[1][-6], "/home/ugrp1/MasterPi/tools/masterpi_control.py")
        self.assertTrue(any(event["kind"] == "servo" for event in self.controller.events()))
        # Arm does not store background active process
        self.assertIsNone(self.controller._active_process)

    def test_arm_failure_returns_502(self):
        self.runner.stop = CommandResult(1, "", "servo failed")
        status, body = self.request("POST", "/api/arm", {"servo": 3, "pulse": 1500, "duration": 0.5})
        self.assertEqual(status, 502)
        self.assertFalse(body["success"])
        self.assertEqual(body["error"]["code"], "servo_failed")

    def test_new_drive_cancels_previous_before_starting_next(self):
        self.request("POST", "/api/drive", {"direction": "left", "speed": 35, "duration": 0.2})
        first_process = self.runner.processes[0]
        self.request("POST", "/api/drive", {"direction": "right", "speed": 36, "duration": 0.2})
        self.assertTrue(first_process.terminated)
        self.assertEqual([call[0] for call in self.runner.calls], ["start", "start"])

    def test_stop_terminates_tracked_process_then_runs_remote_stop(self):
        self.request("POST", "/api/drive", {"direction": "forward", "speed": 35, "duration": 0.5})
        process = self.runner.processes[0]
        status, body = self.request("POST", "/api/stop")
        self.assertEqual(status, 200)
        self.assertTrue(body["success"])
        self.assertTrue(process.terminated)
        self.assertEqual(self.runner.calls[-1][0], "run")
        self.assertEqual(self.runner.calls[-1][1][-1], "stop")

    def test_stop_failure_is_not_reported_as_success(self):
        self.runner.stop = CommandResult(1, "", "failed")
        status, body = self.request("POST", "/api/stop")
        self.assertEqual(status, 502)
        self.assertFalse(body["success"])

    def test_stop_retries_one_transient_ssh_failure(self):
        class TransientStopRunner(FakeRunner):
            def __init__(self):
                super().__init__()
                self.stop_attempts = 0

            def run(self, argv, *, timeout):
                if argv[-1] == "stop":
                    self.calls.append(("run", list(argv), timeout))
                    self.stop_attempts += 1
                    return CommandResult(255 if self.stop_attempts == 1 else 0)
                return super().run(argv, timeout=timeout)

        runner = TransientStopRunner()
        controller = DashboardController(runner=runner, register_shutdown=False)
        status, body = controller.stop()
        self.assertEqual(status, 200)
        self.assertTrue(body["success"])
        self.assertEqual(runner.stop_attempts, 2)

    def test_events_are_bounded_json(self):
        for _ in range(130):
            self.controller._event("test", "safe")
        status, body = self.request("GET", "/api/events")
        self.assertEqual(status, 200)
        self.assertLessEqual(len(body["events"]), 100)

    def test_static_file_and_spa_fallback(self):
        conn = HTTPConnection(self.host, self.port, timeout=2)
        conn.request("GET", "/app.js")
        response = conn.getresponse()
        self.assertEqual(response.status, 200)
        self.assertIn("console.log", response.read().decode())
        conn.close()

        conn = HTTPConnection(self.host, self.port, timeout=2)
        conn.request("GET", "/control/forward")
        response = conn.getresponse()
        self.assertEqual(response.status, 200)
        self.assertIn("MasterPi", response.read().decode())
        conn.close()

    def test_actuator_api_requires_token_when_configured(self):
        self.server.auth_token = "secret-token"
        try:
            status, body = self.request("POST", "/api/stop")
            self.assertEqual(status, 401)
            conn = HTTPConnection(self.host, self.port, timeout=2)
            conn.request("POST", "/api/stop", body=b"", headers={"X-Dashboard-Token": "secret-token", "Content-Length": "0"})
            response = conn.getresponse(); response.read(); conn.close()
            self.assertNotEqual(response.status, 401)
            # Read-only status stays available without the token.
            status, _ = self.request("GET", "/api/status")
            self.assertNotEqual(status, 401)
        finally:
            self.server.auth_token = ""

    def test_non_loopback_bind_refuses_to_start_without_token(self):
        from dashboard.server import main as dashboard_main
        with patch.dict(os.environ, {"MASTERPI_DASHBOARD_TOKEN": ""}):
            with self.assertRaises(SystemExit):
                dashboard_main(["--host", "0.0.0.0", "--port", "0", "--transport", "mock"])

    def test_static_path_traversal_is_rejected(self):
        status, body = self.request("GET", "/../secret.txt")
        self.assertEqual(status, 404)
        self.assertEqual(body["success"], False)
        status, body = self.request("GET", "/%2e%2e/%2e%2e/etc/passwd")
        self.assertEqual(status, 404)
        self.assertEqual(body["success"], False)


class ValidationAndSeamTests(unittest.TestCase):
    def test_validation_boundaries(self):
        self.assertEqual(validate_drive_payload({"direction": "forward", "speed": 31, "duration": 0.05})[1:], (31, 0.05))
        self.assertEqual(validate_drive_payload({"direction": "rotate-right", "speed": 40, "duration": 2.0})[1:], (40, 2.0))
        for payload in (
            {"direction": "bad", "speed": 31, "duration": 0.1},
            {"direction": "forward", "speed": 30, "duration": 0.1},
            {"direction": "forward", "speed": 41, "duration": 0.1},
            {"direction": "forward", "speed": True, "duration": 0.1},
            {"direction": "forward", "speed": 31, "duration": 0.049},
            {"direction": "forward", "speed": 31, "duration": 2.01},
            {"direction": "forward", "speed": 31, "duration": float("nan")},
            {"direction": "forward", "speed": 31, "duration": True},
        ):
            with self.assertRaises(ValueError):
                validate_drive_payload(payload)

    def test_runner_is_injectable_and_no_subprocess_is_called(self):
        runner = FakeRunner()
        controller = DashboardController(runner=runner, register_shutdown=False)
        with patch("dashboard.server.subprocess.run", side_effect=AssertionError("real SSH invoked")):
            status, body = controller.status()
        self.assertEqual(status, 200)
        self.assertTrue(body["connected"])
        self.assertEqual(len(runner.calls), 2)  # auth check + hardware probe

    def test_servo_validation_boundaries(self):
        self.assertEqual(validate_servo_payload({"servo": 1, "pulse": 500, "duration": 0.1}), (1, 500, 0.1))
        self.assertEqual(validate_servo_payload({"servo": 6, "pulse": 2500, "duration": 3.0}), (6, 2500, 3.0))
        for payload in (
            {"servo": 2, "pulse": 1500, "duration": 1},
            {"servo": True, "pulse": 1500, "duration": 1},
            {"servo": 1, "pulse": 499, "duration": 1},
            {"servo": 1, "pulse": 2501, "duration": 1},
            {"servo": 1, "pulse": True, "duration": 1},
            {"servo": 1, "pulse": 1500, "duration": 0.099},
            {"servo": 1, "pulse": 1500, "duration": 3.001},
            {"servo": 1, "pulse": 1500, "duration": float("inf")},
            {"servo": 1, "pulse": 1500, "duration": True},
        ):
            with self.assertRaises(ValueError):
                validate_servo_payload(payload)

    def test_argv_helpers_contain_no_shell_string(self):
        for argv in (status_argv(), stop_argv()):
            self.assertTrue(all(isinstance(part, str) for part in argv))
            self.assertNotIn(";", " ".join(argv))

    def test_servo_argv_contract(self):
        self.assertEqual(servo_argv(5, 1800, 1.25)[-5:], ["servo", "5", "1800", "--duration", "1.25"])

    def test_production_runner_disables_shell(self):
        with patch("dashboard.server.subprocess.Popen") as popen:
            CommandRunner().start(["ssh", "ugrp1"])
        kwargs = popen.call_args.kwargs
        self.assertFalse(kwargs["shell"])

    def test_launcher_executes_server_directly_without_npm(self):
        launcher = (Path(__file__).parents[1] / "scripts" / "start_masterpi_dashboard.sh").read_text()
        self.assertNotIn("npm", launcher)
        self.assertNotIn("dist", launcher)
        self.assertIn('exec python3 "$ROOT_DIR/dashboard/server.py" "$@"', launcher)


class TransportTests(unittest.TestCase):
    def test_ssh_transport_defaults_and_argv_generation(self):
        transport = SSHTransport()
        self.assertEqual(transport.target_name, "ugrp1")
        self.assertEqual(transport.status_argv()[-1], "true")
        self.assertIn("BatchMode=yes", transport.status_argv())
        self.assertIn("ControlMaster=auto", transport.status_argv())
        self.assertIn("ControlPersist=120", transport.status_argv())
        self.assertTrue(any(part.startswith("ControlPath=") for part in transport.status_argv()))
        self.assertEqual(transport.probe_argv()[-3:], ["python3", "/home/ugrp1/MasterPi/tools/masterpi_control.py", "probe"])
        self.assertEqual(transport.stop_argv()[-3:], ["python3", "/home/ugrp1/MasterPi/tools/masterpi_control.py", "stop"])
        self.assertEqual(
            transport.drive_argv("forward", 35, 1.0)[-8:],
            ["python3", "/home/ugrp1/MasterPi/tools/masterpi_control.py", "drive", "forward", "--speed", "35", "--duration", "1"],
        )
        self.assertEqual(
            transport.servo_argv(3, 1200, 0.5)[-7:],
            ["python3", "/home/ugrp1/MasterPi/tools/masterpi_control.py", "servo", "3", "1200", "--duration", "0.5"],
        )

    def test_ssh_transport_custom_target_and_dry_run(self):
        transport = SSHTransport(
            target="192.168.1.100",
            script_path="/opt/control.py",
            python_bin="/usr/bin/python3",
            connect_timeout=5,
            dry_run=True,
            extra_ssh_opts=["-p", "2222"],
        )
        self.assertEqual(transport.target_name, "192.168.1.100")
        status = transport.status_argv()
        self.assertIn("ConnectTimeout=5", status)
        self.assertIn("-p", status)
        self.assertIn("2222", status)
        self.assertEqual(status[-2:], ["192.168.1.100", "true"])

        stop = transport.stop_argv()
        self.assertEqual(stop[-4:], ["/usr/bin/python3", "/opt/control.py", "stop", "--dry-run"])

        drive = transport.drive_argv("backward", 35, 0.25)
        self.assertEqual(drive[-1], "--dry-run")
        self.assertEqual(drive[-9:-1], ["/usr/bin/python3", "/opt/control.py", "drive", "backward", "--speed", "35", "--duration", "0.25"])

        servo = transport.servo_argv(1, 1500, 1.0)
        self.assertEqual(servo[-1], "--dry-run")
        self.assertEqual(servo[-8:-1], ["/usr/bin/python3", "/opt/control.py", "servo", "1", "1500", "--duration", "1"])

    def test_ssh_transport_with_resolved_address_uses_host_alias(self):
        transport = SSHTransport(target="ugrp1", cached_address="172.20.10.2")
        argv = transport.probe_argv()
        self.assertIn("-o", argv)
        self.assertIn("HostName=172.20.10.2", argv)
        self.assertIn("HostKeyAlias=ugrp1.local", argv)
        self.assertEqual(argv[-4:], ["ugrp1", "python3", "/home/ugrp1/MasterPi/tools/masterpi_control.py", "probe"])
        self.assertNotIn("StrictHostKeyChecking=no", " ".join(argv))
        self.assertNotIn("/dev/null", " ".join(argv))

    def test_local_transport_argv_generation(self):
        transport = LocalTransport(script_path="/home/pi/control.py", python_bin="python3", dry_run=False)
        self.assertEqual(transport.target_name, "local")
        self.assertEqual(transport.status_argv(), ["python3", "/home/pi/control.py", "probe"])
        self.assertEqual(transport.probe_argv(), ["python3", "/home/pi/control.py", "probe"])
        self.assertEqual(transport.stop_argv(), ["python3", "/home/pi/control.py", "stop"])
        self.assertEqual(
            transport.drive_argv("left", 35, 0.4),
            ["python3", "/home/pi/control.py", "drive", "left", "--speed", "35", "--duration", "0.4"],
        )
        self.assertEqual(
            transport.servo_argv(4, 1600, 0.8),
            ["python3", "/home/pi/control.py", "servo", "4", "1600", "--duration", "0.8"],
        )

    def test_transport_rejects_dead_zone_drive_speed(self):
        for transport in (
            LocalTransport(script_path="/tmp/control.py"),
            SSHTransport(target="ugrp1", cached_address="127.0.0.1"),
            MockTransport(),
        ):
            with self.assertRaises(ValueError):
                transport.drive_argv("forward", 30, 0.2)

    def test_local_transport_dry_run(self):
        transport = LocalTransport(dry_run=True)
        self.assertEqual(transport.stop_argv()[-1], "--dry-run")
        self.assertEqual(transport.drive_argv("right", 35, 0.5)[-1], "--dry-run")
        self.assertEqual(transport.servo_argv(5, 1000, 0.5)[-1], "--dry-run")

    def test_mock_transport_argv_generation(self):
        transport = MockTransport()
        self.assertEqual(transport.target_name, "mock")
        self.assertIn("-c", transport.status_argv())
        self.assertIn("-c", transport.probe_argv())
        self.assertIn("-c", transport.stop_argv())
        self.assertIn("-c", transport.drive_argv("forward", 35, 0.5))
        self.assertIn("-c", transport.servo_argv(1, 1500, 1.0))

    def test_create_transport_factory_and_env_vars(self):
        t1 = create_transport("ssh", target="test-host")
        self.assertIsInstance(t1, SSHTransport)
        self.assertEqual(t1.target_name, "test-host")

        t2 = create_transport("local")
        self.assertIsInstance(t2, LocalTransport)
        self.assertEqual(t2.target_name, "local")

        t3 = create_transport("mock")
        self.assertIsInstance(t3, MockTransport)
        self.assertEqual(t3.target_name, "mock")

        with patch.dict(os.environ, {"MASTERPI_TRANSPORT": "local", "MASTERPI_DRY_RUN": "1"}):
            t_env = create_transport()
            self.assertIsInstance(t_env, LocalTransport)
            self.assertTrue(t_env.dry_run)

        with self.assertRaises(ValueError):
            create_transport("unknown_transport")

    def test_controller_and_server_with_local_transport(self):
        runner = FakeRunner(status=CommandResult(0, '{"ok": true, "controller": "MasterPi-local", "battery_mv": 8200}'))
        transport = LocalTransport(script_path="/tmp/control.py")
        controller = DashboardController(runner=runner, transport=transport, register_shutdown=False)
        status, body = controller.status()
        self.assertEqual(status, 200)
        self.assertEqual(body["target"], "local")
        self.assertEqual(body["connected"], True)
        self.assertEqual(body["detail"], "ready")
        self.assertEqual(runner.calls[0][1], ["python3", "/tmp/control.py", "probe"])

        controller.drive({"direction": "forward", "speed": 35, "duration": 0.5})
        motion_call = runner.calls[1]  # [run status], [start drive]
        self.assertEqual(motion_call[0], "start")
        self.assertEqual(motion_call[1], ["python3", "/tmp/control.py", "drive", "forward", "--speed", "35", "--duration", "0.5"])

    def test_cli_parser_transport_options(self):
        parser = build_parser()
        args = parser.parse_args(["--transport", "local", "--target", "custom-pi", "--dry-run", "--port", "9000"])
        self.assertEqual(args.transport, "local")
        self.assertEqual(args.target, "custom-pi")
        self.assertTrue(args.dry_run)
        self.assertEqual(args.port, 9000)


class AddressResolverTests(unittest.TestCase):
    def test_find_candidates_precedence_order(self):
        resolver = AddressResolver(
            mdns_resolver=lambda host: "172.20.10.2",
            port_prober=lambda ip, port, timeout: ip == "172.20.10.5",
            local_ip_getter=lambda: {"172.20.10.4", "127.0.0.1"},
        )
        candidates = resolver.find_candidates(explicit_target="192.168.1.50", cached_address="172.20.10.99")
        self.assertEqual(candidates[0], "192.168.1.50")
        self.assertEqual(candidates[1], "172.20.10.99")
        self.assertEqual(candidates[2], "172.20.10.2")
        self.assertIn("172.20.10.5", candidates)

    def test_ssh_auth_check_required_before_accepting_scanned_candidate(self):
        # Subnet has open port 22 on 172.20.10.7 and 172.20.10.8, but only 172.20.10.8 authenticates via SSH
        resolver = AddressResolver(
            mdns_resolver=lambda host: None,
            port_prober=lambda ip, port, timeout: ip in {"172.20.10.7", "172.20.10.8"},
            local_ip_getter=lambda: {"172.20.10.4", "127.0.0.1"},
        )
        transport = SSHTransport(resolver=resolver)

        class AuthCheckRunner:
            def __init__(self):
                self.calls = []
            def run(self, argv, timeout):
                self.calls.append(list(argv))
                # Only 172.20.10.8 passes SSH auth check
                if "HostName=172.20.10.8" in " ".join(argv) and argv[-1] == "true":
                    return CommandResult(0)
                return CommandResult(255)

        runner = AuthCheckRunner()
        resolved = transport.resolve_and_authenticate(runner)
        self.assertEqual(resolved, "172.20.10.8")
        self.assertEqual(transport.cached_address, "172.20.10.8")

    def test_cache_invalidation_on_command_failure(self):
        transport = SSHTransport(cached_address="172.20.10.2")
        self.assertEqual(transport.cached_address, "172.20.10.2")

        runner = FakeRunner()
        controller = DashboardController(runner=runner, transport=transport, register_shutdown=False)

        # Command failure invalidates cache
        runner.stop = CommandResult(255, "", "SSH connection lost")
        controller.stop()
        self.assertIsNone(transport.cached_address)


if __name__ == "__main__":
    unittest.main()
