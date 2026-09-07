#!/usr/bin/env python3
"""Small, local-only HTTP dashboard backend for the MasterPi robot."""

from __future__ import annotations

import argparse
import atexit
import hmac
import json
import math
import os
from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
from functools import partial
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
from typing import Any, Callable, Deque, Iterable, Mapping, Sequence
from urllib.parse import unquote, urlsplit

try:
    from .transport import (
        ControlTransport,
        DEFAULT_PI_SCRIPT,
        DEFAULT_TARGET,
        LocalTransport,
        MockTransport,
        SSHTransport,
        create_transport,
    )
except ImportError:  # Direct execution fallback
    from transport import (
        ControlTransport,
        DEFAULT_PI_SCRIPT,
        DEFAULT_TARGET,
        LocalTransport,
        MockTransport,
        SSHTransport,
        create_transport,
    )

TARGET = DEFAULT_TARGET
PI_SCRIPT = DEFAULT_PI_SCRIPT
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 8765
STATUS_TIMEOUT = 3.0
STOP_TIMEOUT = 4.0
ARM_TIMEOUT = 5.0
MAX_BODY_BYTES = 16 * 1024
MAX_EVENTS = 100
STATUS_SUCCESS_CACHE_SECONDS = 3.0
STATUS_FAILURE_CACHE_SECONDS = 10.0
MOTION_DIRECTIONS = frozenset(
    {"forward", "backward", "left", "right", "rotate-left", "rotate-right"}
)
SERVO_IDS = frozenset({1, 3, 4, 5, 6})
MIN_EFFECTIVE_WHEEL_SPEED = 31
MAX_CHASSIS_SPEED = 40


@dataclass(frozen=True)
class CommandResult:
    returncode: int
    stdout: str = ""
    stderr: str = ""


class CommandRunner:
    """Subprocess seam. Production calls are always argv-based and shell-free."""

    def run(self, argv: Sequence[str], *, timeout: float) -> CommandResult:
        completed = subprocess.run(
            list(argv),
            shell=False,
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
        return CommandResult(completed.returncode, completed.stdout, completed.stderr)

    def start(self, argv: Sequence[str]) -> subprocess.Popen[bytes]:
        return subprocess.Popen(
            list(argv),
            shell=False,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def _safe_result_detail(result: CommandResult | None, *, timed_out: bool = False) -> str:
    if timed_out:
        return "timeout"
    if result is None:
        return "ssh_unreachable"
    return "ready" if result.returncode == 0 else "ssh_unreachable"


_DEFAULT_TRANSPORT = SSHTransport()


def status_argv() -> list[str]:
    return _DEFAULT_TRANSPORT.status_argv()


def probe_argv() -> list[str]:
    return _DEFAULT_TRANSPORT.probe_argv()


def stop_argv() -> list[str]:
    return _DEFAULT_TRANSPORT.stop_argv()


def drive_argv(direction: str, speed: int, duration: float) -> list[str]:
    return _DEFAULT_TRANSPORT.drive_argv(direction, speed, duration)


def servo_argv(servo: int, pulse: int, duration: float) -> list[str]:
    return _DEFAULT_TRANSPORT.servo_argv(servo, pulse, duration)


def validate_drive_payload(payload: Mapping[str, Any]) -> tuple[str, int, float]:
    if not isinstance(payload, Mapping):
        raise ValueError("JSON body must be an object")

    direction = payload.get("direction")
    if not isinstance(direction, str) or direction not in MOTION_DIRECTIONS:
        raise ValueError("direction must be one of the supported directions")

    speed = payload.get("speed")
    if isinstance(speed, bool) or not isinstance(speed, int) or not MIN_EFFECTIVE_WHEEL_SPEED <= speed <= MAX_CHASSIS_SPEED:
        raise ValueError(f"speed must be an integer from {MIN_EFFECTIVE_WHEEL_SPEED} to {MAX_CHASSIS_SPEED}; values <=30 do not move this chassis")

    duration = payload.get("duration")
    if isinstance(duration, bool) or not isinstance(duration, (int, float)):
        raise ValueError("duration must be a number from 0.05 to 2.0")
    duration_float = float(duration)
    if not math.isfinite(duration_float) or not 0.05 <= duration_float <= 2.0:
        raise ValueError("duration must be a number from 0.05 to 2.0")
    return direction, speed, duration_float


def validate_servo_payload(payload: Mapping[str, Any]) -> tuple[int, int, float]:
    if not isinstance(payload, Mapping):
        raise ValueError("JSON body must be an object")

    servo = payload.get("servo")
    if isinstance(servo, bool) or not isinstance(servo, int) or servo not in SERVO_IDS:
        raise ValueError("servo must be one of 1, 3, 4, 5, or 6")

    pulse = payload.get("pulse")
    if isinstance(pulse, bool) or not isinstance(pulse, int) or not 500 <= pulse <= 2500:
        raise ValueError("pulse must be an integer from 500 to 2500")

    duration = payload.get("duration")
    if isinstance(duration, bool) or not isinstance(duration, (int, float)):
        raise ValueError("duration must be a number from 0.1 to 3.0")
    duration_float = float(duration)
    if not math.isfinite(duration_float) or not 0.1 <= duration_float <= 3.0:
        raise ValueError("duration must be a number from 0.1 to 3.0")
    return servo, pulse, duration_float


class DashboardController:
    """Serialized controller state with injectable command and process seams."""

    def __init__(
        self,
        *,
        runner: CommandRunner | Any | None = None,
        process_factory: Callable[[Sequence[str]], Any] | None = None,
        register_shutdown: bool = True,
        transport: ControlTransport | None = None,
    ) -> None:
        self.runner = runner or CommandRunner()
        self.process_factory = process_factory or self.runner.start
        self.transport = transport or SSHTransport()
        self._lock = threading.RLock()
        self._active_process: Any | None = None
        self._last_probe: dict[str, Any] | None = None
        self._last_status: tuple[float, tuple[int, dict[str, Any]]] | None = None
        self._events: Deque[dict[str, str]] = deque(maxlen=MAX_EVENTS)
        self._shutdown = False
        if register_shutdown:
            atexit.register(self.shutdown)

    def _event(self, kind: str, detail: str) -> None:
        evt: dict[str, Any] = {"at": _utc_now(), "kind": kind, "detail": detail}
        addr = self.transport.cached_address
        if addr:
            evt["address"] = addr
        self._events.append(evt)

    def events(self) -> list[dict[str, str]]:
        with self._lock:
            return list(self._events)

    def status(self) -> tuple[int, dict[str, Any]]:
        # Status reads and actuator commands share the same physical I2C bus.
        # Never queue polling requests behind one another or behind a command.
        now = time.monotonic()
        if self._last_status is not None:
            recorded_at, cached = self._last_status
            ttl = STATUS_SUCCESS_CACHE_SECONDS if cached[1].get("connected") else STATUS_FAILURE_CACHE_SECONDS
            if now - recorded_at < ttl:
                return cached

        if not self._lock.acquire(blocking=False):
            if self._last_status is not None:
                return self._last_status[1]
            return HTTPStatus.OK, {
                "connected": False,
                "target": self.transport.target_name,
                "address": self.transport.cached_address,
                "detail": "checking",
                "probe": None,
            }
        try:
            result = self._status_locked()
            self._last_status = (time.monotonic(), result)
            return result
        finally:
            self._lock.release()

    def _status_locked(self) -> tuple[int, dict[str, Any]]:
        if self._active_process is not None:
            returncode = self._active_process.poll()
            if returncode is None and self._last_probe is not None:
                return HTTPStatus.OK, {
                    "connected": True,
                    "target": self.transport.target_name,
                    "address": self.transport.cached_address or self.transport.target_name,
                    "detail": "motion_active",
                    "probe": self._last_probe,
                }
            if returncode is not None:
                self._active_process = None
                if returncode != 0:
                    if returncode == 255:
                        self.transport.invalidate_cache()
                    self._event("drive", "failed")

        if isinstance(self.transport, SSHTransport):
            addr = self.transport.resolve_and_authenticate(self.runner, timeout=STATUS_TIMEOUT)
            if addr is None and not self.transport.cached_address:
                with self._lock:
                    self._event("status", "ssh_unreachable")
                return HTTPStatus.OK, {
                    "connected": False,
                    "target": self.transport.target_name,
                    "address": None,
                    "detail": "ssh_unreachable",
                    "probe": None,
                }

        def run_probe() -> tuple[CommandResult | None, str | None]:
            try:
                return self.runner.run(self.transport.probe_argv(), timeout=STATUS_TIMEOUT), None
            except subprocess.TimeoutExpired:
                return None, "timeout"
            except OSError:
                return None, "ssh_unreachable"

        result, probe_error = run_probe()

        # Once hardware has been proven, tolerate one transient hotspot/SSH or
        # I2C miss before flipping the dashboard offline.
        if self._last_probe is not None and (probe_error is not None or (result is not None and result.returncode != 0)):
            if probe_error is not None or (result is not None and result.returncode == 255):
                self.transport.invalidate_cache()
                if isinstance(self.transport, SSHTransport):
                    self.transport.resolve_and_authenticate(self.runner, timeout=STATUS_TIMEOUT)
            time.sleep(0.1)
            result, probe_error = run_probe()

        if probe_error == "timeout":
            self.transport.invalidate_cache()
            with self._lock:
                self._event("status", "timeout")
            return HTTPStatus.OK, {
                "connected": False,
                "target": self.transport.target_name,
                "address": self.transport.cached_address,
                "detail": "timeout",
                "probe": None,
            }
        if probe_error == "ssh_unreachable":
            self.transport.invalidate_cache()
            with self._lock:
                self._event("status", "ssh_unreachable")
            return HTTPStatus.OK, {
                "connected": False,
                "target": self.transport.target_name,
                "address": self.transport.cached_address,
                "detail": "ssh_unreachable",
                "probe": None,
            }

        assert result is not None
        if result.returncode == 255:
            self.transport.invalidate_cache()
            with self._lock:
                self._event("status", "ssh_unreachable")
            return HTTPStatus.OK, {
                "connected": False,
                "target": self.transport.target_name,
                "address": self.transport.cached_address,
                "detail": "ssh_unreachable",
                "probe": None,
            }

        if result.returncode != 0:
            with self._lock:
                self._event("status", "controller_unreachable")
            return HTTPStatus.OK, {
                "connected": False,
                "target": self.transport.target_name,
                "address": self.transport.cached_address,
                "detail": "controller_unreachable",
                "probe": None,
            }

        try:
            parsed = json.loads(result.stdout.strip())
            if isinstance(parsed, dict) and parsed.get("ok") is True:
                connected = True
                detail = "ready"
                probe = parsed
                self._last_probe = parsed
            else:
                connected = False
                detail = "controller_unreachable"
                probe = None
        except Exception:
            connected = False
            detail = "controller_unreachable"
            probe = None

        with self._lock:
            self._event("status", detail)
        return HTTPStatus.OK, {
            "connected": connected,
            "target": self.transport.target_name,
            "address": self.transport.cached_address,
            "detail": detail,
            "probe": probe,
        }

    def _cancel_active_locked(self) -> None:
        process = self._active_process
        self._active_process = None
        if process is None:
            return
        try:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=1.0)
                self._event("motion_cancelled", "previous motion cancelled")
        except (OSError, subprocess.TimeoutExpired):
            self._event("motion_cancelled", "previous motion cancellation incomplete")

    def _remote_stop_locked(self, reason: str) -> tuple[bool, str]:
        last_detail = "stop command failed"
        for attempt in range(2):
            try:
                result = self.runner.run(self.transport.stop_argv(), timeout=STOP_TIMEOUT)
            except subprocess.TimeoutExpired:
                self.transport.invalidate_cache()
                last_detail = "stop timed out"
            except OSError:
                self.transport.invalidate_cache()
                last_detail = "stop could not reach the Pi"
            else:
                if result.returncode == 0:
                    self._event("stop", reason)
                    return True, "stopped"
                if result.returncode == 255:
                    self.transport.invalidate_cache()
                last_detail = "stop command failed"

            if attempt == 0:
                if isinstance(self.transport, SSHTransport):
                    self.transport.resolve_and_authenticate(self.runner, timeout=STATUS_TIMEOUT)
                time.sleep(0.1)

        self._event("stop", "failed")
        return False, last_detail

    def stop(self) -> tuple[int, dict[str, Any]]:
        with self._lock:
            self._cancel_active_locked()
            ok, detail = self._remote_stop_locked("requested")
        if ok:
            return HTTPStatus.OK, {"success": True, "detail": detail}
        return HTTPStatus.BAD_GATEWAY, {"success": False, "error": {"code": "stop_failed", "message": detail}}

    def drive(self, payload: Mapping[str, Any]) -> tuple[int, dict[str, Any]]:
        try:
            direction, speed, duration = validate_drive_payload(payload)
        except ValueError as exc:
            return HTTPStatus.BAD_REQUEST, {"success": False, "error": {"code": "invalid_request", "message": str(exc)}}

        with self._lock:
            self._cancel_active_locked()
            # masterpi_control.py performs stop_all() before applying every
            # drive command and again in its finally block.  A separate,
            # synchronous SSH stop here only delayed motion by a full remote
            # process launch without adding another safety boundary.
            argv = self.transport.drive_argv(direction, speed, duration)
            try:
                process = self.process_factory(argv)
            except OSError:
                self.transport.invalidate_cache()
                self._event("drive", "launch failed")
                return HTTPStatus.BAD_GATEWAY, {"success": False, "error": {"code": "launch_failed", "message": "motion command could not start"}}
            self._active_process = process if process.poll() is None else None
            self._event("drive", direction)
        return HTTPStatus.ACCEPTED, {"success": True, "direction": direction, "speed": speed, "duration": duration}

    def arm(self, payload: Mapping[str, Any]) -> tuple[int, dict[str, Any]]:
        try:
            servo, pulse, duration = validate_servo_payload(payload)
        except ValueError as exc:
            return HTTPStatus.BAD_REQUEST, {"success": False, "error": {"code": "invalid_request", "message": str(exc)}}

        timeout = duration + 2.0
        with self._lock:
            argv = self.transport.servo_argv(servo, pulse, duration)
            try:
                result = self.runner.run(argv, timeout=timeout)
            except subprocess.TimeoutExpired:
                self.transport.invalidate_cache()
                self._event("servo", "timeout")
                return HTTPStatus.BAD_GATEWAY, {"success": False, "error": {"code": "servo_failed", "message": "servo command timed out"}}
            except OSError:
                self.transport.invalidate_cache()
                self._event("servo", "launch failed")
                return HTTPStatus.BAD_GATEWAY, {"success": False, "error": {"code": "servo_failed", "message": "servo command could not start"}}

            if result.returncode != 0:
                if result.returncode == 255:
                    self.transport.invalidate_cache()
                self._event("servo", "failed")
                return HTTPStatus.BAD_GATEWAY, {"success": False, "error": {"code": "servo_failed", "message": "servo command failed"}}

            self._event("servo", f"servo {servo}")
        return HTTPStatus.OK, {"success": True, "servo": servo, "pulse": pulse, "duration": duration}

    def shutdown(self) -> None:
        with self._lock:
            if self._shutdown:
                return
            self._shutdown = True
            self._cancel_active_locked()
            try:
                self._remote_stop_locked("shutdown")
            except Exception:
                # Shutdown must not mask interpreter termination.
                pass


class DashboardRequestHandler(BaseHTTPRequestHandler):
    server_version = "MasterPiDashboard/1.0"
    protocol_version = "HTTP/1.1"

    def __init__(self, request: Any, client_address: Any, server: Any) -> None:
        self.controller: DashboardController = server.controller
        self.dist_dir: Path = server.dist_dir
        super().__init__(request, client_address, server)

    def log_message(self, format: str, *args: Any) -> None:
        return

    def _json(self, status: int, payload: Mapping[str, Any]) -> None:
        encoded = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(encoded)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            self.wfile.write(encoded)
        except (BrokenPipeError, ConnectionResetError):
            # The browser may abandon an offline status request; do not turn a
            # bounded connectivity failure into a noisy server traceback.
            pass

    def _error(self, status: int, code: str, message: str) -> None:
        self._json(status, {"success": False, "error": {"code": code, "message": message}})

    def _read_json(self) -> Mapping[str, Any]:
        raw_length = self.headers.get("Content-Length")
        if raw_length is None:
            raise ValueError("Content-Length is required")
        try:
            length = int(raw_length)
        except ValueError as exc:
            raise ValueError("invalid Content-Length") from exc
        if length < 0 or length > MAX_BODY_BYTES:
            raise ValueError("request body is too large")
        try:
            value = json.loads(self.rfile.read(length).decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise ValueError("request body must be valid JSON") from exc
        if not isinstance(value, Mapping):
            raise ValueError("JSON body must be an object")
        return value

    def do_GET(self) -> None:
        path = urlsplit(self.path).path
        if path == "/api/status":
            status, payload = self.controller.status()
            self._json(status, payload)
            return
        if path == "/api/events":
            self._json(HTTPStatus.OK, {"events": self.controller.events()})
            return
        if path.startswith("/api/"):
            self._error(HTTPStatus.NOT_FOUND, "not_found", "API route not found")
            return
        self._serve_static(path)

    def _authorized(self) -> bool:
        token = getattr(self.server, "auth_token", "") or ""
        if not token:
            return True
        supplied = self.headers.get("X-Dashboard-Token", "")
        if not supplied:
            auth = self.headers.get("Authorization", "")
            if auth.lower().startswith("bearer "):
                supplied = auth[7:].strip()
        return hmac.compare_digest(supplied, token)

    def do_POST(self) -> None:
        path = urlsplit(self.path).path
        if not self._authorized():
            self._error(HTTPStatus.UNAUTHORIZED, "unauthorized", "actuator API requires the dashboard token")
            return
        try:
            if path == "/api/drive":
                status, payload = self.controller.drive(self._read_json())
            elif path == "/api/arm":
                status, payload = self.controller.arm(self._read_json())
            elif path == "/api/stop":
                status, payload = self.controller.stop()
            else:
                self._error(HTTPStatus.NOT_FOUND, "not_found", "API route not found")
                return
        except ValueError as exc:
            self._error(HTTPStatus.BAD_REQUEST, "invalid_request", str(exc))
            return
        self._json(status, payload)

    def _safe_static_path(self, request_path: str) -> Path | None:
        decoded = unquote(request_path)
        if "\x00" in decoded:
            return None
        relative = decoded.lstrip("/")
        candidate = (self.dist_dir / relative).resolve()
        try:
            candidate.relative_to(self.dist_dir)
        except ValueError:
            return None
        return candidate

    def _serve_static(self, request_path: str) -> None:
        if request_path == "":
            request_path = "/"
        candidate = self._safe_static_path(request_path)
        if candidate is None:
            self._error(HTTPStatus.NOT_FOUND, "not_found", "resource not found")
            return
        if candidate.is_file():
            self._send_file(candidate)
            return
        if self.dist_dir.is_dir():
            index = (self.dist_dir / "index.html").resolve()
            try:
                index.relative_to(self.dist_dir)
            except ValueError:
                index = None  # pragma: no cover
            if index is not None and index.is_file():
                self._send_file(index)
                return
        self._error(HTTPStatus.NOT_FOUND, "not_found", "dashboard frontend not found")

    def _send_file(self, path: Path) -> None:
        try:
            content = path.read_bytes()
        except OSError:
            self._error(HTTPStatus.NOT_FOUND, "not_found", "resource not found")
            return
        content_type = {
            ".html": "text/html; charset=utf-8",
            ".css": "text/css; charset=utf-8",
            ".js": "text/javascript; charset=utf-8",
            ".json": "application/json; charset=utf-8",
            ".svg": "image/svg+xml",
        }.get(path.suffix.lower(), "application/octet-stream")
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(content)))
        self.end_headers()
        self.wfile.write(content)


class DashboardHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def handle_error(self, request: Any, client_address: Any) -> None:
        if isinstance(sys.exception(), (BrokenPipeError, ConnectionResetError)):
            return
        super().handle_error(request, client_address)


def make_server(
    host: str = DEFAULT_HOST,
    port: int = DEFAULT_PORT,
    *,
    controller: DashboardController | None = None,
    transport: ControlTransport | None = None,
    dist_dir: str | os.PathLike[str] | None = None,
    auth_token: str | None = None,
) -> DashboardHTTPServer:
    repo_root = Path(__file__).resolve().parent.parent
    resolved_dist = Path(dist_dir).resolve() if dist_dir is not None else repo_root / "dashboard" / "frontend"
    active_controller = controller or DashboardController(transport=transport)
    handler = partial(DashboardRequestHandler)
    server = DashboardHTTPServer((host, port), handler)
    server.controller = active_controller
    server.dist_dir = resolved_dist
    server.auth_token = auth_token or ""
    return server


def _is_loopback_host(host: str) -> bool:
    return host in {"127.0.0.1", "::1", "localhost", ""}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Local MasterPi dashboard backend")
    parser.add_argument(
        "--host",
        default=os.environ.get("MASTERPI_HOST", DEFAULT_HOST),
        help="bind host (default: 127.0.0.1)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=int(os.environ.get("MASTERPI_PORT", str(DEFAULT_PORT))),
        help="bind port (default: 8765)",
    )
    parser.add_argument(
        "--transport",
        choices=["ssh", "local", "mock"],
        default=None,
        help="control transport mode: ssh (default), local, or mock",
    )
    parser.add_argument(
        "--target",
        "--ssh-target",
        dest="target",
        default=None,
        help="target hostname or SSH alias for remote transport (default: ugrp1)",
    )
    parser.add_argument(
        "--script-path",
        default=None,
        help="path to masterpi_control.py on target or local host",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        default=None,
        help="pass --dry-run to control commands",
    )
    return parser


def main(argv: Iterable[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(list(argv) if argv is not None else None)
    auth_token = os.environ.get("MASTERPI_DASHBOARD_TOKEN", "").strip()
    if not _is_loopback_host(args.host) and not auth_token:
        parser.error(
            f"refusing to expose the actuator API on {args.host!r} without MASTERPI_DASHBOARD_TOKEN; "
            "bind to 127.0.0.1 or set a token"
        )
    transport = create_transport(
        transport_type=args.transport,
        target=args.target,
        script_path=args.script_path,
        dry_run=args.dry_run,
    )
    server = make_server(args.host, args.port, transport=transport, auth_token=auth_token)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.controller.shutdown()
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
