"""Last commanded chassis/arm values, with bounded live adjustments."""

from __future__ import annotations

import importlib.util
import json
import os
import subprocess
import threading
from pathlib import Path
from typing import Any

from dashboard.server import CommandResult, CommandRunner, validate_drive_payload, validate_servo_payload


ROOT = Path(__file__).resolve().parents[1]
# Legacy default retained; multi-REAL children override through the same env as
# scripts/red_block/deploy.py so manual panel controls target the selected robot.
REMOTE_CONTROL = os.environ.get("UGRP_ROBOT_REMOTE_CONTROL", f"/home/{os.environ.get('UGRP_ROBOT_USER', 'ugrp1')}/MasterPi/tools/masterpi_control.py")
ARM_JOINTS = (
    (1, "집게"),
    (3, "어깨"),
    (4, "팔꿈치"),
    (5, "손목"),
    (6, "베이스"),
)
DEFAULT_ARM = {1: 2000, 3: 740, 4: 2320, 5: 1320, 6: 1500}


class RobotError(RuntimeError):
    """Raised when a motor or arm command cannot be delivered."""


def _load_deploy():
    path = ROOT / "scripts" / "red_block" / "deploy.py"
    spec = importlib.util.spec_from_file_location("ugrp_red_block_deploy", path)
    if spec is None or spec.loader is None:
        raise RobotError(f"missing {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _drive_speeds(direction: str, speed: int) -> list[tuple[int, int]]:
    path = ROOT / "scripts" / "masterpi_control.py"
    spec = importlib.util.spec_from_file_location("ugrp_masterpi_control", path)
    if spec is None or spec.loader is None:
        raise RobotError(f"missing {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.drive_speeds(direction, speed)


class RobotPanel:
    def __init__(
        self,
        *,
        host: str = "ugrp1",
        runner: CommandRunner | None = None,
        ssh_args: tuple[str, list[str]] | None = None,
        dry_run: bool = False,
    ) -> None:
        self.host = host
        self.runner = runner or CommandRunner()
        self.ssh_args = ssh_args
        self.dry_run = dry_run
        self.connected = False
        self.detail = "unchecked"
        self.battery_mv: int | None = None
        self.arm_pulses = dict(DEFAULT_ARM)
        self.motor_speeds = {1: 0, 2: 0, 3: 0, 4: 0}
        self.motion = {"direction": "stop", "speed": 0, "duration": 0.3}
        self.error: str | None = None
        self._lock = threading.Lock()

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {
                "connected": self.connected,
                "detail": self.detail,
                "battery_mv": self.battery_mv,
                "error": self.error,
                "motion": dict(self.motion),
                "motors": [
                    {"id": motor, "speed": self.motor_speeds[motor]}
                    for motor in range(1, 5)
                ],
                "arm": [
                    {
                        "id": servo,
                        "name": label,
                        "pulse": self.arm_pulses[servo],
                        "min": 500,
                        "max": 2500,
                    }
                    for servo, label in ARM_JOINTS
                ],
                "note": "PWM 서보는 위치 피드백이 없습니다. 숫자는 마지막 명령값입니다.",
            }

    def refresh(self) -> dict[str, Any]:
        try:
            result = self._run(["probe"], timeout=10.0)
            probe = _parse_probe(result.stdout)
            with self._lock:
                self.connected = True
                self.detail = "ready"
                self.battery_mv = probe.get("battery_mv")
                self.error = None
        except RobotError as exc:
            with self._lock:
                self.connected = False
                self.detail = "unreachable"
                self.error = str(exc)
        return self.snapshot()

    def drive(self, payload: dict[str, Any]) -> dict[str, Any]:
        direction, speed, duration = validate_drive_payload(payload)
        speeds = dict(_drive_speeds(direction, speed))
        self._run(
            ["drive", direction, "--speed", str(speed), "--duration", format(duration, ".6g")],
            timeout=duration + 8.0,
        )
        with self._lock:
            self.connected = True
            self.detail = "ready"
            self.error = None
            self.motion = {
                "direction": direction,
                "speed": speed,
                "duration": duration,
            }
            self.motor_speeds = {motor: speeds.get(motor, 0) for motor in range(1, 5)}
        return self.snapshot()

    def arm(self, payload: dict[str, Any]) -> dict[str, Any]:
        servo, pulse, duration = validate_servo_payload(payload)
        self._run(
            ["servo", str(servo), str(pulse), "--duration", format(duration, ".6g")],
            timeout=duration + 8.0,
        )
        with self._lock:
            self.connected = True
            self.detail = "ready"
            self.error = None
            self.arm_pulses[servo] = pulse
        return self.snapshot()

    def stop(self) -> dict[str, Any]:
        self._run(["stop"], timeout=10.0)
        with self._lock:
            self.connected = True
            self.detail = "ready"
            self.error = None
            self.motion = {"direction": "stop", "speed": 0, "duration": 0}
            self.motor_speeds = {1: 0, 2: 0, 3: 0, 4: 0}
        return self.snapshot()

    def _remote_argv(self, parts: list[str]) -> list[str]:
        if self.ssh_args is None:
            try:
                destination, opts = _load_deploy().ssh_connection_args(self.host)
            except Exception as exc:
                raise RobotError("로봇 SSH 대상을 정하지 못했습니다.") from exc
        else:
            destination, opts = self.ssh_args
        cmd = ["python3", REMOTE_CONTROL, *parts]
        if self.dry_run:
            cmd.append("--dry-run")
        return ["ssh", *opts, destination, *cmd]

    def _run(self, parts: list[str], *, timeout: float) -> CommandResult:
        argv = self._remote_argv(parts)
        try:
            result = self.runner.run(argv, timeout=timeout)
        except subprocess.TimeoutExpired as exc:
            raise RobotError("로봇 응답이 늦습니다. 카메라 스트림이 열려 있으면 잠시 후 다시 눌러 보세요.") from exc
        except Exception as exc:
            raise RobotError("로봇 명령을 보내지 못했습니다.") from exc
        if result.returncode != 0:
            raise RobotError("로봇 명령이 실패했습니다.")
        return result


def _parse_probe(stdout: str) -> dict[str, Any]:
    text = (stdout or "").strip()
    if not text:
        return {}
    try:
        data = json.loads(text)
    except json.JSONDecodeError:
        return {}
    if not isinstance(data, dict):
        return {}
    battery = data.get("battery_mv")
    if isinstance(battery, bool) or not isinstance(battery, int):
        data.pop("battery_mv", None)
    return data
