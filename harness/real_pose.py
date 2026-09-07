"""Read-only commanded-pose reader for the physical MasterPi."""

from __future__ import annotations

import json
import subprocess
import time
from dataclasses import dataclass
from typing import Any, Callable

from harness.pi_camera import _load_deploy

POSE_CMD = "cat /tmp/ugrp-masterpi-pose.json 2>/dev/null || true"


@dataclass(frozen=True)
class CommandedPose:
    pose: dict[int, int]
    updated_at: float | None
    age_s: float | None
    last_writer: str | None
    last_servo: int | None
    last_pulse: int | None


def parse_commanded_pose(raw: str | bytes, *, now: float | None = None) -> CommandedPose | None:
    if isinstance(raw, bytes):
        raw = raw.decode("utf-8", errors="replace")
    try:
        data = json.loads(raw.strip())
    except (ValueError, TypeError):
        return None
    if not isinstance(data, dict) or not isinstance(data.get("pose"), dict):
        return None
    pose: dict[int, int] = {}
    for key, value in data["pose"].items():
        try:
            servo, pulse = int(key), int(value)
        except (TypeError, ValueError):
            continue
        if servo in {1, 3, 4, 5, 6} and 400 <= pulse <= 2600:
            pose[servo] = pulse
    if not pose:
        return None
    try:
        updated_at = float(data["updated_at"])
    except (KeyError, TypeError, ValueError):
        updated_at = None
    wall = time.time() if now is None else now
    age_s = None if updated_at is None else max(0.0, wall - updated_at)
    try:
        last_servo = int(data["last_servo"])
    except (KeyError, TypeError, ValueError):
        last_servo = None
    try:
        last_pulse = int(data["last_pulse"])
    except (KeyError, TypeError, ValueError):
        last_pulse = None
    return CommandedPose(
        pose=pose,
        updated_at=updated_at,
        age_s=age_s,
        last_writer=str(data.get("last_writer")) if data.get("last_writer") else None,
        last_servo=last_servo,
        last_pulse=last_pulse,
    )


def read_commanded_pose(
    *,
    host: str = "ugrp1",
    ssh_run: Callable[..., subprocess.CompletedProcess] | None = None,
    ssh_args: tuple[str, list[str]] | None = None,
) -> CommandedPose | None:
    runner = ssh_run or subprocess.run
    if ssh_args is None:
        deploy = _load_deploy()
        destination, opts = deploy.ssh_connection_args(host)
    else:
        destination, opts = ssh_args
    try:
        result = runner(
            ["ssh", *opts, destination, POSE_CMD],
            check=False,
            capture_output=True,
            timeout=5,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    if result.returncode != 0:
        return None
    return parse_commanded_pose(result.stdout)


class PoseStabilityTracker:
    """Require two equal commanded poses before metric projection is trusted."""

    def __init__(self, *, min_settle_s: float = 0.18) -> None:
        self.min_settle_s = min_settle_s
        self._last: tuple[tuple[int, int], ...] | None = None
        self._same_count = 0

    def observe(self, value: CommandedPose | None) -> bool:
        if value is None:
            self._last = None
            self._same_count = 0
            return False
        key = tuple(sorted(value.pose.items()))
        if key == self._last:
            self._same_count += 1
        else:
            self._last = key
            self._same_count = 1
        age_ok = value.age_s is None or value.age_s >= self.min_settle_s
        return self._same_count >= 2 and age_ok


POSE_STREAM_CMD = r"while true; do cat /tmp/ugrp-masterpi-pose.json 2>/dev/null || true; printf '\n'; sleep 0.10; done"


def open_commanded_pose_stream(
    *,
    host: str = "ugrp1",
    ssh_popen: Callable[..., subprocess.Popen] | None = None,
    ssh_args: tuple[str, list[str]] | None = None,
) -> subprocess.Popen:
    """Open one persistent, read-only pose telemetry SSH process.

    This avoids paying a fresh SSH handshake for every camera frame.  The
    remote command only cats the commanded-pose JSON file; it never touches
    motors, servos, services, or the actuator lease.
    """
    popen = ssh_popen or subprocess.Popen
    if ssh_args is None:
        deploy = _load_deploy()
        destination, opts = deploy.ssh_connection_args(host)
    else:
        destination, opts = ssh_args
    return popen(
        ["ssh", *opts, destination, POSE_STREAM_CMD],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
        bufsize=1,
    )
