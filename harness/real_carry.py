"""Read-only causal carry evidence for the physical MasterPi.

This module never actuates the robot.  REAL precision pick writes a short-lived
Pi-side handoff after the pickup site is visually clear and the gripper was
commanded closed.  Oracle may remember ``PROBABLE_HELD`` only while that causal
handoff is still valid.  Positive ``HELD`` evidence, if hardware ever provides
it, is intentionally outside this weak-evidence reconciliation path.
"""
from __future__ import annotations

import json
import subprocess
import time
from dataclasses import dataclass, asdict
from typing import Callable

from harness.pi_camera import _load_deploy

CARRY_PATH = "/tmp/ugrp-red-carry-handoff.json"
CARRY_MAX_AGE_SECONDS = 600.0
GRIPPER_CLOSED_MAX_PULSE = 1700
CARRY_CMD = f"cat {CARRY_PATH} 2>/dev/null || true"


@dataclass(frozen=True)
class CarryEvidence:
    known: bool
    valid: bool
    reason: str
    target_color: str | None = None
    age_s: float | None = None
    gripper_pulse: int | None = None

    def public(self) -> dict:
        return asdict(self)


def parse_carry_evidence(raw: str | bytes, *, now: float | None = None) -> CarryEvidence:
    if isinstance(raw, bytes):
        raw = raw.decode("utf-8", errors="replace")
    text = raw.strip()
    if not text:
        return CarryEvidence(True, False, "missing")
    try:
        payload = json.loads(text)
    except (TypeError, ValueError):
        return CarryEvidence(True, False, "unreadable")
    if not isinstance(payload, dict):
        return CarryEvidence(True, False, "invalid_payload")
    if payload.get("version") != 1 or payload.get("source") != "precision_pick_floor_clear":
        return CarryEvidence(True, False, "unsupported_provenance")
    if payload.get("grasp_state") != "PROBABLE_HELD":
        return CarryEvidence(True, False, "unsupported_grasp_state")
    color = str(payload.get("target_color") or "").lower()
    if color not in {"red", "blue", "yellow"}:
        return CarryEvidence(True, False, "invalid_target_color")
    created = payload.get("created_at")
    if isinstance(created, bool) or not isinstance(created, (int, float)):
        return CarryEvidence(True, False, "invalid_timestamp", target_color=color)
    wall = time.time() if now is None else float(now)
    age = wall - float(created)
    pulse = payload.get("gripper_pulse")
    if isinstance(pulse, bool) or not isinstance(pulse, int):
        return CarryEvidence(True, False, "invalid_gripper_command", target_color=color, age_s=age)
    if pulse > GRIPPER_CLOSED_MAX_PULSE:
        return CarryEvidence(True, False, "gripper_not_closed", target_color=color, age_s=age, gripper_pulse=pulse)
    if age < -2.0:
        return CarryEvidence(True, False, "future_timestamp", target_color=color, age_s=age, gripper_pulse=pulse)
    if age > CARRY_MAX_AGE_SECONDS:
        return CarryEvidence(True, False, "stale", target_color=color, age_s=age, gripper_pulse=pulse)
    return CarryEvidence(True, True, "fresh_precision_pick", target_color=color, age_s=age, gripper_pulse=pulse)


def read_carry_evidence(
    *,
    host: str = "ugrp1",
    ssh_run: Callable[..., subprocess.CompletedProcess] | None = None,
    ssh_args: tuple[str, list[str]] | None = None,
    now: float | None = None,
) -> CarryEvidence:
    """Read the Pi handoff over SSH; unknown transport state never clears carry."""
    runner = ssh_run or subprocess.run
    if ssh_args is None:
        deploy = _load_deploy()
        destination, opts = deploy.ssh_connection_args(host)
    else:
        destination, opts = ssh_args
    try:
        result = runner(
            ["ssh", *opts, destination, CARRY_CMD],
            check=False,
            capture_output=True,
            timeout=5,
        )
    except (OSError, subprocess.TimeoutExpired):
        return CarryEvidence(False, False, "transport_unavailable")
    if result.returncode != 0:
        return CarryEvidence(False, False, "transport_failed")
    return parse_carry_evidence(result.stdout, now=now)


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description="Read-only MasterPi probable-carry evidence")
    ap.add_argument("--host", default="ugrp1")
    args = ap.parse_args()
    print(json.dumps(read_carry_evidence(host=args.host).public(), ensure_ascii=False, sort_keys=True))
