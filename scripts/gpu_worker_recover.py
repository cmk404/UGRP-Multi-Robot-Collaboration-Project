#!/usr/bin/env python3
"""Recover the current Mac MuJoCo worker used by the simulation bridge.

Cloud simulation providers are retired. This compatibility entry point now
owns only the Mac worker path used through the existing bridge.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from sim.worker_contract import REMOTE_WORKER_CONTRACT

MAC = ROOT / "scripts" / "mac_worker_recover.py"
HEALTH = os.environ.get("UGRP_SIM_HEALTH", "http://127.0.0.1:8091/health")
STATUS_FILE = Path(os.environ.get("UGRP_GPU_RECOVERY_STATUS", "/tmp/ugrp_gpu_recovery_status.json"))


def provider_order() -> list[str]:
    """Return the sole supported worker location."""
    return ["mac"]


def provider_configured(name: str) -> bool:
    return name == "mac" and os.environ.get("UGRP_ALLOW_MAC_WORKER") == "1" and MAC.exists()


def configured_providers() -> list[str]:
    return ["mac"] if provider_configured("mac") else []


def remote_health() -> dict:
    try:
        with urlopen(HEALTH, timeout=2) as response:
            obj = json.loads(response.read().decode())
        return obj if isinstance(obj, dict) else {}
    except Exception:
        return {}


def remote_ready(provider: str | None = None) -> bool:
    obj = remote_health()
    if not (obj.get("remote_ws_connected") and obj.get("remote_authoritative")):
        return False
    if str(obj.get("remote_worker_contract") or "").strip() != REMOTE_WORKER_CONTRACT:
        return False
    actual = str(obj.get("remote_provider") or "").strip().lower()
    return actual == "mac" and (provider is None or provider.strip().lower() == "mac")


def wait_remote_ready(timeout: float = 15.0) -> bool:
    deadline = time.monotonic() + max(0.0, timeout)
    while time.monotonic() < deadline:
        if remote_ready("mac"):
            return True
        time.sleep(0.25)
    return remote_ready("mac")


def command_for(name: str) -> list[str]:
    if name != "mac":
        raise ValueError(name)
    return [sys.executable, str(MAC)]


def write_status(**data) -> None:
    try:
        STATUS_FILE.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n")
    except OSError:
        pass


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--preflight", action="store_true")
    args = parser.parse_args()
    configured = configured_providers()
    if args.preflight:
        print(json.dumps({"order": provider_order(), "configured": configured}))
        return 0 if configured else 3
    if remote_ready("mac"):
        write_status(ok=True, state="connected", provider="mac", configured=configured)
        return 0
    if not configured:
        detail = "Mac simulation worker recovery is not enabled"
        write_status(
            ok=False,
            state="manual_start_required",
            provider=None,
            configured=[],
            detail=detail,
        )
        print(f"worker recovery: {detail}", flush=True)
        return 0
    try:
        proc = subprocess.run(
            command_for("mac"), cwd=ROOT, capture_output=True, text=True, timeout=300
        )
        detail = ((proc.stdout or "") + (proc.stderr or ""))[-1800:].strip()
    except Exception as exc:
        proc = None
        detail = f"{type(exc).__name__}: {exc}"
    if proc is not None and proc.returncode == 0 and wait_remote_ready():
        write_status(ok=True, state="connected", provider="mac", configured=configured)
        return 0
    write_status(
        ok=False,
        state="recovery_failed",
        provider="mac",
        configured=configured,
        detail=detail[-700:],
    )
    return 4


if __name__ == "__main__":
    raise SystemExit(main())
