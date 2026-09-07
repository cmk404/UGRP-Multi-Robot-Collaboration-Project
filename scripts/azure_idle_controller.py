#!/usr/bin/env python3
"""Cost guard for the dedicated Azure UGRP GPU worker.

The preferred deallocation path asks the running VM to deallocate itself through
its system-assigned managed identity.  This keeps the safety guard independent
of an interactive Azure login.  The authenticated Mac Azure CLI is retained as
a fallback control plane.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shlex
import subprocess
from urllib.request import urlopen

HEALTH = os.environ.get("UGRP_SIM_HEALTH", "http://127.0.0.1:8091/health")
CONTROL_MAC = os.environ.get("UGRP_AZURE_CONTROL_MAC", "changmin@100.98.90.69")
SUBSCRIPTION = os.environ.get("UGRP_AZURE_SUBSCRIPTION", "UGRP GPU PAYG")
RG = os.environ.get("UGRP_AZURE_RG", "ugrp-gpu-rg")
VM = os.environ.get("UGRP_AZURE_VM", "ugrp-a10-sim")
SSH_USER = os.environ.get("UGRP_AZURE_SSH_USER", "azureuser")
SSH_HOST = os.environ.get("UGRP_AZURE_SSH_HOST", "").strip()
SSH_KEY = Path(os.environ.get("UGRP_AZURE_SSH_KEY", "/home/ubuntu/azure_keys/ugrp-a10-sim_key.pem"))
KNOWN_HOSTS = Path(os.environ.get("UGRP_AZURE_KNOWN_HOSTS", "/home/ubuntu/azure_keys/known_hosts"))
IDLE = float(os.environ.get("UGRP_AZURE_IDLE_SECONDS", "900"))


def health() -> dict:
    try:
        with urlopen(HEALTH, timeout=2) as response:
            obj = json.loads(response.read())
        return obj if isinstance(obj, dict) else {}
    except Exception:
        return {}


def mac_az(command: str, timeout: float = 30.0) -> subprocess.CompletedProcess[str]:
    remote = "export PATH=/opt/homebrew/bin:/usr/local/bin:$PATH; " + command
    return subprocess.run(
        ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5", CONTROL_MAC, remote],
        capture_output=True,
        text=True,
        timeout=timeout,
    )


def public_ip() -> str:
    if SSH_HOST:
        return SSH_HOST
    command = (
        f"az vm show -d -g {shlex.quote(RG)} -n {shlex.quote(VM)} "
        f"--subscription {shlex.quote(SUBSCRIPTION)} --query publicIps -o tsv"
    )
    try:
        proc = mac_az(command, timeout=15)
    except (OSError, subprocess.TimeoutExpired):
        return ""
    return (proc.stdout or "").strip().split(",")[0].strip() if proc.returncode == 0 else ""


def self_deallocate() -> bool:
    host = public_ip()
    if not host or not SSH_KEY.exists() or not KNOWN_HOSTS.exists():
        return False
    cmd = [
        "ssh", "-i", str(SSH_KEY),
        "-o", "BatchMode=yes",
        "-o", "StrictHostKeyChecking=yes",
        "-o", f"UserKnownHostsFile={KNOWN_HOSTS}",
        "-o", "ConnectTimeout=8",
        f"{SSH_USER}@{host}",
        "sudo /usr/local/sbin/ugrp-self-deallocate",
    ]
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=25)
    except (OSError, subprocess.TimeoutExpired):
        return False
    text = (proc.stdout or "") + (proc.stderr or "")
    return proc.returncode == 0 or "DEALLOCATE_REQUESTED" in text


def mac_deallocate() -> bool:
    command = (
        f"az vm deallocate -g {shlex.quote(RG)} -n {shlex.quote(VM)} "
        f"--subscription {shlex.quote(SUBSCRIPTION)} --no-wait -o none"
    )
    try:
        proc = mac_az(command, timeout=20)
    except (OSError, subprocess.TimeoutExpired):
        return False
    if proc.returncode:
        print("Azure deallocate fallback unavailable: " + (proc.stderr or proc.stdout)[-500:])
        return False
    return True


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    h = health()
    if not h.get("remote_ws_connected") or str(h.get("remote_provider") or "").lower() != "azure":
        return 0
    if h.get("inflight") or int(h.get("queue_depth") or 0) > 0:
        return 0
    age = float(h.get("user_activity_age_s") or 0.0)
    if age < IDLE:
        return 0
    if args.dry_run:
        print(f"WOULD_DEALLOCATE age_s={age:.1f} threshold_s={IDLE:.0f}")
        return 0

    if self_deallocate():
        print(f"Azure self-deallocate requested after {age:.0f}s idle", flush=True)
        return 0
    if mac_deallocate():
        print(f"Azure deallocate requested through fallback control plane after {age:.0f}s idle", flush=True)
        return 0
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
