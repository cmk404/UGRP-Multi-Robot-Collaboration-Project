#!/usr/bin/env python3
"""Provider-neutral GPU worker recovery for UGRP."""
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
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
LIGHTNING_PY = ROOT / ".venv-lightning" / "bin" / "python"
LIGHTNING_CLI = ROOT / ".venv-lightning" / "bin" / "lightning"
COLAB = Path(os.environ.get("UGRP_COLAB_CLI", "/home/ubuntu/.local/bin/colab"))
AZURE = ROOT / "scripts" / "azure_worker_recover.py"
MAC = ROOT / "scripts" / "mac_worker_recover.py"
AZURE_KEY = Path(os.environ.get("UGRP_AZURE_SSH_KEY", "/home/ubuntu/azure_keys/ugrp-a10-sim_key.pem"))
HEALTH = os.environ.get("UGRP_SIM_HEALTH", "http://127.0.0.1:8091/health")
STATUS_FILE = Path(os.environ.get("UGRP_GPU_RECOVERY_STATUS", "/tmp/ugrp_gpu_recovery_status.json"))


def provider_order() -> list[str]:
    raw = os.environ.get("UGRP_GPU_PROVIDERS", "lightning")
    out: list[str] = []
    for name in raw.split(","):
        name = name.strip().lower()
        if name and name not in out:
            out.append(name)
    return out


def _cmd_ok(args: list[str], timeout: float = 15.0) -> bool:
    try:
        proc = subprocess.run(
            args, cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=timeout
        )
        return proc.returncode == 0
    except (OSError, subprocess.TimeoutExpired):
        return False


def provider_configured(name: str) -> bool:
    if name == "lightning":
        return (
            LIGHTNING_PY.exists()
            and LIGHTNING_CLI.exists()
            and _cmd_ok([str(LIGHTNING_CLI), "auth", "whoami"])
        )
    if name == "azure":
        # Azure is paid capacity, so merely having a key on disk must never make
        # it an automatic provider.  It becomes eligible only after explicit
        # opt-in; the dedicated recovery script owns VM power/deploy handling.
        return (
            os.environ.get("UGRP_ALLOW_AZURE_WORKER") == "1"
            and AZURE.exists()
            and AZURE_KEY.exists()
            and bool(os.environ.get("UGRP_AZURE_SUBSCRIPTION", "UGRP GPU PAYG").strip())
            and bool(os.environ.get("UGRP_AZURE_RG", "ugrp-gpu-rg").strip())
            and bool(os.environ.get("UGRP_AZURE_VM", "ugrp-a10-sim").strip())
        )
    if name == "colab":
        # Colab remains explicitly opt-in. The free managed runtime is not used
        # as an automatic persistent worker just because the CLI happens to exist.
        return os.environ.get("UGRP_ALLOW_COLAB_WORKER") == "1" and COLAB.exists()
    if name == "mac":
        # The Mac is a free local fallback/primary worker reached through Tailscale SSH.
        # Keep it opt-in so unattended deployments never assume the laptop is available.
        return os.environ.get("UGRP_ALLOW_MAC_WORKER") == "1" and MAC.exists()
    return False


def configured_providers() -> list[str]:
    return [name for name in provider_order() if provider_configured(name)]


def remote_health() -> dict:
    try:
        with urlopen(HEALTH, timeout=2) as response:
            obj=json.loads(response.read().decode())
        return obj if isinstance(obj, dict) else {}
    except Exception:
        return {}

def remote_ready(provider: str | None = None) -> bool:
    obj=remote_health()
    if not (obj.get("remote_ws_connected") and obj.get("remote_authoritative")):
        return False
    if str(obj.get("remote_worker_contract") or "").strip() != REMOTE_WORKER_CONTRACT:
        return False
    if provider is None:
        return True
    return str(obj.get("remote_provider") or "").strip().lower() == provider.strip().lower()

def wait_remote_ready(provider: str, timeout: float = 15.0) -> bool:
    deadline=time.monotonic()+max(0.0, timeout)
    while time.monotonic() < deadline:
        if remote_ready(provider):
            return True
        time.sleep(0.25)
    return remote_ready(provider)


def command_for(name: str) -> list[str]:
    if name == "lightning":
        # provider_configured() already performed the CLI auth check in this
        # recovery process; avoid repeating it in the child and wasting outage time.
        return [str(LIGHTNING_PY), str(ROOT / "scripts/lightning_worker_recover.py"), "--auth-prechecked"]
    if name == "azure":
        return ["/usr/bin/python3", str(AZURE)]
    if name == "colab":
        return ["/usr/bin/python3", str(ROOT / "scripts/colab_worker_recover.py")]
    if name == "mac":
        return ["/usr/bin/python3", str(MAC)]
    raise ValueError(name)


def write_status(**data) -> None:
    try:
        STATUS_FILE.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n")
    except OSError:
        pass


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--preflight", action="store_true")
    args = ap.parse_args()
    configured = configured_providers()
    if args.preflight:
        print(json.dumps({"order": provider_order(), "configured": configured}))
        return 0 if configured else 3
    existing=remote_health()
    existing_provider=str(existing.get("remote_provider") or "").strip().lower()
    if remote_ready() and existing_provider in configured:
        write_status(ok=True, state="connected", provider=existing_provider, order=provider_order(), configured=configured, detail="configured remote already connected")
        return 0
    if not configured:
        detail = "no configured automatic GPU provider; simulation remains GPU_OFFLINE"
        print(f"gpu recovery: {detail}", flush=True)
        write_status(ok=False, state="auth_required", provider=None, order=provider_order(), configured=configured, detail=detail)
        return 0

    failures = []
    for provider in configured:
        print(f"gpu recovery: trying {provider}", flush=True)
        try:
            proc = subprocess.run(
                command_for(provider), cwd=ROOT, capture_output=True, text=True, timeout=300
            )
            detail = ((proc.stdout or "") + (proc.stderr or ""))[-1800:].strip()
        except Exception as exc:
            proc = None
            detail = f"{type(exc).__name__}: {exc}"
        if proc is not None and proc.returncode == 0 and wait_remote_ready(provider):
            print(f"gpu recovery: {provider} connected", flush=True)
            write_status(ok=True, state="connected", provider=provider, order=provider_order(), configured=configured, detail="connected")
            return 0
        failures.append({"provider": provider, "detail": detail[-700:]})
        print(f"gpu recovery: {provider} unavailable", flush=True)
    write_status(ok=False, state="recovery_failed", provider=None, order=provider_order(), configured=configured, detail="all configured providers failed", failures=failures)
    return 4


if __name__ == "__main__":
    raise SystemExit(main())
