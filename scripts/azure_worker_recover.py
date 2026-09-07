#!/usr/bin/env python3
"""Recover the production UGRP MuJoCo worker on the dedicated Azure GPU VM.

Azure is an explicitly opt-in provider.  Power control is delegated to the
already-authenticated Mac Azure CLI, while worker deployment and verification
run directly from Oracle over SSH.  No Azure credential is stored in this
repository or copied to the GPU VM.
"""
from __future__ import annotations

import fcntl
import hashlib
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import tarfile
import tempfile
import time
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from scripts.gpu_worker_bundle import BUNDLE_FILES
from scripts.lightning_worker_recover import make_package
from sim.worker_contract import REMOTE_WORKER_CONTRACT

HEALTH = os.environ.get("UGRP_SIM_HEALTH", "http://127.0.0.1:8091/health")
CONTROL_MAC = os.environ.get("UGRP_AZURE_CONTROL_MAC", "changmin@100.98.90.69")
SUBSCRIPTION = os.environ.get("UGRP_AZURE_SUBSCRIPTION", "UGRP GPU PAYG")
RG = os.environ.get("UGRP_AZURE_RG", "ugrp-gpu-rg")
VM = os.environ.get("UGRP_AZURE_VM", "ugrp-a10-sim")
SSH_USER = os.environ.get("UGRP_AZURE_SSH_USER", "azureuser")
SSH_KEY = Path(os.environ.get("UGRP_AZURE_SSH_KEY", "/home/ubuntu/azure_keys/ugrp-a10-sim_key.pem"))
KNOWN_HOSTS = Path(os.environ.get("UGRP_AZURE_KNOWN_HOSTS", "/home/ubuntu/azure_keys/known_hosts"))
SSH_HOST_OVERRIDE = os.environ.get("UGRP_AZURE_SSH_HOST", "").strip()
MACHINE = os.environ.get("UGRP_AZURE_MACHINE", "NV6ADS_A10_V5").strip().upper() or "NV6ADS_A10_V5"
REMOTE_DIR = "/home/azureuser/ugrp-worker"
REMOTE_VENV = "/home/azureuser/ugrp-venv"
REMOTE_ARCHIVE = "/home/azureuser/ugrp-worker.tgz"
REMOTE_SERVICE = "ugrp-sim-worker.service"
LOCK = Path("/tmp/ugrp_azure_recover.lock")


def health() -> dict:
    try:
        with urlopen(HEALTH, timeout=2) as response:
            obj = json.loads(response.read().decode())
        return obj if isinstance(obj, dict) else {}
    except Exception:
        return {}


def connected() -> bool:
    obj = health()
    return bool(
        obj.get("remote_ws_connected")
        and str(obj.get("remote_provider") or "").lower() == "azure"
        and str(obj.get("remote_worker_contract") or "") == REMOTE_WORKER_CONTRACT
    )


def authoritative() -> bool:
    obj = health()
    return bool(
        obj.get("remote_ws_connected")
        and obj.get("remote_authoritative")
        and str(obj.get("remote_provider") or "").lower() == "azure"
        and str(obj.get("remote_worker_contract") or "") == REMOTE_WORKER_CONTRACT
    )


def _mac_az(command: str, timeout: float = 90.0) -> subprocess.CompletedProcess[str]:
    remote = "export PATH=/opt/homebrew/bin:/usr/local/bin:$PATH; " + command
    return subprocess.run(
        ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5", CONTROL_MAC, remote],
        cwd=ROOT,
        capture_output=True,
        text=True,
        timeout=timeout,
    )


def _az_base() -> str:
    return f"--subscription {shlex.quote(SUBSCRIPTION)}"


def vm_public_ip() -> str:
    if SSH_HOST_OVERRIDE:
        return SSH_HOST_OVERRIDE
    command = (
        f"az vm show -d -g {shlex.quote(RG)} -n {shlex.quote(VM)} {_az_base()} "
        "--query publicIps -o tsv"
    )
    proc = _mac_az(command, timeout=20)
    if proc.returncode:
        return ""
    return (proc.stdout or "").strip().split(",")[0].strip()


def start_vm() -> bool:
    command = f"az vm start -g {shlex.quote(RG)} -n {shlex.quote(VM)} {_az_base()} -o none"
    proc = _mac_az(command, timeout=120)
    if proc.returncode:
        print("azure recovery: VM start failed: " + (proc.stderr or proc.stdout)[-600:], file=sys.stderr)
        return False
    return True


def ssh_base(host: str) -> list[str]:
    return [
        "ssh", "-i", str(SSH_KEY),
        "-o", "BatchMode=yes",
        "-o", "StrictHostKeyChecking=yes",
        "-o", f"UserKnownHostsFile={KNOWN_HOSTS}",
        "-o", "ConnectTimeout=8",
        f"{SSH_USER}@{host}",
    ]


def scp_base(host: str) -> list[str]:
    return [
        "scp", "-q", "-i", str(SSH_KEY),
        "-o", "BatchMode=yes",
        "-o", "StrictHostKeyChecking=yes",
        "-o", f"UserKnownHostsFile={KNOWN_HOSTS}",
        "-o", "ConnectTimeout=8",
    ]


def ssh_ok(host: str) -> bool:
    if not host or not SSH_KEY.exists() or not KNOWN_HOSTS.exists():
        return False
    try:
        proc = subprocess.run(
            ssh_base(host) + ["true"], cwd=ROOT,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=12,
        )
        return proc.returncode == 0
    except (OSError, subprocess.TimeoutExpired):
        return False


def wait_ssh(timeout: float = 180.0) -> str:
    deadline = time.monotonic() + timeout
    last_host = ""
    while time.monotonic() < deadline:
        host = vm_public_ip()
        if host:
            last_host = host
            if ssh_ok(host):
                return host
        time.sleep(4)
    return last_host if ssh_ok(last_host) else ""


def bundle_digest() -> str:
    h = hashlib.sha256()
    for rel in BUNDLE_FILES:
        p = ROOT / rel
        h.update(rel.encode() + b"\0")
        h.update(p.read_bytes())
        h.update(b"\0")
    token = ROOT / ".sim_bridge_token"
    h.update(token.read_bytes() if token.exists() else b"")
    h.update(REMOTE_WORKER_CONTRACT.encode())
    return h.hexdigest()


def make_azure_archive(target: Path, digest: str) -> None:
    with tempfile.TemporaryDirectory(prefix="ugrp-azure-worker-") as td:
        package = Path(td) / "ugrp-worker"
        package.mkdir(parents=True)
        make_package(package, machine=MACHINE)
        launcher = package / "start_worker.sh"
        text = launcher.read_text()
        text = text.replace("export UGRP_GPU_PROVIDER=lightning", "export UGRP_GPU_PROVIDER=azure")
        text = text.replace(
            "exec python3 scripts/run_mujoco_ws_worker.py --seed 11",
            f"exec {REMOTE_VENV}/bin/python scripts/run_mujoco_ws_worker.py --seed 11",
        )
        launcher.write_text(text)
        launcher.chmod(0o700)
        (package / ".ugrp_bundle_digest").write_text(digest + "\n")
        with tarfile.open(target, "w:gz") as tf:
            tf.add(package, arcname="ugrp-worker")


def remote_digest(host: str) -> str:
    try:
        proc = subprocess.run(
            ssh_base(host) + [f"cat {REMOTE_DIR}/.ugrp_bundle_digest 2>/dev/null || true"],
            cwd=ROOT, capture_output=True, text=True, timeout=15,
        )
        return (proc.stdout or "").strip()
    except (OSError, subprocess.TimeoutExpired):
        return ""


def deploy(host: str) -> bool:
    digest = bundle_digest()
    if remote_digest(host) == digest:
        print("azure recovery: production bundle already current", flush=True)
    else:
        with tempfile.TemporaryDirectory(prefix="ugrp-azure-upload-") as td:
            archive = Path(td) / "ugrp-worker.tgz"
            make_azure_archive(archive, digest)
            proc = subprocess.run(
                scp_base(host) + [str(archive), f"{SSH_USER}@{host}:{REMOTE_ARCHIVE}"],
                cwd=ROOT, capture_output=True, text=True, timeout=60,
            )
            if proc.returncode:
                print("azure recovery: bundle upload failed: " + (proc.stderr or proc.stdout)[-600:], file=sys.stderr)
                return False
        install = f"""
set -euo pipefail
rm -rf {REMOTE_DIR}.new
mkdir -p {REMOTE_DIR}.new
tar -xzf {REMOTE_ARCHIVE} -C /home/{SSH_USER}
rm -f {REMOTE_ARCHIVE}
if [ ! -x {REMOTE_VENV}/bin/python ]; then
  python3 -m venv {REMOTE_VENV}
fi
{REMOTE_VENV}/bin/python -m pip install -q --upgrade pip
{REMOTE_VENV}/bin/python -m pip install -q -r {REMOTE_DIR}/requirements.txt
chmod 600 {REMOTE_DIR}/.ugrp_token {REMOTE_DIR}/.ugrp_speed
chmod 700 {REMOTE_DIR}/start_worker.sh {REMOTE_DIR}/supervise_worker.sh
"""
        # The tar already expands atomically enough for this single-worker host;
        # keep the stable directory so systemd paths never change.
        proc = subprocess.run(
            ssh_base(host) + [install], cwd=ROOT,
            capture_output=True, text=True, timeout=240,
        )
        if proc.returncode:
            print("azure recovery: dependency/deploy setup failed: " + (proc.stderr or proc.stdout)[-1200:], file=sys.stderr)
            return False

    unit = f"""[Unit]
Description=UGRP Azure production MuJoCo GPU worker
After=network-online.target nvidia-persistenced.service
Wants=network-online.target

[Service]
Type=simple
User={SSH_USER}
WorkingDirectory={REMOTE_DIR}
Environment=HOME=/home/{SSH_USER}
ExecStart=/bin/bash {REMOTE_DIR}/start_worker.sh
Restart=always
RestartSec=2

[Install]
WantedBy=multi-user.target
"""
    encoded = __import__("base64").b64encode(unit.encode()).decode()
    command = (
        f"echo {shlex.quote(encoded)} | base64 -d | sudo tee /etc/systemd/system/{REMOTE_SERVICE} >/dev/null && "
        "sudo systemctl daemon-reload && "
        f"sudo systemctl enable {REMOTE_SERVICE} >/dev/null && "
        f"sudo systemctl restart {REMOTE_SERVICE}"
    )
    proc = subprocess.run(
        ssh_base(host) + [command], cwd=ROOT,
        capture_output=True, text=True, timeout=30,
    )
    if proc.returncode:
        print("azure recovery: worker service restart failed: " + (proc.stderr or proc.stdout)[-1000:], file=sys.stderr)
        return False
    return True


def main() -> int:
    LOCK.parent.mkdir(parents=True, exist_ok=True)
    with LOCK.open("a+") as fh:
        try:
            fcntl.flock(fh, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            return 0

        if authoritative():
            return 0

        host = vm_public_ip()
        if not ssh_ok(host):
            print("azure recovery: starting dedicated GPU VM", flush=True)
            if not start_vm():
                return 2
            host = wait_ssh(180)
            if not host:
                print("azure recovery: VM started but SSH did not become ready", file=sys.stderr)
                return 3

        if not deploy(host):
            return 4

        deadline = time.monotonic() + 60
        while time.monotonic() < deadline:
            if connected():
                print("azure recovery: production worker connected", flush=True)
                return 0
            time.sleep(1)
        print("azure recovery: worker service started but bridge did not observe Azure", file=sys.stderr)
        return 5


if __name__ == "__main__":
    raise SystemExit(main())
