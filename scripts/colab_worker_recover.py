#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import tarfile
import tempfile
import time
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
COLAB = Path(os.environ.get("UGRP_COLAB_CLI", "/home/ubuntu/.local/bin/colab"))
BRIDGE_HEALTH = os.environ.get("UGRP_SIM_HEALTH", "http://127.0.0.1:8091/health")
BRIDGE_SPEED = os.environ.get("UGRP_SIM_SPEED_URL", "http://127.0.0.1:8091/sim/speed")
TOKEN_FILE = ROOT / ".sim_bridge_token"
DEFAULT_SESSION = os.environ.get("UGRP_COLAB_SESSION", "ugrp-gpu")
DEFAULT_GPU = os.environ.get("UGRP_COLAB_GPU", "T4")
WS_URL = os.environ.get(
    "UGRP_SIM_WS_URL",
    "wss://instance-20260627-1243.taileb87bd.ts.net:8443",
)

try:
    from scripts.gpu_worker_bundle import BUNDLE_FILES, REQUIREMENTS
except ModuleNotFoundError:  # direct script execution
    from gpu_worker_bundle import BUNDLE_FILES, REQUIREMENTS
from sim.worker_contract import REMOTE_WORKER_CONTRACT


def run_cmd(args: list[str], *, timeout: float = 120.0, check: bool = True) -> subprocess.CompletedProcess[str]:
    proc = subprocess.run(args, cwd=ROOT, capture_output=True, text=True, timeout=timeout)
    if check and proc.returncode != 0:
        detail = (proc.stderr or proc.stdout or "command failed").strip()
        raise RuntimeError(f"{Path(args[0]).name} command failed: {detail[-1200:]}")
    return proc


def bridge_remote_connected() -> bool:
    try:
        with urlopen(BRIDGE_HEALTH, timeout=2.0) as response:
            obj = json.loads(response.read().decode())
        return bool(
            obj.get("remote_ws_connected")
            and obj.get("remote_authoritative")
            and str(obj.get("remote_provider") or "").strip().lower() == 'colab'
            and str(obj.get("remote_worker_contract") or "").strip() == REMOTE_WORKER_CONTRACT
        )
    except Exception:
        return False


def current_speed() -> int:
    try:
        with urlopen(BRIDGE_SPEED, timeout=2.0) as response:
            value = int(json.loads(response.read().decode()).get("sim_speed", 1))
        return value if value in (1, 2, 3) else 1
    except Exception:
        return 1


def parse_sessions(text: str) -> set[str]:
    return set(re.findall(r"^\[([^\]]+)\]", text, flags=re.MULTILINE))


def active_sessions() -> set[str]:
    proc = run_cmd([str(COLAB), "sessions"], timeout=30, check=False)
    return parse_sessions((proc.stdout or "") + "\n" + (proc.stderr or ""))


def ensure_session(session: str, gpu: str) -> bool:
    """Return True when a new runtime was created."""
    if session in active_sessions():
        return False
    print(f"colab recovery: creating {gpu} session {session}", flush=True)
    run_cmd([str(COLAB), "new", "-s", session, "--gpu", gpu], timeout=180)
    deadline = time.monotonic() + 90
    while time.monotonic() < deadline:
        if session in active_sessions():
            return True
        time.sleep(3)
    raise RuntimeError(f"Colab session {session!r} did not become active")


def make_bundle(path: Path) -> None:
    with tarfile.open(path, "w:gz") as archive:
        for rel in BUNDLE_FILES:
            source = ROOT / rel
            if not source.exists():
                raise FileNotFoundError(source)
            archive.add(source, arcname=rel)


def upload(session: str, local: Path, remote: str) -> None:
    run_cmd([str(COLAB), "upload", "-s", session, str(local), remote], timeout=90)


def write_launcher(path: Path, ws_url: str, gpu: str = DEFAULT_GPU) -> None:
    # Secrets are read from the uploaded token file inside Colab and are never
    # embedded in this generated launcher or printed to recovery logs.
    code = f'''\
import os, shutil, subprocess, tarfile, time
from pathlib import Path
root=Path('/content/ugrp')
shutil.rmtree(root, ignore_errors=True)
root.mkdir(parents=True, exist_ok=True)
with tarfile.open('/ugrp_colab_latest.tgz','r:gz') as tf:
    tf.extractall(root)
(root/'harness').mkdir(parents=True,exist_ok=True)
(root/'harness/__init__.py').write_text('')
subprocess.run("pkill -f 'scripts/run_mujoco_ws_worker.py'", shell=True, check=False)
env=os.environ.copy()
env.update({{
    'MUJOCO_GL':'egl',
    'PYTHONPATH':str(root),
    'UGRP_SIM_WS_URL':{ws_url!r},
    'UGRP_SIM_TOKEN':Path('/ugrp_token').read_text().strip(),
    'UGRP_SIM_SPEED_FILE':'/ugrp_sim_speed',
    'UGRP_GPU_PROVIDER':'colab',
    'UGRP_GPU_MACHINE':{gpu!r},
    'UGRP_PICK_REQUIRE_BILATERAL':'1',
    'UGRP_GENTLE_PLACE_TRANSPORT':'1',
}})
log=open('/content/ugrp_ws_worker.log','ab', buffering=0)
p=subprocess.Popen(
    ['python3','scripts/run_mujoco_ws_worker.py','--seed','11'],
    cwd=root, env=env, stdout=log, stderr=subprocess.STDOUT,
    start_new_session=True,
)
Path('/content/ugrp_ws_worker.pid').write_text(str(p.pid))
time.sleep(2)
print({{'worker_pid':p.pid,'alive':p.poll() is None}})
'''
    path.write_text(code)
    path.chmod(0o600)


def deploy(session: str, gpu: str = DEFAULT_GPU) -> None:
    if not TOKEN_FILE.exists() or not TOKEN_FILE.read_text().strip():
        raise RuntimeError("simulation bridge token is missing")
    with tempfile.TemporaryDirectory(prefix="ugrp-colab-recover-") as temp_dir:
        temp = Path(temp_dir)
        bundle = temp / "ugrp_colab_latest.tgz"
        speed_file = temp / "ugrp_sim_speed"
        launcher = temp / "launch.py"
        make_bundle(bundle)
        speed_file.write_text(f"{current_speed()}\n")
        speed_file.chmod(0o600)
        write_launcher(launcher, WS_URL, gpu)
        upload(session, bundle, "/ugrp_colab_latest.tgz")
        upload(session, TOKEN_FILE, "/ugrp_token")
        upload(session, speed_file, "/ugrp_sim_speed")
        # Idempotent on a live runtime, required after Colab reclaimed the VM.
        run_cmd(
            [str(COLAB), "install", "-s", session, *REQUIREMENTS],
            timeout=180,
        )
        result = run_cmd(
            [str(COLAB), "exec", "-s", session, "-f", str(launcher), "--timeout", "60"],
            timeout=90,
        )
        if "'alive': True" not in result.stdout and '"alive": true' not in result.stdout.lower():
            print("colab recovery: launcher returned without explicit alive marker", flush=True)


def wait_remote(timeout: float = 35.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if bridge_remote_connected():
            return True
        time.sleep(1)
    return False


def main() -> int:
    ap = argparse.ArgumentParser(description="Recreate/redeploy the UGRP Colab GPU worker safely")
    ap.add_argument("--session", default=DEFAULT_SESSION)
    ap.add_argument("--gpu", default=DEFAULT_GPU)
    ap.add_argument("--force", action="store_true", help="redeploy even if a remote worker is already connected")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    if bridge_remote_connected() and not args.force:
        print("colab recovery: remote GPU already connected", flush=True)
        return 0
    if args.dry_run:
        print({"session": args.session, "gpu": args.gpu, "active_sessions": sorted(active_sessions())})
        return 0

    created = ensure_session(args.session, args.gpu)
    print(f"colab recovery: session ready (created={created})", flush=True)
    deploy(args.session, args.gpu)
    if not wait_remote():
        raise RuntimeError("Colab worker launch completed but bridge never observed remote WebSocket")
    print("colab recovery: remote GPU connected", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
