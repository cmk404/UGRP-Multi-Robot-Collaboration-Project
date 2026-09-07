#!/usr/bin/env python3
"""Recover the production UGRP MuJoCo WebSocket worker on Lightning Studio."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
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
LIGHTNING_CLI = ROOT / ".venv-lightning" / "bin" / "lightning"
TOKEN_FILE = ROOT / ".sim_bridge_token"
BRIDGE_HEALTH = os.environ.get("UGRP_SIM_HEALTH", "http://127.0.0.1:8091/health")
BRIDGE_SPEED = os.environ.get("UGRP_SIM_SPEED_URL", "http://127.0.0.1:8091/sim/speed")
WS_URL = os.environ.get("UGRP_SIM_WS_URL", "wss://instance-20260627-1243.taileb87bd.ts.net:8443")
DEFAULT_STUDIO = os.environ.get("UGRP_LIGHTNING_STUDIO", "ugrp-sim")
DEFAULT_TEAMSPACE = os.environ.get("UGRP_LIGHTNING_TEAMSPACE", "").strip() or os.environ.get("LIGHTNING_TEAMSPACE", "").strip()
DEFAULT_MACHINE = os.environ.get("UGRP_LIGHTNING_MACHINE", "T4")
DEFAULT_MACHINES = tuple(x.strip() for x in os.environ.get("UGRP_LIGHTNING_MACHINES", DEFAULT_MACHINE).split(",") if x.strip()) or (DEFAULT_MACHINE,)
REMOTE_DIR = "ugrp-worker"

try:
    from scripts.gpu_worker_bundle import BUNDLE_FILES, REQUIREMENTS
except ModuleNotFoundError:  # direct script execution
    from gpu_worker_bundle import BUNDLE_FILES, REQUIREMENTS
from sim.worker_contract import REMOTE_WORKER_CONTRACT


def run_local(args: list[str], *, timeout: float = 30.0) -> subprocess.CompletedProcess[str]:
    return subprocess.run(args, cwd=ROOT, capture_output=True, text=True, timeout=timeout)


def credentials_available() -> bool:
    if not LIGHTNING_CLI.exists():
        return False
    try:
        proc = run_local([str(LIGHTNING_CLI), "auth", "whoami"], timeout=15)
    except (OSError, subprocess.TimeoutExpired):
        return False
    return proc.returncode == 0


def bridge_remote_connected() -> bool:
    try:
        with urlopen(BRIDGE_HEALTH, timeout=2.0) as response:
            obj = json.loads(response.read().decode())
        return bool(
            obj.get("remote_ws_connected")
            and obj.get("remote_authoritative")
            and str(obj.get("remote_provider") or "").strip().lower() == 'lightning'
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


def make_package(target: Path, machine: str = DEFAULT_MACHINE) -> None:
    if not TOKEN_FILE.exists() or not TOKEN_FILE.read_text().strip():
        raise RuntimeError("simulation bridge token is missing")
    for rel in BUNDLE_FILES:
        source = ROOT / rel
        if not source.exists():
            raise FileNotFoundError(source)
        dest = target / rel
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, dest)
    # Minimal package marker: the production GPU worker only needs
    # harness.real_geometry. The full harness/__init__.py imports web/catalog
    # modules that intentionally are not shipped to the GPU bundle.
    harness_init = target / "harness" / "__init__.py"
    harness_init.parent.mkdir(parents=True, exist_ok=True)
    harness_init.write_text("")
    (target / "requirements.txt").write_text("\n".join(REQUIREMENTS) + "\n")
    token = target / ".ugrp_token"
    token.write_text(TOKEN_FILE.read_text().strip() + "\n")
    token.chmod(0o600)
    speed = target / ".ugrp_speed"
    speed.write_text(f"{current_speed()}\n")
    speed.chmod(0o600)
    launcher = target / "start_worker.sh"
    launcher.write_text(
        "#!/usr/bin/env bash\n"
        "set -euo pipefail\n"
        "cd \"$(dirname \"$0\")\"\n"
        "export MUJOCO_GL=egl\n"
        "export PYTHONPATH=\"$PWD\"\n"
        f"export UGRP_SIM_WS_URL={WS_URL!r}\n"
        "export UGRP_SIM_TOKEN=\"$(cat .ugrp_token)\"\n"
        "export UGRP_SIM_SPEED_FILE=\"$PWD/.ugrp_speed\"\n"
        "export UGRP_GPU_PROVIDER=lightning\n"
        f"export UGRP_GPU_MACHINE={machine!r}\n"
        "export UGRP_PICK_REQUIRE_BILATERAL=1\n"
        "export UGRP_GENTLE_PLACE_TRANSPORT=1\n"
        "exec python3 scripts/run_mujoco_ws_worker.py --seed 11\n"
    )
    launcher.chmod(0o700)
    supervisor = target / "supervise_worker.sh"
    supervisor.write_text(
        "#!/usr/bin/env bash\n"
        "set +e\n"
        "cd \"$(dirname \"$0\")\"\n"
        "while true; do\n"
        "  bash start_worker.sh\n"
        "  rc=$?\n"
        "  echo \"UGRP GPU worker exited rc=$rc; restarting\" >&2\n"
        "  sleep 1\n"
        "done\n"
    )
    supervisor.chmod(0o700)


def wait_remote(timeout: float = 45.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if bridge_remote_connected():
            return True
        time.sleep(0.5)
    return False


def launch_supervisor(studio) -> None:
    """Launch the worker through Lightning's real detached-command API.

    Do not emulate detachment with `nohup ... &` inside Studio.run(): the Studio
    command shell may retain/clean up the child and Studio.run waits for command
    completion. run_and_detach submits the command with detached=True in the
    Lightning API, so its lifetime is explicitly independent of this SDK call.
    """
    studio.run("pkill -f '[s]upervise_worker.sh' || true; pkill -f '[r]un_mujoco_ws_worker.py' || true")
    studio.run_and_detach(
        f"cd {REMOTE_DIR} && exec bash supervise_worker.sh >> worker_supervisor.log 2>&1",
        timeout=2.0,
        check_interval=0.5,
    )


def fast_restart_existing(studio, timeout: float = 12.0) -> bool:
    """Recover a dead worker inside an already-running Studio without redeploy."""
    try:
        if str(getattr(studio, "status", "")).strip().lower() != "running":
            return False
        _out, code = studio.run_with_exit_code(
            f"cd {REMOTE_DIR} && test -f start_worker.sh && test -f supervise_worker.sh && "
            "python3 -c 'import mujoco,numpy,PIL,websockets,cv2'"
        )
        if code:
            return False
        print(f"lightning recovery: fast-restarting existing bundle on {getattr(studio, 'machine', 'GPU')}", flush=True)
        launch_supervisor(studio)
        return wait_remote(timeout)
    except Exception as exc:
        print(f"lightning recovery: fast restart unavailable: {type(exc).__name__}: {exc}", flush=True)
        return False


def main() -> int:
    ap = argparse.ArgumentParser(description="Deploy/recover UGRP GPU worker on Lightning")
    ap.add_argument("--studio", default=DEFAULT_STUDIO)
    ap.add_argument("--machine", default=None, help="single machine override; otherwise UGRP_LIGHTNING_MACHINES order")
    ap.add_argument("--force", action="store_true")
    ap.add_argument("--preflight", action="store_true")
    ap.add_argument("--auth-prechecked", action="store_true", help=argparse.SUPPRESS)
    args = ap.parse_args()

    if not args.auth_prechecked and not credentials_available():
        print("lightning recovery unavailable: authentication required (`lightning login`)", flush=True)
        return 3
    if args.preflight:
        machines = [args.machine] if args.machine else list(DEFAULT_MACHINES)
        print(json.dumps({"provider": "lightning", "configured": True, "machines": machines}))
        return 0
    if bridge_remote_connected() and not args.force:
        print("lightning recovery: remote GPU already connected", flush=True)
        return 0

    try:
        from lightning_sdk import Studio
        machines = [args.machine] if args.machine else list(DEFAULT_MACHINES)
        failures = []
        for machine in machines:
            try:
                studio = Studio(args.studio, teamspace=DEFAULT_TEAMSPACE or None, create_ok=True)
                if not args.force and fast_restart_existing(studio):
                    print(f"lightning recovery: existing {getattr(studio, 'machine', machine)} worker reconnected", flush=True)
                    return 0
                with tempfile.TemporaryDirectory(prefix="ugrp-lightning-worker-") as td:
                    package = Path(td) / REMOTE_DIR
                    package.mkdir(parents=True)
                    make_package(package, machine)
                    print(f"lightning recovery: starting {args.studio} on {machine}", flush=True)
                    studio.start(machine)
                    archive = Path(td) / f"{REMOTE_DIR}.tgz"
                    with tarfile.open(archive, "w:gz") as tf:
                        tf.add(package, arcname=REMOTE_DIR)
                    studio.upload_file(str(archive), remote_path=f"{REMOTE_DIR}.tgz", progress_bar=False)
                    studio.run(
                        f"rm -rf {REMOTE_DIR} && tar -xzf {REMOTE_DIR}.tgz && rm -f {REMOTE_DIR}.tgz"
                    )
                    out, code = studio.run_with_exit_code(
                        f"cd {REMOTE_DIR} && chmod 600 .ugrp_token .ugrp_speed && "
                        "(python3 -c 'import mujoco,numpy,PIL,websockets,cv2' >/dev/null 2>&1 || python3 -m pip install -q -r requirements.txt)"
                    )
                    if code:
                        raise RuntimeError(f"Lightning dependency setup failed: {out[-1000:]}")
                    launch_supervisor(studio)
                if wait_remote():
                    print(f"lightning recovery: remote GPU connected on {machine}", flush=True)
                    return 0
                raise RuntimeError("worker launched but bridge did not observe its WebSocket")
            except Exception as exc:
                failures.append(f"{machine}: {type(exc).__name__}: {exc}")
                print(f"lightning recovery: {machine} failed: {type(exc).__name__}: {exc}", flush=True)
        raise RuntimeError("; ".join(failures))
    except Exception as exc:
        print(f"lightning recovery failed: {type(exc).__name__}: {exc}", flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
