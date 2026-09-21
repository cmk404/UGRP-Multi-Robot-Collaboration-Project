#!/usr/bin/env python3
"""Recover a configured UGRP MuJoCo worker on macOS through SSH/Tailscale."""
from __future__ import annotations

import json
import os
from pathlib import Path
import shlex
import subprocess
import time
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parents[1]
TOKEN_FILE = ROOT / ".sim_bridge_token"
HEALTH = os.environ.get("UGRP_SIM_HEALTH", "http://127.0.0.1:8091/health")
SPEED_URL = os.environ.get("UGRP_SIM_SPEED_URL", "http://127.0.0.1:8091/sim/speed")

# Deployment-specific network and host values must come from untracked local
# configuration (for example .env.gpu, which is excluded by .gitignore).
WS_URL = os.environ.get("UGRP_SIM_WS_URL", "").strip()
MAC_HOST = os.environ.get("UGRP_MAC_SIM_HOST", "").strip()
MAC_ROOT = os.environ.get("UGRP_MAC_SIM_ROOT", "").strip()
MAC_PYTHON = os.environ.get("UGRP_MAC_SIM_PYTHON", ".venv-sim-worker-mac/bin/python").strip()
LOCAL_MODE = os.environ.get("UGRP_MAC_SIM_LOCAL", "").strip().lower() in {"1", "true", "yes", "on"}
REMOTE_TOKEN = "/tmp/ugrp_sim_bridge_token"
REMOTE_SPEED = "/tmp/ugrp_sim_speed"
REMOTE_PID = "/tmp/ugrp_mac_sim_worker.pid"
REMOTE_LOG = "/tmp/ugrp_mac_sim_worker.log"


def ssh_base() -> list[str]:
    return ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8", MAC_HOST]


def run_ssh(command: str, *, timeout: float = 20.0, stdin: str | None = None) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ssh_base() + [command], input=stdin, capture_output=True, text=True, timeout=timeout
    )


def mac_ready() -> bool:
    if LOCAL_MODE:
        python = ROOT / MAC_PYTHON if not Path(MAC_PYTHON).is_absolute() else Path(MAC_PYTHON)
        if not python.exists():
            return False
        env = os.environ.copy()
        env["PATH"] = "/opt/homebrew/bin:/usr/local/bin:/usr/bin:/bin:/usr/sbin:/sbin"
        try:
            probe = subprocess.run(
                [str(python), "-c", "import mujoco,cv2,websockets,numpy"],
                cwd=ROOT, env=env, capture_output=True, text=True, timeout=12,
            )
            return probe.returncode == 0
        except (OSError, subprocess.TimeoutExpired):
            return False
    if not MAC_HOST or not MAC_ROOT or not MAC_PYTHON:
        return False
    probe = (
        f"cd {shlex.quote(MAC_ROOT)} && "
        f"test -x {shlex.quote(MAC_PYTHON)} && "
        f"{shlex.quote(MAC_PYTHON)} -c 'import mujoco,cv2,websockets,numpy'"
    )
    try:
        return run_ssh(probe, timeout=12).returncode == 0
    except (OSError, subprocess.TimeoutExpired):
        return False


def current_speed() -> int:
    try:
        with urlopen(SPEED_URL, timeout=2.0) as response:
            value = int(json.loads(response.read().decode()).get("sim_speed", 1))
        return value if value in (1, 2, 3) else 1
    except Exception:
        return 1


def bridge_connected() -> bool:
    try:
        with urlopen(HEALTH, timeout=2.0) as response:
            obj = json.loads(response.read().decode())
        return bool(
            obj.get("remote_ws_connected")
            and str(obj.get("remote_provider") or "").strip().lower() == "mac"
        )
    except Exception:
        return False


def install_runtime_inputs() -> bool:
    if not TOKEN_FILE.exists() or not TOKEN_FILE.read_text().strip():
        return False
    if LOCAL_MODE:
        return True
    token = TOKEN_FILE.read_text().strip() + "\n"
    token_cmd = f"umask 077; cat > {shlex.quote(REMOTE_TOKEN)}"
    speed_cmd = f"umask 077; cat > {shlex.quote(REMOTE_SPEED)}"
    try:
        if run_ssh(token_cmd, timeout=12, stdin=token).returncode != 0:
            return False
        if run_ssh(speed_cmd, timeout=12, stdin=f"{current_speed()}\n").returncode != 0:
            return False
    except (OSError, subprocess.TimeoutExpired):
        return False
    return True


def start_worker() -> bool:
    if LOCAL_MODE:
        token = TOKEN_FILE.read_text().strip()
        python = ROOT / MAC_PYTHON if not Path(MAC_PYTHON).is_absolute() else Path(MAC_PYTHON)
        env = os.environ.copy()
        env.update({
            "PATH": "/opt/homebrew/bin:/usr/local/bin:/usr/bin:/bin:/usr/sbin:/sbin",
            "PYTHONPATH": str(ROOT),
            "UGRP_SIM_WS_URL": WS_URL,
            "UGRP_SIM_TOKEN": token,
            "UGRP_GPU_PROVIDER": "mac",
            "UGRP_GPU_MACHINE": "APPLE_M3",
            "UGRP_PICK_REQUIRE_BILATERAL": "1",
            "UGRP_GENTLE_PLACE_TRANSPORT": "1",
        })
        try:
            log = Path("/private/tmp/ugrp_local_worker.log").open("wb")
            proc = subprocess.Popen(
                [str(python), "scripts/run_mujoco_ws_worker.py", "--seed", "11"],
                cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT,
                stdin=subprocess.DEVNULL, start_new_session=True, close_fds=True,
            )
            Path("/private/tmp/ugrp_local_worker.pid").write_text(str(proc.pid))
            time.sleep(2.0)
            alive = proc.poll() is None
            log.close()
            return alive
        except OSError:
            return False
    root = shlex.quote(MAC_ROOT)
    python = shlex.quote(MAC_PYTHON)
    ws_url = shlex.quote(WS_URL)
    command = f"""
set -e
cd {root}
if [ -f {REMOTE_PID} ]; then
  old_pid=$(cat {REMOTE_PID} 2>/dev/null || true)
  if [ -n "$old_pid" ] && kill -0 "$old_pid" 2>/dev/null; then
    kill "$old_pid" 2>/dev/null || true
  fi
fi
nohup env \
  PYTHONPATH={root} \
  UGRP_SIM_WS_URL={ws_url} \
  UGRP_SIM_TOKEN="$(cat {REMOTE_TOKEN})" \
  UGRP_SIM_SPEED_FILE={REMOTE_SPEED} \
  UGRP_GPU_PROVIDER=mac \
  UGRP_GPU_MACHINE=APPLE_M3 \
  UGRP_PICK_REQUIRE_BILATERAL=1 \
  UGRP_GENTLE_PLACE_TRANSPORT=1 \
  {python} scripts/run_mujoco_ws_worker.py --seed 11 \
  > {REMOTE_LOG} 2>&1 < /dev/null &
echo $! > {REMOTE_PID}
sleep 2
kill -0 "$(cat {REMOTE_PID})"
rm -f {REMOTE_TOKEN}
""".strip()
    try:
        return run_ssh(command, timeout=20).returncode == 0
    except (OSError, subprocess.TimeoutExpired):
        return False


def main() -> int:
    if bridge_connected():
        print("mac recovery: worker already connected", flush=True)
        return 0
    if not WS_URL:
        print("mac recovery: set UGRP_SIM_WS_URL in local untracked configuration", flush=True)
        return 2
    if not LOCAL_MODE and (not MAC_HOST or not MAC_ROOT):
        print(
            "mac recovery: set UGRP_MAC_SIM_HOST and UGRP_MAC_SIM_ROOT in local untracked configuration",
            flush=True,
        )
        return 2
    if not mac_ready():
        print("mac recovery: Mac worker runtime unavailable", flush=True)
        return 2
    if not install_runtime_inputs():
        print("mac recovery: failed to install runtime inputs", flush=True)
        return 3
    if not start_worker():
        print("mac recovery: worker failed to start", flush=True)
        return 4
    deadline = time.monotonic() + 20.0
    while time.monotonic() < deadline:
        if bridge_connected():
            print("mac recovery: worker connected", flush=True)
            return 0
        time.sleep(0.25)
    print("mac recovery: worker did not reach bridge", flush=True)
    return 5


if __name__ == "__main__":
    raise SystemExit(main())
