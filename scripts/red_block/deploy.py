"""Shared copy-to-Pi helper for red-block skills."""

from __future__ import annotations

import hashlib
import io
import json
import os
import tarfile
import ipaddress
import time
import uuid
import shutil
import socket
import subprocess
import sys
import threading
from pathlib import Path
from typing import Sequence


HERE = Path(__file__).resolve().parent
# R1 keeps every legacy default exactly. Multi-REAL child processes set these
# before importing the action package, so one codebase can address three Pi
# endpoints without rewriting host/user/path constants in each skill.
DEFAULT_HOST = os.environ.get("UGRP_ROBOT_HOST", "ugrp1").strip() or "ugrp1"
DEFAULT_USER = os.environ.get("UGRP_ROBOT_USER", "ugrp1").strip() or "ugrp1"
HOST_KEY_ALIAS = os.environ.get(
    "UGRP_ROBOT_HOSTKEY_ALIAS",
    "ugrp1.local" if DEFAULT_HOST == "ugrp1" else (DEFAULT_HOST if DEFAULT_HOST.endswith(".local") else f"{DEFAULT_HOST}.local"),
).strip()
ROBOT_ID = os.environ.get("UGRP_ROBOT_ID", "").strip().lower()
LOCAL_SUFFIX = f"-{ROBOT_ID}" if ROBOT_ID else ""
LAST_REMOTE_ERROR_PATH = Path(f"/tmp/ugrp{LOCAL_SUFFIX}-last-remote-error.txt")
REMOTE_DIR = Path(os.environ.get("UGRP_ROBOT_REMOTE_DIR", f"/home/{DEFAULT_USER}/MasterPi/tools"))
REMOTE_PACKAGE = REMOTE_DIR / "red_block"  # legacy shared path; never execute new jobs here
REMOTE_VERSIONS = REMOTE_DIR / ".ugrp_versions"
REMOTE_DEBUG = Path(f"/tmp/{DEFAULT_USER}-pick-red-debug.jpg")
LOCAL_DEBUG = Path(f"/tmp/ugrp{LOCAL_SUFFIX}-pick-red-debug.jpg")
CONTROL_SCRIPT = HERE.parent / "masterpi_control.py"
REMOTE_CONTROL = REMOTE_DIR / "masterpi_control.py"
SSH_CONTROL_PATH = "/tmp/ugrp-ssh-%C"
PROJECT_ROOT = HERE.parents[1]
LOCAL_TRACE_ROOT = PROJECT_ROOT / "outputs" / "real_traces"
REMOTE_TRACE_ROOT = Path("/tmp/ugrp-real-traces")
LAST_REAL_TRACE_PATH = Path(f"/tmp/ugrp{LOCAL_SUFFIX}-last-real-trace.json")

# Hard wall-clock budgets are deliberately above each skill's own bounded-loop
# budget. They are a final fail-safe for dead cameras, stuck drivers, or a
# controller loop that stops making progress while SSH itself remains healthy.
REMOTE_SKILL_TIMEOUTS = {
    "primitive.py": 15.0,
    "search.py": 55.0,
    "track.py": 20.0,
    "approach.py": 120.0,
    "pick.py": 75.0,
    "task_runner.py": 260.0,
    "place.py": 180.0,
    "put_down.py": 60.0,
    "fetch.py": 180.0,
    "carry.py": 15.0,
}
DEFAULT_REMOTE_SKILL_TIMEOUT = 120.0
SSH_TIMEOUT_MARGIN_SECONDS = 20.0


def _set_last_remote_error(message: str) -> None:
    text = " ".join(str(message).strip().split())
    if text:
        LAST_REMOTE_ERROR_PATH.write_text(text + "\n", encoding="utf-8")


def _timeout_text(value) -> str:
    if value is None:
        return ""
    if isinstance(value, bytes):
        return value.decode("utf-8", "replace")
    return str(value)


def new_trace_id(script_name: str) -> str:
    stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    skill = Path(script_name).stem.replace("_", "-")[:24]
    return f"{stamp}-{skill}-{uuid.uuid4().hex[:8]}"


def _safe_trace_component(value: str, fallback: str) -> str:
    cleaned = "".join(ch if ch.isalnum() or ch in "-_." else "-" for ch in str(value))
    cleaned = cleaned.strip("-.")[:96]
    return cleaned or fallback


def _trace_context(script_name: str) -> tuple[str, str, Path, Path]:
    run_id = _safe_trace_component(os.environ.get("UGRP_REAL_RUN_ID", ""), "")
    if not run_id:
        stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
        run_id = f"{stamp}-standalone-{uuid.uuid4().hex[:8]}"
    span_id = new_trace_id(script_name)
    local_span = LOCAL_TRACE_ROOT / run_id / "skills" / span_id
    remote_span = REMOTE_TRACE_ROOT / run_id / span_id
    return run_id, span_id, local_span, remote_span


def _write_local_trace_result(
    run_id: str,
    span_id: str,
    local_dir: Path,
    *,
    script_name: str,
    extra: Sequence[str],
    package_sha256: str,
    host: str,
    started_wall_s: float,
    ended_wall_s: float,
    exit_code: int,
    stdout_text: str,
    stderr_text: str,
    remote_trace_copied: bool,
    execution_started_wall_s: float | None = None,
    execution_ended_wall_s: float | None = None,
    trace_copy_duration_s: float | None = None,
) -> Path:
    metadata = {
        "schema_version": 2,
        "run_id": run_id,
        "span_id": span_id,
        "trace_id": span_id,
        "source": "real_masterpi",
        "script": script_name,
        "skill": Path(script_name).stem,
        "argv": list(extra),
        "package_sha256": package_sha256,
        "host_alias": host,
        "started_wall_s": started_wall_s,
        "ended_wall_s": ended_wall_s,
        "duration_s": max(0.0, ended_wall_s - started_wall_s),
        "execution_started_wall_s": execution_started_wall_s,
        "execution_ended_wall_s": execution_ended_wall_s,
        "execution_duration_s": (
            None
            if execution_started_wall_s is None or execution_ended_wall_s is None
            else max(0.0, execution_ended_wall_s - execution_started_wall_s)
        ),
        "trace_copy_duration_s": trace_copy_duration_s,
        "exit_code": int(exit_code),
        "execution_status": "COMPLETED" if exit_code == 0 else "FAILED",
        "remote_trace_copied": bool(remote_trace_copied),
    }
    try:
        local_dir.mkdir(parents=True, exist_ok=True)
        (local_dir / "result.json").write_text(
            json.dumps(metadata, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        (local_dir / "stdout.log").write_text(stdout_text or "", encoding="utf-8")
        (local_dir / "stderr.log").write_text(stderr_text or "", encoding="utf-8")
        run_dir = LOCAL_TRACE_ROOT / run_id
        LAST_REAL_TRACE_PATH.write_text(
            json.dumps({
                "run_id": run_id,
                "span_id": span_id,
                "trace_id": span_id,
                "trace_dir": str(local_dir),
                "run_dir": str(run_dir),
                "result": metadata,
            }, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
    except Exception as exc:
        print(f"warning: could not persist REAL trace metadata: {exc}", file=sys.stderr, flush=True)
    return local_dir


def _copy_remote_trace(destination: str, ssh_opts: list[str], remote_dir: Path, local_dir: Path) -> bool:
    """Fetch one completed REAL trace as a single compressed stream.

    ``scp -r`` pays a round-trip-heavy file protocol cost for every JPEG/event
    file.  Over the robot's often DERP-relayed Tailscale path that turned a
    100--300 KiB trace into 10--20 seconds on the action hot path.  A single
    tar stream preserves exactly the same evidence while requiring one payload
    transfer.  Delete the remote source only after safe local extraction.
    """
    try:
        local_dir.mkdir(parents=True, exist_ok=True)
        packed = subprocess.run(
            [
                "ssh", *ssh_opts, destination,
                "tar", "-C", str(remote_dir), "-czf", "-", ".",
            ],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            timeout=12.0,
        )
        if packed.returncode != 0 or not packed.stdout:
            return False
        try:
            with tarfile.open(fileobj=io.BytesIO(packed.stdout), mode="r:gz") as archive:
                archive.extractall(local_dir, filter="data")
        except (tarfile.TarError, OSError, ValueError):
            return False
        try:
            subprocess.run(
                ["ssh", *ssh_opts, destination, "rm", "-rf", str(remote_dir)],
                check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=8.0,
            )
        except (OSError, subprocess.TimeoutExpired):
            pass
        return True
    except (OSError, subprocess.TimeoutExpired):
        return False


def _run_tee(argv: list[str], timeout_s: float) -> tuple[int, str, str, bool]:
    """Run SSH while retaining stdout/stderr without hiding live diagnostics."""
    proc = subprocess.Popen(argv, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, bufsize=1)
    stdout_parts: list[str] = []
    stderr_parts: list[str] = []

    def pump(stream, target, sink: list[str]) -> None:
        if stream is None:
            return
        for line in iter(stream.readline, ""):
            sink.append(line)
            try:
                target.write(line)
                target.flush()
            except Exception:
                pass

    threads = [
        threading.Thread(target=pump, args=(proc.stdout, sys.stdout, stdout_parts), daemon=True),
        threading.Thread(target=pump, args=(proc.stderr, sys.stderr, stderr_parts), daemon=True),
    ]
    for t in threads:
        t.start()
    timed_out = False
    try:
        code = int(proc.wait(timeout=timeout_s))
    except subprocess.TimeoutExpired:
        timed_out = True
        proc.terminate()
        try:
            code = int(proc.wait(timeout=3.0))
        except subprocess.TimeoutExpired:
            proc.kill()
            code = int(proc.wait())
    for t in threads:
        t.join(timeout=1.0)
    return code, "".join(stdout_parts), "".join(stderr_parts), timed_out


def package_files() -> list[Path]:
    return sorted(path for path in HERE.glob("*.py") if path.name != "__init__.py")


def is_ipv4(value: str) -> bool:
    try:
        return ipaddress.IPv4Address(value).version == 4
    except ValueError:
        return False


def host_resolves(host: str) -> bool:
    try:
        socket.getaddrinfo(host, 22, socket.AF_INET)
        return True
    except OSError:
        return False


def tailscale_ipv4(name: str = DEFAULT_HOST) -> str | None:
    binary = shutil.which("tailscale")
    if not binary:
        return None
    try:
        result = subprocess.run(
            [binary, "ip", "-4", name],
            check=False,
            capture_output=True,
            text=True,
            timeout=3,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    if result.returncode != 0:
        return None
    for token in result.stdout.split():
        if is_ipv4(token):
            return token
    return None


def ssh_connection_args(host: str) -> tuple[str, list[str]]:
    """Return (ssh destination, extra -o options).

    The SSH alias `ugrp1` is configured to use `ugrp1.local`. When mDNS is
    down, override HostName with the Tailscale IPv4 or an explicit IP.
    """
    extra = [
        "-o", "BatchMode=yes",
        "-o", "ConnectTimeout=8",
        # Tailscale changes the address but not the robot identity. Accept a
        # previously unseen alias key once, then OpenSSH pins it normally.
        "-o", "StrictHostKeyChecking=accept-new",
        "-o", f"User={DEFAULT_USER}",
        # Reuse the camera/service SSH transport instead of paying a ~2.3 s
        # Tailscale/OpenSSH handshake for every mkdir/scp/action subprocess.
        "-o", "ControlMaster=auto",
        "-o", "ControlPersist=600",
        "-o", f"ControlPath={SSH_CONTROL_PATH}",
        "-o", "ServerAliveInterval=10",
        "-o", "ServerAliveCountMax=2",
    ]
    if is_ipv4(host):
        extra.extend(
            ["-o", f"HostName={host}", "-o", f"HostKeyAlias={HOST_KEY_ALIAS}"]
        )
        return DEFAULT_HOST, extra
    if host in {DEFAULT_HOST, HOST_KEY_ALIAS}:
        ts_ip = tailscale_ipv4(DEFAULT_HOST)
        if ts_ip:
            extra.extend(
                ["-o", f"HostName={ts_ip}", "-o", f"HostKeyAlias={HOST_KEY_ALIAS}"]
            )
            return DEFAULT_HOST, extra
        if host_resolves(HOST_KEY_ALIAS):
            return DEFAULT_HOST, extra
        raise RuntimeError(
            f"cannot reach {DEFAULT_HOST}: Tailscale has no IPv4 for it and "
            f"{HOST_KEY_ALIAS} did not resolve. Run `tailscale status`, or pass "
            "--host <IP>."
        )
    return host, extra


def package_signature() -> str:
    """Content signature for the code that actually runs on the Pi."""
    digest = hashlib.sha256()
    for path in [CONTROL_SCRIPT, *package_files()]:
        digest.update(path.name.encode("utf-8"))
        digest.update(b"\0")
        digest.update(path.read_bytes())
        digest.update(b"\0")
    return digest.hexdigest()


def package_manifest() -> str:
    """sha256sum-compatible manifest for one immutable remote version."""
    lines: list[str] = []
    files = [(CONTROL_SCRIPT, "masterpi_control.py")]
    files.extend((path, f"red_block/{path.name}") for path in package_files())
    for path, rel in files:
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        lines.append(f"{digest}  {rel}")
    return "\n".join(lines) + "\n"


def remote_version_root(signature: str) -> Path:
    return REMOTE_VERSIONS / signature


def remote_package(signature: str) -> Path:
    return remote_version_root(signature) / "red_block"


def _remote_package_current(destination: str, ssh_opts: list[str], signature: str) -> bool:
    root = remote_version_root(signature)
    command = (
        f"cd {root} 2>/dev/null && "
        f"test -f .ugrp-files.sha256 && "
        f"sha256sum -c .ugrp-files.sha256 >/dev/null 2>&1"
    )
    try:
        result = subprocess.run(
            ["ssh", *ssh_opts, destination, command],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=15.0,
        )
    except subprocess.TimeoutExpired:
        return False
    return result.returncode == 0


def _add_bytes(archive: tarfile.TarFile, name: str, data: bytes) -> None:
    info = tarfile.TarInfo(name=name)
    info.size = len(data)
    info.mtime = 0
    archive.addfile(info, io.BytesIO(data))


def _deployment_archive(signature: str, manifest: str) -> bytes:
    """One immutable, self-verifying code bundle for the Pi."""
    buf = io.BytesIO()
    with tarfile.open(fileobj=buf, mode="w:gz") as archive:
        archive.add(CONTROL_SCRIPT, arcname="masterpi_control.py")
        for path in package_files():
            archive.add(path, arcname=f"red_block/{path.name}")
        _add_bytes(archive, ".ugrp-package-sha256", signature.encode("ascii"))
        _add_bytes(archive, ".ugrp-files.sha256", manifest.encode("utf-8"))
    return buf.getvalue()


def _ensure_deployed(
    destination: str,
    ssh_opts: list[str],
    host: str,
    *,
    signature: str | None = None,
) -> bool:
    signature = signature or package_signature()
    if _remote_package_current(destination, ssh_opts, signature):
        return True

    manifest = package_manifest()
    payload = _deployment_archive(signature, manifest)
    root = remote_version_root(signature)
    # Extract into a signature-specific directory. Existing legacy controllers
    # may overwrite /tools/red_block, but can no longer create a mixed version
    # inside the path used by this request. Verify every file before execution.
    command = (
        f"mkdir -p {root} && "
        f"tar -xzf - -C {root} && "
        f"cd {root} && sha256sum -c .ugrp-files.sha256 >/dev/null 2>&1"
    )
    try:
        sync = subprocess.run(
            ["ssh", *ssh_opts, destination, command],
            input=payload,
            check=False,
            timeout=30.0,
        )
    except subprocess.TimeoutExpired:
        message = f"deployment sync timed out while contacting {host}"
        _set_last_remote_error(message)
        print(f"error: {message}", file=sys.stderr)
        return False
    if sync.returncode != 0:
        message = f"could not sync verified red_block package to {host}"
        _set_last_remote_error(message)
        print(f"error: {message}", file=sys.stderr)
        return False
    return _remote_package_current(destination, ssh_opts, signature)


def deploy_and_run(
    script_name: str,
    extra: Sequence[str],
    *,
    host: str = DEFAULT_HOST,
    tty: bool = False,
) -> int:
    try:
        LAST_REMOTE_ERROR_PATH.unlink()
    except FileNotFoundError:
        pass

    run_id, span_id, local_span, remote_span = _trace_context(script_name)
    started_wall_s = time.time()
    signature = package_signature()
    stdout_text = ""
    stderr_text = ""
    result = 1
    copied = False
    destination = ""
    ssh_opts: list[str] = []
    execution_started_wall_s: float | None = None
    execution_ended_wall_s: float | None = None
    trace_copy_duration_s: float | None = None

    try:
        destination, ssh_opts = ssh_connection_args(host)
    except RuntimeError as exc:
        stderr_text = f"error: {exc}\n"
        _set_last_remote_error(str(exc))
        print(stderr_text, file=sys.stderr, end="")
        _write_local_trace_result(
            run_id, span_id, local_span, script_name=script_name, extra=extra,
            package_sha256=signature, host=host, started_wall_s=started_wall_s,
            ended_wall_s=time.time(), exit_code=result, stdout_text=stdout_text,
            stderr_text=stderr_text, remote_trace_copied=False,
        )
        return result

    if not _ensure_deployed(destination, ssh_opts, host, signature=signature):
        message = (
            LAST_REMOTE_ERROR_PATH.read_text(encoding="utf-8").strip()
            if LAST_REMOTE_ERROR_PATH.is_file() else f"verified package deployment failed for {host}"
        )
        stderr_text = f"error: {message}\n"
        _write_local_trace_result(
            run_id, span_id, local_span, script_name=script_name, extra=extra,
            package_sha256=signature, host=host, started_wall_s=started_wall_s,
            ended_wall_s=time.time(), exit_code=result, stdout_text=stdout_text,
            stderr_text=stderr_text, remote_trace_copied=False,
        )
        return result

    version_package = remote_package(signature)
    remote_timeout = float(REMOTE_SKILL_TIMEOUTS.get(script_name, DEFAULT_REMOTE_SKILL_TIMEOUT))
    trace_quality = os.environ.get("UGRP_REAL_TRACE_JPEG_QUALITY", "55")
    trace_interval = os.environ.get("UGRP_REAL_TRACE_FRAME_INTERVAL_S", "0.25")
    trace_all_frames = os.environ.get("UGRP_REAL_TRACE_ALL_FRAMES", "1")
    remote_env = [
        "env",
        "UGRP_REAL_TRACE_ENABLE=1",
        f"UGRP_REAL_RUN_ID={run_id}",
        f"UGRP_REAL_TRACE_SPAN_ID={span_id}",
        f"UGRP_REAL_TRACE_DIR={remote_span}",
        f"UGRP_REAL_TRACE_SKILL={Path(script_name).stem}",
        f"UGRP_REAL_TRACE_PHASE={Path(script_name).stem}",
        f"UGRP_REAL_TRACE_JPEG_QUALITY={trace_quality}",
        f"UGRP_REAL_TRACE_FRAME_INTERVAL_S={trace_interval}",
        f"UGRP_REAL_TRACE_ALL_FRAMES={trace_all_frames}",
    ]
    ssh = ["ssh", *ssh_opts]
    if tty:
        ssh.append("-t")
    ssh.extend([
        destination,
        *remote_env,
        "python3", str(version_package / "remote_watchdog.py"),
        "--timeout", str(remote_timeout), "--",
        "python3", str(version_package / script_name), "--on-robot", *extra,
    ])
    local_timeout = remote_timeout + SSH_TIMEOUT_MARGIN_SECONDS
    execution_started_wall_s = time.time()
    try:
        result, stdout_text, stderr_text, timed_out = _run_tee(ssh, local_timeout)
    except OSError as exc:
        result, stdout_text, stderr_text, timed_out = 1, "", f"error: ssh launch failed: {exc}\n", False
        print(stderr_text, file=sys.stderr, end="")
    execution_ended_wall_s = time.time()
    if timed_out:
        result = 124
        message = (
            f"ssh transport exceeded {local_timeout:.1f}s while running {script_name}; "
            "remote watchdog did not return in time"
        )
        _set_last_remote_error(message)
        stderr_text += f"error: {message}\n"
        print(f"error: {message}", file=sys.stderr)

    trace_copy_started = time.perf_counter()
    copied = _copy_remote_trace(destination, ssh_opts, remote_span, local_span)
    trace_copy_duration_s = max(0.0, time.perf_counter() - trace_copy_started)

    if result != 0:
        error_lines = [
            line.strip()[6:].strip() for line in stderr_text.splitlines()
            if line.strip().lower().startswith("error:")
        ]
        if error_lines:
            _set_last_remote_error(error_lines[-1])
        elif result == 124:
            _set_last_remote_error(f"remote skill safety timeout while running {script_name}")
        else:
            tail = next((line.strip() for line in reversed(stderr_text.splitlines()) if line.strip()), "")
            _set_last_remote_error(f"remote skill {script_name} exited {result}" + (f": {tail}" if tail else ""))
        # Keep the legacy latest-debug path, but also preserve an immutable copy
        # inside this span so a later failure can never overwrite the evidence.
        try:
            debug_copy = subprocess.run(
                ["scp", *ssh_opts, f"{destination}:{REMOTE_DEBUG}", str(local_span / "debug-failure.jpg")],
                check=False, capture_output=True, timeout=8.0,
            )
        except subprocess.TimeoutExpired:
            debug_copy = None
        if debug_copy is not None and debug_copy.returncode == 0:
            try:
                shutil.copy2(local_span / "debug-failure.jpg", LOCAL_DEBUG)
            except OSError:
                pass
            print(f"debug frame copied to {local_span / 'debug-failure.jpg'}", flush=True)

    _write_local_trace_result(
        run_id, span_id, local_span, script_name=script_name, extra=extra,
        package_sha256=signature, host=host, started_wall_s=started_wall_s,
        ended_wall_s=time.time(), exit_code=result, stdout_text=stdout_text,
        stderr_text=stderr_text, remote_trace_copied=copied,
        execution_started_wall_s=execution_started_wall_s,
        execution_ended_wall_s=execution_ended_wall_s,
        trace_copy_duration_s=trace_copy_duration_s,
    )
    return result

