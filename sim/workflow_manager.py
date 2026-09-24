"""Versioned local workflow launcher and durable process/evidence records.

Legacy scripts remain the execution adapters; this module never changes their
physics, model selection, or success criteria.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import importlib.metadata
import json
import math
import os
from pathlib import Path
import platform
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
from urllib.parse import parse_qsl, urlencode, urlsplit, urlunsplit
from uuid import uuid4

SCHEMA = "ugrp.simulation_run.v1"
CATALOG = Path("configs/simulation_workflows.json")
RECORDS = Path("outputs/simulation-runs")
SOURCE_DIRS = ("harness", "sim", "scripts", "config", "configs", "maps", "calibration", "examples")
SOURCE_SUFFIXES = {".py", ".json", ".jsonl", ".xml", ".yaml", ".yml", ".toml", ".command",
                   ".txt", ".csv", ".png", ".jpg", ".jpeg", ".npy", ".npz"}
SECRET = re.compile(r"(?:^|[-_])(?:token|password|secret|api[-_]?key|authorization|cookie|credential)(?:$|[-_])", re.I)
MANAGED_CHILD = "UGRP_SIM_MANAGED_CHILD"
INPUT_PATH_FLAGS = ("--plan-replay", "--grasp-model-dir", "--stage-model-dir", "--protocol", "--manifest",
                    "--artifacts", "--map-file", "--map", "--act-python", "--mjpython", "--grasp",
                    "--stages", "--cases-json", "--dataset", "--evidence-root", "--inventory", "--config",
                    "--carry-act-model", "--carry-act-python", "--reference-top", "--spec")


def _sha(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _write(path: Path, value: object) -> None:
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")
    temporary.replace(path)


def _tree_files(path: Path) -> list[Path]:
    if path.is_file():
        return [path]
    if not path.is_dir():
        raise ValueError(f"input does not exist: {path}")
    return sorted(p for p in path.rglob("*") if p.is_file() and not p.is_symlink())


def path_receipt(path: Path) -> dict:
    path = path.resolve()
    files = _tree_files(path)
    entries = [{"path": str(p.relative_to(path)) if path.is_dir() else p.name, "sha256": _sha(p),
                "bytes": p.stat().st_size} for p in files]
    digest = hashlib.sha256(json.dumps(entries, sort_keys=True).encode()).hexdigest()
    return {"path": str(path), "kind": "directory" if path.is_dir() else "file", "sha256": digest,
            "files": entries}


def source_fingerprint(root: Path) -> dict:
    def selected(relative: str) -> bool:
        path = Path(relative)
        return (path.parts and path.parts[0] in SOURCE_DIRS and path.suffix in SOURCE_SUFFIXES or
                len(path.parts) == 1 and (relative.startswith("requirements") and path.suffix == ".txt" or
                                          relative in ("pyproject.toml", "setup.cfg")))
    entries = []
    present = set()
    for dirname in SOURCE_DIRS:
        folder = root / dirname
        if folder.exists():
            for path in sorted(folder.rglob("*")):
                if path.is_file() and path.suffix in SOURCE_SUFFIXES and "__pycache__" not in path.parts:
                    relative = str(path.relative_to(root))
                    present.add(relative)
                    entries.append({"path": relative, "sha256": _sha(path)})
    for path in sorted(root.glob("requirements*.txt")) + [root / "pyproject.toml", root / "setup.cfg"]:
        if path.is_file():
            relative = str(path.relative_to(root))
            present.add(relative)
            entries.append({"path": relative, "sha256": _sha(path)})
    tracked_available = False
    try:
        raw = subprocess.check_output(["git", "-C", str(root), "ls-files", "-z", "--cached"], stderr=subprocess.DEVNULL)
        tracked = {name for name in raw.decode().split("\0") if name and selected(name)}
        tracked_available = True
    except (OSError, subprocess.CalledProcessError):
        tracked = set()
    missing = sorted(tracked - present)
    entries.extend({"path": name, "sha256": None, "missing_tracked": True} for name in missing)
    entries.sort(key=lambda row: row["path"])
    digest = hashlib.sha256(json.dumps(entries, sort_keys=True).encode()).hexdigest()
    return {"sha256": digest, "files": entries, "scope": [*SOURCE_DIRS, "requirements*.txt",
            "pyproject.toml", "setup.cfg"], "missing_tracked_files": missing,
            "tracked_inventory_available": tracked_available, "includes_untracked_execution_files": True}


def git_identity(root: Path) -> dict:
    def command(*parts):
        return subprocess.check_output(["git", "-C", str(root), *parts], text=True, stderr=subprocess.DEVNULL).strip()
    try:
        return {"source_sha": command("rev-parse", "HEAD"),
                "source_dirty": bool(command("status", "--porcelain", "--untracked-files=all"))}
    except (OSError, subprocess.CalledProcessError):
        return {"source_sha": None, "source_dirty": None}


def environment_identity() -> dict:
    packages = {}
    for name in ("mujoco", "numpy", "opencv-python", "opencv-python-headless", "Pillow", "torch", "gymnasium"):
        try:
            packages[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            packages[name] = None
    return {"python": platform.python_version(), "executable": sys.executable,
            "platform": platform.platform(), "packages": packages}


def redact_argv(argv: list[str]) -> list[str]:
    def redact_url(value: str) -> str:
        if "://" not in value:
            return value
        try:
            parts = urlsplit(value)
            netloc = parts.netloc
            if "@" in netloc:
                netloc = "[REDACTED]@" + netloc.rsplit("@", 1)[1]
            # Query names vary across providers (key, sig, auth, etc.).  Keep
            # the URL shape for diagnosis without persisting any query value.
            query = urlencode([(key, "[REDACTED]" if item else "")
                               for key, item in parse_qsl(parts.query, keep_blank_values=True)])
            fragment = "[REDACTED]" if parts.fragment else ""
            return urlunsplit((parts.scheme, netloc, parts.path, query, fragment))
        except ValueError:
            return "[REDACTED_URL]"
    result = []
    hide_next = False
    for arg in argv:
        if hide_next:
            result.append("[REDACTED]")
            hide_next = False
        elif arg.startswith("-") and "=" in arg and SECRET.search(arg.split("=", 1)[0]):
            result.append(arg.split("=", 1)[0] + "=[REDACTED]")
        else:
            result.append(arg.split("=", 1)[0] + "=" + redact_url(arg.split("=", 1)[1])
                          if arg.startswith("-") and "=" in arg else redact_url(arg))
            hide_next = arg.startswith("-") and bool(SECRET.search(arg)) and "=" not in arg
    return result


def catalog(root: Path) -> tuple[dict, str]:
    path = root / CATALOG
    data = json.loads(path.read_text(encoding="utf-8"))
    if data.get("schema") != "ugrp.local_workflow_catalog.v1":
        raise ValueError("unsupported workflow catalog schema")
    rows = data.get("workflows", [])
    if not rows or len({r["id"] for r in rows}) != len(rows):
        raise ValueError("workflow catalog must have distinct workflows")
    for row in rows:
        if not all(k in row for k in ("id", "version", "runner", "entry", "output_flag", "output_kind", "required_inputs", "side_effect")):
            raise ValueError(f"incomplete workflow declaration: {row.get('id')}")
        if not (root / row["entry"]).is_file():
            raise ValueError(f"workflow entry missing: {row['entry']}")
    return data, _sha(path)


def _row(root: Path, workflow_id: str) -> tuple[dict, str]:
    data, digest = catalog(root)
    row = next((r for r in data["workflows"] if r["id"] == workflow_id), None)
    if row is None:
        raise ValueError(f"unknown workflow: {workflow_id}")
    return row, digest


def _option(argv: list[str], flag: str) -> str | None:
    values = []
    for index, arg in enumerate(argv):
        if arg == flag:
            if index + 1 == len(argv):
                raise ValueError(f"{flag} requires a value")
            if argv[index + 1].startswith("--"):
                raise ValueError(f"{flag} requires a value, got another option")
            values.append(argv[index + 1])
        if arg.startswith(flag + "="):
            value = arg.split("=", 1)[1]
            if not value:
                raise ValueError(f"{flag} requires a value")
            values.append(value)
    if len(values) > 1:
        raise ValueError(f"duplicate {flag} is ambiguous")
    return values[0] if values else None


def _at_root(root: Path, value: str | Path) -> Path:
    path = Path(value).expanduser()
    return (path if path.is_absolute() else root / path).resolve()


def _input_paths(root: Path, argv: list[str], extra: list[Path]) -> list[Path]:
    paths = list(extra)
    redacted = redact_argv(argv)
    for original, safe in zip(argv, redacted):
        if original != safe:
            continue
        if original.startswith("-") and "=" not in original:
            continue
        value = original.split("=", 1)[1] if "=" in original else original
        path = _at_root(root, value)
        if path.exists():
            paths.append(path)
    return list(dict.fromkeys(_at_root(root, p) for p in paths))


def _local_config_inputs(root: Path, argv: list[str]) -> list[Path]:
    """Resolve local config dependencies without importing user extensions."""
    from sim.session_config import load_config, validate_config
    from sim.session_extensions import validate_reference

    config_path = _at_root(root, argv[1])
    if not config_path.is_file():
        raise ValueError(f"local config does not exist: {argv[1]}")
    config = load_config(config_path)
    scene = _option(argv, "--scene")
    if scene is not None:
        config["scene"]["layout"] = scene
    builder = _option(argv, "--scene-builder")
    if builder is not None:
        config["scene"]["builder"] = builder
    for index, arg in enumerate(argv):
        if arg != "--controller" and not arg.startswith("--controller="):
            continue
        value = argv[index + 1] if arg == "--controller" and index + 1 < len(argv) else arg.partition("=")[2]
        if "=" not in value:
            raise ValueError("--controller requires ROBOT=FILE.py:FACTORY")
        robot, ref = value.split("=", 1)
        config["controllers"][robot] = {**config["controllers"].get(robot, {}), "factory": ref}
    config = validate_config(config)
    base = config_path.parent
    paths = [config_path]
    if config["scene"]["map_file"] is not None:
        paths.append(base / config["scene"]["map_file"])
    refs = [config["scene"]["builder"], *config["action_plugins"].values(),
            *(entry["factory"] for entry in config["controllers"].values())]
    for ref in refs:
        if ref is not None:
            filename, _ = validate_reference(ref)
            paths.append(base / filename)
    resolved = list(dict.fromkeys(path.resolve() for path in paths))
    for path in resolved:
        if not path.is_file():
            raise ValueError(f"local config input does not exist: {path}")
    return resolved


def _workflow_inputs(root: Path, workflow_id: str, argv: list[str], extra: list[Path]) -> list[Path]:
    paths = list(extra)
    if workflow_id == "local":
        paths.extend(_local_config_inputs(root, argv))
    if workflow_id == "dispatch" and _option(argv, "--grasp-model-dir") is None:
        paths.append(root / "experiments/dispatch-skill-integration-20260917/models.zip")
    if workflow_id == "dispatch-skills" and _option(argv, "--reference-top") is None:
        paths.append(root / "tests/fixtures/camera_goal_transport/reference-top.jpg")
    return _input_paths(root, argv, paths)


def _validate_input_paths(root: Path, workflow_id: str, argv: list[str]) -> None:
    if workflow_id == "local":
        _local_config_inputs(root, argv)
    if workflow_id == "act-map-suite":
        spec = _option(argv, "--spec")
        if spec is not None and not _at_root(root, spec).is_file():
            raise ValueError(f"--spec input is not a file: {spec}")
    for flag in INPUT_PATH_FLAGS:
        value = _option(argv, flag)
        if value is not None and not _at_root(root, value).exists():
            raise ValueError(f"{flag} input does not exist: {value}")
    if workflow_id == "dispatch" and _option(argv, "--grasp-model-dir") is None:
        archive = root / "experiments/dispatch-skill-integration-20260917/models.zip"
        if not archive.is_file():
            raise ValueError(f"dispatch default model archive missing: {archive}")


def _validate_args(row: dict, argv: list[str]) -> str | None:
    workflow_id = row["id"]
    if workflow_id == "local":
        if not argv or argv[0] not in ("run", "console") or len(argv) < 2 or argv[1].startswith("-"):
            raise ValueError("local requires explicit 'run CONFIG' or 'console CONFIG'")
    elif workflow_id == "communication":
        if not argv or argv[0] not in ("prepare", "validate", "schedule", "admit", "evaluate"):
            raise ValueError("communication requires an explicit subcommand")
    elif workflow_id == "communication-study":
        if not argv or argv[0] not in ("prepare", "check", "run", "trial"):
            raise ValueError("communication-study requires an explicit subcommand")
    elif workflow_id == "physical":
        if not argv or argv[0].startswith("-") or argv[0] == "latest":
            raise ValueError("physical requires an explicit trace directory; 'latest' is not managed")
    elif workflow_id == "worker":
        if _option(argv, "--token") is not None:
            raise ValueError("worker token must be supplied through UGRP_SIM_TOKEN, not process argv")
        if not _option(argv, "--url") or not os.environ.get("UGRP_SIM_TOKEN"):
            raise ValueError("worker requires explicit --url and UGRP_SIM_TOKEN in its environment")
    elif workflow_id == "dispatch-skills":
        if _option(argv, "--executor") is not None:
            raise ValueError("dispatch-skills fixes the skills executor; --executor is not accepted")
        if "--live-replan" in argv:
            raise ValueError("dispatch-skills is saved-plan replay; live replan requires dispatch")
        if not _option(argv, "--grasp-model-dir") or not _option(argv, "--stage-model-dir"):
            raise ValueError("dispatch-skills requires explicit grasp and stage model directories")
    elif workflow_id == "act-input-training" and any(arg == "--resume" or arg.startswith("--resume=") for arg in argv):
        raise ValueError("act-input-training requires a fresh output; --resume is not managed")
    if workflow_id == "jev" and "--worker" in argv:
        raise ValueError("jev --worker is an internal worker entry point")
    output_flag = row["output_flag"]
    if isinstance(output_flag, dict):
        output_flag = output_flag.get(argv[0])
    required = row["required_inputs"]
    if isinstance(required, dict):
        required = required.get(argv[0], [])
    for key in required:
        if key in ("CONFIG", "TRACE_DIRECTORY"):
            continue
        if key == "--execute":
            if key not in argv:
                raise ValueError(f"{workflow_id} requires {key}")
            continue
        if key == "--source":
            if not any(arg == key or arg.startswith(key + "=") for arg in argv):
                raise ValueError(f"{workflow_id} requires {key}")
        elif key == "--token" and workflow_id == "worker":
            continue
        elif _option(argv, key) is None:
            raise ValueError(f"{workflow_id} requires {key}")
    if output_flag:
        _option(argv, output_flag)  # reject duplicate output switches
        return output_flag
    return None


def _runner_command(root: Path, row: dict, args: list[str], *, record: Path | None) -> tuple[list[str], Path | None]:
    argv = list(args)
    output_flag = _validate_args(row, argv)
    output = _at_root(root, _option(argv, output_flag)) if output_flag and _option(argv, output_flag) else None
    if output is not None and output.exists():
        raise FileExistsError(f"workflow output already exists: {output}")
    if record is not None and output_flag and output is None:
        output = record / ("artifacts.json" if row["output_kind"] == "file" or
                           isinstance(row["output_kind"], dict) and row["output_kind"].get(args[0]) == "file"
                           else "artifacts")
        argv.extend([output_flag, str(output)])
    if row["id"] == "physical":
        # The historical analyzer writes alongside its input. Operate on a
        # recorded copy so an analysis cannot overwrite the original trace.
        source = _at_root(root, argv[0])
        if not source.is_dir():
            raise ValueError(f"trace directory does not exist: {source}")
        if record is not None:
            output = record / "artifacts"
            shutil.copytree(source, output)
            argv[0] = str(output)
    if row["id"] == "dispatch-skills":
        argv = ["--executor", "skills", *argv]
    if row["id"] == "dispatch":
        argv = ["dispatch", *argv]
    runner = row["runner"]
    return [sys.executable, "-m", runner, *argv], output


def plan(root: Path, workflow_id: str, args: list[str], *, inputs: list[Path] | None = None) -> dict:
    row, catalog_hash = _row(root, workflow_id)
    argv = list(args)
    _validate_args(row, argv)
    _validate_input_paths(root, workflow_id, argv)
    if row["id"] == "physical" and not _at_root(root, argv[0]).is_dir():
        raise ValueError("physical trace directory does not exist")
    if row["id"] == "physical" and (root / RECORDS).resolve().is_relative_to(_at_root(root, argv[0])):
        raise ValueError("physical trace cannot contain the managed record root")
    # Planning is read-only and never chooses a model, starts a subprocess, or
    # changes a missing output into a supposedly completed run.
    command, output = _runner_command(root, row, argv, record=None)
    input_receipts = [path_receipt(p) for p in _workflow_inputs(root, workflow_id, argv, inputs or [])]
    proposed_output = str(output) if output else ("<record>/artifacts" if row["id"] == "physical" else
        "<record>/artifacts.json" if row["output_kind"] == "file" or
        isinstance(row["output_kind"], dict) and row["output_kind"].get(argv[0]) == "file" else
        "<record>/artifacts" if _validate_args(row, argv) else None)
    planned_command = list(command)
    if _validate_args(row, argv) and output is None:
        planned_command.extend([_validate_args(row, argv), proposed_output])
    if workflow_id == "physical":
        planned_command[-1] = "<record>/artifacts"
    return {"workflow_id": workflow_id, "workflow_version": row["version"], "catalog_sha256": catalog_hash,
            "runner": row["runner"], "adapter": row.get("adapter", "legacy_cli"),
            "side_effect": row["side_effect"], "argv": redact_argv(argv),
            "command": redact_argv(planned_command), "output": proposed_output,
            "output_flag": _validate_args(row, argv), "inputs": input_receipts,
            "readiness": "required_inputs_present; legacy_parser_and_runtime_not_executed", "execution_started": False}


def _new_record(root: Path, workflow_id: str, chosen: Path | None) -> Path:
    if chosen is not None:
        path = chosen.expanduser().resolve()
    else:
        path = root / RECORDS / f"{datetime.now():%Y%m%d-%H%M%S}-{workflow_id}-{uuid4().hex[:8]}"
    path.mkdir(parents=True, exist_ok=False)
    _write(path / "manifest.json", {"schema": SCHEMA, "run_id": path.name, "workflow_id": workflow_id,
                                    "status": "initializing", "started_utc": datetime.now(timezone.utc).isoformat()})
    if chosen is not None:
        index = root / RECORDS / ".index"
        index.mkdir(parents=True, exist_ok=True)
        _write(index / f"{path.name}-{uuid4().hex[:8]}.json", {"record": str(path)})
    return path


def _records(root: Path) -> list[Path]:
    folder = root / RECORDS
    paths = list(folder.glob("*/manifest.json")) if folder.exists() else []
    for pointer in (folder / ".index").glob("*.json") if (folder / ".index").exists() else []:
        try:
            paths.append(Path(json.loads(pointer.read_text())["record"]) / "manifest.json")
        except (OSError, ValueError, KeyError):
            continue
    return sorted(set(paths), reverse=True)


def _artifact_receipts(path: Path | None) -> dict | None:
    return path_receipt(path) if path is not None and path.exists() else None


def _reported_outcomes(output: Path | None) -> dict:
    if output is None:
        return {}
    paths = ([output / "result.json", output / "summary.json", output / "report.json",
              output / "artifact-finalization.json"] if output.is_dir() else [output])
    outcomes = {}
    for path in paths:
        if not path.is_file() or path.suffix != ".json":
            continue
        try:
            result = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            continue
        if not isinstance(result, dict):
            continue
        for key in ("physical_success", "protocol_complete", "success", "completed", "ready"):
            if type(result.get(key)) is bool:
                outcomes[key] = {"value": result[key], "evidence": str(path)}
    return outcomes


def _base_manifest(root: Path, workflow_id: str, argv: list[str], inputs: list[Path],
                   record: Path, *, mode: str) -> dict:
    row, catalog_hash = _row(root, workflow_id)
    return {"schema": SCHEMA, "run_id": record.name, "project_root": str(root), "workflow_id": workflow_id,
            "workflow_version": row["version"], "catalog_sha256": catalog_hash,
            "adapter": row.get("adapter", "legacy_cli"), "mode": mode,
            "source": {**git_identity(root), "execution_tree": source_fingerprint(root)},
            "environment": environment_identity(), "argv": redact_argv(argv),
            "inputs_before": [path_receipt(p) for p in _workflow_inputs(root, workflow_id, argv, inputs)],
            "started_utc": datetime.now(timezone.utc).isoformat(), "status": "running",
            "parent_pid": os.getpid(), "child_pid": None, "output": None,
            "reported_outcomes": {}, "physical_success": None}


def _finish(manifest: dict, record: Path, *, exit_code: int | None, output: Path | None,
            status: str, failure: str | None) -> None:
    manifest["ended_utc"] = datetime.now(timezone.utc).isoformat()
    manifest["runtime_s"] = time.monotonic() - manifest.pop("_monotonic")
    manifest["exit_code"] = exit_code
    manifest["status"] = status
    manifest["failure"] = failure
    manifest["output"] = str(output) if output else None
    errors = []
    def safe(label, thunk):
        try:
            return thunk()
        except (OSError, ValueError) as error:
            errors.append(f"{label}: {type(error).__name__}: {error}")
            return None
    manifest["output_receipt"] = safe("output_receipt", lambda: _artifact_receipts(output))
    outcomes = safe("reported_outcomes", lambda: _reported_outcomes(output)) or {}
    manifest["reported_outcomes"] = outcomes
    manifest["physical_success"] = outcomes.get("physical_success", {}).get("value")
    manifest["inputs_after"] = [entry for entry in (
        safe("input_after", lambda path=Path(row["path"]): path_receipt(path)) for row in manifest["inputs_before"])
        if entry is not None]
    before_inputs = {row["path"]: row["sha256"] for row in manifest["inputs_before"]}
    after_inputs = {row["path"]: row["sha256"] for row in manifest["inputs_after"]}
    manifest["inputs_changed_during_run"] = before_inputs != after_inputs
    root = Path(manifest["project_root"])
    after = safe("source_after", lambda: source_fingerprint(root))
    manifest["source_after"] = after
    manifest["source_changed_during_run"] = (after["sha256"] != manifest["source"]["execution_tree"]["sha256"]) if after else None
    manifest["logs"] = safe("logs", lambda: _artifact_receipts(record / "console.log"))
    manifest["finalization_errors"] = errors
    _write(record / "manifest.json", manifest)


def _select_python(row: dict, argv: list[str]) -> str:
    isolated = row["id"] in ("dispatch", "dispatch-skills") and "--realtime-control" in argv
    native = (row["id"] == "local" and "--headless" not in argv or
              row["id"] == "dispatch" and "--headless" not in argv and not isolated or
              row["id"] == "dispatch-skills" and "--viewer" in argv and not isolated or
              row["id"] == "act-map-suite" and "--render" in argv)
    if native and sys.platform == "darwin":
        candidate = Path(sys.executable).with_name("mjpython")
        if not candidate.is_file() or not os.access(candidate, os.X_OK):
            raise ValueError(f"native macOS viewer requires mjpython next to {sys.executable}; use --headless")
        return str(candidate)
    return sys.executable


def run_workflow(root: Path, workflow_id: str, argv: list[str], *, inputs: list[Path] | None = None,
                 record_dir: Path | None = None, timeout: float | None = None) -> Path:
    root = root.resolve()
    row, _ = _row(root, workflow_id)
    if timeout is not None and (not math.isfinite(timeout) or timeout <= 0):
        raise ValueError("timeout must be positive and finite")
    if workflow_id == "worker" and timeout is None:
        raise ValueError("worker requires --timeout because it is a persistent service")
    # Reject unsupported/missing selection and output collisions before a record is made.
    plan(root, workflow_id, argv, inputs=inputs)
    if workflow_id == "physical" and record_dir is not None and record_dir.expanduser().resolve().is_relative_to(_at_root(root, argv[0])):
        raise ValueError("physical trace cannot contain its record directory")
    record = _new_record(root, workflow_id, record_dir)
    try:
        manifest = _base_manifest(root, workflow_id, argv, inputs or [], record, mode="subprocess")
    except BaseException as error:
        _write(record / "manifest.json", {"schema": SCHEMA, "run_id": record.name, "workflow_id": workflow_id,
                "status": "launcher_failed", "failure": f"{type(error).__name__}: {error}",
                "ended_utc": datetime.now(timezone.utc).isoformat(), "exit_code": 2})
        raise
    manifest["_monotonic"] = time.monotonic()
    _write(record / "manifest.json", {k: v for k, v in manifest.items() if not k.startswith("_")})
    code = None
    output = None
    status = "failed"
    failure = None
    child = None
    output_thread = None
    log = None
    previous = {}
    try:
        command, output = _runner_command(root, row, argv, record=record)
        command[0] = _select_python(row, argv)
        manifest["command"] = redact_argv(command)
        manifest["output"] = str(output) if output else None
        env = os.environ.copy()
        env[MANAGED_CHILD] = "1"
        print(f"Simulation record: {record / 'manifest.json'}", flush=True)
        log = (record / "console.log").open("xb")
        child = subprocess.Popen(command, cwd=root, env=env, stdin=None, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                 start_new_session=True)
        def tee():
            assert child is not None and child.stdout is not None
            for chunk in iter(lambda: os.read(child.stdout.fileno(), 4096), b""):
                log.write(chunk)
                log.flush()
                try:
                    sys.stdout.buffer.write(chunk)
                    sys.stdout.buffer.flush()
                except (AttributeError, BrokenPipeError, OSError):
                    pass
        output_thread = threading.Thread(target=tee, daemon=True)
        output_thread.start()
        manifest["child_pid"] = child.pid
        _write(record / "manifest.json", {k: v for k, v in manifest.items() if not k.startswith("_")})
        def forward(signum, _frame):
            if child and child.poll() is None:
                os.killpg(child.pid, signum)
            raise KeyboardInterrupt
        for sig in (signal.SIGINT, signal.SIGTERM):
            previous[sig] = signal.signal(sig, forward)
        try:
            code = child.wait(timeout=timeout)
            status = "process_completed" if code == 0 else "process_failed"
        except subprocess.TimeoutExpired:
            status, failure = "timeout", f"timeout after {timeout}s"
            os.killpg(child.pid, signal.SIGTERM)
            try:
                child.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait()
            code = 124
    except KeyboardInterrupt:
        status, failure, code = "interrupted", "keyboard interrupt", 130
        if child is not None and child.poll() is None:
            os.killpg(child.pid, signal.SIGTERM)
            try:
                child.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait()
    except BaseException as error:
        failure = f"{type(error).__name__}: {error}"
        status, code = "launcher_failed", 2
    finally:
        for sig, handler in previous.items():
            signal.signal(sig, handler)
        if child is not None:
            # The leader can exit while its children continue running. Always
            # clean our own process group, including on process exit 0.
            for sig in (signal.SIGTERM, signal.SIGKILL):
                try:
                    os.killpg(child.pid, sig)
                except ProcessLookupError:
                    break
                if sig == signal.SIGTERM:
                    time.sleep(0.1)
            child.wait()
        if output_thread is not None:
            output_thread.join(timeout=3)
        if child is not None and child.stdout is not None:
            child.stdout.close()
        if log is not None:
            log.close()
        _finish(manifest, record, exit_code=code, output=output, status=status, failure=failure)
    return record


def run_inprocess(root: Path, workflow_id: str, argv: list[str], invoke, *, output: Path | None,
                  inputs: list[Path] | None = None) -> int:
    if os.environ.get(MANAGED_CHILD) == "1":
        return invoke(output)
    root = root.resolve()
    if workflow_id == "local":
        _validate_input_paths(root, workflow_id, argv)
    if output is not None and output.exists():
        raise FileExistsError(f"workflow output already exists: {output}")
    record = _new_record(root, workflow_id, None)
    try:
        manifest = _base_manifest(root, workflow_id, argv, inputs or [], record, mode="in_process")
    except BaseException as error:
        _write(record / "manifest.json", {"schema": SCHEMA, "run_id": record.name, "workflow_id": workflow_id,
                "status": "launcher_failed", "failure": f"{type(error).__name__}: {error}",
                "ended_utc": datetime.now(timezone.utc).isoformat(), "exit_code": 2})
        raise
    manifest["_monotonic"] = time.monotonic()
    chosen = output or record / "artifacts"
    code = None
    status = "failed"
    failure = None
    try:
        code = invoke(chosen)
        status = "process_completed" if code == 0 else "process_failed"
        return code
    except KeyboardInterrupt:
        status, failure, code = "interrupted", "keyboard interrupt", 130
        raise
    except BaseException as error:
        status, failure, code = "launcher_failed", f"{type(error).__name__}: {error}", 2
        raise
    finally:
        _finish(manifest, record, exit_code=code, output=chosen, status=status, failure=failure)
        print(f"Simulation record: {record / 'manifest.json'}", flush=True)


def workflow_cli(argv: list[str], *, root: Path) -> int:
    parser = argparse.ArgumentParser(prog="sim_cli workflow", description="Plan, execute, and inspect versioned simulation workflows")
    sub = parser.add_subparsers(dest="action", required=True)
    sub.add_parser("list")
    for name in ("plan", "run"):
        command = sub.add_parser(name)
        command.add_argument("workflow_id")
        if name == "run":
            command.add_argument("--record", type=Path)
            command.add_argument("--timeout", type=float)
        command.add_argument("--input", type=Path, action="append", default=[])
    sub.add_parser("runs")
    show = sub.add_parser("show")
    show.add_argument("run_id")
    head, tail = [], []
    if "--" in argv:
        index = argv.index("--")
        head, tail = argv[:index], argv[index + 1:]
    else:
        head = argv
    args = parser.parse_args(head)
    try:
        if args.action == "list":
            data, digest = catalog(root)
            print(json.dumps({"catalog_sha256": digest, "workflows": data["workflows"]}, indent=2, ensure_ascii=False))
        elif args.action == "plan":
            print(json.dumps(plan(root, args.workflow_id, tail, inputs=args.input), indent=2, ensure_ascii=False))
        elif args.action == "run":
            record = run_workflow(root, args.workflow_id, tail, inputs=args.input,
                                  record_dir=args.record, timeout=args.timeout)
            manifest = json.loads((record / "manifest.json").read_text())
            print(f"Simulation record: {record / 'manifest.json'}")
            return int(manifest["exit_code"] or 0)
        elif args.action == "runs":
            for path in _records(root):
                try:
                    row = json.loads(path.read_text())
                    print(f"{row['run_id']}\t{row['workflow_id']}\t{row['status']}\tphysical_success={row.get('physical_success')}")
                except (OSError, ValueError, KeyError):
                    continue
        elif args.action == "show":
            name = args.run_id
            if "/" in name:
                path = Path(name).expanduser().resolve() / "manifest.json"
            else:
                if not re.fullmatch(r"[A-Za-z0-9._-]+", name) or name in (".", ".."):
                    raise ValueError("run_id must be a record directory name or explicit directory path")
                matches = [p for p in _records(root) if p.parent.name == name]
                if len(matches) != 1:
                    raise ValueError(f"run_id has {len(matches)} matches; use an explicit record path")
                path = matches[0]
            print(path.read_text(encoding="utf-8"))
        return 0
    except (ValueError, OSError) as error:
        parser.exit(2, f"workflow: {error}\n")
