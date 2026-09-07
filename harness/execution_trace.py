"""Durable per-turn execution tracing for REAL MasterPi runs.

The web harness is multi-threaded, so request identity is carried with
``contextvars`` rather than process-global environment variables. Physical skill
children inherit a sanitized run id via ``scripts.robot_actions`` and create
separate Pi-side spans under the same run directory.
"""
from __future__ import annotations

import contextlib
import contextvars
import dataclasses
import hashlib
import json
import os
import threading
import time
import uuid
from pathlib import Path
from typing import Any, Iterator


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_TRACE_ROOT = PROJECT_ROOT / "outputs" / "real_traces"
_RUN_ID: contextvars.ContextVar[str | None] = contextvars.ContextVar("ugrp_real_run_id", default=None)
_RUN_DIR: contextvars.ContextVar[Path | None] = contextvars.ContextVar("ugrp_real_run_dir", default=None)
_WRITE_LOCK = threading.Lock()
_EVENT_SEQ: dict[str, int] = {}


def trace_root() -> Path:
    return Path(os.environ.get("UGRP_REAL_TRACE_ROOT", str(DEFAULT_TRACE_ROOT)))


def _jsonable(value: Any) -> Any:
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, Path):
        return str(value)
    if dataclasses.is_dataclass(value):
        return _jsonable(dataclasses.asdict(value))
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [_jsonable(v) for v in value]
    if hasattr(value, "__dict__"):
        return {"type": type(value).__name__, **_jsonable(vars(value))}
    return str(value)


def _new_run_id() -> str:
    stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    return f"{stamp}-real-{uuid.uuid4().hex[:8]}"


def current_run_id() -> str | None:
    return _RUN_ID.get()


def current_run_dir() -> Path | None:
    return _RUN_DIR.get()




def _safe_run_dir(run_id: str) -> Path | None:
    rid = str(run_id or "").strip()
    if not rid or Path(rid).name != rid or rid in {".", ".."}:
        return None
    display_root = trace_root()
    root = display_root.resolve()
    display_candidate = display_root / rid
    candidate = display_candidate.resolve()
    try:
        candidate.relative_to(root)
    except ValueError:
        return None
    if not candidate.is_dir():
        return None
    return display_candidate


def _read_json_file(path: Path) -> dict[str, Any]:
    try:
        obj = json.loads(path.read_text(encoding="utf-8"))
        return obj if isinstance(obj, dict) else {}
    except Exception:
        return {}


def _tree_size(path: Path) -> int:
    total = 0
    try:
        for item in path.rglob("*"):
            if item.is_file():
                try:
                    total += item.stat().st_size
                except OSError:
                    pass
    except OSError:
        pass
    return total


def trace_index(limit: int = 12) -> dict[str, Any]:
    """Return a small read-only index of recent REAL trace runs."""
    root = trace_root()
    try:
        dirs = [d for d in root.iterdir() if d.is_dir()]
    except OSError:
        dirs = []
    dirs.sort(key=lambda d: d.stat().st_mtime if d.exists() else 0.0, reverse=True)
    limit = max(1, min(int(limit), 50))
    runs: list[dict[str, Any]] = []
    for run_dir in dirs[:limit]:
        meta = _read_json_file(run_dir / "run.json")
        result = _read_json_file(run_dir / "result.json")
        analysis = _read_json_file(run_dir / "analysis.json")
        finished = bool(result)
        status = analysis.get("status") if analysis else ("RUNNING" if not finished else "UNANALYZED")
        failure = analysis.get("failure") if isinstance(analysis.get("failure"), dict) else None
        created = meta.get("created_wall_time_s")
        if created is None:
            try:
                created = run_dir.stat().st_mtime
            except OSError:
                created = None
        runs.append({
            "run_id": meta.get("run_id") or run_dir.name,
            "created_wall_time_s": created,
            "finished": finished,
            "status": status,
            "user_command": meta.get("message"),
            "skill_count": analysis.get("skill_count") if analysis else len(list((run_dir / "skills").glob("*"))) if (run_dir / "skills").exists() else 0,
            "failure": failure,
            "root_cause": analysis.get("root_cause") if analysis else None,
            "failure_boundary": analysis.get("failure_boundary") if analysis else None,
            "bytes": _tree_size(run_dir),
        })
    return {
        "schema_version": 2,
        "root": str(root),
        "run_count": len(dirs),
        "returned": len(runs),
        "bytes": sum(_tree_size(d) for d in dirs),
        "runs": runs,
    }


def trace_detail(run_id: str) -> dict[str, Any] | None:
    """Return metadata + automatic RCA for one run without mutating evidence."""
    run_dir = _safe_run_dir(run_id)
    if run_dir is None:
        return None
    meta = _read_json_file(run_dir / "run.json")
    result = _read_json_file(run_dir / "result.json")
    analysis = _read_json_file(run_dir / "analysis.json")
    if not analysis and result:
        try:
            from .trace_analysis import analyze_run
            analysis = analyze_run(run_dir, write=False)
        except Exception:
            analysis = {}
    if not result:
        # Active/incomplete runs should never be mislabeled as successful merely
        # because no failure has been recorded yet.
        analysis = {
            **analysis,
            "schema_version": 2,
            "run_id": meta.get("run_id") or run_dir.name,
            "status": "RUNNING",
            "user_command": meta.get("message"),
        }
    return {
        "schema_version": 2,
        "run_id": meta.get("run_id") or run_dir.name,
        "meta": meta,
        "result": result,
        "analysis": analysis,
        "bytes": _tree_size(run_dir),
    }


def trace_asset(run_id: str, relative_path: str) -> Path | None:
    """Resolve an image artifact while preventing path traversal."""
    run_dir = _safe_run_dir(run_id)
    if run_dir is None:
        return None
    rel = Path(str(relative_path or ""))
    if rel.is_absolute() or not rel.parts or any(part in {"..", ""} for part in rel.parts):
        return None
    display_candidate = run_dir / rel
    candidate = display_candidate.resolve()
    try:
        candidate.relative_to(run_dir.resolve())
    except ValueError:
        return None
    if not candidate.is_file() or candidate.suffix.lower() not in {".jpg", ".jpeg", ".png", ".webp"}:
        return None
    # Preserve the caller/root spelling (macOS may expose /var as
    # /private/var) after validating the canonical path above.
    return display_candidate
def create_run(
    *,
    message: str,
    execute: bool,
    initial_world_state: Any = None,
    initial_image: str | Path | None = None,
    initial_commanded_pose: Any = None,
    initial_pose_age_s: float | None = None,
    initial_pose_stable: bool | None = None,
    root: str | Path | None = None,
    source: str = "real_masterpi",
) -> tuple[str, Path]:
    root = Path(root) if root is not None else trace_root()
    root.mkdir(parents=True, exist_ok=True)
    run_id = _new_run_id()
    run_dir = root / run_id
    (run_dir / "harness_frames").mkdir(parents=True, exist_ok=False)
    (run_dir / "skills").mkdir(parents=True, exist_ok=True)
    meta = {
        "schema_version": 2,
        "run_id": run_id,
        "source": str(source),
        "created_wall_time_s": time.time(),
        "message": str(message),
        "execute": bool(execute),
        "initial_world_state": _jsonable(initial_world_state),
        "initial_commanded_pose": _jsonable(initial_commanded_pose),
        "initial_pose_age_s": initial_pose_age_s,
        "initial_pose_stable": initial_pose_stable,
        "pose_semantics": "commanded_pose_only_no_joint_encoder_feedback",
        "base_motion_semantics": "commanded_motion_only_no_wheel_odometry",
    }
    (run_dir / "run.json").write_text(json.dumps(meta, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    _EVENT_SEQ[run_id] = 0
    if initial_image:
        copy_frame(
            initial_image, label="turn-input", source="harness_initial", run_id=run_id, run_dir=run_dir,
            commanded_pose=initial_commanded_pose, pose_age_s=initial_pose_age_s, pose_stable=initial_pose_stable,
        )
    append_event(
        "turn_start", run_id=run_id, run_dir=run_dir, message=message, execute=execute,
        world_state=initial_world_state, commanded_pose=initial_commanded_pose,
        pose_age_s=initial_pose_age_s, pose_stable=initial_pose_stable,
    )
    return run_id, run_dir


@contextlib.contextmanager
def activate_run(run_id: str, run_dir: str | Path) -> Iterator[None]:
    run_dir = Path(run_dir)
    token_id = _RUN_ID.set(str(run_id))
    token_dir = _RUN_DIR.set(run_dir)
    try:
        yield
    finally:
        _RUN_DIR.reset(token_dir)
        _RUN_ID.reset(token_id)


def append_event(kind: str, *, run_id: str | None = None, run_dir: str | Path | None = None, **payload: Any) -> None:
    rid = run_id or current_run_id()
    directory = Path(run_dir) if run_dir is not None else current_run_dir()
    if not rid or directory is None:
        return
    record = {
        "schema_version": 2,
        "run_id": rid,
        "kind": str(kind),
        "wall_time_s": time.time(),
        "monotonic_s": time.monotonic(),
        **{str(k): _jsonable(v) for k, v in payload.items()},
    }
    try:
        with _WRITE_LOCK:
            seq = _EVENT_SEQ.get(rid, 0) + 1
            _EVENT_SEQ[rid] = seq
            record["event_seq"] = seq
            directory.mkdir(parents=True, exist_ok=True)
            with (directory / "events.jsonl").open("a", encoding="utf-8") as fh:
                fh.write(json.dumps(record, ensure_ascii=False, sort_keys=True, separators=(",", ":")) + "\n")
    except Exception:
        # Observability must never become the reason robot control fails.
        return


def copy_frame(
    path: str | Path,
    *,
    label: str,
    source: str,
    run_id: str | None = None,
    run_dir: str | Path | None = None,
    commanded_pose: Any = None,
    pose_age_s: float | None = None,
    pose_stable: bool | None = None,
) -> str | None:
    rid = run_id or current_run_id()
    directory = Path(run_dir) if run_dir is not None else current_run_dir()
    if not rid or directory is None:
        return None
    src = Path(path)
    try:
        data = src.read_bytes()
    except OSError:
        append_event("harness_frame_missing", run_id=rid, run_dir=directory, label=label, source=source, input_path=str(src))
        return None
    safe = "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in str(label))[:48] or "frame"
    frames = directory / "harness_frames"
    frames.mkdir(parents=True, exist_ok=True)
    with _WRITE_LOCK:
        existing = sum(1 for item in frames.iterdir() if item.is_file())
        suffix = src.suffix.lower() if src.suffix.lower() in {".jpg", ".jpeg", ".png", ".webp"} else ".jpg"
        dst = frames / f"{existing + 1:06d}-{safe}{suffix}"
        dst.write_bytes(data)
    rel = str(dst.relative_to(directory))
    append_event(
        "harness_frame",
        run_id=rid,
        run_dir=directory,
        label=label,
        source=source,
        image_path=rel,
        bytes=len(data),
        sha256=hashlib.sha256(data).hexdigest(),
        commanded_pose=commanded_pose,
        pose_age_s=pose_age_s,
        pose_stable=pose_stable,
    )
    return str(dst)


def record_step(step: Any) -> None:
    append_event(
        "harness_step",
        raw=getattr(step, "raw", None),
        action=getattr(step, "action", None),
        result=getattr(step, "result", None),
        error=getattr(step, "error", None),
    )


def record_activity(kind: str, name: str | None) -> None:
    append_event("harness_activity", activity=kind, tool=name)


def finish_run(result: Any = None, *, error: str | None = None) -> dict[str, Any] | None:
    rid = current_run_id()
    directory = current_run_dir()
    if not rid or directory is None:
        return None
    payload = {
        "schema_version": 2,
        "run_id": rid,
        "finished_wall_time_s": time.time(),
        "error": error,
        "result": _jsonable(result),
    }
    try:
        (directory / "result.json").write_text(json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    except Exception:
        pass
    append_event("turn_finish", error=error, result=result)
    analysis = None
    try:
        from .trace_analysis import analyze_run
        analysis = analyze_run(directory, write=True)
    except Exception as exc:
        append_event("analysis_error", error=str(exc))
    _EVENT_SEQ.pop(rid, None)
    return analysis
