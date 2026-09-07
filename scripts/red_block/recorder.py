"""Low-overhead REAL execution recorder for later sim replay/calibration.

The recorder is deliberately opt-in.  Production/SIM code is unchanged unless
``UGRP_REAL_TRACE_ENABLE=1`` and ``UGRP_REAL_TRACE_DIR`` are provided by the
Oracle-side deploy wrapper.  Recording failures are swallowed: observability
must never become an actuator failure.
"""
from __future__ import annotations

import atexit
import json
import os
import threading
import time
from collections import OrderedDict
from pathlib import Path
from typing import Any, Mapping


_SCHEMA_VERSION = 2
_LOCK = threading.Lock()
_FRAME_SEQ = 0
_EVENT_SEQ = 0
_LAST_SAVED_MONO = float("-inf")
_FRAME_IDS: "OrderedDict[int, int]" = OrderedDict()
_STATS = {
    "events": 0,
    "event_write_total_ms": 0.0,
    "event_write_max_ms": 0.0,
    "frames_seen": 0,
    "frames_saved": 0,
    "frame_record_total_ms": 0.0,
    "frame_record_max_ms": 0.0,
    "jpeg_save_total_ms": 0.0,
    "jpeg_save_max_ms": 0.0,
    "pose_read_total_ms": 0.0,
    "pose_read_max_ms": 0.0,
}
_SUMMARY_WRITTEN = False


def enabled() -> bool:
    return os.environ.get("UGRP_REAL_TRACE_ENABLE", "").strip().lower() in {"1", "true", "yes", "on"} and bool(
        os.environ.get("UGRP_REAL_TRACE_DIR")
    )


def trace_dir() -> Path | None:
    if not enabled():
        return None
    try:
        path = Path(os.environ["UGRP_REAL_TRACE_DIR"])
        path.mkdir(parents=True, exist_ok=True)
        return path
    except Exception:
        return None


def _jsonable(value: Any) -> Any:
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, Mapping):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [_jsonable(v) for v in value]
    try:
        item = value.item()
    except Exception:
        return str(value)
    return _jsonable(item)


def event(kind: str, **payload: Any) -> float | None:
    """Append one structured event and return local write cost in milliseconds.

    Callers may ignore the return value. Recording failures remain non-fatal.
    """
    global _EVENT_SEQ
    root = trace_dir()
    if root is None:
        return None
    with _LOCK:
        _EVENT_SEQ += 1
        event_seq = _EVENT_SEQ
    record = {
        "schema_version": _SCHEMA_VERSION,
        "event_seq": event_seq,
        "run_id": os.environ.get("UGRP_REAL_RUN_ID"),
        "span_id": os.environ.get("UGRP_REAL_TRACE_SPAN_ID"),
        "kind": str(kind),
        "wall_time_s": time.time(),
        "monotonic_s": time.monotonic(),
        "pid": os.getpid(),
        "skill": os.environ.get("UGRP_REAL_TRACE_SKILL"),
        "phase": os.environ.get("UGRP_REAL_TRACE_PHASE") or os.environ.get("UGRP_REAL_TRACE_SKILL"),
        **{str(k): _jsonable(v) for k, v in payload.items()},
    }
    started = time.perf_counter()
    try:
        line = json.dumps(record, ensure_ascii=False, sort_keys=True, separators=(",", ":")) + "\n"
        with _LOCK:
            with (root / "events.jsonl").open("a", encoding="utf-8") as fh:
                fh.write(line)
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        with _LOCK:
            _STATS["events"] += 1
            _STATS["event_write_total_ms"] += elapsed_ms
            _STATS["event_write_max_ms"] = max(_STATS["event_write_max_ms"], elapsed_ms)
        return elapsed_ms
    except Exception:
        return None


def _remember_frame(frame: Any, seq: int) -> None:
    try:
        key = id(frame)
        _FRAME_IDS[key] = int(seq)
        _FRAME_IDS.move_to_end(key)
        while len(_FRAME_IDS) > 128:
            _FRAME_IDS.popitem(last=False)
    except Exception:
        pass


def frame_seq_for(frame: Any) -> int | None:
    try:
        return _FRAME_IDS.get(id(frame))
    except Exception:
        return None


def _commanded_pose_snapshot() -> tuple[dict[int, int] | None, float | None]:
    """Read the latest commanded PWM pose without depending on robot.py."""
    path = Path(os.environ.get("UGRP_ROBOT_POSE_STATE", "/tmp/ugrp-masterpi-pose.json"))
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
        pose_raw = raw.get("pose") if isinstance(raw, dict) else None
        updated_at = float(raw.get("updated_at")) if isinstance(raw, dict) else None
        if not isinstance(pose_raw, dict) or updated_at is None:
            return None, None
        pose = {int(k): int(v) for k, v in pose_raw.items()}
        return pose, max(0.0, time.time() - updated_at)
    except (OSError, TypeError, ValueError, json.JSONDecodeError):
        return None, None


def record_camera_frame(frame: Any, *, source: str | None = None, force: bool = False, label: str | None = None) -> int | None:
    """Record metadata for every frame and a throttled JPEG sample.

    By default JPEG encoding is throttled (4 Hz). Production REAL tracing sets
    ``UGRP_REAL_TRACE_ALL_FRAMES=1`` so every frame actually consumed by a
    controller decision is preserved; unused MJPEG stream frames are not.
    Important/debug frames can also request ``force=True``.
    """
    global _FRAME_SEQ, _LAST_SAVED_MONO
    record_started = time.perf_counter()
    if not enabled() or frame is None:
        return None
    root = trace_dir()
    if root is None:
        return None
    with _LOCK:
        _FRAME_SEQ += 1
        seq = _FRAME_SEQ
    _remember_frame(frame, seq)
    now = time.monotonic()
    all_frames = os.environ.get("UGRP_REAL_TRACE_ALL_FRAMES", "").strip().lower() in {"1", "true", "yes", "on"}
    try:
        interval = max(0.05, float(os.environ.get("UGRP_REAL_TRACE_FRAME_INTERVAL_S", "0.25")))
    except ValueError:
        interval = 0.25
    save = bool(force or all_frames or (now - _LAST_SAVED_MONO >= interval))
    rel_path: str | None = None
    image_save_error: str | None = None
    jpeg_ms = 0.0
    if save:
        jpeg_started = time.perf_counter()
        try:
            import cv2

            frames_dir = root / "frames"
            frames_dir.mkdir(parents=True, exist_ok=True)
            quality = int(os.environ.get("UGRP_REAL_TRACE_JPEG_QUALITY", "60"))
            quality = max(35, min(90, quality))
            suffix = "" if not label else "-" + "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in str(label))[:48]
            path = frames_dir / f"frame-{seq:06d}{suffix}.jpg"
            ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
            if ok:
                path.write_bytes(encoded.tobytes())
                rel_path = str(path.relative_to(root))
                _LAST_SAVED_MONO = now
        except Exception as exc:
            rel_path = None
            image_save_error = f"{type(exc).__name__}: {exc}"
        finally:
            jpeg_ms = (time.perf_counter() - jpeg_started) * 1000.0
    try:
        shape = [int(v) for v in frame.shape]
    except Exception:
        shape = None
    pose_started = time.perf_counter()
    pose, pose_age_s = _commanded_pose_snapshot()
    pose_ms = (time.perf_counter() - pose_started) * 1000.0
    pre_event_ms = (time.perf_counter() - record_started) * 1000.0
    event(
        "camera_frame",
        frame_seq=seq,
        source=source,
        shape=shape,
        sampled_jpeg=rel_path,
        important=bool(force),
        label=label,
        commanded_pose=pose,
        pose_state_age_s=pose_age_s,
        image_save_error=image_save_error,
        jpeg_save_ms=round(jpeg_ms, 4),
        pose_read_ms=round(pose_ms, 4),
        pre_event_record_ms=round(pre_event_ms, 4),
    )
    total_ms = (time.perf_counter() - record_started) * 1000.0
    with _LOCK:
        _STATS["frames_seen"] += 1
        if rel_path:
            _STATS["frames_saved"] += 1
        _STATS["frame_record_total_ms"] += total_ms
        _STATS["frame_record_max_ms"] = max(_STATS["frame_record_max_ms"], total_ms)
        _STATS["jpeg_save_total_ms"] += jpeg_ms
        _STATS["jpeg_save_max_ms"] = max(_STATS["jpeg_save_max_ms"], jpeg_ms)
        _STATS["pose_read_total_ms"] += pose_ms
        _STATS["pose_read_max_ms"] = max(_STATS["pose_read_max_ms"], pose_ms)
    return seq


def _write_recorder_summary() -> None:
    """Best-effort one-shot aggregate overhead summary for completed skill processes."""
    global _SUMMARY_WRITTEN
    if _SUMMARY_WRITTEN or not enabled():
        return
    _SUMMARY_WRITTEN = True
    root = trace_dir()
    if root is None:
        return
    with _LOCK:
        stats = dict(_STATS)
    frames = int(stats.get("frames_seen") or 0)
    events = int(stats.get("events") or 0)
    payload = {
        "schema_version": _SCHEMA_VERSION,
        "event_seq": _EVENT_SEQ + 1,
        "run_id": os.environ.get("UGRP_REAL_RUN_ID"),
        "span_id": os.environ.get("UGRP_REAL_TRACE_SPAN_ID"),
        "kind": "recorder_summary",
        "wall_time_s": time.time(),
        "monotonic_s": time.monotonic(),
        "pid": os.getpid(),
        "skill": os.environ.get("UGRP_REAL_TRACE_SKILL"),
        "phase": os.environ.get("UGRP_REAL_TRACE_PHASE") or os.environ.get("UGRP_REAL_TRACE_SKILL"),
        "frames_seen": frames,
        "frames_saved": int(stats.get("frames_saved") or 0),
        "events_before_summary": events,
        "frame_record_avg_ms": round(float(stats.get("frame_record_total_ms") or 0.0) / frames, 4) if frames else 0.0,
        "frame_record_max_ms": round(float(stats.get("frame_record_max_ms") or 0.0), 4),
        "jpeg_save_avg_ms": round(float(stats.get("jpeg_save_total_ms") or 0.0) / frames, 4) if frames else 0.0,
        "jpeg_save_max_ms": round(float(stats.get("jpeg_save_max_ms") or 0.0), 4),
        "pose_read_avg_ms": round(float(stats.get("pose_read_total_ms") or 0.0) / frames, 4) if frames else 0.0,
        "pose_read_max_ms": round(float(stats.get("pose_read_max_ms") or 0.0), 4),
        "event_write_avg_ms": round(float(stats.get("event_write_total_ms") or 0.0) / events, 4) if events else 0.0,
        "event_write_max_ms": round(float(stats.get("event_write_max_ms") or 0.0), 4),
    }
    try:
        line = json.dumps(payload, ensure_ascii=False, sort_keys=True, separators=(",", ":")) + "\n"
        with _LOCK:
            with (root / "events.jsonl").open("a", encoding="utf-8") as fh:
                fh.write(line)
    except Exception:
        pass


atexit.register(_write_recorder_summary)


def _blob_payload(blob: Any) -> dict[str, Any] | None:
    if blob is None:
        return None
    keys = ("cx", "cy", "area", "width", "height", "nx", "ny", "box_points", "rectangularity")
    return {key: _jsonable(getattr(blob, key)) for key in keys if hasattr(blob, key)}


def record_detection(
    frame: Any,
    *,
    color: str,
    blob: Any,
    detector: str,
    min_area: int | None = None,
    crop_left: int | None = None,
) -> None:
    event(
        "detection",
        frame_seq=frame_seq_for(frame),
        color=str(color),
        detector=str(detector),
        visible=blob is not None,
        blob=_blob_payload(blob),
        min_area=min_area,
        crop_left=crop_left,
    )


def record_pose(pose: Mapping[int, int] | None, *, reason: str, pose_state_age_s: float | None = None) -> None:
    event(
        "commanded_pose",
        reason=reason,
        pose={} if pose is None else {int(k): int(v) for k, v in pose.items()},
        pose_state_age_s=pose_state_age_s,
    )
