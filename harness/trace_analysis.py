"""Deterministic RCA summary for REAL execution trace directories."""
from __future__ import annotations

import json
import re
from pathlib import Path
from typing import Any


ERROR_RE = re.compile(r"^error:\s*(.+)$", re.IGNORECASE)



def _classify_failure(reason: str | None, failure_code: str | None, evidence: dict[str, Any]) -> dict[str, Any]:
    text = (reason or "").lower()
    code = (failure_code or "").upper()
    transition = evidence.get("visibility_transition") if isinstance(evidence, dict) else None
    commands = transition.get("commands_between", []) if isinstance(transition, dict) else []
    command_kinds = {str(item.get("kind") or "") for item in commands if isinstance(item, dict)}

    if "mjpeg" in text or "camera" in text and any(word in text for word in ("stream", "timeout", "transport", "http")):
        return {"category": "CAMERA_TRANSPORT_FAILURE", "confidence": "software_proven", "basis": "camera transport error was recorded explicitly"}
    if code == "OBJECT_ALREADY_CARRIED" or "object_already_carried" in text:
        return {"category": "EXECUTIVE_STATE_CONFLICT", "confidence": "software_proven", "basis": "executive precondition rejected acquisition because carry state was occupied"}
    if "carry handoff" in text or "handoff" in text and any(word in text for word in ("missing", "stale", "changed", "invalid")):
        return {"category": "CAUSAL_HANDOFF_INVALID", "confidence": "software_proven", "basis": "causal handoff contract failed before unsafe continuation"}
    if "range changed implausibly" in text or "range" in text and "implaus" in text:
        return {"category": "RANGE_INCONSISTENCY_AFTER_MOTION", "confidence": "sensor_observed", "basis": "range observation violated the controller's physically plausible bound"}
    if "diagonal" in text or "face" in text and any(word in text for word in ("align", "normal")) and any(word in text for word in ("failed", "remains", "could not")):
        return {"category": "FACE_ALIGNMENT_NONCONVERGENCE", "confidence": "sensor_observed", "basis": "vision face-normal alignment failed its bounded convergence guard"}
    if "pan limit" in text:
        return {"category": "CAMERA_ALIGNMENT_LIMIT", "confidence": "sensor_observed", "basis": "camera alignment exhausted the allowed pan envelope"}
    if "still visible after" in text or "grasp" in text and any(word in text for word in ("miss", "visible")):
        return {"category": "GRASP_VISUAL_MISS", "confidence": "sensor_observed", "basis": "post-grasp vision still supported the target remaining at the pickup site"}
    if transition:
        if "servo_command" in command_kinds or "servo_batch_command" in command_kinds:
            return {"category": "TARGET_LOST_AFTER_ARM_POSE_CHANGE", "confidence": "sensor_observed", "basis": "target changed visible→lost after recorded servo actuation; mechanical/occlusion cause is not directly sensed"}
        if "chassis_command" in command_kinds:
            return {"category": "TARGET_LOST_AFTER_CHASSIS_MOTION", "confidence": "sensor_observed", "basis": "target changed visible→lost after recorded chassis actuation; slip/actual displacement is not directly sensed"}
        return {"category": "TARGET_LOST_WITHOUT_RECORDED_ACTUATION", "confidence": "sensor_observed", "basis": "vision changed visible→lost without an actuator command recorded between the paired observations"}
    if code:
        return {"category": code, "confidence": "software_proven", "basis": "structured failure code emitted by the execution stack"}
    return {"category": "UNCLASSIFIED_FAILURE", "confidence": "unknown", "basis": "failure is recorded but evidence is insufficient for a more specific deterministic class"}


def _read_json(path: Path) -> dict[str, Any]:
    try:
        obj = json.loads(path.read_text(encoding="utf-8"))
        return obj if isinstance(obj, dict) else {}
    except Exception:
        return {}


def _read_jsonl(path: Path) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    try:
        for line in path.read_text(encoding="utf-8").splitlines():
            try:
                item = json.loads(line)
            except json.JSONDecodeError:
                continue
            if isinstance(item, dict):
                out.append(item)
    except OSError:
        pass
    return out


def _last_error(stderr_path: Path) -> str | None:
    try:
        lines = stderr_path.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError:
        return None
    matches = []
    for line in lines:
        m = ERROR_RE.match(line.strip())
        if m:
            matches.append(m.group(1).strip())
    return matches[-1] if matches else next((line.strip() for line in reversed(lines) if line.strip()), None)


def _event_time(item: dict[str, Any]) -> float:
    try:
        return float(item.get("wall_time_s", 0.0))
    except (TypeError, ValueError):
        return 0.0


def _frame_for_detection(events: list[dict[str, Any]], detection: dict[str, Any] | None) -> dict[str, Any] | None:
    if not detection:
        return None
    seq = detection.get("frame_seq")
    candidates = [e for e in events if e.get("kind") == "camera_frame" and e.get("frame_seq") == seq]
    return candidates[-1] if candidates else None


def _last(events: list[dict[str, Any]], kinds: set[str], *, before: float | None = None) -> dict[str, Any] | None:
    candidates = [e for e in events if e.get("kind") in kinds and (before is None or _event_time(e) <= before)]
    return max(candidates, key=_event_time) if candidates else None


def _normalize_image(run_dir: Path, span_dir: Path, frame: dict[str, Any] | None) -> str | None:
    if not frame:
        return None
    rel = frame.get("sampled_jpeg") or frame.get("image_path")
    if not rel:
        return None
    path = (span_dir / str(rel)).resolve()
    try:
        return str(path.relative_to(run_dir.resolve()))
    except ValueError:
        return str(path)


def analyze_run(run_dir: str | Path, *, write: bool = True) -> dict[str, Any]:
    run_dir = Path(run_dir)
    meta = _read_json(run_dir / "run.json")
    top_result = _read_json(run_dir / "result.json")
    harness_events = _read_jsonl(run_dir / "events.jsonl")
    spans: list[dict[str, Any]] = []
    for span_dir in sorted((run_dir / "skills").glob("*")) if (run_dir / "skills").exists() else []:
        if not span_dir.is_dir():
            continue
        result = _read_json(span_dir / "result.json")
        events = _read_jsonl(span_dir / "events.jsonl")
        recorder_summary = _last(events, {"recorder_summary"})
        stderr_reason = _last_error(span_dir / "stderr.log")
        spans.append({
            "dir": span_dir, "result": result, "events": events,
            "stderr_reason": stderr_reason, "recorder_summary": recorder_summary,
        })

    spans.sort(key=lambda s: float(s["result"].get("started_wall_s", 0.0) or 0.0))

    harness_failure = None
    for event in harness_events:
        if event.get("kind") != "harness_step":
            continue
        step_error = event.get("error")
        result = event.get("result") if isinstance(event.get("result"), dict) else {}
        nested = result.get("result") if isinstance(result.get("result"), dict) else result
        if step_error or result.get("ok") is False or (isinstance(nested, dict) and nested.get("execution_status") == "FAILED"):
            harness_failure = event
            break

    failed_spans = [s for s in spans if int(s["result"].get("exit_code", 0) or 0) != 0]
    first_failed_span = min(failed_spans, key=lambda s: float(s["result"].get("started_wall_s", 0.0) or 0.0)) if failed_spans else None
    failure_time = None
    if first_failed_span:
        failure_time = float(first_failed_span["result"].get("ended_wall_s", 0.0) or 0.0)
    elif harness_failure:
        failure_time = _event_time(harness_failure)

    evidence: dict[str, Any] = {}
    if first_failed_span:
        events = first_failed_span["events"]
        span_dir = first_failed_span["dir"]
        before = failure_time
        detection = _last(events, {"detection"}, before=before)
        pose = _last(events, {"commanded_pose"}, before=before)
        command = _last(events, {"chassis_command", "servo_command", "servo_batch_command", "motor_stop"}, before=before)
        frame = _frame_for_detection(events, detection) or _last(events, {"camera_frame"}, before=before)
        camera_frames = [e for e in events if e.get("kind") == "camera_frame" and (before is None or _event_time(e) <= before)]
        camera_frames.sort(key=_event_time)
        previous_frame = camera_frames[-2] if len(camera_frames) >= 2 else None
        detections = [e for e in events if e.get("kind") == "detection" and (before is None or _event_time(e) <= before)]
        detections.sort(key=_event_time)
        visibility_transition = None
        if detections and detections[-1].get("visible") is False:
            prior_visible = next((e for e in reversed(detections[:-1]) if e.get("visible") is True), None)
            if prior_visible is not None:
                prior_frame = _frame_for_detection(events, prior_visible)
                lost_frame = _frame_for_detection(events, detections[-1])
                start_t = _event_time(prior_visible)
                end_t = _event_time(detections[-1])
                causal_commands = [
                    e for e in events
                    if e.get("kind") in {"chassis_command", "servo_command", "servo_batch_command", "motor_stop", "commanded_pose"}
                    and start_t <= _event_time(e) <= end_t
                ]
                causal_commands.sort(key=_event_time)
                visibility_transition = {
                    "from_visible": True,
                    "to_visible": False,
                    "before_detection": prior_visible,
                    "after_detection": detections[-1],
                    "before_frame": _normalize_image(run_dir, span_dir, prior_frame),
                    "after_frame": _normalize_image(run_dir, span_dir, lost_frame),
                    "before_commanded_pose": (prior_frame or {}).get("commanded_pose"),
                    "after_commanded_pose": (lost_frame or {}).get("commanded_pose"),
                    "commands_between": causal_commands,
                }
        try:
            stdout_lines = [line.strip() for line in (span_dir / "stdout.log").read_text(encoding="utf-8", errors="replace").splitlines() if line.strip()]
        except OSError:
            stdout_lines = []
        evidence = {
            "last_detection": detection,
            "last_commanded_pose": pose,
            "last_actuator_command": command,
            "last_input_frame": _normalize_image(run_dir, span_dir, frame),
            "previous_input_frame": _normalize_image(run_dir, span_dir, previous_frame),
            "visibility_transition": visibility_transition,
            "decision_log_tail": stdout_lines[-25:],
            "skill_stdout_log": str((span_dir / "stdout.log").relative_to(run_dir)) if (span_dir / "stdout.log").exists() else None,
            "skill_stderr_log": str((span_dir / "stderr.log").relative_to(run_dir)) if (span_dir / "stderr.log").exists() else None,
        }

    reason = None
    failure_code = None
    failed_skill = None
    if harness_failure:
        result = harness_failure.get("result") if isinstance(harness_failure.get("result"), dict) else {}
        nested = result.get("result") if isinstance(result.get("result"), dict) else result
        if isinstance(nested, dict):
            reason = nested.get("reason")
            failure_code = nested.get("failure_code")
            failed_skill = nested.get("skill") or result.get("tool")
        reason = reason or harness_failure.get("error")
    if first_failed_span:
        sr = first_failed_span["result"]
        failed_skill = failed_skill or sr.get("skill") or Path(str(sr.get("script") or "")).stem
        reason = reason or first_failed_span.get("stderr_reason")

    failure_boundary = None
    if first_failed_span:
        failed_idx = spans.index(first_failed_span)
        previous_success = next(
            (s for s in reversed(spans[:failed_idx]) if int(s["result"].get("exit_code", 0) or 0) == 0),
            None,
        )
        failure_boundary = {
            "last_successful_skill": (
                previous_success["result"].get("skill")
                or Path(str(previous_success["result"].get("script") or "")).stem
            ) if previous_success else None,
            "first_failed_skill": failed_skill,
            "first_failed_span_id": first_failed_span["result"].get("span_id") or first_failed_span["dir"].name,
        }
    elif harness_failure:
        failure_boundary = {
            "last_successful_skill": None,
            "first_failed_skill": failed_skill or "harness",
            "first_failed_span_id": None,
        }

    root_cause = _classify_failure(reason, failure_code, evidence) if (harness_failure or first_failed_span or top_result.get("error")) else None

    has_failure_evidence = bool(harness_failure or first_failed_span or top_result.get("error"))
    loop_result = top_result.get("result") if isinstance(top_result.get("result"), dict) else {}
    run_reached_final = loop_result.get("stopped") == "final"
    if top_result.get("error"):
        status = "FAILED"
    elif has_failure_evidence and run_reached_final:
        status = "COMPLETED_WITH_FAILURES"
    elif has_failure_evidence:
        status = "FAILED"
    else:
        status = "COMPLETED"
    observability = {
        "trace_copy_total_s": round(sum(float(s["result"].get("trace_copy_duration_s") or 0.0) for s in spans), 4),
        "skill_execution_total_s": round(sum(float(s["result"].get("execution_duration_s") or 0.0) for s in spans), 4),
        "recorder": [
            {
                "span_id": s["result"].get("span_id") or s["dir"].name,
                "skill": s["result"].get("skill") or Path(str(s["result"].get("script") or "")).stem,
                "frames_seen": (s.get("recorder_summary") or {}).get("frames_seen"),
                "frames_saved": (s.get("recorder_summary") or {}).get("frames_saved"),
                "frame_record_avg_ms": (s.get("recorder_summary") or {}).get("frame_record_avg_ms"),
                "frame_record_max_ms": (s.get("recorder_summary") or {}).get("frame_record_max_ms"),
                "jpeg_save_avg_ms": (s.get("recorder_summary") or {}).get("jpeg_save_avg_ms"),
                "pose_read_avg_ms": (s.get("recorder_summary") or {}).get("pose_read_avg_ms"),
                "event_write_avg_ms": (s.get("recorder_summary") or {}).get("event_write_avg_ms"),
                "trace_copy_duration_s": s["result"].get("trace_copy_duration_s"),
            }
            for s in spans
            if s.get("recorder_summary") or s["result"].get("trace_copy_duration_s") is not None
        ],
    }

    summary = {
        "schema_version": 2,
        "run_id": meta.get("run_id") or run_dir.name,
        "status": status,
        "has_failure_evidence": has_failure_evidence,
        "run_reached_final": run_reached_final,
        "user_command": meta.get("message"),
        "skill_count": len(spans),
        "observability": observability,
        "failure_boundary": failure_boundary,
        "root_cause": root_cause,
        "skills": [
            {
                "span_id": s["result"].get("span_id") or s["dir"].name,
                "skill": s["result"].get("skill") or Path(str(s["result"].get("script") or "")).stem,
                "exit_code": s["result"].get("exit_code"),
                "duration_s": s["result"].get("duration_s"),
                "execution_duration_s": s["result"].get("execution_duration_s"),
                "trace_copy_duration_s": s["result"].get("trace_copy_duration_s"),
                "recorder_summary": s.get("recorder_summary"),
                "trace_dir": str(s["dir"].relative_to(run_dir)),
            }
            for s in spans
        ],
        "failure": None if not has_failure_evidence else {
            "skill": failed_skill,
            "failure_code": failure_code,
            "reason": reason or "failure detected but no explicit reason was recorded",
            "wall_time_s": failure_time,
        },
        "evidence": evidence,
        "limitations": [
            "Servo pose is the commanded PWM state; MasterPi has no independent joint encoder feedback in this stack.",
            "Chassis motion records commands and timing; no validated wheel odometry is available for exact traveled distance.",
        ],
    }

    if write:
        (run_dir / "analysis.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        lines = [
            f"# REAL Trace Analysis — {summary['run_id']}",
            "",
            f"- status: **{status}**",
            f"- command: `{summary.get('user_command') or ''}`",
            f"- skill spans: {len(spans)}",
        ]
        if summary["failure"]:
            f = summary["failure"]
            lines += [
                f"- first failure skill: **{f.get('skill') or 'unknown'}**",
                f"- failure code: `{f.get('failure_code') or 'unknown'}`",
                f"- reason: {f.get('reason')}",
            ]
            if root_cause:
                lines.append(
                    f"- RCA class: **{root_cause.get('category')}** "
                    f"({root_cause.get('confidence')}) — {root_cause.get('basis')}"
                )
            if failure_boundary:
                lines.append(
                    f"- boundary: `{failure_boundary.get('last_successful_skill') or 'start'}` → "
                    f"`{failure_boundary.get('first_failed_skill') or 'unknown'}`"
                )
        if evidence:
            lines += ["", "## Failure evidence"]
            if evidence.get("last_input_frame"):
                lines.append(f"- last input frame: `{evidence['last_input_frame']}`")
            if evidence.get("previous_input_frame"):
                lines.append(f"- previous input frame: `{evidence['previous_input_frame']}`")
            pose = evidence.get("last_commanded_pose") or {}
            if pose.get("pose") is not None:
                lines.append(f"- commanded pose: `{json.dumps(pose.get('pose'), ensure_ascii=False, sort_keys=True)}`")
            det = evidence.get("last_detection") or {}
            if det:
                lines.append(f"- last detection: `{json.dumps({k: det.get(k) for k in ('color','visible','blob','detector')}, ensure_ascii=False)}`")
            transition = evidence.get("visibility_transition") or {}
            if transition:
                lines.append(
                    f"- visibility transition: visible → lost; before=`{transition.get('before_frame')}`, "
                    f"after=`{transition.get('after_frame')}`"
                )
                if transition.get("commands_between"):
                    lines.append(
                        f"- commands between visible/lost frames: "
                        f"`{json.dumps(transition.get('commands_between'), ensure_ascii=False, sort_keys=True)}`"
                    )
            cmd = evidence.get("last_actuator_command") or {}
            if cmd:
                lines.append(f"- last actuator command: `{json.dumps(cmd, ensure_ascii=False, sort_keys=True)}`")
            if evidence.get("decision_log_tail"):
                lines += ["", "### Controller decision tail", *[f"- {line}" for line in evidence["decision_log_tail"]]]
        if observability.get("recorder"):
            lines += [
                "", "## Trace overhead",
                f"- skill execution total: `{observability.get('skill_execution_total_s')} s`",
                f"- post-skill trace copy total: `{observability.get('trace_copy_total_s')} s`",
            ]
            for item in observability["recorder"]:
                if item.get("frame_record_avg_ms") is not None:
                    lines.append(
                        f"- {item.get('skill')}: recorder avg `{item.get('frame_record_avg_ms')} ms/frame`, "
                        f"max `{item.get('frame_record_max_ms')} ms/frame`, "
                        f"copy `{item.get('trace_copy_duration_s')} s`"
                    )
        lines += ["", "## Sensor limitations", *[f"- {x}" for x in summary["limitations"]]]
        (run_dir / "analysis.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return summary
