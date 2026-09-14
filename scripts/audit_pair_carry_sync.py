#!/usr/bin/env python3
"""Replay a paired carry run from its saved actor inputs and referee trace."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path, PurePath
import sys
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from harness.camera_short_transport_student import predict_transport
from harness.pair_carry_policy import PairCarryPolicy, ROBOTS, payload_skew
from scripts.audit_camera_grasp_student import audit as audit_grasp
from scripts.evaluate_camera_short_transport import evaluate_transport_samples
from scripts.run_camera_short_transport_student import choose_carry_actions

ROW_KEYS = {
    "index", "observed_at_s", "request_wall_s", "decision_wall_s", "frame_ids",
    "images", "own_command_histories", "decisions", "skew_px", "delivered_reports",
    "control", "actions", "command_issued_wall_s", "execution_end_s", "execution_end_wall_s",
}
IMAGE_KEYS = {"own", "top"}
RGB_KEYS = {"path", "sha256"}
ACTION_KEYS = {"kind", "forward", "turn", "duration_s"}


def _digest(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _safe_file(root: Path, relative: Any) -> Path:
    if not isinstance(relative, str) or not relative or PurePath(relative).is_absolute():
        raise ValueError("artifact path must be a non-empty relative path")
    if any(part in ("", ".", "..") for part in PurePath(relative).parts):
        raise ValueError("artifact path traversal refused")
    current = root
    for part in PurePath(relative).parts:
        current = current / part
        if current.is_symlink():
            raise ValueError("symlink artifact path refused")
    resolved = current.resolve()
    if not resolved.is_relative_to(root) or not resolved.is_file():
        raise ValueError("artifact path escapes root or is not a file")
    return resolved


def _rgb(run_dir: Path, record: Any) -> bytes:
    if not isinstance(record, dict) or set(record) != RGB_KEYS:
        raise ValueError("RGB record schema mismatch or has hidden input fields")
    path = _safe_file(run_dir, record["path"])
    data = path.read_bytes()
    if not isinstance(record["sha256"], str) or _digest(data) != record["sha256"]:
        raise ValueError(f"RGB hash mismatch: {record.get('path')}")
    return data


def _strict_models(root: Path, report: dict[str, Any]) -> dict[str, dict[str, Any]]:
    manifest_path = _safe_file(root, "short-transport-skill.json")
    manifest_bytes = manifest_path.read_bytes()
    if report.get("transport_skill_sha256") != _digest(manifest_bytes):
        raise ValueError("transport manifest hash mismatch")
    manifest = json.loads(manifest_bytes)
    records, reported = manifest.get("models"), report.get("transport_model_sha256")
    if (manifest.get("schema") != "ugrp.camera_short_transport_skill.v1"
            or not isinstance(records, dict) or set(records) != set(ROBOTS)
            or not isinstance(reported, dict) or set(reported) != set(ROBOTS)):
        raise ValueError("transport manifest/model coverage mismatch")
    models = {}
    for rid in ROBOTS:
        record = records[rid]
        if not isinstance(record, dict) or set(record) != {"path", "sha256"}:
            raise ValueError(f"transport model record schema mismatch: {rid}")
        path = _safe_file(root, record["path"])
        raw = path.read_bytes()
        digest = _digest(raw)
        if digest != record["sha256"] or digest != reported[rid]:
            raise ValueError(f"transport model hash mismatch: {rid}")
        model = json.loads(raw)
        if model.get("schema") != "ugrp.camera_short_transport_model.v1" or model.get("robot_id") != rid:
            raise ValueError(f"transport model identity mismatch: {rid}")
        models[rid] = model
    return models


def _finite(value: Any) -> bool:
    return (not isinstance(value, bool) and isinstance(value, (int, float))
            and math.isfinite(float(value)))


def _same(expected: Any, saved: Any) -> bool:
    if isinstance(expected, bool) or isinstance(saved, bool):
        return expected is saved
    if isinstance(expected, (int, float)) and isinstance(saved, (int, float)):
        return _finite(expected) and _finite(saved) and math.isclose(float(expected), float(saved), rel_tol=1e-9, abs_tol=1e-9)
    if isinstance(expected, dict) and isinstance(saved, dict):
        return set(expected) == set(saved) and all(_same(expected[k], saved[k]) for k in expected)
    if isinstance(expected, list) and isinstance(saved, list):
        return len(expected) == len(saved) and all(_same(a, b) for a, b in zip(expected, saved))
    return expected == saved


def _action(value: Any, rid: str) -> dict[str, Any]:
    if not isinstance(value, dict) or set(value) != ACTION_KEYS or value.get("kind") != "drive":
        raise ValueError(f"action schema mismatch: {rid}")
    if any(not _finite(value[key]) for key in ("forward", "turn", "duration_s")):
        raise ValueError(f"non-finite action: {rid}")
    if float(value["turn"]) != 0.0 or not 0.0 <= float(value["forward"]) <= .10 or not 0 < float(value["duration_s"]) <= .25:
        raise ValueError(f"action outside bounds: {rid}")
    return value


def _actor(run_dir: Path, model_dir: Path, report: dict[str, Any]) -> dict[str, Any]:
    if report.get("schema") != "ugrp.pair_carry_sync.v1":
        raise ValueError("result schema mismatch")
    condition = report.get("condition")
    if condition not in ("baseline", "sync"):
        raise ValueError("condition must be baseline or sync")
    models = _strict_models(model_dir, report)
    anchors = report.get("anchor_images")
    if not isinstance(anchors, dict) or set(anchors) != set(ROBOTS):
        raise ValueError("anchor robot coverage mismatch")
    anchor_bytes = {}
    for rid in ROBOTS:
        if not isinstance(anchors[rid], dict) or set(anchors[rid]) != IMAGE_KEYS:
            raise ValueError(f"anchor input schema mismatch: {rid}")
        anchor_bytes[rid] = (_rgb(run_dir, anchors[rid]["own"]), _rgb(run_dir, anchors[rid]["top"]))

    rows = report.get("steps")
    if not isinstance(rows, list) or not rows:
        raise ValueError("missing actor step rows")
    histories = {rid: [] for rid in ROBOTS}
    frames_seen = {rid: set() for rid in ROBOTS}
    previous_observed = -math.inf
    policy = PairCarryPolicy(report.get("case_id", ""))
    confirming, consecutive = False, 0
    done = False
    for index, row in enumerate(rows):
        if not isinstance(row, dict) or set(row) != ROW_KEYS:
            raise ValueError(f"step {index} schema mismatch or has hidden input fields")
        if (type(row["index"]) is not int or row["index"] != index
                or not _finite(row["observed_at_s"]) or float(row["observed_at_s"]) <= previous_observed):
            raise ValueError(f"step {index} observation time/index is not strictly monotonic")
        if index == 0 and float(row["observed_at_s"]) != 0.0:
            raise ValueError("first observation time must be zero")
        if index and not _same(rows[index - 1]["execution_end_s"], row["observed_at_s"]):
            raise ValueError(f"step {index} does not begin at previous execution end")
        previous_observed = float(row["observed_at_s"])
        for field in ("request_wall_s", "decision_wall_s", "command_issued_wall_s", "execution_end_s", "execution_end_wall_s"):
            if not _finite(row[field]):
                raise ValueError(f"step {index} has non-finite timing field: {field}")
        if float(row["execution_end_s"]) <= float(row["observed_at_s"]):
            raise ValueError(f"step {index} execution end must follow observation")
        for field in ("frame_ids", "images", "own_command_histories", "decisions", "actions"):
            if not isinstance(row[field], dict) or set(row[field]) != set(ROBOTS):
                raise ValueError(f"step {index} robot coverage mismatch: {field}")
        delivered = row["delivered_reports"]
        if (not isinstance(delivered, list) or len(delivered) != len(set(delivered))
                or any(rid not in ROBOTS for rid in delivered)):
            raise ValueError(f"step {index} delivered report trace is invalid")
        decisions, tops = {}, {}
        for rid in ROBOTS:
            frame_id = row["frame_ids"][rid]
            if not isinstance(frame_id, (str, int)) or isinstance(frame_id, bool) or str(frame_id) in frames_seen[rid]:
                raise ValueError(f"step {index} frame is duplicate or invalid: {rid}")
            frames_seen[rid].add(str(frame_id))
            images = row["images"][rid]
            if not isinstance(images, dict) or set(images) != IMAGE_KEYS:
                raise ValueError(f"step {index} image input schema mismatch: {rid}")
            own, top = _rgb(run_dir, images["own"]), _rgb(run_dir, images["top"])
            tops[rid] = (_digest(top), top)
            if row["own_command_histories"][rid] != histories[rid]:
                raise ValueError(f"step {index} exact own command history mismatch: {rid}")
            decision = predict_transport(models[rid], own, top, anchor_bytes[rid][0], anchor_bytes[rid][1],
                                         own_command_history=list(histories[rid]))
            if not _same(decision, row["decisions"][rid]):
                raise ValueError(f"step {index} transport decision replay mismatch: {rid}")
            decisions[rid] = decision
        if index == 0 and not _same(row["images"], anchors):
            raise ValueError("first step images differ from carry anchors")
        if tops["r1"][0] != tops["r3"][0]:
            raise ValueError(f"step {index} robots do not share exact top RGB")
        skew = payload_skew(tops["r1"][1])
        if not _same(skew, row["skew_px"]):
            raise ValueError(f"step {index} payload skew replay mismatch")

        if condition == "sync":
            received_decisions = {rid: decisions[rid] for rid in delivered}
            control = policy.step(received_decisions, skew, row["frame_ids"], row["observed_at_s"], delivered)
        else:
            base = choose_carry_actions(decisions, confirming)
            control = {**base, "mode": "CONFIRM" if confirming else "CRUISE",
                       "abort": not base["valid"], "done": False, "permission": None,
                       "skew_error_px": None, "recovery_count": 0}
            if confirming:
                consecutive = consecutive + 1 if base["ready"] else 0
                control["done"] = consecutive >= 2
                if not base["ready"]:
                    confirming = False
            elif base["ready"]:
                confirming = True
        if not _same(control, row["control"]):
            raise ValueError(f"step {index} control replay mismatch")
        expected_actions = {rid: {"kind": "drive", "forward": control["forwards"][rid], "turn": 0.,
                                  "duration_s": control["duration_s"]} for rid in ROBOTS}
        for rid in ROBOTS:
            _action(row["actions"][rid], rid)
            if not _same(expected_actions[rid], row["actions"][rid]):
                raise ValueError(f"step {index} action replay mismatch: {rid}")
            histories[rid].append(row["actions"][rid])
        done = bool(control["done"])
        if done and index != len(rows) - 1:
            raise ValueError("actor rows continue after terminal done")
        if control["abort"] and index != len(rows) - 1:
            raise ValueError("actor rows continue after terminal abort")

    if bool(report.get("carry_ready")) != done:
        raise ValueError("saved carry_ready differs from actor replay")
    expected_sync = policy.sync.events if condition == "sync" else []
    expected_policy = policy.events if condition == "sync" else []
    if not _same(expected_sync, report.get("sync_events")):
        raise ValueError("synchronization event replay mismatch")
    if not _same(expected_policy, report.get("policy_events")):
        raise ValueError("policy event replay mismatch")
    return {"steps": len(rows), "commands": sum(len(v) for v in histories.values()),
            "condition": condition, "carry_ready": done}


def _evaluation(run_dir: Path, report: dict[str, Any]) -> tuple[dict[str, Any], int]:
    path = _safe_file(run_dir, "evaluation-only.jsonl")
    samples = []
    for line_number, line in enumerate(path.read_text().splitlines(), 1):
        if not line.strip():
            raise ValueError(f"blank referee row: {line_number}")
        row = json.loads(line)
        if not isinstance(row, dict):
            raise ValueError(f"invalid referee row: {line_number}")
        samples.append(row)
    replay = evaluate_transport_samples(samples)
    if not _same(replay, report.get("evaluation")):
        raise ValueError("saved physical evaluation differs from referee replay")
    return replay, len(samples)


def _invariants(value: Any, label: str) -> dict[str, Any]:
    geometry_keys = {"body_inertia", "body_mass", "geom_friction", "geom_pos",
                     "geom_quat", "geom_rgba", "geom_size"}
    camera_keys = {"cctv_top", "r1__robot_cam", "r3__robot_cam"}
    if not isinstance(value, dict) or set(value) != {"geometry_sha256", "policy_cameras"}:
        raise ValueError(f"{label} invariant metadata schema mismatch")
    geometry, cameras = value["geometry_sha256"], value["policy_cameras"]
    if (not isinstance(geometry, dict) or set(geometry) != geometry_keys
            or any(not isinstance(v, str) or len(v) != 64 for v in geometry.values())):
        raise ValueError(f"{label} source model geometry metadata mismatch")
    if not isinstance(cameras, dict) or set(cameras) != camera_keys:
        raise ValueError(f"{label} policy camera coverage mismatch")
    for name, camera in cameras.items():
        if (not isinstance(camera, dict) or set(camera) != {"fov_y_deg", "position", "quaternion"}
                or not _finite(camera["fov_y_deg"])
                or not isinstance(camera["position"], list) or len(camera["position"]) != 3
                or not isinstance(camera["quaternion"], list) or len(camera["quaternion"]) != 4
                or not all(_finite(v) for v in camera["position"] + camera["quaternion"])):
            raise ValueError(f"{label} policy camera metadata mismatch: {name}")
    return value


def audit(run_dir: Path | str, transport_model_dir: Path | str,
          grasp_model_dir: Path | str) -> dict[str, Any]:
    run_dir = Path(run_dir).resolve()
    result: dict[str, Any] = {"success": False, "physical_success": False, "errors": [], "counts": {}}
    try:
        report = json.loads(_safe_file(run_dir, "result.json").read_text())
        grasp_skill = _safe_file(Path(grasp_model_dir).resolve(), "student-skill.json")
        if report.get("grasp_skill_sha256") != _digest(grasp_skill.read_bytes()):
            raise ValueError("grasp skill hash mismatch")
        actor = _actor(run_dir, Path(transport_model_dir).resolve(), report)
        grasp = audit_grasp(run_dir, Path(grasp_model_dir).resolve(), report_name="grasp-result.json")
        if not isinstance(grasp, dict) or grasp.get("ok") is not True:
            raise ValueError("grasp audit did not pass")
        evaluation, sample_count = _evaluation(run_dir, report)
        initial = _invariants(report.get("invariants_initial"), "initial")
        final = _invariants(report.get("invariants_final"), "final")
        if type(report.get("weld_active_ticks")) is not int or report["weld_active_ticks"] < 0:
            raise ValueError("weld_active_ticks must be a non-negative integer")
        if type(report.get("carry_ready")) is not bool or type(report.get("success")) is not bool:
            raise ValueError("saved carry_ready/success must be booleans")
        if report.get("error") is not None and not isinstance(report.get("error"), str):
            raise ValueError("saved error must be null or a string")
        physical_success = bool(report.get("error") is None and report["carry_ready"]
                                and evaluation["success"] and report["weld_active_ticks"] == 0
                                and initial == final)
        if report["success"] is not physical_success:
            raise ValueError("saved physical success differs from recomputed runner predicate")
        result.update({"success": True, "actor": actor, "grasp": grasp,
                       "physical_success": physical_success,
                       "evaluation_success": evaluation["success"],
                       "counts": {"steps": actor["steps"], "actions": actor["commands"],
                                  "evaluation_samples": sample_count,
                                  "grasp_calls": grasp.get("calls")}})
    except Exception as exc:
        result["errors"].append(f"{type(exc).__name__}: {exc}")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--transport-model-dir", type=Path, required=True)
    parser.add_argument("--grasp-model-dir", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    result = audit(args.run_dir, args.transport_model_dir, args.grasp_model_dir)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, sort_keys=True, indent=2, allow_nan=False) + "\n")
    print(json.dumps(result, sort_keys=True, allow_nan=False))
    return 0 if result["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
