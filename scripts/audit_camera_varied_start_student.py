#!/usr/bin/env python3
"""Replay a varied-start RGB controller from saved evidence only."""
from __future__ import annotations

import argparse
import ast
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from scripts.camera_approach_scene import ROBOTS
from scripts.run_camera_pair_transport import evaluate_grasp_samples
from scripts.run_camera_varied_start_student import AXES, LIMITS, PHASES, choose_stage_actions


def _sha_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def _sha(path: Path) -> str:
    return _sha_bytes(path.read_bytes())


def _canonical(value: Any) -> Any:
    return json.loads(json.dumps(value, sort_keys=True, allow_nan=False))


def _inside(root: Path, value: Any) -> Path:
    if not isinstance(value, str) or not value:
        raise ValueError("artifact path must be a non-empty relative string")
    path = (root / value).resolve()
    if not path.is_relative_to(root):
        raise ValueError("artifact path escapes its artifact root")
    return path


def _image(run_dir: Path, record: Any) -> bytes:
    if not isinstance(record, dict) or not isinstance(record.get("sha256"), str):
        raise ValueError("invalid RGB record")
    data = _inside(run_dir, record.get("path")).read_bytes()
    if _sha_bytes(data) != record["sha256"]:
        raise ValueError(f"RGB hash mismatch: {record.get('path')}")
    return data


def _predict_stage(model, own, top):
    from harness.camera_varied_start_student import predict_stage
    return predict_stage(model, own, top)


def _predict_straight(model, own, top):
    from harness.camera_approach_student import predict_approach
    return predict_approach(model, own, top)


def _audit_grasp(run_dir: Path, model_dir: Path):
    from scripts.audit_camera_grasp_student import audit
    return audit(run_dir, model_dir, report_name="grasp-result.json")


def _flat_models(root: Path, report: dict[str, Any], manifest_name: str,
                 skill_key: str, model_key: str) -> dict[str, dict[str, Any]]:
    manifest = root / manifest_name
    if report.get(skill_key) != _sha(manifest):
        raise ValueError(f"{manifest_name} hash mismatch")
    skill = json.loads(manifest.read_text())
    records, reported = skill.get("models"), report.get(model_key)
    if not isinstance(records, dict) or set(records) != set(ROBOTS) or not isinstance(reported, dict) or set(reported) != set(ROBOTS):
        raise ValueError(f"invalid {manifest_name} model coverage")
    result = {}
    for rid in ROBOTS:
        rec = records[rid]
        path = _inside(root, rec.get("path") if isinstance(rec, dict) else None)
        digest = _sha(path)
        if digest != rec.get("sha256") or digest != reported[rid]:
            raise ValueError(f"model hash mismatch: {rid}")
        result[rid] = json.loads(path.read_text())
    return result


def _stage_models(root: Path, report: dict[str, Any]) -> dict[str, dict[str, dict[str, Any]]]:
    manifest = root / "varied-start-skill.json"
    if report.get("stage_skill_sha256") != _sha(manifest):
        raise ValueError("varied-start skill hash mismatch")
    skill = json.loads(manifest.read_text())
    records, reported = skill.get("models"), report.get("stage_model_sha256")
    if not isinstance(records, dict) or set(records) != set(ROBOTS) or not isinstance(reported, dict) or set(reported) != set(ROBOTS):
        raise ValueError("invalid staged model coverage")
    result = {}
    for rid in ROBOTS:
        if set(records[rid]) != set(AXES) or set(reported[rid]) != set(AXES):
            raise ValueError(f"stage model coverage mismatch: {rid}")
        result[rid] = {}
        for stage in AXES:
            rec = records[rid][stage]
            path = _inside(root, rec.get("path") if isinstance(rec, dict) else None)
            digest = _sha(path)
            if digest != rec.get("sha256") or digest != reported[rid][stage]:
                raise ValueError(f"stage model hash mismatch: {rid}/{stage}")
            model = json.loads(path.read_text())
            if model.get("robot_id") != rid or model.get("stage") != stage:
                raise ValueError(f"stage model identity mismatch: {rid}/{stage}")
            result[rid][stage] = model
    return result


def _source_boundary() -> None:
    """Reject direct post-setup pose mutation or privileged predictor arguments."""
    tree = ast.parse((ROOT / "scripts/run_camera_varied_start_student.py").read_text())
    forbidden = {"set_base_pose_for_test", "set_free_body_pose_for_reset", "mj_forward"}
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            name = node.func.attr if isinstance(node.func, ast.Attribute) else node.func.id if isinstance(node.func, ast.Name) else ""
            if name in forbidden:
                raise ValueError(f"runner directly mutates physics pose: {name}")
            if name in ("predict_stage", "predict_approach") and (len(node.args) != 3 or node.keywords):
                raise ValueError(f"{name} receives inputs beyond model and two RGB views")
        if isinstance(node, (ast.Assign, ast.AugAssign, ast.AnnAssign)):
            targets = node.targets if isinstance(node, ast.Assign) else [node.target]
            if any(isinstance(target, ast.Subscript) and isinstance(target.value, ast.Attribute)
                   and target.value.attr in ("qpos", "qvel") for target in targets):
                raise ValueError("runner directly mutates qpos/qvel")


def _zero_drive_actions() -> dict[str, dict[str, Any]]:
    return {r: {"kind": "drive", "forward": 0.0, "turn": 0.0, "duration_s": .25} for r in ROBOTS}


def audit(run_dir: Path | str, stage_model_dir: Path | str,
          straight_model_dir: Path | str, grasp_model_dir: Path | str) -> dict[str, Any]:
    run_dir, stage_root = Path(run_dir).resolve(), Path(stage_model_dir).resolve()
    straight_root, grasp_root = Path(straight_model_dir).resolve(), Path(grasp_model_dir).resolve()
    report = json.loads((run_dir / "result.json").read_text())
    if report.get("error") is not None:
        raise ValueError("cannot audit a runner error")
    source = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    if report.get("source_sha") != source:
        raise ValueError("runner source SHA differs from audit source")
    _source_boundary()
    stage_models = _stage_models(stage_root, report)
    _flat_models(grasp_root, report, "student-skill.json", "grasp_skill_sha256", "grasp_model_sha256")
    condition = report.get("config", {}).get("condition")
    if condition not in ("visual", "straight"):
        raise ValueError("invalid varied-start condition")
    straight_models = (_flat_models(straight_root, report, "approach-skill.json",
                       "straight_skill_sha256", "straight_model_sha256") if condition == "straight" else None)
    config = report["config"]
    if (config.get("phases") != list(PHASES) or config.get("limits") != LIMITS
            or config.get("slice_s") != .2 or config.get("stop_dwell_s") != .25
            or config.get("max_confirmation_frames") != 4
            or config.get("required_consecutive_stationary_ready_frames") != 2
            or config.get("weld") is not False):
        raise ValueError("saved protocol differs from varied-start controller")

    calls = report.get("approach_calls")
    if not isinstance(calls, list) or not calls or len(calls) % 2:
        raise ValueError("invalid paired approach calls")
    phases = PHASES if condition == "visual" else ("forward",)
    histories = {r: [] for r in ROBOTS}
    last_frame = {r: 0 for r in ROBOTS}
    call_offset, expected_trace, stage_results = 0, [], []
    approach_ok = True
    for phase_index, stage in enumerate(phases):
        confirming = False
        confirmations = consecutive = movements = 0
        record = {"phase_index": phase_index, "stage": stage, "ok": False, "confirmations": []}
        for index in range(LIMITS[stage] + 5):
            pair = calls[call_offset:call_offset + 2]
            if len(pair) != 2 or [row.get("robot_id") for row in pair] != list(ROBOTS):
                raise ValueError(f"missing paired calls: phase {phase_index} index {index}")
            if any(row.get("phase_index") != phase_index or row.get("stage") != stage or row.get("index") != index for row in pair):
                raise ValueError("call phase/index order mismatch")
            decisions, image_records = {}, {}
            for row in pair:
                rid = row["robot_id"]
                fid = row.get("frame_id")
                if isinstance(fid, bool) or not isinstance(fid, int) or fid <= last_frame[rid]:
                    raise ValueError("approach frame is not fresh")
                last_frame[rid] = fid
                if row.get("stationary") is not confirming or row.get("own_command_history") != histories[rid]:
                    raise ValueError("stationary flag or own command history mismatch")
                images = row.get("images")
                if not isinstance(images, dict):
                    raise ValueError("missing approach RGB")
                own, top = _image(run_dir, images.get("own")), _image(run_dir, images.get("top"))
                if condition == "visual":
                    decision = _predict_stage(stage_models[rid][stage], own, top)
                else:
                    raw = _predict_straight(straight_models[rid], own, top)
                    decision = {**raw, "command": raw["forward"]}
                if _canonical(decision) != row.get("decision"):
                    raise ValueError(f"RGB decision replay mismatch: {rid}/{stage}/{index}")
                decisions[rid], image_records[rid] = decision, images
            if pair[0]["frame_id"] != pair[1]["frame_id"] or image_records[ROBOTS[0]]["top"] != image_records[ROBOTS[1]]["top"]:
                raise ValueError("paired calls do not share fresh frame/top RGB")
            control = choose_stage_actions(decisions, stage, confirming)
            trace_actions = {}
            for row in pair:
                rid = row["robot_id"]
                action = {"kind": "mecanum", **control["commands"][rid], "duration_s": control["duration_s"]}
                if _canonical(action) != row.get("action"):
                    raise ValueError(f"raw action replay mismatch: {rid}/{stage}/{index}")
                if confirming and any(action[axis] != 0.0 for axis in AXES.values()):
                    raise ValueError("confirmation issued a nonzero command")
                histories[rid].append(action)
                trace_actions[rid] = action
            expected_trace.append(trace_actions)
            call_offset += 2
            if not control["valid"]:
                record["reason"] = "RGB outside learned stage support"
                break
            if confirming:
                confirmations += 1
                consecutive = consecutive + 1 if all(control["ready"].values()) else 0
                record["confirmations"].append({"frame_ids": {r: pair[i]["frame_id"] for i, r in enumerate(ROBOTS)},
                    "ready": control["ready"], "stationary": True})
                if consecutive >= 2:
                    record["ok"] = True
                    record["reason"] = "two consecutive fresh stationary RGB confirmations"
                    break
                if confirmations >= 4:
                    record["reason"] = "stationary RGB confirmation budget exhausted"
                    break
            elif control["enter_confirmation"]:
                confirming = True
            else:
                movements += 1
                if movements >= LIMITS[stage]:
                    expected_trace.append(_zero_drive_actions())
                    record["reason"] = "bounded phase movement budget exhausted"
                    break
        record["movement_slices"] = movements
        stage_results.append(record)
        if not record["ok"]:
            approach_ok = False
            break
    if call_offset != len(calls) or report.get("stage_results") != stage_results:
        raise ValueError("stage result/call termination mismatch")
    approach_ok = approach_ok and len(stage_results) == len(phases)

    final_checks = report.get("final_alignment_checks")
    if approach_ok and condition == "visual":
        if not isinstance(final_checks, list) or len(final_checks) != 2:
            raise ValueError("successful varied approach needs two final all-axis checks")
        for check_index, check in enumerate(final_checks):
            frames, images, saved = check.get("frame_ids"), check.get("images"), check.get("decisions")
            if not all(isinstance(value, dict) for value in (frames, images, saved)) or set(frames) != set(ROBOTS):
                raise ValueError("invalid final alignment check")
            top_record = None
            for rid in ROBOTS:
                fid = frames[rid]
                if isinstance(fid, bool) or not isinstance(fid, int) or fid <= last_frame[rid]:
                    raise ValueError("final alignment frame is not fresh")
                last_frame[rid] = fid
                own, top = _image(run_dir, images[rid].get("own")), _image(run_dir, images[rid].get("top"))
                if top_record is not None and images[rid]["top"] != top_record:
                    raise ValueError("final alignment robots do not share top RGB")
                top_record = images[rid]["top"]
                for axis in AXES:
                    decision = _predict_stage(stage_models[rid][axis], own, top)
                    if _canonical(decision) != saved[rid][axis] or not decision["ok"] or not decision["ready"]:
                        raise ValueError(f"final alignment replay failed: {rid}/{axis}/{check_index}")
            expected_trace.append(_zero_drive_actions())
    elif final_checks is not None:
        raise ValueError("unexpected final alignment checks")
    if bool(report.get("approach_ok")) != approach_ok:
        raise ValueError("approach_ok differs from replayed staged confirmations")

    trace = json.loads((run_dir / "execution-trace.json").read_text())
    approach_trace = [row for row in trace if row.get("stage") in ("approach", "approach_stop_dwell")]
    if len(approach_trace) != len(expected_trace):
        raise ValueError("approach execution trace length mismatch")
    for index, actions in enumerate(expected_trace):
        if _canonical(approach_trace[index].get("actions")) != _canonical(actions):
            raise ValueError(f"execution trace action mismatch: {index}")
    if any(row.get("stage") == "folded_setup" for row in trace[1:]):
        raise ValueError("setup replay appears after execution began")

    samples = []
    for line in (run_dir / "evaluation-only.jsonl").read_text().splitlines():
        sample = json.loads(line)
        if not isinstance(sample.get("phase"), str):
            raise ValueError("evaluation sample lacks phase tag")
        samples.append(sample)
    evaluation = evaluate_grasp_samples(samples)
    if _canonical(evaluation) != report.get("evaluation") or evaluation["grasp_success"] is not bool(report.get("grasp_success")):
        raise ValueError("physics evaluation replay mismatch")
    steps, contacts, events = report.get("approach_physics_steps"), report.get("approach_payload_contact_steps"), report.get("approach_collision_events")
    if (any(isinstance(v, bool) or not isinstance(v, int) or v < 0 for v in (steps, contacts))
            or not isinstance(events, list) or len(events) != contacts or contacts > steps):
        raise ValueError("approach collision evidence is inconsistent")

    grasp_audit = None
    if approach_ok:
        grasp_report_path = run_dir / "grasp-result.json"
        if not grasp_report_path.is_file() or not isinstance(report.get("calls"), list):
            raise ValueError("successful approach lacks grasp evidence")
        grasp = json.loads(grasp_report_path.read_text())
        if grasp.get("source_sha") != source or grasp.get("calls") != report["calls"] or grasp.get("evaluation") != report["evaluation"]:
            raise ValueError("outer/grasp report mismatch")
        grasp_audit = _audit_grasp(run_dir, grasp_root)
    elif report.get("calls") != [] or (run_dir / "grasp-result.json").exists():
        raise ValueError("grasp attempted after failed approach")
    success = approach_ok and evaluation["grasp_success"] and contacts == 0
    if bool(report.get("success")) != success:
        raise ValueError("success predicate mismatch")
    return {"ok": True, "condition": condition, "approach_ok": approach_ok,
            "approach_calls": len(calls), "stage_results": stage_results,
            "evaluation": evaluation, "grasp_audit": grasp_audit,
            "checked": "source and model/RGB hashes; exact RGB predictions, phases, raw actions and histories; stationary confirmations and final all-axis checks; trace, grasp and physics replay; no direct runner pose mutation"}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--stage-model-dir", type=Path, required=True)
    parser.add_argument("--straight-model-dir", type=Path, required=True)
    parser.add_argument("--grasp-model-dir", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    result = audit(args.run_dir, args.stage_model_dir, args.straight_model_dir, args.grasp_model_dir)
    args.out.write_text(json.dumps(result, indent=2, sort_keys=True, allow_nan=False) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
