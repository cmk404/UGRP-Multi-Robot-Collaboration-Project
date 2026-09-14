#!/usr/bin/env python3
"""Replay and audit an RGB-only wheel approach from saved artifacts."""
from __future__ import annotations

import argparse
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

from scripts.camera_approach_scene import MAX_APPROACH_ROUNDS, ROBOTS
from scripts.run_camera_approach_student import choose_actions
from scripts.run_camera_pair_transport import evaluate_grasp_samples


def _predict_approach(model: dict[str, Any], own: bytes, top: bytes) -> dict[str, Any]:
    from harness.camera_approach_student import predict_approach
    return predict_approach(model, own, top)


def _audit_grasp(run_dir: Path, model_dir: Path) -> dict[str, Any]:
    from scripts.audit_camera_grasp_student import audit
    return audit(run_dir, model_dir, report_name="grasp-result.json")


def _sha_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def _sha(path: Path) -> str:
    return _sha_bytes(path.read_bytes())


def _inside(root: Path, value: Any) -> Path:
    if not isinstance(value, str) or not value:
        raise ValueError("artifact path must be a non-empty relative string")
    path = (root / value).resolve()
    if not path.is_relative_to(root):
        raise ValueError(f"artifact path escapes run directory: {value}")
    return path


def _image(run_dir: Path, record: Any) -> bytes:
    if not isinstance(record, dict) or not isinstance(record.get("sha256"), str):
        raise ValueError("invalid RGB record")
    data = _inside(run_dir, record.get("path")).read_bytes()
    if _sha_bytes(data) != record["sha256"]:
        raise ValueError(f"RGB hash mismatch: {record.get('path')}")
    return data


def _canonical(value: Any) -> Any:
    return json.loads(json.dumps(value, sort_keys=True, allow_nan=False))


def _models(model_dir: Path, report: dict[str, Any], manifest_name: str,
            skill_key: str, model_key: str) -> dict[str, dict[str, Any]]:
    manifest = model_dir / manifest_name
    if report.get(skill_key) != _sha(manifest):
        raise ValueError(f"{manifest_name} hash mismatch")
    skill = json.loads(manifest.read_text())
    records = skill.get("models")
    if not isinstance(records, dict) or set(records) != set(ROBOTS):
        raise ValueError(f"{manifest_name} must contain exactly the two robots")
    reported = report.get(model_key)
    if not isinstance(reported, dict) or set(reported) != set(ROBOTS):
        raise ValueError(f"invalid {model_key}")
    result = {}
    for rid in ROBOTS:
        rec = records[rid]
        if not isinstance(rec, dict):
            raise ValueError(f"invalid model record: {rid}")
        path = _inside(model_dir, rec.get("path"))
        digest = _sha(path)
        if digest != rec.get("sha256") or digest != reported[rid]:
            raise ValueError(f"model hash mismatch: {rid}")
        result[rid] = json.loads(path.read_text())
    return result


def _number(value: Any, label: str, *, minimum: float = 0.0) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < minimum:
        raise ValueError(f"invalid {label}")
    return float(value)


def audit(run_dir: Path | str, approach_model_dir: Path | str,
          grasp_model_dir: Path | str) -> dict[str, Any]:
    run_dir = Path(run_dir).resolve()
    approach_model_dir, grasp_model_dir = Path(approach_model_dir).resolve(), Path(grasp_model_dir).resolve()
    report = json.loads((run_dir / "result.json").read_text())
    if report.get("error") is not None:
        raise ValueError("cannot audit a runner error")
    head = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    if report.get("source_sha") != head:
        raise ValueError("runner source SHA differs from audit source")

    approach_models = _models(approach_model_dir, report, "approach-skill.json",
                              "approach_skill_sha256", "approach_model_sha256")
    _models(grasp_model_dir, report, "student-skill.json",
            "grasp_skill_sha256", "grasp_model_sha256")
    config = report.get("config")
    if not isinstance(config, dict) or config.get("condition") not in ("visual", "playback"):
        raise ValueError("invalid approach config")
    if config.get("max_rounds") != MAX_APPROACH_ROUNDS or config.get("slice_s") != .2 or config.get("confirmation_dwell_s") != .25:
        raise ValueError("approach timing or budget differs from protocol")
    playback = _number(config.get("playback_seconds"), "playback_seconds")
    if not 0 < playback <= 20:
        raise ValueError("playback_seconds is outside the runner bound")

    calls = report.get("approach_calls")
    if not isinstance(calls, list) or not calls or len(calls) % len(ROBOTS):
        raise ValueError("invalid approach call coverage")
    histories: dict[str, list[dict[str, Any]]] = {r: [] for r in ROBOTS}
    confirmations: dict[str, list[dict[str, Any]]] = {r: [] for r in ROBOTS}
    frame_ids = {r: 0 for r in ROBOTS}
    replay = []
    phase, cruise_index = "cruise", 0
    terminal = False
    completed_confirmation = False
    trace_expected = []
    for round_index in range(len(calls) // 2):
        pair = calls[round_index * 2:(round_index + 1) * 2]
        if [c.get("robot_id") for c in pair] != list(ROBOTS):
            raise ValueError(f"robot call order/coverage mismatch: round {round_index}")
        decisions, images_by_robot = {}, {}
        for call in pair:
            rid = call["robot_id"]
            if call.get("index") != round_index or call.get("phase") != phase or call.get("cruise_index") != cruise_index:
                raise ValueError(f"approach phase/index mismatch: {rid} round {round_index}")
            if call.get("stationary") is not (phase != "cruise"):
                raise ValueError(f"stationary flag mismatch: {rid} round {round_index}")
            fid = call.get("frame_id")
            if isinstance(fid, bool) or not isinstance(fid, int) or fid <= frame_ids[rid]:
                raise ValueError(f"frame is not fresh: {rid} round {round_index}")
            frame_ids[rid] = fid
            if call.get("own_command_history") != histories[rid]:
                raise ValueError(f"own command history mismatch: {rid} round {round_index}")
            images = call.get("images")
            if not isinstance(images, dict):
                raise ValueError("missing approach RGB records")
            own, top = _image(run_dir, images.get("own")), _image(run_dir, images.get("top"))
            decisions[rid] = _predict_approach(approach_models[rid], own, top)
            if _canonical(decisions[rid]) != call.get("decision"):
                raise ValueError(f"approach decision replay mismatch: {rid} round {round_index}")
            images_by_robot[rid] = images
        if len({call["frame_id"] for call in pair}) != 1:
            raise ValueError(f"paired robots have different frame ids: round {round_index}")
        if images_by_robot[ROBOTS[0]]["top"] != images_by_robot[ROBOTS[1]]["top"]:
            raise ValueError(f"robots did not share one fixed-top RGB: round {round_index}")
        control = choose_actions(decisions, config["condition"], phase, cruise_index, playback)
        for call in pair:
            rid = call["robot_id"]
            action = control["actions"][rid]
            if _canonical(action) != call.get("action"):
                raise ValueError(f"approach action replay mismatch: {rid} round {round_index}")
            histories[rid].append(action)
            if phase != "cruise":
                confirmations[rid].append({"frame_id": call["frame_id"], "stationary": True,
                    "ok": not control["blocked"], "ready": control["ready"][rid], "index": round_index})
        trace_expected.append(control["actions"])
        replay.append({"index": round_index, "phase": phase, "cruise_index": cruise_index,
                       "actions": control["actions"]})
        if control["blocked"]:
            terminal = True
        elif phase == "confirmation-2":
            terminal = True
            completed_confirmation = True
        elif phase == "confirmation-1":
            phase = "confirmation-2"
        elif control["enter_confirmation"]:
            phase = "confirmation-1"
        else:
            cruise_index += 1
            if cruise_index >= MAX_APPROACH_ROUNDS:
                terminal = True
        if terminal and round_index != len(calls) // 2 - 1:
            raise ValueError("approach calls continue after terminal controller state")

    if not terminal:
        raise ValueError("approach evidence ends before a terminal state")
    if report.get("goal_confirmation") != confirmations:
        raise ValueError("goal confirmation evidence mismatch")
    if bool(report.get("approach_ok")) != completed_confirmation:
        raise ValueError("approach_ok does not represent two completed confirmations")

    trace = json.loads((run_dir / "execution-trace.json").read_text())
    drive_trace = [row for row in trace if row.get("stage") in ("approach", "approach_stop_dwell")]
    budget_stop = not completed_confirmation and cruise_index >= MAX_APPROACH_ROUNDS
    if len(drive_trace) != len(trace_expected) + int(budget_stop):
        raise ValueError("approach execution trace length mismatch")
    for index, expected in enumerate(trace_expected):
        if _canonical(drive_trace[index].get("actions")) != _canonical(expected):
            raise ValueError(f"execution trace action mismatch: round {index}")
    if budget_stop:
        final = drive_trace[-1].get("actions")
        expected_stop = {r: {"kind": "drive", "forward": 0.0, "turn": 0.0, "duration_s": .25} for r in ROBOTS}
        if _canonical(final) != _canonical(expected_stop):
            raise ValueError("missing exact final budget stop dwell")

    samples = []
    with (run_dir / "evaluation-only.jsonl").open() as stream:
        for line in stream:
            sample = json.loads(line)
            if not isinstance(sample.get("phase"), str):
                raise ValueError("evaluation sample lacks phase tag")
            samples.append(sample)
    evaluation = evaluate_grasp_samples(samples)
    if _canonical(evaluation) != report.get("evaluation") or bool(report.get("grasp_success")) != evaluation["grasp_success"]:
        raise ValueError("evaluation replay mismatch")

    physics_steps = report.get("approach_physics_steps")
    contact_steps = report.get("approach_payload_contact_steps")
    if any(isinstance(value, bool) or not isinstance(value, int) or value < 0
           for value in (physics_steps, contact_steps)):
        raise ValueError("approach physics/contact counts must be nonnegative integers")
    events = report.get("approach_collision_events")
    if not isinstance(events, list) or len(events) != contact_steps or contact_steps > physics_steps:
        raise ValueError("approach collision count/event bounds mismatch")
    for event in events:
        _number(event.get("sim_time_s"), "collision sim time")
        if not isinstance(event.get("robot_geom_ids"), list) or not event["robot_geom_ids"]:
            raise ValueError("invalid approach collision event")

    grasp_audit = None
    if completed_confirmation:
        if not isinstance(report.get("calls"), list) or not (run_dir / "grasp-result.json").is_file():
            raise ValueError("successful approach lacks grasp evidence")
        grasp_report = json.loads((run_dir / "grasp-result.json").read_text())
        if grasp_report.get("source_sha") != report.get("source_sha") or grasp_report.get("calls") != report["calls"] or grasp_report.get("evaluation") != report["evaluation"]:
            raise ValueError("outer and grasp reports disagree")
        grasp_audit = _audit_grasp(run_dir, grasp_model_dir)
    elif report.get("calls") != [] or (run_dir / "grasp-result.json").exists():
        raise ValueError("grasp was attempted without a completed approach")

    expected_success = completed_confirmation and evaluation["grasp_success"] and contact_steps == 0
    if bool(report.get("success")) != expected_success:
        raise ValueError("overall success predicate mismatch")
    return {"ok": True, "condition": config["condition"], "approach_calls": len(calls),
            "approach_ok": completed_confirmation, "grasp_audit": grasp_audit,
            "evaluation": evaluation, "replay": replay,
            "checked": "source, model and RGB hashes; fresh paired frames; exact RGB-only decisions, actions, own-command history, confirmation state, execution trace, collision bounds, grasp audit, and phase-tagged evaluation"}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--approach-model-dir", type=Path, required=True)
    parser.add_argument("--grasp-model-dir", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    result = audit(args.run_dir, args.approach_model_dir, args.grasp_model_dir)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=2, sort_keys=True, allow_nan=False) + "\n")
    print(json.dumps({k: v for k, v in result.items() if k != "replay"}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
