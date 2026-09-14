#!/usr/bin/env python3
"""Run source-frozen paired visual/straight varied-start cohorts."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from scripts.camera_approach_scene import ROBOTS
from scripts.run_grasp_recovery_cohort import compare_initial_rgb


def _write(path, value):
    path.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n")


def _sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _git(*args):
    return subprocess.check_output(["git", *args], cwd=ROOT, text=True).strip()


def _artifact_hashes(*roots):
    return {str(path): _sha(path) for root in roots for path in sorted(root.glob("*.json"))}


def load_cases(path: Path) -> list[dict]:
    value = json.loads(path.read_text())
    rows = value.get("cases")
    if not isinstance(rows, list) or not rows:
        raise ValueError("cases must be a non-empty list")
    seen = set()
    for row in rows:
        case_id = row.get("case_id") if isinstance(row, dict) else None
        if not isinstance(case_id, str) or not re.fullmatch(r"[A-Za-z0-9_-]+", case_id) or case_id in seen:
            raise ValueError("case_id must be safe and unique")
        seen.add(case_id)
        poses = row.get("start_poses")
        if not isinstance(poses, dict) or set(poses) != set(ROBOTS):
            raise ValueError("each case needs independent r1/r3 start poses")
        for rid in ROBOTS:
            pose = poses[rid]
            bounds = {"distance_m": (.15, .40), "lateral_m": (-.06, .06), "yaw_deg": (-10., 10.)}
            if not isinstance(pose, dict) or set(pose) != set(bounds):
                raise ValueError(f"invalid start pose fields: {case_id}/{rid}")
            for key, (low, high) in bounds.items():
                item = pose[key]
                if isinstance(item, bool) or not isinstance(item, (int, float)) or not math.isfinite(item) or not low <= item <= high:
                    raise ValueError(f"start pose outside protocol: {case_id}/{rid}/{key}")
    return rows


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cases-json", type=Path, required=True)
    parser.add_argument("--stage-model-dir", type=Path, required=True)
    parser.add_argument("--straight-model-dir", type=Path, required=True)
    parser.add_argument("--grasp-model-dir", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--mjpython", type=Path, required=True)
    parser.add_argument("--conditions", nargs="+", choices=("visual", "straight"),
                        default=("visual", "straight"))
    args = parser.parse_args()
    if len(set(args.conditions)) != len(args.conditions):
        raise ValueError("duplicate conditions")
    if _git("status", "--porcelain=v1"):
        raise ValueError("commit source and protocol before cohort")
    source = _git("rev-parse", "HEAD")
    cases_path = args.cases_json.resolve()
    stage, straight, grasp = (args.stage_model_dir.resolve(), args.straight_model_dir.resolve(),
                              args.grasp_model_dir.resolve())
    hashes = _artifact_hashes(stage, straight, grasp)
    cases = load_cases(cases_path)
    out = args.out_dir.resolve()
    out.mkdir(parents=True, exist_ok=False)
    setup_dir = out / "case-inputs"
    setup_dir.mkdir()
    runs = []
    started = time.monotonic()
    summary = {"complete": False, "source_sha": source, "cases_sha256": _sha(cases_path),
               "model_artifact_hashes": hashes, "conditions": list(args.conditions),
               "case_count": len(cases), "runs": runs}
    _write(out / "cohort-report.json", summary)
    from scripts.audit_camera_varied_start_student import audit
    for case in cases:
        first = None
        setup_path = setup_dir / f"{case['case_id']}.json"
        _write(setup_path, case)
        for condition in args.conditions:
            if (_git("rev-parse", "HEAD") != source or _git("status", "--porcelain=v1")
                    or _artifact_hashes(stage, straight, grasp) != hashes):
                raise RuntimeError("source or models changed during frozen cohort")
            destination = out / f"{case['case_id']}-{condition}"
            command = [str(args.mjpython.resolve()), "scripts/run_camera_varied_start_student.py",
                       "--stage-model-dir", str(stage), "--straight-model-dir", str(straight),
                       "--grasp-model-dir", str(grasp), "--case-json", str(setup_path),
                       "--condition", condition, "--out-dir", str(destination)]
            completed = subprocess.run(command, cwd=ROOT, capture_output=True, text=True)
            (out / f"{case['case_id']}-{condition}.stdout.txt").write_text(completed.stdout)
            (out / f"{case['case_id']}-{condition}.stderr.txt").write_text(completed.stderr)
            row = {"case_id": case["case_id"], "condition": condition, "command": command,
                   "raw_dir": str(destination), "exit_code": completed.returncode, "audit_ok": False}
            runs.append(row)
            if (_git("rev-parse", "HEAD") != source or _git("status", "--porcelain=v1")
                    or _artifact_hashes(stage, straight, grasp) != hashes):
                row["error"] = "source or models changed while runner was active"
                _write(out / "cohort-report.json", summary)
                raise RuntimeError(row["error"])
            if completed.returncode:
                row["error"] = "runner exception; raw evidence preserved"
                _write(out / "cohort-report.json", summary)
                raise RuntimeError(f"runner exception: {destination}")
            report = json.loads((destination / "result.json").read_text())
            if report.get("source_sha") != source:
                raise ValueError("runner source differs from frozen cohort")
            replay = audit(destination, stage, straight, grasp)
            _write(destination / "audit.json", replay)
            first_pair = report["approach_calls"][:2]
            initial_images = {call["robot_id"]: call["images"] for call in first_pair}
            if set(initial_images) != set(ROBOTS):
                raise ValueError("run lacks paired initial RGB")
            if first is None:
                first = (destination, initial_images, report["evaluation_initial_state"])
            if report["evaluation_initial_state"] != first[2]:
                raise ValueError("paired conditions have different initial physics")
            comparison = compare_initial_rgb(first[0], first[1], destination, initial_images)
            actions = [call["action"] for call in report["approach_calls"]]
            row.update({"audit_ok": True, "success": report["success"],
                        "approach_ok": report["approach_ok"], "evaluation": report["evaluation"],
                        "approach_payload_contact_steps": report["approach_payload_contact_steps"],
                        "approach_elapsed_sim_s": report["approach_elapsed_sim_s"],
                        "wall_elapsed_s": report["wall_elapsed_s"],
                        "approach_calls": len(report["approach_calls"]),
                        "grasp_calls": len(report.get("calls", [])),
                        "action_counts": {axis: sum(abs(action.get(axis, 0.0)) > 0 for action in actions)
                                          for axis in ("forward", "left", "turn")},
                        "matched_initial_physics": True, "initial_rgb_comparison": comparison,
                        "result_sha256": _sha(destination / "result.json")})
            _write(out / "cohort-report.json", summary)
            print(json.dumps({key: row[key] for key in ("case_id", "condition", "success",
                "approach_ok", "approach_elapsed_sim_s", "approach_payload_contact_steps")}), flush=True)
    summary.update({"complete": True, "wall_elapsed_s": time.monotonic() - started,
                    "success_counts": {condition: sum(row["success"] for row in runs if row["condition"] == condition)
                                       for condition in args.conditions}})
    _write(out / "cohort-report.json", summary)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
