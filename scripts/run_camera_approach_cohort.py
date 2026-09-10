#!/usr/bin/env python3
"""Run a source-frozen paired RGB wheel approach and full grasp cohort."""
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
from scripts.run_grasp_recovery_cohort import compare_initial_rgb


def write(path, value):
    path.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n")


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def git(*args):
    return subprocess.check_output(["git", *args], cwd=ROOT, text=True).strip()


def load_cases(path):
    value = json.loads(path.read_text())
    rows = value.get("cases")
    if not isinstance(rows, list) or not rows:
        raise ValueError("cases must be a nonempty list")
    seen = set()
    for row in rows:
        case_id = row.get("id")
        if not isinstance(case_id, str) or not re.fullmatch(r"[A-Za-z0-9_-]+", case_id) or case_id in seen:
            raise ValueError("case ids must be safe and unique")
        seen.add(case_id)
        distances = row.get("distance_m")
        if not isinstance(distances, dict) or set(distances) != {"r1", "r3"}:
            raise ValueError("independent robot distances required")
        for distance in distances.values():
            if isinstance(distance, bool) or not isinstance(distance, (int, float)) or not math.isfinite(distance) or not .2 <= distance <= .3:
                raise ValueError("cohort is bounded to20-30cm")
    return rows


def artifact_hashes(*roots):
    return {str(p): sha(p) for root in roots for p in sorted(root.glob("*.json"))}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--cases-json", type=Path, required=True)
    p.add_argument("--grasp-model-dir", type=Path, required=True)
    p.add_argument("--approach-model-dir", type=Path, required=True)
    p.add_argument("--out-dir", type=Path, required=True)
    p.add_argument("--mjpython", type=Path, required=True)
    p.add_argument("--playback-seconds", type=float, required=True)
    p.add_argument("--conditions", nargs="+", choices=["visual", "playback"], default=["visual", "playback"])
    args = p.parse_args()
    if not math.isfinite(args.playback_seconds) or not 0 < args.playback_seconds <= 20:
        raise ValueError("bounded fixed playback duration required")
    if len(set(args.conditions)) != len(args.conditions):
        raise ValueError("duplicate conditions")
    if git("status", "--porcelain=v1"):
        raise ValueError("commit source and protocol before cohort")
    from scripts.audit_camera_approach_student import audit
    source = git("rev-parse", "HEAD")
    grasp, approach = args.grasp_model_dir.resolve(), args.approach_model_dir.resolve()
    hashes = artifact_hashes(grasp, approach)
    cases = load_cases(args.cases_json)
    out = args.out_dir.resolve(); out.mkdir(parents=True, exist_ok=False)
    rows = []; started = time.monotonic()
    summary = {"complete": False, "source_sha": source, "cases_sha256": sha(args.cases_json),
               "model_artifact_hashes": hashes, "playback_seconds": args.playback_seconds,
               "conditions": args.conditions, "runs": rows, "case_count": len(cases)}
    write(out / "cohort-report.json", summary)
    for case in cases:
        first = None
        for condition in args.conditions:
            if git("rev-parse", "HEAD") != source or git("status", "--porcelain=v1") or artifact_hashes(grasp, approach) != hashes:
                raise RuntimeError("source or model changed during frozen cohort")
            destination = out / f"{case['id']}-{condition}"
            command = [str(args.mjpython.resolve()), "scripts/run_camera_approach_student.py",
                       "--grasp-model-dir", str(grasp), "--approach-model-dir", str(approach),
                       "--distance", str(case["distance_m"]["r1"]), str(case["distance_m"]["r3"]),
                       "--condition", condition, "--playback-seconds", str(args.playback_seconds),
                       "--out-dir", str(destination)]
            completed = subprocess.run(command, cwd=ROOT, capture_output=True, text=True)
            (out / f"{case['id']}-{condition}.stdout.txt").write_text(completed.stdout)
            (out / f"{case['id']}-{condition}.stderr.txt").write_text(completed.stderr)
            row = {"case_id": case["id"], "condition": condition, "command": command,
                   "raw_dir": str(destination), "exit_code": completed.returncode, "audit_ok": False}
            rows.append(row)
            if completed.returncode:
                row["error"] = "runner exception; preserved raw evidence"
                write(out / "cohort-report.json", summary)
                raise RuntimeError(f"runner exception: {destination}")
            report = json.loads((destination / "result.json").read_text())
            if report["source_sha"] != source:
                raise ValueError("runner source differs")
            replay = audit(destination, approach, grasp)
            write(destination / "audit.json", replay)
            initial_images = {c["robot_id"]: c["images"] for c in report["approach_calls"][:2]}
            if first is None:
                first = (destination, initial_images, report["evaluation_initial_state"])
            if first[2] != report["evaluation_initial_state"]:
                raise ValueError("paired initial physics differ")
            image_match = compare_initial_rgb(first[0], first[1], destination, initial_images)
            row.update({"audit_ok": True, "success": report["success"],
                        "approach_ok": report["approach_ok"], "evaluation": report["evaluation"],
                        "approach_payload_contact_steps": report["approach_payload_contact_steps"],
                        "wall_elapsed_s": report["wall_elapsed_s"],
                        "approach_elapsed_sim_s": report["approach_elapsed_sim_s"],
                        "approach_calls": len(report["approach_calls"]),
                        "grasp_calls": len(report.get("calls", [])),
                        "matched_initial_physics": True, "initial_rgb_comparison": image_match,
                        "result_sha256": sha(destination / "result.json")})
            write(out / "cohort-report.json", summary)
            print(json.dumps({key: row[key] for key in ("case_id", "condition", "success", "approach_ok", "approach_payload_contact_steps", "approach_elapsed_sim_s")}), flush=True)
    summary.update({"complete": True, "wall_elapsed_s": time.monotonic() - started,
                    "success_counts": {c: sum(r["success"] for r in rows if r["condition"] == c) for c in args.conditions}})
    write(out / "cohort-report.json", summary)
    print(json.dumps({k: summary[k] for k in ("complete", "case_count", "success_counts", "wall_elapsed_s")}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
