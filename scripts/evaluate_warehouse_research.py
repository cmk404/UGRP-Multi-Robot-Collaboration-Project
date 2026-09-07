#!/usr/bin/env python3
"""Run matched, independently reset warehouse conditions and persist evidence."""
from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from harness.warehouse_evaluation import EvaluationLedger, EpisodeResult
from harness.warehouse_runtime import EpisodeBudget, LLMPolicy, LocalEnvironment, RulePolicy, run_episode


def source_signature():
    files = sorted(p for folder in ("sim", "harness", "scripts")
                   for p in (ROOT / folder).rglob("*") if p.is_file() and "__pycache__" not in p.parts
                   and (p.suffix == ".py" or (folder == "sim" and p.suffix in {".xml", ".json", ".png"})))
    hashes = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    return {"sha256": hashlib.sha256(json.dumps(hashes, sort_keys=True).encode()).hexdigest(), "files": hashes}


class VideoRecorder:
    """Observer is for the human reviewer only; never passed to an actor."""
    def __init__(self, world, path):
        self.world = world
        self.count = 0
        self.last_sim_time = -1.0
        self.path = Path(path)
        self.process = subprocess.Popen([
            "ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "bgr24",
            "-s", "640x480", "-r", "10", "-i", "-", "-an", "-c:v", "libx264",
            "-preset", "veryfast", "-crf", "22", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(path)
        ], stdin=subprocess.PIPE)

    def capture(self, force=False):
        import cv2
        import numpy as np
        now = float(self.world.data.time)
        if not force and now - self.last_sim_time < 0.20:
            return
        self.last_sim_time = now
        jpeg = self.world.render_team_jpeg(camera="cctv_warehouse", quality=85)
        frame = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        frame = cv2.resize(frame, (640, 480))
        last = self.world.warehouse_trace[-1] if self.world.warehouse_trace else {}
        cv2.rectangle(frame, (0, 0), (640, 48), (24, 28, 32), -1)
        cv2.putText(frame, "PEER DECISIONS | SIM RGB-D | shared transport skill", (8, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, .43, (255, 255, 255), 1)
        cv2.putText(frame, f"t={now:.1f}s  {last.get('phase', 'negotiating')}", (8, 38),
                    cv2.FONT_HERSHEY_SIMPLEX, .43, (100, 230, 255), 1)
        self.process.stdin.write(frame.tobytes())
        self.count += 1
        if force:
            cv2.imwrite(str(self.path.with_suffix(".png")), frame)

    def close(self):
        self.capture(force=True)
        self.process.stdin.close()
        if self.process.wait(timeout=30) != 0:
            raise RuntimeError("video encoding failed")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", default="11,12")
    parser.add_argument("--conditions", default="rule,llm_no_comm,llm_peer_comm")
    parser.add_argument("--scenarios", default="normal,gripper_failure")
    parser.add_argument("--destination", choices=("B", "C"), default="B")
    parser.add_argument("--selector", choices=("all", "plank", "pipe", "crate"), default="all")
    parser.add_argument("--max-rounds", type=int, default=12)
    parser.add_argument("--max-actions", type=int, default=6)
    parser.add_argument("--backend", default="gemini")
    parser.add_argument("--model", default="gemini-3.7-flash")
    parser.add_argument("--record-first-peer", action="store_true")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    seeds = [int(s) for s in args.seeds.split(",")]
    conditions = args.conditions.split(",")
    scenarios = args.scenarios.split(",")
    output = Path(args.output).expanduser().resolve()
    output.mkdir(parents=True, exist_ok=False)
    budget = EpisodeBudget(max_rounds=args.max_rounds, max_actions=args.max_actions)
    profile = "parallel_rgbd_fiducial_memory_v4"
    controller = "independent_crew_forward_uncalibrated_v4"
    ledger = EvaluationLedger(conditions, seeds, scenarios, budget=asdict(budget),
        sensor_profiles={c: profile for c in conditions}, controller_profiles={c: controller for c in conditions})
    signature = source_signature()
    (output / "design.json").write_text(json.dumps({
        "seeds": seeds, "conditions": conditions, "scenarios": scenarios,
        "budget": asdict(budget), "source_signature": signature,
        "sensor_profile": profile, "controller_profile": controller,
        "model": args.model, "backend": args.backend, "selector": args.selector,
        "destination": args.destination, "purpose": "pilot; not evidence of population superiority",
    }, indent=2), encoding="utf-8")
    from sim.multi_masterpi_production import MultiMasterPiProductionV2
    from harness.groq import live_completer
    results = []
    recorded = False
    for seed in seeds:
        for scenario in scenarios:
            # Rotate condition order across matched cases to reduce load/order confounding.
            rotation = (seeds.index(seed) + scenarios.index(scenario)) % len(conditions)
            ordered = conditions[rotation:] + conditions[:rotation]
            for condition in ordered:
                name = f"seed{seed}-{scenario}-{condition}"
                world = None
                recorder = None
                started = time.perf_counter()
                try:
                    world = MultiMasterPiProductionV2(seed=seed, width=640, height=480, render=True)
                    if condition == "rule":
                        policies = {rid: RulePolicy(rid) for rid in ("r1", "r2", "r3")}
                    else:
                        policies = {rid: LLMPolicy(rid, live_completer(args.backend, args.model)[0], response_tokens=budget.response_tokens)
                                    for rid in ("r1", "r2", "r3")}
                    if args.record_first_peer and condition == "llm_peer_comm" and not recorded:
                        recorder = VideoRecorder(world, output / f"{name}.mp4")
                        recorder.capture(force=True)
                        world.frame_callback = recorder.capture
                        recorded = True
                    result = run_episode(LocalEnvironment(world), policies, condition=condition,
                        seed=seed, destination=args.destination, selector=args.selector, scenario=scenario,
                        budget=budget, journal_path=output / f"{name}.jsonl")
                    result["timing_comparable"] = recorder is None
                    (output / f"{name}.physics.json").write_text(json.dumps(world.warehouse_state(), indent=2), encoding="utf-8")
                except Exception as exc:
                    result = {"condition": condition, "seed": seed, "scenario": scenario,
                              "success": False, "reason": f"{type(exc).__name__}: {exc}",
                              "elapsed_s": time.perf_counter()-started, "budget": asdict(budget),
                              "sensor_profile": profile, "controller_profile": controller,
                              "evidence_kind": "rule" if condition == "rule" else "live_llm"}
                finally:
                    if recorder:
                        world.frame_callback = None
                        recorder.close()
                    if world:
                        world.close()
                if source_signature()["sha256"] != signature["sha256"]:
                    raise RuntimeError("source changed during comparison; discard and rerun this suite")
                ledger.append(EpisodeResult(
                    condition=condition, seed=seed, scenario=scenario, success=result["success"],
                    elapsed_s=result.get("elapsed_s") if result.get("timing_comparable", True) else None,
                    rounds=result.get("rounds"), actions=result.get("actions"),
                    tokens_total=result.get("tokens_total"), error=None if result["success"] else result["reason"],
                    provenance="live", budget=asdict(budget), sensor_profile=profile, controller_profile=controller))
                results.append(result)
                summary = {**ledger.summary(), "results": results,
                           "paired": ledger.paired("llm_no_comm", "llm_peer_comm")
                           if {"llm_no_comm", "llm_peer_comm"} <= set(conditions) else None}
                (output / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
                print(json.dumps({k: result.get(k) for k in ("condition", "seed", "scenario", "success", "reason", "rounds", "actions", "elapsed_s")}), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
