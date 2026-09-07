"""Run matched CoELA-inspired A/B/C pilot episodes, retaining all failures."""
import argparse
import hashlib
import json
from pathlib import Path
import random
import threading
import time

from harness.coela_modules import Planner, ROBOTS, MODES
from harness.coela_runtime import run_coela_episode
from harness.gemini_proxy import GeminiProxyCompleter
from harness.warehouse_runtime import LocalEnvironment
from sim.multi_masterpi_production import MultiMasterPiProductionV2


def signature():
    digest = hashlib.sha256()
    root = Path(__file__).resolve().parents[1]
    files = sorted(p for folder in ("harness", "sim", "scripts") for p in (root/folder).rglob("*.py"))
    for path in files:
        digest.update(str(path.relative_to(root)).encode()); digest.update(path.read_bytes())
    return digest.hexdigest()


def score_scenario(result, injected, recovery_history, scenario):
    """Retain original outcome and score the injected event, not any recovery."""
    result = dict(result)
    original_success = bool(result.get("success"))
    result["execution_success"] = original_success
    result["execution_reason"] = result.get("reason")
    history = result.get("status", {}).get("history", [])
    result["cleanup_failures"] = [e for e in history if e.get("event") == "failed" and e.get("reason") == "MIXED_EXECUTION_CANCELLED"]
    failures = [e for e in history if e.get("event") == "failed" and e not in result["cleanup_failures"]]
    result["first_execution_failure"] = failures[0] if failures else None
    expected = {e["event_id"] for e in injected}
    recovered = {e.get("event_id") for e in recovery_history if e.get("event") == "recovered" and e.get("action") == "replan"}
    result["scenario_triggered"] = bool(expected)
    result["injected_event_recovered"] = bool(expected) and expected <= recovered
    if scenario == "private_obstacle":
        result["scenario_verdict"] = ("NOT_TRIGGERED" if not expected else "RECOVERED" if expected <= recovered else "NOT_RECOVERED")
        result["success"] = original_success and result["injected_event_recovered"]
        if original_success and not result["success"]:
            result["reason"] = "SCENARIO_" + result["scenario_verdict"]
    else:
        result["scenario_verdict"] = "NOT_APPLICABLE"
    if failures and not result["success"]:
        result["primary_failure"] = failures[0]["reason"]
    elif not result["success"]:
        result["primary_failure"] = result.get("reason", "INCOMPLETE")
    else:
        result["primary_failure"] = None
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", required=True)
    p.add_argument("--seeds", default="11")
    p.add_argument("--modes", default="none,structured,natural")
    p.add_argument("--scenarios", default="normal,private_obstacle")
    p.add_argument("--repeats", type=int, default=1)
    p.add_argument("--timeout", type=float, default=120.)
    p.add_argument("--record", action="store_true", help="Record natural mode; exclude recordings from comparative wall time")
    args = p.parse_args()
    modes, scenarios, seeds = args.modes.split(","), args.scenarios.split(","), [int(s) for s in args.seeds.split(",")]
    if set(modes)-set(MODES) or set(scenarios)-{"normal", "private_obstacle"} or args.repeats < 1:
        p.error("invalid modes, scenarios or repeats")
    out = Path(args.output).resolve(); out.mkdir(parents=True, exist_ok=False)
    source_hash = signature()
    schedule = [(seed, scenario, repeat, mode) for seed in seeds for scenario in scenarios
                for repeat in range(args.repeats) for mode in modes]
    random.Random(20260905).shuffle(schedule)
    design = {"architecture": "coela_inspired_local_v2", "source_sha256": source_hash,
        "schedule": schedule, "modes": modes, "scenarios": scenarios, "seeds": seeds, "repeats": args.repeats,
        "max_calls_per_robot": 24, "response_tokens": 768, "timeout_s": args.timeout, "decision_period_s": 3.,
        "communication": {"max_messages_per_sender": 24, "max_bytes_per_sender": 24000,
            "intent_ttl_receiver_seconds": 12., "latency": "in_process; measured delivery times"},
        "planning_budget": {"normal_cap": 18, "recovery_reserve": 6},
        "preflight": "serial local scans followed by same-clock passive local refresh",
        "controller": "shared ideal_sim_geometry; same skill layer in A/B/C",
        "no_comm_definition": "no explicit high-level messages; shared actuator synchronization remains",
        "scenario": "controlled obstacle event after 6 SIM seconds in approach_grip; seeded observer among participants; not camera detection",
        "recording": args.record, "comparative_wall_time_includes_recordings": False,
        "analysis": "pilot only; all failures retained; no superiority claim from one seed"}
    (out/"design.json").write_text(json.dumps(design, ensure_ascii=False, indent=2))
    results = []
    for seed, scenario, repeat, mode in schedule:
        run = out/f"{seed}-{scenario}-{repeat}-{mode}"; run.mkdir()
        world = None; video = None; dialogue = None
        injected = []; recording = args.record and mode == "natural"
        try:
            if signature() != source_hash: raise RuntimeError("SOURCE_CHANGED_DURING_COMPARISON")
            world = MultiMasterPiProductionV2(seed=seed, warehouse_layout="mixed", render=True)
            if recording:
                from scripts.record_parallel_warehouse import CrewVideo
                video = CrewVideo(world, run/"motion-1x.mp4")
                video_lock = threading.Lock()
                def capture():
                    with video_lock: video.capture()
                video.capture(force=True); world.frame_callback = capture
                dialogue = (run/"dialogue.jsonl").open("w")
            message_lock = threading.Lock()
            def on_message(event):
                print(mode, event["sender_id"], event["content"], flush=True)
                if dialogue:
                    with message_lock:
                        dialogue.write(json.dumps({"robot_id": event["sender_id"], "recipients": event["recipients"],
                            "message": event["content"], "sim_time": event["sent_sim_time"],
                            "video_time": max(0., event["sent_sim_time"]-video.start),
                            "source": "actual_coela_message_sent"}, ensure_ascii=False)+"\n"); dialogue.flush()
            def scenario_hook(environment, eid, status):
                if scenario != "private_obstacle" or injected: return
                engine = world._mixed_engine
                a = engine.joint_active
                if not a: return
                accepted = next((e for e in status["history"] if e["event"] == "accepted" and e["assignment_id"] == a.assignment_id), None)
                crew = getattr(world, "_warehouse_crew", None)
                if not accepted or not crew or world.data.time-accepted["sim_time"] < 6.: return
                if not any(crew.activities.get(r) == "approach_grip" for r in a.participants): return
                # Same seeded choice for the same crew across communication modes.
                observer = a.participants[random.Random(seed * 101 + repeat).randrange(len(a.participants))]
                event = environment.request("r1", {"operation": "mixed_obstacle", "episode_id": eid,
                    "assignment_id": a.assignment_id, "position_xy": [.72, -.64], "known_to": [observer]})
                if not event.get("event_id"): raise RuntimeError("SCENARIO_INJECTION_FAILED")
                injected.append(event)
            planners = {r: Planner(r, GeminiProxyCompleter(max_tokens=768, timeout=30)) for r in ROBOTS}
            result = run_coela_episode(LocalEnvironment(world), planners, mode=mode, timeout_s=args.timeout,
                journal_path=run/"episode.jsonl", scenario_hook=scenario_hook, event_callback=on_message)
            engine = world._mixed_engine
            result.update(sim_time=float(world.data.time), task_overlap_seconds=engine.task_overlap_seconds,
                          cargo_motion_overlap_seconds=engine.cargo_motion_overlap_seconds,
                          peer_collision_events=engine.peer_collision_events, recovery_history=engine.recovery_history,
                          peer_contact_seconds=engine.peer_contact_seconds,
                          motion=engine.recorder.summary(), injected_events=injected)
            result = score_scenario(result, injected, engine.recovery_history, scenario)
        except Exception as exc:
            result = {"success": False, "reason": str(exc)}
        finally:
            if world:
                world.frame_callback = None
                if getattr(world, "_mixed_engine", None): world._mixed_engine.close()
                if video: video.close()
                if dialogue: dialogue.close()
                world.close()
        result.update(seed=seed, scenario=scenario, repeat=repeat, mode=mode, recorded=recording,
                      source_consistent=signature() == source_hash)
        if not result["source_consistent"]:
            result["success"] = False; result["reason"] = "SOURCE_CHANGED_DURING_COMPARISON"
        (run/"result.json").write_text(json.dumps(result, ensure_ascii=False, indent=2))
        results.append(result)
        summary = {m: {"successes": sum(r["success"] for r in results if r["mode"] == m),
            "denominator": sum(s[3] == m for s in schedule), "recorded_episodes": sum(r["mode"] == m for r in results)} for m in modes}
        (out/"summary.json").write_text(json.dumps({"summary": summary, "results": results}, ensure_ascii=False, indent=2))
        if video:
            from scripts.render_warehouse_dialogue import render_dialogue_video
            render_dialogue_video(run/"motion-1x.mp4", run/"dialogue.jsonl", run/"dialogue-1x.mp4")
        print("RESULT", seed, scenario, mode, result["success"], result["reason"], flush=True)
        if not result["source_consistent"]: raise RuntimeError("SOURCE_CHANGED_DURING_COMPARISON")


if __name__ == "__main__": main()
