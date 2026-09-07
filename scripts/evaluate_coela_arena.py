"""Run the frozen nine-run CoELA arena matched cohort."""
from __future__ import annotations

import argparse
from dataclasses import asdict, is_dataclass
import hashlib
import json
from pathlib import Path
import random
import threading

import mujoco

from harness.coela_modules import MODES, ROBOTS, Planner
from harness.coela_runtime import run_coela_episode
from harness.gemini_proxy import GeminiProxyCompleter
from harness.warehouse_runtime import LocalEnvironment
from sim.multi_masterpi_production import MultiMasterPiProductionV2


DEFAULT_SEEDS = (41, 58, 73)


def signature() -> str:
    """Use the same source-cohort signature as the original evaluator."""
    digest = hashlib.sha256()
    root = Path(__file__).resolve().parents[1]
    files = sorted(
        path
        for folder in ("harness", "sim", "scripts")
        for path in (root / folder).rglob("*.py")
    )
    for path in files:
        digest.update(str(path.relative_to(root)).encode())
        digest.update(path.read_bytes())
    return digest.hexdigest()


def _jsonable(value):
    if is_dataclass(value):
        return asdict(value)
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_jsonable(item) for item in value]
    return value


def layout_record(world) -> dict:
    """Capture the seeded arena fixture without exposing it to actor inputs."""
    arena = _jsonable(world.warehouse_arena)
    return {
        "arena": arena,
        "warehouse_zone_positions": _jsonable(world.warehouse_zone_positions),
    }


def static_geometry_snapshot(world) -> list[dict]:
    """Read each configured obstacle from the live MuJoCo scene."""
    terrain = getattr(world.warehouse_arena, "terrain", ())
    snapshot = []
    for item in terrain:
        terrain_id = str(item.terrain_id)
        geom_id = int(mujoco.mj_name2id(
            world.model, mujoco.mjtObj.mjOBJ_GEOM, terrain_id,
        ))
        if geom_id < 0:
            raise RuntimeError(f"ARENA_STATIC_GEOM_MISSING:{terrain_id}")
        snapshot.append({
            "terrain_id": terrain_id,
            "geom_id": geom_id,
            "world_position": [float(v) for v in world.data.geom_xpos[geom_id]],
            "model_position": [float(v) for v in world.model.geom_pos[geom_id]],
            "size": [float(v) for v in world.model.geom_size[geom_id]],
            "quaternion": [float(v) for v in world.model.geom_quat[geom_id]],
            "geom_type": int(world.model.geom_type[geom_id]),
            "body_id": int(world.model.geom_bodyid[geom_id]),
            "contype": int(world.model.geom_contype[geom_id]),
            "conaffinity": int(world.model.geom_conaffinity[geom_id]),
        })
    return snapshot


def score_arena_result(result: dict, static_before, static_after) -> dict:
    """Preserve execution evidence and require the arena to remain static."""
    scored = dict(result)
    original_success = bool(scored.get("success"))
    history = scored.get("status", {}).get("history", [])
    cleanup = [
        event for event in history
        if event.get("event") == "failed"
        and event.get("reason") == "MIXED_EXECUTION_CANCELLED"
    ]
    failures = [
        event for event in history
        if event.get("event") == "failed" and event not in cleanup
    ]
    present = (
        bool(static_before)
        and len(static_before) == len(static_after)
        and all(
            item.get("body_id") == 0 and item.get("contype", 0) != 0
            for item in (*static_before, *static_after)
        )
    )
    unchanged = present and static_before == static_after
    scored.update(
        execution_success=original_success,
        execution_reason=scored.get("reason"),
        cleanup_failures=cleanup,
        first_execution_failure=failures[0] if failures else None,
        static_geometry_before=static_before,
        static_geometry_after=static_after,
        static_geometry_present=present,
        static_geometry_unchanged=unchanged,
        overall_task_success=original_success and unchanged,
    )
    scored["success"] = scored["overall_task_success"]
    if not present:
        scored["reason"] = "STATIC_GEOMETRY_MISSING"
    elif not unchanged:
        scored["reason"] = "STATIC_GEOMETRY_CHANGED"
    if failures and not scored["success"]:
        scored["primary_failure"] = failures[0].get("reason", "INCOMPLETE")
    elif not scored["success"]:
        scored["primary_failure"] = scored.get("reason", "INCOMPLETE")
    else:
        scored["primary_failure"] = None
    return scored


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True)
    parser.add_argument("--seeds", default=",".join(map(str, DEFAULT_SEEDS)))
    parser.add_argument("--modes", default="none,structured,natural")
    parser.add_argument("--repeats", type=int, default=1)
    parser.add_argument("--timeout", type=float, default=240.0)
    parser.add_argument("--controller", choices=("camera", "oracle-baseline"), default="camera",
                        help="Camera-only is the default; oracle-baseline preserves the old controller experiment")
    parser.add_argument(
        "--record", action="store_true",
        help="Record natural mode separately; recording time is not comparative wall time",
    )
    args = parser.parse_args()
    modes = args.modes.split(",")
    seeds = [int(seed) for seed in args.seeds.split(",")]
    if set(modes) - set(MODES) or args.repeats < 1 or args.timeout <= 0:
        parser.error("invalid modes, repeats, or timeout")

    if args.controller == "camera":
        from scripts.evaluate_coela_camera import evaluate
        evaluate(output=args.output,seeds=seeds,modes=modes,repeats=args.repeats,
                 timeout=args.timeout,record=args.record)
        return

    out = Path(args.output).resolve()
    out.mkdir(parents=True, exist_ok=False)
    source_hash = signature()
    schedule = [
        (seed, repeat, mode)
        for seed in seeds
        for repeat in range(args.repeats)
        for mode in modes
    ]
    random.Random(20260906).shuffle(schedule)
    design = {
        "architecture": "coela_inspired_local_arena_v1",
        "source_sha256": source_hash,
        "schedule": schedule,
        "modes": modes,
        "seeds": seeds,
        "repeats": args.repeats,
        "matched_cohort_runs": len(schedule),
        "warehouse_layout": "arena",
        "static_obstacles": "seeded before episode; never injected",
        "timeout_s": args.timeout,
        "max_calls_per_robot": 24,
        "response_tokens": 768,
        "decision_period_s": 3.0,
        "information_mode": "strict_local; layout is not included in actor inputs",
        "recording": args.record,
        "comparative_wall_time_includes_recordings": False,
    }
    (out / "design.json").write_text(json.dumps(design, ensure_ascii=False, indent=2))

    results = []
    for seed, repeat, mode in schedule:
        run = out / f"{seed}-arena-{repeat}-{mode}"
        run.mkdir()
        world = video = dialogue = None
        recording = args.record and mode == "natural"
        static_before = []
        try:
            if signature() != source_hash:
                raise RuntimeError("SOURCE_CHANGED_DURING_COMPARISON")
            world = MultiMasterPiProductionV2(
                warehouse_layout="arena", seed=seed, render=True,
            )
            static_before = static_geometry_snapshot(world)
            layout = layout_record(world)
            layout["static_geometry_before"] = static_before
            (run / "layout.json").write_text(
                json.dumps(layout, ensure_ascii=False, indent=2)
            )
            if recording:
                from scripts.record_parallel_warehouse import CrewVideo
                video = CrewVideo(world, run / "motion-1x.mp4")
                video_lock = threading.Lock()

                def capture():
                    with video_lock:
                        video.capture()

                video.capture(force=True)
                world.frame_callback = capture
                dialogue = (run / "dialogue.jsonl").open("w")

            message_lock = threading.Lock()

            def on_message(event):
                print(mode, event["sender_id"], event["content"], flush=True)
                if dialogue:
                    with message_lock:
                        dialogue.write(json.dumps({
                            "robot_id": event["sender_id"],
                            "recipients": event["recipients"],
                            "message": event["content"],
                            "sim_time": event["sent_sim_time"],
                            "video_time": max(0.0, event["sent_sim_time"] - video.start),
                            "source": "actual_coela_message_sent",
                        }, ensure_ascii=False) + "\n")
                        dialogue.flush()

            planners = {
                rid: Planner(rid, GeminiProxyCompleter(max_tokens=768, timeout=30))
                for rid in ROBOTS
            }
            result = run_coela_episode(
                LocalEnvironment(world), planners, mode=mode,
                destination=world.warehouse_arena.destination_zone,
                timeout_s=args.timeout, journal_path=run / "episode.jsonl",
                event_callback=on_message,
            )
            engine = world._mixed_engine
            result.update(
                sim_time=float(world.data.time),
                task_overlap_seconds=engine.task_overlap_seconds,
                cargo_motion_overlap_seconds=engine.cargo_motion_overlap_seconds,
                peer_collision_events=engine.peer_collision_events,
                peer_contact_seconds=engine.peer_contact_seconds,
                terrain_contact_seconds=engine.terrain_contact_seconds,
                motion=engine.recorder.summary(),
                static_obstacle_count=len(static_before),
            )
            (run / "physics-trace.json").write_text(json.dumps(world.warehouse_trace, ensure_ascii=False, indent=2))
            result = score_arena_result(
                result, static_before, static_geometry_snapshot(world),
            )
            result["evidence"] = {
                "task": {
                    "complete": bool(result.get("status", {}).get("complete")),
                    "overall_task_success": result["overall_task_success"],
                    "first_execution_failure": result["first_execution_failure"],
                },
                "contact": {
                    "peer_collision_events": result["peer_collision_events"],
                    "peer_contact_seconds": result["peer_contact_seconds"],
                },
                "motion": result["motion"],
                "static_geometry": {
                    "before": result["static_geometry_before"],
                    "after": result["static_geometry_after"],
                    "present": result["static_geometry_present"],
                    "unchanged": result["static_geometry_unchanged"],
                },
            }
        except Exception as exc:
            after = []
            snapshot_error = None
            if world:
                try:
                    after = static_geometry_snapshot(world)
                except Exception as geometry_exc:
                    snapshot_error = str(geometry_exc)
            result = score_arena_result(
                {"success": False, "reason": str(exc)}, static_before, after,
            )
            if snapshot_error:
                result["static_geometry_snapshot_error"] = snapshot_error
        finally:
            if world:
                world.frame_callback = None
                if getattr(world, "_mixed_engine", None):
                    world._mixed_engine.close()
                if video:
                    video.close()
                if dialogue:
                    dialogue.close()
                world.close()
        result.update(
            seed=seed, scenario="arena", repeat=repeat, mode=mode,
            recorded=recording, source_consistent=signature() == source_hash,
        )
        if not result["source_consistent"]:
            result.update(
                success=False, overall_task_success=False,
                reason="SOURCE_CHANGED_DURING_COMPARISON",
                primary_failure="SOURCE_CHANGED_DURING_COMPARISON",
            )
        (run / "result.json").write_text(
            json.dumps(result, ensure_ascii=False, indent=2)
        )
        results.append(result)
        summary = {
            candidate: {
                "successes": sum(
                    bool(item["overall_task_success"])
                    for item in results if item["mode"] == candidate
                ),
                "denominator": sum(item[2] == candidate for item in schedule),
                "completed_episodes": sum(
                    item["mode"] == candidate for item in results
                ),
            }
            for candidate in modes
        }
        (out / "summary.json").write_text(json.dumps(
            {"summary": summary, "results": results},
            ensure_ascii=False, indent=2,
        ))
        if video:
            from scripts.render_warehouse_dialogue import render_dialogue_video
            render_dialogue_video(
                run / "motion-1x.mp4", run / "dialogue.jsonl",
                run / "dialogue-1x.mp4",
            )
        print(
            "RESULT", seed, "arena", mode,
            result["overall_task_success"], result["reason"], flush=True,
        )
        if not result["source_consistent"]:
            raise RuntimeError("SOURCE_CHANGED_DURING_COMPARISON")


if __name__ == "__main__":
    main()
