"""Replay a Gemini team episode from logged raw commands, without an LLM."""

from __future__ import annotations

import argparse
import base64
import hashlib
import json
import math
import subprocess
import sys
from collections import Counter
from pathlib import Path
from types import SimpleNamespace

import mujoco

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from sim.camera_robot_port import CameraRobotPort
from sim.multi_masterpi_production import MultiMasterPiProductionV2
from scripts.record_visual_team import VisualTeamVideo


TOLERANCE_M = 0.001


def _json(path: Path) -> dict:
    value = json.loads(path.read_text())
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON object: {path}")
    return value


def _rows(path: Path) -> list[dict]:
    result = []
    for line in path.read_text().splitlines():
        value = json.loads(line)
        if isinstance(value, dict):
            result.append(value)
    return result


def _is_recorded_stop(row: dict) -> bool:
    if row.get("event") in {"raw_stop", "explicit_stop"}:
        return True
    raw = row.get("raw_action")
    return row.get("event") == "raw_action" and isinstance(raw, dict) and raw.get("kind") == "wait"


def _position_errors(actual: dict, expected: dict, cargo_ids: tuple[str, ...]) -> dict:
    errors = {}
    for cargo_id in cargo_ids:
        a = actual["cargo"][cargo_id]["position"]
        b = expected["cargo"][cargo_id]["position"]
        errors[cargo_id] = math.dist(a, b)
    return errors


def _source_check(run_dir: Path) -> dict:
    recorded = _json(run_dir / "source-manifest.json")
    checked = {}
    missing = []
    changed = []
    # These files define initial geometry and physical integration. Reporter and
    # replay code do not affect the raw-command trajectory.
    for name, expected in recorded.items():
        if not (name.startswith("sim/") or name.startswith("calibration/")):
            continue
        path = Path(name)
        if not path.is_file():
            missing.append(name)
            continue
        actual = hashlib.sha256(path.read_bytes()).hexdigest()
        checked[name] = actual
        if actual != expected:
            changed.append(name)
    return {"physical_sources_match": not missing and not changed,
            "checked_files": len(checked), "missing": missing, "changed": changed}


def _observation(path: Path, rid: str, camera: str, sha256: str | None = None) -> dict:
    jpeg = path.read_bytes()
    return {"robot_id": rid, "frame_id": 0, "sim_time": 0.0,
            "image": base64.b64encode(jpeg).decode("ascii"),
            "sha256": sha256 or hashlib.sha256(jpeg).hexdigest(), "camera": camera}


def _mark_invalid(video_path: Path) -> None:
    temporary = video_path.with_name(video_path.stem + ".invalid-overlay.mp4")
    subprocess.run([
        "ffmpeg", "-y", "-loglevel", "error", "-i", str(video_path),
        "-vf", "drawbox=x=0:y=0:w=iw:h=72:color=red@0.88:t=fill,"
               "drawtext=text='INVALID PHYSICS REPLAY':fontcolor=white:fontsize=34:x=24:y=18",
        "-an", "-c:v", "libx264", "-preset", "veryfast", "-crf", "21",
        "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(temporary),
    ], check=True)
    temporary.replace(video_path)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--output-video", type=Path)
    parser.add_argument("--fps", type=int, choices=(4, 8), default=8)
    parser.add_argument("--no-video", action="store_true", help="run physics validation without rendering")
    args = parser.parse_args()
    run_dir = args.run_dir.resolve()
    result = _json(run_dir / "result.json")
    commands = _rows(run_dir / "commands.jsonl")
    controls = _rows(run_dir / "control.jsonl")
    llm = _rows(run_dir / "llm-decisions.jsonl")
    truth = _rows(run_dir / "evaluation-only.jsonl")
    monitors = _rows(run_dir / "visual-guard.jsonl") if (run_dir / "visual-guard.jsonl").exists() else []
    messages = _rows(run_dir / "messages.jsonl") if (run_dir / "messages.jsonl").exists() else []
    active = tuple(result["active_robots"])
    cargo_ids = tuple(result["outcomes"][rid]["cargo_id"] for rid in active)
    output = (args.output_video or run_dir / "motion-1x.mp4").resolve()
    if output.exists() and not args.no_video:
        raise FileExistsError(output)

    physics = result.get("physics", {})
    world = MultiMasterPiProductionV2(warehouse_layout="camera_team", seed=int(result["seed"]),
                                      render=True, warehouse_cargo_ids=cargo_ids)
    world.model.opt.impratio = float(physics.get("impratio", world.model.opt.impratio))
    world.model.opt.noslip_iterations = int(physics.get("noslip_iterations", world.model.opt.noslip_iterations))
    for zone in "abc":
        gid = mujoco.mj_name2id(world.model, mujoco.mjtObj.mjOBJ_GEOM, "warehouse_zone_" + zone)
        world.model.geom_group[gid] = 0
    ports = {rid: CameraRobotPort(world, rid) for rid in active}
    actors = {rid: SimpleNamespace(phase="idle", destination_zone=result["outcomes"][rid]["destination_zone"],
                                   last_llm_reason="", last_message=None, held=None) for rid in active}
    video = None
    if not args.no_video:
        old_fps = VisualTeamVideo.FPS
        VisualTeamVideo.FPS = args.fps
        video = VisualTeamVideo(world, output, actors)
        video.FPS = args.fps
        VisualTeamVideo.FPS = old_fps

    source_check = _source_check(run_dir)
    replay_initial = world.warehouse_state()
    initial_errors = _position_errors(replay_initial, result["initial"], cargo_ids)
    command_events = [dict(row) for row in commands if row.get("event") in {"raw_action", "raw_stop", "explicit_stop"}]
    recorded_stops = any(_is_recorded_stop(row) for row in command_events)
    inferred_stops = []
    # Older runs logged drive-lease stops but omitted some direct port.stop()
    # calls. Reconstruct only stops whose time and cause exist in other logs.
    candidates = []
    for row in llm:
        disposition = str(row.get("disposition") or "")
        action = row.get("decision", {}).get("action", {}) if isinstance(row.get("decision"), dict) else {}
        if disposition.startswith("rejected_") or (disposition == "accepted" and action.get("kind") == "finish"):
            candidates.append({"event": "inferred_stop", "time": row["time"],
                               "robot_id": row["robot_id"], "cause": disposition or "finish"})
    for row in monitors:
        if row.get("skill_state") != "carrying":
            candidates.append({"event": "inferred_stop", "time": row["time"],
                               "robot_id": row["robot_id"], "cause": "visual_guard_state_change"})
    for row in controls:
        action = row.get("action", {})
        if isinstance(action, dict) and action.get("kind") == "finish":
            candidates.append({"event": "inferred_stop", "time": row["time"],
                               "robot_id": row["robot_id"], "cause": "deterministic_finish"})
    recorded_stop_rows = [row for row in command_events if _is_recorded_stop(row)]
    for candidate in candidates:
        duplicate = any(str(row.get("robot_id")) == str(candidate["robot_id"]) and
                        abs(float(row["time"]) - float(candidate["time"])) <= .003
                        for row in recorded_stop_rows)
        if not duplicate:
            inferred_stops.append(candidate)
    command_events.extend(inferred_stops)
    command_events.sort(key=lambda row: float(row["time"]))
    control_events = sorted(controls, key=lambda row: float(row.get("time", 0)))
    reason_events = sorted((row for row in llm if row.get("disposition") == "accepted"),
                           key=lambda row: float(row.get("time", 0)))
    message_events = sorted((row for row in messages
                             if str(row.get("sender")) in actors and isinstance(row.get("sent_at"), (int, float))),
                            key=lambda row: float(row["sent_at"]))
    ci = ui = ri = mi = 0
    start = float(world.data.time)
    end = start + float(result["elapsed_sim_s"])
    reference = truth[-1] if truth else None
    reference_time = float(reference["time"]) if reference else None
    replay_robots_at_reference = None
    contacts = Counter()
    r3_near_contacts = {}
    if video:
        video.capture(force=True)
    try:
        while world.data.time < end - 1e-12:
            now = float(world.data.time)
            for port in ports.values():
                port.tick(now)
            while ci < len(command_events) and float(command_events[ci]["time"]) <= now + 1e-9:
                row = command_events[ci]; rid = str(row["robot_id"])
                if row["event"] == "raw_action":
                    ports[rid].apply(row["raw_action"], now)
                else:
                    ports[rid].stop()
                ci += 1
            while ui < len(control_events) and float(control_events[ui].get("time", 0)) <= now + 1e-9:
                row = control_events[ui]; rid = str(row["robot_id"])
                actors[rid].phase = str(row.get("phase", actors[rid].phase))
                step = row.get("step")
                if isinstance(step, int):
                    wrist_path = run_dir / "inputs" / rid / f"{step:04d}-wrist.jpg"
                    nav_path = run_dir / "inputs" / rid / f"{step:04d}-nav.jpg"
                    if wrist_path.exists() and nav_path.exists():
                        if video:
                            video.update_inputs(rid, _observation(wrist_path, rid, "robot_cam", row.get("wrist_sha256")),
                                                _observation(nav_path, rid, "nav_cam", row.get("nav_sha256")), now)
                ui += 1
            while ri < len(reason_events) and float(reason_events[ri].get("time", 0)) <= now + 1e-9:
                row = reason_events[ri]; rid = str(row["robot_id"])
                decision = row.get("decision", {})
                actors[rid].last_llm_reason = str(decision.get("reason", ""))
                ri += 1
            while mi < len(message_events) and float(message_events[mi]["sent_at"]) <= now + 1e-9:
                row = message_events[mi]
                actors[str(row["sender"])].last_message = row
                mi += 1
            if reference_time is not None and replay_robots_at_reference is None and now >= reference_time - 1e-9:
                replay_robots_at_reference = {rid: world.robot(rid).base_xyz().tolist() for rid in active}
            for contact in world.data.contact:
                a = mujoco.mj_id2name(world.model, mujoco.mjtObj.mjOBJ_GEOM, int(contact.geom1)) or ""
                b = mujoco.mj_id2name(world.model, mujoco.mjtObj.mjOBJ_GEOM, int(contact.geom2)) or ""
                pair = " | ".join(sorted((a, b)))
                if "r3" in active and contact.dist < 0.001 and (a.startswith("r3__") or b.startswith("r3__")):
                    entry = r3_near_contacts.setdefault(pair, {"samples": 0, "min_distance_m": float(contact.dist),
                                                               "first_time": now, "last_time": now,
                                                               "samples_after_200s": 0, "samples_after_450s": 0})
                    entry["samples"] += 1;entry["last_time"] = now
                    entry["min_distance_m"] = min(entry["min_distance_m"], float(contact.dist))
                    entry["samples_after_200s"] += now - start >= 200
                    entry["samples_after_450s"] += now - start >= 450
                if contact.dist >= -0.002:
                    continue
                if "barrier" in a or "barrier" in b or any(a.startswith(r + "__") and b.startswith(q + "__") for r in active for q in active if r != q):
                    contacts[pair] += 1
            world._physics_step_for(world.robot("r1"))
            if video and world.data.time >= video.next_frame:
                video.capture()
        for port in ports.values():
            port.stop()
        replay_final = world.warehouse_state()
    finally:
        if video:
            video.close()
        world.close()

    final_errors = _position_errors(replay_final, result["final"], cargo_ids)
    robot_errors = {}
    if reference and replay_robots_at_reference:
        robot_errors = {rid: math.dist(replay_robots_at_reference[rid], reference["robot_positions"][rid])
                        for rid in active}
    initial_ok = max(initial_errors.values(), default=0) <= TOLERANCE_M
    final_ok = max(final_errors.values(), default=0) <= TOLERANCE_M
    robots_ok = max(robot_errors.values(), default=0) <= TOLERANCE_M
    command_complete = ci == len(command_events)
    valid = source_check["physical_sources_match"] and initial_ok and final_ok and robots_ok and command_complete
    validation = {"valid": valid, "tolerance_m": TOLERANCE_M, "source": source_check,
                  "initial_cargo_error_m": initial_errors, "final_cargo_error_m": final_errors,
                  "robot_error_at_last_evaluation_sample_m": robot_errors,
                  "last_evaluation_sample_time": reference_time,
                  "command_events_total": len(command_events), "command_events_applied": ci,
                  "recorded_stop_events_present": recorded_stops,
                  "inferred_stop_events": len(inferred_stops),
                  "critical_contact_samples": dict(contacts.most_common(30)),
                  "r3_contact_pairs_below_1mm": dict(sorted(r3_near_contacts.items(),
                      key=lambda item: item[1]["samples"], reverse=True)[:50]),
                  "video": str(output) if video else None, "diagnostic_overlay": bool(video and not valid)}
    validation_path = output.parent / "replay-validation.json"
    validation_path.write_text(json.dumps(validation, ensure_ascii=False, indent=2))
    if video and not valid:
        _mark_invalid(output)
    print(json.dumps(validation, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
