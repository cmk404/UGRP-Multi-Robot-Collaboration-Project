#!/usr/bin/env python3
"""Diagnostic R2 full-redelivery probe from the dev-01 232 s raw prefix.

The prefix is reconstructed by applying recorded actuator commands to current
physics.  This script never writes qpos, teleports a body, creates a weld, or
uses simulator truth to control the recovery.  Truth is sampled only for the
final offline result.
"""
from __future__ import annotations

import base64
import hashlib
import json
import os
import sys
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
PREFIX = ROOT / "outputs/warehouse_research/gemini38-team-dev-01"
OUT = Path(os.environ.get(
    "CAMERA_REDELIVERY_OUT",
    ROOT / "outputs/warehouse_research/recovery-redelivery-01",
)).resolve()
RID, CARGO, ZONE = "r2", "small_box_02", "B"
CUTOFF_S = 232.0
MAX_EXTRA_S = 180.0
try:
    MAX_MODEL_CALLS = max(1, int(os.environ.get("CAMERA_REDELIVERY_MAX_CALLS", "60")))
except ValueError:
    MAX_MODEL_CALLS = 60

if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def rows(path: Path) -> list[dict[str, Any]]:
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n")


def append_jsonl(path: Path, value: Any) -> None:
    with path.open("a") as stream:
        stream.write(json.dumps(value, ensure_ascii=False, allow_nan=False) + "\n")


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def prefix_events(commands: list[dict[str, Any]], controls: list[dict[str, Any]],
                  decisions: list[dict[str, Any]], guards: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Recover raw actions and stops exactly as the earlier probe did."""
    events = [dict(item) for item in commands
              if item.get("event") in {"raw_action", "raw_stop", "explicit_stop"}
              and float(item.get("time", 1e9)) <= CUTOFF_S]
    stops = [item for item in events if item.get("event") in {"raw_stop", "explicit_stop"}]
    inferred: list[dict[str, Any]] = []
    for item in decisions:
        disposition = str(item.get("disposition") or "")
        decision = item.get("decision")
        action = decision.get("action", {}) if isinstance(decision, dict) else {}
        if disposition.startswith("rejected_") or (disposition == "accepted" and action.get("kind") == "finish"):
            inferred.append({"event": "inferred_stop", "time": item["time"], "robot_id": item["robot_id"]})
    for item in guards:
        if item.get("skill_state") != "carrying":
            inferred.append({"event": "inferred_stop", "time": item["time"], "robot_id": item["robot_id"]})
    for item in controls:
        if isinstance(item.get("action"), dict) and item["action"].get("kind") == "finish":
            inferred.append({"event": "inferred_stop", "time": item["time"], "robot_id": item["robot_id"]})
    for candidate in inferred:
        if float(candidate["time"]) > CUTOFF_S:
            continue
        duplicate = any(str(item.get("robot_id")) == str(candidate["robot_id"])
                        and abs(float(item["time"]) - float(candidate["time"])) <= .003
                        for item in stops)
        if not duplicate:
            events.append(candidate)
    return sorted(events, key=lambda item: float(item["time"]))


class FreshFrameGuard:
    """Reject reuse of either capture across model decisions."""

    def __init__(self) -> None:
        self._previous: dict[str, tuple[Any, str, float]] = {}

    def accept(self, wrist: dict[str, Any], nav: dict[str, Any]) -> None:
        current = {"robot_cam": wrist, "nav_cam": nav}
        for camera, obs in current.items():
            signature = (obs["frame_id"], obs["sha256"], float(obs["sim_time"]))
            if camera in self._previous:
                old = self._previous[camera]
                # A stationary scene may legitimately encode to the same JPEG.
                # Freshness comes from the camera capture identity and sim time;
                # the digest remains recorded for evidence integrity.
                if signature[0] == old[0] or signature[2] <= old[2]:
                    raise ValueError("STALE_CAMERA_FRAME")
            self._previous[camera] = signature


def save_pair(wrist: dict[str, Any], nav: dict[str, Any], call: int) -> dict[str, Any]:
    folder = OUT / "inputs" / RID
    folder.mkdir(parents=True, exist_ok=True)
    saved = []
    for label, obs in (("wrist", wrist), ("nav", nav)):
        relative = Path("inputs") / RID / f"{call:04d}-{label}.jpg"
        (OUT / relative).write_bytes(base64.b64decode(obs["image"]))
        saved.append({"camera": label, "path": str(relative), "frame_id": obs["frame_id"],
                      "sim_time": obs["sim_time"], "sha256": obs["sha256"]})
    return {"images": saved}


def source_record() -> dict[str, Any]:
    original_manifest_path = PREFIX / "source-manifest.json"
    original = json.loads(original_manifest_path.read_text())
    changed, missing = [], []
    for name, expected in original.items():
        if not (name.startswith("sim/") or name.startswith("calibration/")):
            continue
        path = ROOT / name
        if not path.exists():
            missing.append(name)
        elif sha256(path) != expected:
            changed.append(name)
    current = {str(path.relative_to(ROOT)): sha256(path)
               for base in ("sim", "harness", "scripts", "calibration")
               for path in sorted((ROOT / base).rglob("*"))
               if path.is_file() and path.suffix in {".py", ".xml", ".json", ".png", ".yaml", ".yml"}
               and "__pycache__" not in path.parts}
    write_json(OUT / "source-manifest.json", current)
    return {
        "original_prefix": str(PREFIX.relative_to(ROOT)),
        "original_prefix_manifest_sha256": sha256(original_manifest_path),
        "current_physics_reconstruction": True,
        "original_physical_sources_match": not changed and not missing,
        "changed_since_original": changed,
        "missing_since_original": missing,
        "current_source_manifest_sha256": hashlib.sha256(
            json.dumps(current, sort_keys=True).encode()).hexdigest(),
    }


def initialize_checkpoint() -> tuple[Any, dict[str, Any], dict[str, Any]]:
    """Replay the raw dev-01 prefix into a newly initialized current world."""
    import mujoco
    from sim.camera_robot_port import CameraRobotPort
    from sim.multi_masterpi_production import MultiMasterPiProductionV2

    result = json.loads((PREFIX / "result.json").read_text())
    commands, controls, decisions = (rows(PREFIX / name) for name in
                                     ("commands.jsonl", "control.jsonl", "llm-decisions.jsonl"))
    guard_path = PREFIX / "visual-guard.jsonl"
    guards = rows(guard_path) if guard_path.exists() else []
    events = prefix_events(commands, controls, decisions, guards)
    active = tuple(result["active_robots"])
    cargo_ids = tuple(result["outcomes"][rid]["cargo_id"] for rid in active)
    world = MultiMasterPiProductionV2(warehouse_layout="camera_team", seed=int(result["seed"]),
                                      render=True, warehouse_cargo_ids=cargo_ids)
    physics = result.get("physics", {})
    world.model.opt.impratio = float(physics.get("impratio", world.model.opt.impratio))
    world.model.opt.noslip_iterations = int(physics.get("noslip_iterations", world.model.opt.noslip_iterations))
    for zone in "abc":
        gid = mujoco.mj_name2id(world.model, mujoco.mjtObj.mjOBJ_GEOM, "warehouse_zone_" + zone)
        world.model.geom_group[gid] = 0
    ports = {rid: CameraRobotPort(world, rid) for rid in active}
    event_index = 0
    while float(world.data.time) < CUTOFF_S - 1e-12:
        now = float(world.data.time)
        for port in ports.values():
            port.tick(now)
        while event_index < len(events) and float(events[event_index]["time"]) <= now + 1e-9:
            event = events[event_index]
            port = ports[str(event["robot_id"])]
            port.apply(event["raw_action"], now) if event["event"] == "raw_action" else port.stop()
            event_index += 1
        world._physics_step_for(world.robot("r1"))
    for port in ports.values():
        port.stop()
    return world, ports, result


def inside_destination(world: Any, state: dict[str, Any]) -> bool:
    from scripts.evaluate_gemini_team import footprint_inside

    box = state["cargo"][CARGO]
    zone = world.warehouse_zones[ZONE]
    dimensions = world.warehouse_spec_by_id[CARGO].dimensions_m
    return footprint_inside(box["position"], box.get("yaw", 0), dimensions,
                            zone.center_xy, zone.half_extents_xy)


def main() -> None:
    from harness.gemini_proxy import GeminiProxyCompleter
    from harness.gemini_transport_policy import GeminiTransportPlanner
    from harness.llm_transport_skill import LLMTransportSkill
    from harness.visual_drive_guard import validate_visual_drive
    from harness.visual_macro_runtime import VisualMacroExecutor
    from harness.visual_placement import inspect_placement
    from scripts.record_visual_team import VisualTeamVideo

    OUT.mkdir(parents=True, exist_ok=False)
    for name in ("commands.jsonl", "decisions.jsonl"):
        (OUT / name).write_text("")
    outcome: dict[str, Any] = {
        "diagnostic_only": True,
        "claim_scope": "single_R2_full_redelivery_diagnostic_not_original_replay_or_team_success",
        "reconstruction": "dev01 raw actuator prefix through 232 s under current physics",
        "truth_control": False,
        "teleport_or_constraint_control": False,
        "active_recovery_robot": RID,
        "stationary_robots": ["r1", "r3"],
        "limits": {"model_calls": MAX_MODEL_CALLS, "extra_sim_s": MAX_EXTRA_S},
        "diagnostic_limitations": {
            "synchronous_inference_freezes_physics": True,
            "normal_team_latency_comparison_valid": False,
            "transient_inference_recovery_tested": False,
            "inference_failure_policy": "fail honestly; no automatic model replacement",
        },
    }
    world = video = None
    try:
        outcome["source"] = source_record()
        world, ports, _ = initialize_checkpoint()
        initial = world.warehouse_state()
        wrist, nav = ports[RID].capture("robot_cam"), ports[RID].capture("nav_cam")
        initial_evidence = inspect_placement(wrist, nav, cargo_id=CARGO, destination_zone=ZONE,
                                             stage="released", held_identity_confirmed=False)
        outcome["checkpoint"] = {"sim_time": float(world.data.time),
                                 "placement_evidence": initial_evidence}
        if initial_evidence.get("status") != "outside":
            outcome["status"] = "REPLAY_OUTSIDE_PRECONDITION_NOT_RECONSTRUCTED"
            write_json(OUT / "result.json", outcome)
            return

        actor = LLMTransportSkill(RID, CARGO, ZONE)
        actor.state = "released"
        actor.observe_placement(initial_evidence)
        planner = GeminiTransportPlanner(
            RID, GeminiProxyCompleter(model="gemini-3.8-flash", max_tokens=800, timeout=45))
        executor = VisualMacroExecutor(ports[RID], lambda item: append_jsonl(OUT / "commands.jsonl", item))
        memory: list[dict[str, Any]] = [{"placement_evidence": initial_evidence}]
        freshness = FreshFrameGuard()
        calls, terminal = 0, None
        start = float(world.data.time)
        max_z = float(initial["cargo"][CARGO]["position"][2])
        samples = [initial]
        video = VisualTeamVideo(world, OUT / "redelivery-1x.mp4", {RID: actor})

        while float(world.data.time) - start < MAX_EXTRA_S and calls < MAX_MODEL_CALLS:
            now = float(world.data.time)
            executor.tick(now)
            if executor.idle:
                wrist = ports[RID].capture("robot_cam")
                primitive = actor.advance(wrist)
                if primitive is not None:
                    if primitive.get("kind") == "finish":
                        terminal = primitive.get("reason", "primitive_failure")
                        break
                    executor.submit(primitive, wrist, actor.phase, now)
                elif actor.state in {"failure", "finished"}:
                    terminal = actor.reason
                    break
                elif actor.operation is None:
                    nav = ports[RID].capture("nav_cam")
                    freshness.accept(wrist, nav)
                    calls += 1
                    request_evidence = None
                    if actor.state in {"carrying", "released"}:
                        request_evidence = inspect_placement(
                            wrist, nav, cargo_id=CARGO, destination_zone=ZONE,
                            stage="released" if actor.state == "released" else "before_release",
                            held_identity_confirmed=actor.held)
                        actor.observe_placement(request_evidence)
                        memory.append({"placement_evidence": request_evidence})
                    saved = save_pair(wrist, nav, calls)
                    decision = planner.decide(wrist, nav, memory[-16:], cargo_id=CARGO,
                                              destination_zone=ZONE, skill_state=actor.state)
                    fresh_placement = None
                    rejection = None
                    macro = None
                    try:
                        if decision["action"]["kind"] == "drive":
                            guard_nav = ports[RID].capture("nav_cam")
                            guard = validate_visual_drive(guard_nav, decision["action"])
                            if not guard["allowed"]:
                                raise ValueError("OWN_RGB_DRIVE_BLOCKED:" + guard["reason"])
                        if decision["action"]["kind"] in {"release", "finish"}:
                            placement_wrist = ports[RID].capture("robot_cam")
                            placement_nav = ports[RID].capture("nav_cam")
                            fresh_placement = inspect_placement(
                                placement_wrist, placement_nav, cargo_id=CARGO, destination_zone=ZONE,
                                stage="released" if actor.state == "released" else "before_release",
                                held_identity_confirmed=actor.held)
                        macro = actor.request(decision["action"], placement_evidence=fresh_placement)
                    except ValueError as exc:
                        message = str(exc)
                        if not message.startswith(("OWN_RGB_DRIVE_BLOCKED:", "SAFE_INSIDE_",
                                                   "CAMERA_INSIDE_")):
                            raise
                        ports[RID].stop()
                        rejection = message
                        if fresh_placement is not None:
                            memory.append({"placement_evidence": fresh_placement})
                        memory.append({"feedback": message[:300], "skill_state": actor.state})
                    actor.last_llm_reason = decision["reason"]
                    if rejection is None:
                        memory.append({"action": decision["action"], "reason": decision["reason"],
                                       "feedback": "accepted", "skill_state": actor.state})
                    append_jsonl(OUT / "decisions.jsonl", {
                        "call": calls, "time": now, "skill_state": actor.state,
                        "decision": decision, "audit": planner.last_audit,
                        "request_placement": request_evidence,
                        "fresh_placement": fresh_placement,
                        "disposition": "rejected_camera_guard" if rejection else "accepted",
                        "error": rejection, **saved,
                    })
                    memory[:] = memory[-16:]
                    video.update_inputs(RID, wrist, nav, now)
                    if rejection is None and macro is not None:
                        if macro.get("kind") == "finish":
                            terminal = macro.get("reason")
                            break
                        executor.submit(macro, wrist, actor.phase, now)
            for rid, port in ports.items():
                if rid != RID:
                    port.stop()
                port.tick(float(world.data.time))
            world._physics_step_for(world.robot("r1"))
            state = world.warehouse_state()
            samples.append(state)
            max_z = max(max_z, float(state["cargo"][CARGO]["position"][2]))
            if float(world.data.time) >= video.next_frame:
                video.capture()

        for port in ports.values():
            port.stop()
        settle_until = float(world.data.time) + 2.0
        while float(world.data.time) < settle_until:
            for port in ports.values():
                port.tick(float(world.data.time))
            world._physics_step_for(world.robot("r1"))
            samples.append(world.warehouse_state())
            if float(world.data.time) >= video.next_frame:
                video.capture()
        final = world.warehouse_state()
        box = final["cargo"][CARGO]
        constraints = any(any(sample["cargo"][CARGO].get("constraints_active", {}).values())
                          for sample in samples)
        gates = {
            "lift": max_z - float(initial["cargo"][CARGO]["position"][2]) >= .04,
            "inside_destination": inside_destination(world, final),
            "stable": box.get("stable") is True,
            "no_attachment_constraint": not constraints,
            "visual_release": terminal == "VISUAL_RELEASE_CONFIRMED" and actor.state == "finished",
        }
        outcome.update(status="FULL_REDELIVERY_CONFIRMED" if all(gates.values()) else
                       "FULL_REDELIVERY_NOT_CONFIRMED", success=all(gates.values()), gates=gates,
                       terminal=terminal or "limit", actor_state=actor.state, model_calls=calls,
                       extra_sim_s=float(world.data.time) - start,
                       max_lift_m=max_z - float(initial["cargo"][CARGO]["position"][2]),
                       video={"path": "redelivery-1x.mp4", "fps": 8,
                              "includes_prefix_replay": False,
                              "panels": "external overview, R2 own RGB and Gemini reasons; R1/R3 stationary"})
        write_json(OUT / "result.json", outcome)
    except Exception as exc:
        outcome.update(status="PROBE_ERROR", success=False,
                       error={"type": type(exc).__name__, "message": str(exc)[:300]})
        if OUT.exists():
            write_json(OUT / "result.json", outcome)
        raise
    finally:
        if video is not None:
            video.close()
        if world is not None:
            world.close()


if __name__ == "__main__":
    main()
