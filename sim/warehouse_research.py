"""Simulation referee/actuator boundary for robot-authored warehouse decisions.

This module never chooses a cargo or a robot role. Ground truth stays inside the
referee and the shared, simulation-only transport skill, outside actor inputs.
"""
from __future__ import annotations

from dataclasses import replace
import copy
import hashlib
import json
import uuid

from harness.warehouse_protocol import LocalObservation, Proposal, WarehouseProtocol
from sim.warehouse_mission import cargo_pose, warehouse_route_zones

ROBOT_IDS = ("r1", "r2", "r3")
CONDITIONS = ("rule", "llm_no_comm", "llm_peer_comm")
OPERATIONS = ("begin", "observe", "observe_batch", "execute", "status", "revise", "mixed_begin", "mixed_observe", "mixed_submit", "mixed_status", "mixed_stop", "mixed_pause", "mixed_recover", "mixed_obstacle", "mixed_local_status", "mixed_local_submit", "mixed_retire")


def validate_request(request):
    if not isinstance(request, dict) or request.get("operation") not in OPERATIONS:
        raise ValueError("invalid warehouse research operation")
    if len(json.dumps(request, allow_nan=False)) > 48000:
        raise ValueError("warehouse research request too large")
    return dict(request)


def _status(world, episode):
    # Public task outcomes, never the referee's geometry/peer state.
    state = world.warehouse_state()
    delivered = [cid for cid in episode["cargo_ids"]
                 if state["cargo"][cid]["evaluation"]["success"]
                 and state["cargo"][cid]["stable"]]
    return {
        "episode_id": episode["id"], "revision": episode["revision"],
        "destination": episode["destination"],
        "complete": len(delivered) == len(episode["cargo_ids"]),
        "delivered_ids": delivered,
        "remaining_ids": [cid for cid in episode["cargo_ids"] if cid not in delivered],
        "attempts": episode["attempts"],
        "sensor_profile": "parallel_rgbd_fiducial_memory_v4",
        "controller_profile": "independent_crew_forward_uncalibrated_v4",
    }


def _begin(world, request):
    condition = request.get("condition", "llm_peer_comm")
    source, destination = request.get("source_zone", "A"), request.get("destination_zone", "B")
    if condition not in CONDITIONS:
        raise ValueError("unknown research condition")
    route = warehouse_route_zones(source, destination)
    selector = request.get("selector", "all")
    selected = [s.cargo_id for s in world.warehouse_specs
                if selector == "all" or s.cargo_type == selector]
    if not selected:
        raise ValueError("empty manifest selection")
    # The operator manifest is task specification, not a sensor result.
    for cid in selected:
        spec = world.warehouse_spec_by_id[cid]
        pose = cargo_pose(world.data, world.model, spec)
        if world._warehouse_zone_for_position(cid, pose["position"]) != source:
            raise ValueError("INITIAL_SOURCE_MISMATCH")
    disabled = request.get("disabled_carrier")
    if disabled is not None and disabled not in ROBOT_IDS:
        raise ValueError("unknown disabled carrier")
    episode = {
        "id": uuid.uuid4().hex, "condition": condition,
        "mission_id": str(request.get("mission_id") or "warehouse_research"),
        "source": source, "destination": destination, "revision": 1,
        "cargo_ids": selected, "observations": {}, "attempts": 0,
        "used_decisions": set(), "disabled_carrier": disabled,
        "sensor_seed": int(request.get("sensor_seed", world.seed)), "observation_sequence": -1,
        "sensor_memory": {rid: {} for rid in ROBOT_IDS},
    }
    world._research_episode = episode
    world._crew_motion_recorder = None
    world._warehouse_crew_metrics = {}
    world.warehouse_trace = []
    world.warehouse_status = "NEGOTIATING"
    world.warehouse_mission = {
        "mission_id": episode["mission_id"], "source_zone": source,
        "destination_zone": destination, "selector": selector, "route": list(route),
        "decision_mode": condition, "decision_authority": "robot_proposals",
    }
    with world._warehouse_goal_lock:
        world._warehouse_requested_destination = destination
        world._warehouse_goal_revision = 1
        world._warehouse_goal_reason = "research_operator_goal"
    world._sync_warehouse_goal_specs(selected, destination)
    world._record_warehouse_phase("research_started", condition=condition,
                                  decision_authority="robot_proposals")
    result = _status(world, episode)
    result["manifest"] = [{"cargo_id": cid, "label_color": world.warehouse_spec_by_id[cid].label_color}
                          for cid in selected]
    result["world_seed"] = world.seed
    return result


def dispatch(world, robot_id, request):
    """Called under the world's command lock; returns only bounded public data."""
    request = validate_request(request)
    op = request["operation"]
    if op == "begin":
        return _begin(world, request)
    episode = getattr(world, "_research_episode", None)
    if not episode or request.get("episode_id") != episode["id"]:
        raise ValueError("STALE_EPISODE")
    if episode.get("closed"):
        raise ValueError("EPISODE_INTERRUPTED_BY_EXTERNAL_ACTION")
    if op == "status":
        return _status(world, episode)
    if op == "revise":
        destination = request.get("destination")
        if destination not in {"A", "B", "C"}:
            raise ValueError("invalid destination")
        if destination == episode["destination"]:
            raise ValueError("goal revision must change destination")
        episode["destination"] = destination
        episode["revision"] += 1
        episode["observations"].clear()
        with world._warehouse_goal_lock:
            world._warehouse_requested_destination = destination
            world._warehouse_goal_revision = episode["revision"]
            world._warehouse_goal_reason = "research_operator_revision"
        world._sync_warehouse_goal_specs(episode["cargo_ids"], destination)
        world.warehouse_mission["destination_zone"] = destination
        world.warehouse_status = "NEGOTIATING"
        world._record_warehouse_phase("research_goal_revised", revision=episode["revision"], destination=destination)
        return _status(world, episode)
    if op in {"observe", "observe_batch"}:
        from sim.warehouse_observation import observe_robot_scan, observe_robots_scan
        sequence = int(request.get("sequence", 0))
        if sequence < episode["observation_sequence"]:
            raise ValueError("STALE_OBSERVATION_SEQUENCE")
        if sequence > episode["observation_sequence"]:
            episode["observation_sequence"] = sequence
            episode["observations"].clear()
        remaining = _status(world, episode)["remaining_ids"]
        ids = {rid: f"{episode['id']}:{episode['revision']}:{sequence}:{rid}" for rid in ROBOT_IDS}
        sensor_seed = episode["sensor_seed"] + sequence*31
        if op == "observe_batch":
            samples = observe_robots_scan(world, ROBOT_IDS, pending_ids=remaining,
                                          observation_ids=ids, sensor_seed=sensor_seed)
            return {"observations": {rid: _store_observation(world, episode, rid, sequence, samples[rid])
                                     for rid in ROBOT_IDS}}
        sample = observe_robot_scan(world, robot_id, pending_ids=remaining,
                                    observation_id=ids[robot_id], sensor_seed=sensor_seed)
        return _store_observation(world, episode, robot_id, sequence, sample)
    if op == "execute":
        return _execute(world, episode, request)
    raise ValueError("unsupported research operation")


def _store_observation(world, episode, robot_id, sequence, observation):
    oid = f"{episode['id']}:{episode['revision']}:{sequence}:{robot_id}"
    remaining = _status(world, episode)["remaining_ids"]
    observation = copy.deepcopy(observation)
    observation = {key: observation[key] for key in
                   ("camera", "cargo", "scan_pans", "sensor_source", "provenance", "limits")
                   if key in observation}
    memory = episode["sensor_memory"][robot_id]
    for cargo in observation.get("cargo", []):
        memory[cargo["cargo_id"]] = {
            "cargo_id": cargo["cargo_id"], "label_color": cargo.get("label_color"),
            "source_observation_id": oid,
        }
        cargo["current_view"] = True
    visible_ids = {cargo["cargo_id"] for cargo in observation.get("cargo", [])}
    for cid, remembered in memory.items():
        if cid in remaining and cid not in visible_ids:
            # Identity memory is legitimate local evidence. Never reuse an
            # old camera-relative range/bearing after the robot has moved.
            observation.setdefault("cargo", []).append({
                **remembered, "current_view": False, "range_m": None, "bearing_rad": None,
                "confidence": None, "provenance": "robot_local_episodic_fiducial_memory",
            })
    observation.update({
        "robot_id": robot_id, "observation_id": oid, "revision": episode["revision"],
        "sequence": sequence,
        "destination": episode["destination"],
        "capabilities": {"can_carry": robot_id != episode["disabled_carrier"], "can_scout": True},
    })
    remaining = _status(world, episode)["remaining_ids"]
    observation["cargo"] = [{key: c[key] for key in
                             ("cargo_id", "label_color", "range_m", "bearing_rad", "scan_pan", "confidence", "provenance", "current_view", "source_observation_id")
                             if key in c}
                            for c in observation.get("cargo", []) if c["cargo_id"] in remaining]
    episode["observations"][robot_id] = observation
    return observation


def _execute(world, episode, request):
    observations = episode["observations"]
    if set(observations) != set(ROBOT_IDS):
        raise ValueError("FRESH_OBSERVATIONS_REQUIRED")
    if len({o.get("sequence") for o in observations.values()}) != 1:
        raise ValueError("MIXED_OBSERVATION_SEQUENCES")
    proposals = request.get("proposals")
    if not isinstance(proposals, dict) or set(proposals) != set(ROBOT_IDS):
        raise ValueError("THREE_AGENT_DECISIONS_REQUIRED")
    protocol = WarehouseProtocol()
    local = {rid: LocalObservation(
        rid, tuple(c["cargo_id"] for c in obs["cargo"]),
        (episode["destination"],), episode["revision"], obs["observation_id"],
    ) for rid, obs in observations.items()}
    peer_comm = episode["condition"] != "llm_no_comm"
    round_id = protocol.start_round(local, cargo_id=None, destination=episode["destination"],
                                    revision=episode["revision"], peer_comm=peer_comm)
    # Reports contain only actually observed cargo and only enter the peer arm.
    for report in request.get("reports", []):
        protocol.publish_observation_report(round_id, report["sender_id"], report["cargo_id"], report["message"])
    for rid in ROBOT_IDS:
        value = dict(proposals[rid])
        if value.get("robot_id") != rid:
            raise ValueError("DECISION_ORIGIN_MISMATCH")
        protocol.submit_proposal(round_id, value)
    evaluation = protocol.evaluate(round_id)
    episode["attempts"] += 1
    digest = hashlib.sha256(json.dumps(proposals, sort_keys=True).encode()).hexdigest()
    if digest in episode["used_decisions"]:
        raise ValueError("REPLAYED_DECISIONS")
    episode["used_decisions"].add(digest)
    if not evaluation.accepted:
        # No peer proposal or private capability feedback crosses into no-comm.
        return {"ok": False, "reason": "AGREEMENT_REJECTED", **_status(world, episode)}
    p = Proposal.from_value(proposals["r1"])
    assignment = dict(evaluation.assignments)
    carriers = (assignment["carrier_0"], assignment["carrier_1"])
    if episode["disabled_carrier"] in carriers:
        return {"ok": False, "reason": "ACTION_REJECTED", **_status(world, episode)}
    if p.cargo_id not in _status(world, episode)["remaining_ids"]:
        return {"ok": False, "reason": "ACTION_REJECTED", **_status(world, episode)}
    spec = world.warehouse_spec_by_id[p.cargo_id]
    pose = cargo_pose(world.data, world.model, spec)
    current_zone = world._warehouse_zone_for_position(p.cargo_id, pose["position"])
    if current_zone not in {"A", "B", "C"} or current_zone == episode["destination"]:
        return {"ok": False, "reason": "ACTION_REJECTED", **_status(world, episode)}
    spec = replace(spec, carriers=carriers, scout=assignment["scout"], start_xyz=tuple(pose["position"]),
                   goal_xyz=world.warehouse_zone_positions[p.cargo_id][episode["destination"]])
    world._replace_warehouse_spec(spec)
    world.warehouse_status = "RUNNING"
    world._record_warehouse_phase("peer_plan_committed", cargo_id=p.cargo_id,
                                  assignments=assignment, plan_digest=p.plan_digest,
                                  decision_source=episode["condition"])
    try:
        world._transport_warehouse_cargo(spec, warehouse_route_zones(current_zone, episode["destination"]))
        status = _status(world, episode)
        ok = p.cargo_id in status["delivered_ids"]
        world.warehouse_status = "SUCCESS" if status["complete"] else "NEGOTIATING"
        return {"ok": ok, "reason": "CARGO_DELIVERED" if ok else "REFEREE_FAILED",
                "executed_cargo_id": p.cargo_id, "assignments": assignment, **status}
    except Exception as exc:
        for robot in world.controllers.values():
            robot.set_motor_commands([0.0, 0.0, 0.0, 0.0])
        world._release_cargo_constraints(p.cargo_id)
        world.warehouse_status = "NEGOTIATING"
        world._record_warehouse_phase("peer_action_failed", cargo_id=p.cargo_id, error=str(exc))
        return {"ok": False, "reason": "PHYSICAL_ACTION_FAILED", **_status(world, episode)}
    finally:
        # Any physical attempt invalidates every pre-action sensor observation.
        episode["observations"].clear()
