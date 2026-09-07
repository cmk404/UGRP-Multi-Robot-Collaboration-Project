#!/usr/bin/env python3
"""SIM transport for the exact REAL-facing action catalog.

Task policy lives in scripts/red_block and the public action/contract schema lives
in scripts/robot_actions.py. This module only forwards the same action+arguments
to the MuJoCo hardware adapter and translates remote transport/evaluator status.
"""
from __future__ import annotations

import json
import os
import time
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from sim.bridge_client import bridge_headers
from sim.team_batch import first_command_metadata
from scripts.robot_actions import (
    ACTIONS as REAL_ACTIONS,
    ACTION_PARAMETERS as REAL_ACTION_PARAMETERS,
    CONTRACTS as REAL_CONTRACTS,
    PRIMITIVE_MOTIONS,
    classify_failure as classify_real_failure,
)

# One source of truth for what the agent is allowed to ask the robot to do.
ACTIONS = dict(REAL_ACTIONS)
ACTION_PARAMETERS = {name: dict(spec) for name, spec in REAL_ACTION_PARAMETERS.items()}
CONTRACTS = {name: dict(spec) for name, spec in REAL_CONTRACTS.items()}

# SIM team-only calibrated staging primitive. It deliberately does not exist in
# the REAL catalog: the fixed shared-world stack site is a simulator task-space
# affordance, not a claim about unmeasured real-world coordinates.
ACTIONS["stage_base"] = "SIM 협업에서 target_color 블록을 검증된 공용 스택 지점의 바닥층으로 옮겨 놓습니다."
ACTION_PARAMETERS["stage_base"] = {
    "target_color": {
        "type": "str", "required": True,
        "description": "바닥층으로 옮길 블록: red | blue | yellow",
    },
}
CONTRACTS["stage_base"] = {
    "preconditions": [],
    "expected_postconditions": ["sim.team_base_staged=true", "grasp.state=EMPTY"],
    "recoveries": {},
}

# SIM team-only atomic stacking primitive. Unlike the transferable REAL `place`
# contract, this action owns approach, bilateral-contact pick, transport and
# verified release in one shared-world operation. Tower handoffs use this action
# so the LLM does not redundantly rebuild REAL preconditions around an atomic SIM
# controller.
ACTIONS["stack_on"] = (
    "SIM 협업 전용 원자 작업입니다. target_color 블록을 직접 찾아 집고 이동해서 "
    "destination_color 블록 위에 적층·검증합니다. search/approach/pick을 먼저 실행하지 마세요."
)
ACTION_PARAMETERS["stack_on"] = {
    "target_color": {
        "type": "str", "required": True,
        "description": "위에 올릴 블록: red | blue | yellow",
    },
    "destination_color": {
        "type": "str", "required": True,
        "description": "받침 블록: red | blue | yellow",
    },
}
CONTRACTS["stack_on"] = {
    "preconditions": [],
    "expected_postconditions": ["sim.team_stack_verified=true", "grasp.state=EMPTY"],
    "recoveries": {},
}

# SIM-only three-robot joint tower primitive.  This is intentionally not part of
# the REAL catalog: it coordinates the shared MuJoCo world so all three robots
# prepare their assigned blocks concurrently, then serializes only the physical
# support dependency of the final placements.
ACTIONS["team_tower"] = (
    "SIM TEAM 전용 공동 작업입니다. R3=yellow, R1=blue, R2=red를 동시에 준비하고 "
    "yellow→blue→red 순서의 필요한 적층 단계만 직렬화합니다."
)
ACTION_PARAMETERS["team_tower"] = {}
CONTRACTS["team_tower"] = {
    "preconditions": [],
    "expected_postconditions": ["sim.team_tower_verified=true"],
    "recoveries": {},
}

ACTIONS["team_beam_transport"] = (
    "SIM TEAM 협업 미션입니다. 같은 mission_id로 carrier_left, scout, "
    "carrier_right 세 역할이 한 consensus batch에 모두 참여해야 긴 빔을 "
    "출발 구역 A에서 도착 구역 B로 물리적으로 운반합니다. 단독 호출은 거부됩니다."
)
ACTION_PARAMETERS["team_beam_transport"] = {
    "mission_id": {
        "type": "str", "required": True,
        "description": "현재 MVP mission id: beam_transport_v1",
    },
    "role": {
        "type": "str", "required": True,
        "description": "carrier_left | scout | carrier_right",
    },
}
CONTRACTS["team_beam_transport"] = {
    "preconditions": [
        "team.consensus.committed=true",
        "team.roles=carrier_left|scout|carrier_right",
        "payload.zone=START",
    ],
    "expected_postconditions": [
        "payload.zone=DESTINATION",
        "payload.stable=true",
        "payload.dual_grasp_verified=true",
    ],
    "recoveries": {},
}

ACTIONS["team_zone_transfer"] = (
    "중앙 Python 제어 비교군(central_baseline) 전용입니다. LLM 자율 협상 결과가 아닙니다. "
    "SIM source_zone의 선택 화물을 "
    "destination_zone으로 모두 옮깁니다. 세 robot ID가 같은 mission_id와 route를 "
    "한 batch에 제출해야 하며 화물마다 두 운반자가 함께 움직입니다."
)
ACTION_PARAMETERS["team_zone_transfer"] = {
    "mission_id": {"type": "str", "required": True, "description": "TEAM이 생성한 warehouse mission id"},
    "source_zone": {"type": "str", "required": True, "description": "A | B | C"},
    "destination_zone": {"type": "str", "required": True, "description": "A | B | C"},
    "selector": {"type": "str", "required": True, "description": "all | plank | pipe | crate"},
}
CONTRACTS["team_zone_transfer"] = {
    "preconditions": ["team.consensus.committed=true", "warehouse.route.valid=true"],
    "expected_postconditions": ["warehouse.remaining=0", "warehouse.destination.stable=true"],
    "recoveries": {},
}

BRIDGE = os.environ.get("UGRP_SIM_BRIDGE_LOCAL", "http://127.0.0.1:8091")
try:
    GPU_RECONNECT_GRACE = max(0.0, float(os.environ.get("UGRP_SIM_GPU_RECONNECT_GRACE", "30")))
except ValueError:
    GPU_RECONNECT_GRACE = 30.0


def _bridge_health() -> dict:
    try:
        with urlopen(BRIDGE + "/health", timeout=2.0) as response:
            value = json.loads(response.read().decode())
        return value if isinstance(value, dict) else {}
    except Exception:
        return {}


def _post_bridge_action(action: str, params: dict, *, robot_id: str | None = None) -> dict:
    # Legacy callers without UGRP_ROBOT_ID remain R1. Multi-tab workers set the
    # slot explicitly so the shared MuJoCo world can route commands/cameras.
    payload = {
        "action": action,
        "robot_id": str(robot_id or os.environ.get("UGRP_ROBOT_ID", "r1")).strip().lower(),
        **params,
        **first_command_metadata(),
    }
    req = Request(
        BRIDGE + "/command",
        data=json.dumps(payload).encode(),
        headers=bridge_headers(),
        method="POST",
    )
    with urlopen(req, timeout=480) as response:
        value = json.loads(response.read().decode())
    if not isinstance(value, dict):
        raise RuntimeError("simulation bridge returned a non-object response")
    return value


def _http_error_json(exc: HTTPError) -> dict:
    try:
        value = json.loads(exc.read().decode())
        return value if isinstance(value, dict) else {}
    except Exception:
        return {}


def _wait_for_gpu_authority(timeout: float) -> dict:
    deadline = time.monotonic() + max(0.0, timeout)
    while True:
        health = _bridge_health()
        if health.get("remote_ws_connected") and health.get("remote_authoritative"):
            return health
        if time.monotonic() >= deadline:
            return health
        time.sleep(0.25)


def _validate_params(name: str, params: dict) -> dict:
    params = dict(params)
    if name in PRIMITIVE_MOTIONS:
        unknown = sorted(set(params) - {"speed", "duration"})
        if unknown:
            raise ValueError(f"unexpected primitive args: {', '.join(unknown)}")
        if name == "stop_motion":
            return {}
        speed = int(params.get("speed", 35))
        duration = float(params.get("duration", 0.30))
        if not 31 <= speed <= 40:
            raise ValueError("primitive speed must be between 31 and 40; values <=30 do not move this chassis")
        if not 0.10 <= duration <= 0.80:
            raise ValueError("primitive duration must be between 0.10 and 0.80 seconds")
        return {"speed": speed, "duration": duration}

    if name in {"search", "track", "approach", "pick", "search_destination", "place", "stage_base", "stack_on"}:
        allowed = {"target_color"}
        if name in {"search_destination", "place", "stack_on"}:
            allowed.add("destination_color")
        unknown = sorted(set(params) - allowed)
        if unknown:
            raise ValueError(f"unexpected {name} args: {', '.join(unknown)}")
        target_color = str(params.get("target_color", "red")).strip().lower()
        if target_color not in {"red", "blue", "yellow"}:
            raise ValueError("target_color must be red, blue, or yellow")
        out = {"target_color": target_color}
        if name in {"search_destination", "place", "stack_on"}:
            destination_color = str(params.get("destination_color", "")).strip().lower()
            if destination_color not in {"red", "blue", "yellow"}:
                raise ValueError("destination_color must be red, blue, or yellow")
            if destination_color == target_color:
                raise ValueError("destination_color must differ from target_color")
            out["destination_color"] = destination_color
        return out

    if name == "team_tower":
        if params:
            raise ValueError("team_tower does not accept parameters")
        return {}

    if name == "team_beam_transport":
        unknown = sorted(set(params) - {"mission_id", "role"})
        if unknown:
            raise ValueError(f"unexpected team_beam_transport args: {', '.join(unknown)}")
        mission_id = str(params.get("mission_id") or "").strip()
        role = str(params.get("role") or "").strip().lower()
        if mission_id != "beam_transport_v1":
            raise ValueError("mission_id must be beam_transport_v1")
        if role not in {"carrier_left", "scout", "carrier_right"}:
            raise ValueError("role must be carrier_left, scout, or carrier_right")
        return {"mission_id": mission_id, "role": role}

    if name == "team_zone_transfer":
        unknown = sorted(
            set(params) - {"mission_id", "source_zone", "destination_zone", "selector"}
        )
        if unknown:
            raise ValueError(f"unexpected team_zone_transfer args: {', '.join(unknown)}")
        mission_id = str(params.get("mission_id") or "").strip()
        source = str(params.get("source_zone") or "").strip().upper()
        destination = str(params.get("destination_zone") or "").strip().upper()
        selector = str(params.get("selector") or "").strip().lower()
        if not mission_id.startswith("warehouse_"):
            raise ValueError("mission_id must be a canonical warehouse id")
        if source not in {"A", "B", "C"} or destination not in {"A", "B", "C"}:
            raise ValueError("source_zone and destination_zone must be A, B, or C")
        if source == destination:
            raise ValueError("source_zone and destination_zone must differ")
        if selector not in {"all", "plank", "pipe", "crate"}:
            raise ValueError("selector must be all, plank, pipe, or crate")
        return {
            "mission_id": mission_id,
            "source_zone": source,
            "destination_zone": destination,
            "selector": selector,
        }

    if name == "place_on_blue":
        if params:
            raise ValueError("place_on_blue does not accept parameters")
        return {"target_color": "red", "destination_color": "blue"}
    if name == "place_on_yellow":
        if params:
            raise ValueError("place_on_yellow does not accept parameters")
        return {"target_color": "red", "destination_color": "yellow"}
    if params:
        raise ValueError(f"action {name} does not accept parameters")
    return {}


def _failure(name: str, reason: str | None) -> tuple[str, str | None, str | None]:
    text = (reason or "").lower()
    if "peer_too_close" in text:
        # Shared-world referee: a peer robot occupies the space this chassis
        # action needs. The planner is told to wait and retry, never to push.
        return "PEER_TOO_CLOSE", "peer.clearance=SAFE", "wait"
    if name in {"search_destination", "place", "stack_on"} and "lost during delivery" in text:
        return "CARRY_LOST_DURING_DELIVERY", "grasp.state=HELD", None
    if name == "search_destination" and ("target not found" in text or "delivery search" in text):
        return "DESTINATION_NOT_VISIBLE", "destination.visible=true", "search_destination"
    if "sim_episode_reset" in text:
        return "SIM_EPISODE_RESET", None, None
    if "gpu_offline" in text:
        return "GPU_OFFLINE", None, None
    if "shared real pick completed but mujoco postcondition failed" in text:
        return "GRASP_NOT_ACQUIRED", "grasp.state=PROBABLE_HELD|HELD", None
    if "shared real place completed but mujoco stack postcondition failed" in text:
        return "PLACE_NOT_VERIFIED", "task.phase=PLACED", None
    return classify_real_failure(name, reason)


def _transport_failure(name: str, reason: str, *, failure_code: str | None = None) -> dict:
    code, required, recovery = _failure(name, reason)
    if failure_code is not None:
        code = failure_code
    return {
        "ok": False,
        "skill": name,
        "command_status": "UNKNOWN",
        "execution_status": "FAILED",
        "outcome_status": "NOT_ACHIEVED",
        "failure_code": code,
        "required_state": required,
        "recommended_recovery": recovery,
        "reason": reason,
    }


def run(name: str, extra=None, **params) -> dict:
    return run_for_robot(name, None, extra=extra, **params)


def run_for_robot(name: str, robot_id: str | None, extra=None, **params) -> dict:
    if name not in ACTIONS:
        raise ValueError(f"unknown action: {name}")
    if name in {"send_peer_message", "ack_peer_message"}:
        # Communication is environment-independent and must never be sent to the
        # physics bridge as if it were an actuator action.
        from scripts.robot_actions import run as run_real_catalog
        return run_real_catalog(name, extra=extra, **params)
    if extra:
        raise ValueError("SIM actions accept the same typed parameters as REAL; CLI-style extra args are unsupported")
    clean = _validate_params(name, params)
    before = _bridge_health()
    try:
        raw = _post_bridge_action(name, clean, robot_id=robot_id)
    except HTTPError as exc:
        error_obj = _http_error_json(exc)
        reason_token = str(error_obj.get("reason") or error_obj.get("error") or "")
        if exc.code == 503 and reason_token == "GPU_OFFLINE":
            recovered = _wait_for_gpu_authority(GPU_RECONNECT_GRACE)
            if not (recovered.get("remote_ws_connected") and recovered.get("remote_authoritative")):
                return _transport_failure(
                    name,
                    "GPU_OFFLINE: remote GPU did not recover within reconnect grace",
                    failure_code="GPU_OFFLINE",
                )
            before_episode = int(before.get("remote_episode_id") or 0)
            after_episode = int(recovered.get("remote_episode_id") or 0)
            before_instance = before.get("remote_instance_id") or before.get("remote_last_instance_id")
            after_instance = recovered.get("remote_instance_id")
            if (
                (before_episode and after_episode and before_episode != after_episode)
                or (before_instance and after_instance and before_instance != after_instance)
            ):
                return _transport_failure(
                    name,
                    "SIM_EPISODE_RESET: GPU worker process was replaced during recovery",
                    failure_code="SIM_EPISODE_RESET",
                )
            try:
                raw = _post_bridge_action(name, clean, robot_id=robot_id)
            except Exception as retry_exc:
                return _transport_failure(name, f"sim bridge unavailable after GPU reconnect: {retry_exc}")
        else:
            detail = reason_token or f"HTTP {exc.code}"
            return _transport_failure(name, f"sim bridge unavailable: {detail}")
    except Exception as exc:
        return _transport_failure(name, f"sim bridge unavailable: {exc}")

    ok = raw.get("ok") is not False
    reason = str(raw.get("reason") or "command completed")
    vision_by_color = {
        "red": raw.get("vision") if isinstance(raw.get("vision"), dict) else None,
        "blue": raw.get("blue_vision") if isinstance(raw.get("blue_vision"), dict) else None,
        "yellow": raw.get("yellow_vision") if isinstance(raw.get("yellow_vision"), dict) else None,
    }
    target_color = clean.get("target_color")
    destination_color = clean.get("destination_color")
    verified_outcome = ok and name in {
        "search", "track", "approach", "search_destination", "put_down", "place",
        "place_on_blue", "place_on_yellow", "stage_base", "stack_on",
        "team_beam_transport",
        "team_zone_transfer",
    }
    out = {
        "ok": ok,
        "skill": name,
        "command_status": "ACCEPTED",
        "execution_status": "COMPLETED" if ok else "FAILED",
        "outcome_status": "ACHIEVED" if verified_outcome else ("UNKNOWN" if ok else "NOT_ACHIEVED"),
        "reason": reason,
    }
    if target_color is not None:
        out["target_color"] = target_color
    if destination_color is not None:
        out["destination_color"] = destination_color
    if name == "team_beam_transport":
        out["mission_id"] = clean["mission_id"]
        out["role"] = clean["role"]
        if verified_outcome:
            out["mission_verified"] = True
            out["verification_source"] = "opposed_arm_yaw+direct_body_contact+shared_physics+destination_referee"
    if name == "team_zone_transfer":
        out.update({
            "decision_mode": "central_baseline",
            "decision_authority": "omniscient_python_controller",
            "mission_id": clean["mission_id"],
            "source_zone": clean["source_zone"],
            "destination_zone": clean["destination_zone"],
            "selector": clean["selector"],
        })
        if verified_outcome:
            out["mission_verified"] = True
            out["verification_source"] = "warehouse_manifest+all_cargo_destination_referee"
    if name in PRIMITIVE_MOTIONS and ok:
        out["motion_executed"] = True
        out["motion"] = PRIMITIVE_MOTIONS[name]
        out["chassis_motion"] = name != "stop_motion"
        out["verification_source"] = "bounded_driver_completion_needs_post_action_vision"
    elif name == "search" and verified_outcome:
        out["target_vision"] = {"visible": True}
        if target_color == "red":
            out["camera_red"] = {"visible": True}
        out["verification_source"] = f"pi_multiframe_{target_color}_search"
    elif name == "track" and verified_outcome:
        out["target_vision"] = {"visible": True}
        if target_color == "red":
            out["camera_red"] = {"visible": True}
        out["center_verified"] = True
        out["verification_source"] = (
            "pi_multiframe_centered_track"
            if target_color == "red" else f"pi_multiframe_centered_{target_color}_track"
        )
    elif name == "approach" and verified_outcome:
        out["target_vision"] = {"visible": True}
        if target_color == "red":
            out["camera_red"] = {"visible": True}
        out["range_verified"] = True
        out["center_verified"] = True
        # Same label as scripts/robot_actions.py: SIM runs the REAL approach
        # controller through the adapter, so the evidence source must not differ.
        out["verification_source"] = (
            "camera_metric_arm_reach_controller"
            if target_color == "red" else f"camera_metric_{target_color}_arm_reach_controller"
        )
    elif name == "pick" and ok:
        # Hidden MuJoCo truth may reject a false REAL visual success, but it must
        # never give the actor stronger evidence than the physical robot has.
        out["outcome_status"] = "UNKNOWN"
        out["visual_hold"] = {
            "probable": True,
            "confidence": 0.65,
            "source": "whole_view_floor_clear_without_positive_gripper_sensor",
        }
        out["verification_source"] = "camera_floor_clear_probable_only"
    elif name == "search_destination" and verified_outcome:
        out["destination_visible"] = True
        out["verification_source"] = f"camera_delivery_safe_{destination_color}_search"
    elif name == "put_down" and verified_outcome:
        out["release_verified"] = True
        out["verification_source"] = "causal_pick_return_path_plus_open_gripper"
    elif name in {"place", "place_on_blue", "place_on_yellow", "stack_on"} and verified_outcome:
        out["place_verified"] = True
        out["verification_source"] = f"camera_{target_color}_over_{destination_color}_stack_geometry"
    elif name == "stage_base" and verified_outcome:
        # The base of the tower is a verified release too; without this the PLACE
        # goal stays UNKNOWN and the planner burns its whole budget on refused
        # finals instead of handing the next step to a peer.
        out["place_verified"] = True
        out["verification_source"] = f"{target_color}_released_at_shared_stack_site_postcondition"
    elif name == "observe_scene" and ok:
        out["verification_source"] = "robot_camera_rgb+commanded_fk+base_odometry"

    if not ok:
        failure_code, required, recovery = _failure(name, reason)
        out.update({
            "failure_code": failure_code,
            "required_state": required,
            "recommended_recovery": recovery,
        })

    # The detailed per-color detections remain in the bridge trace. A fresh
    # shared camera-only observation populates planner state after every action.
    # raw spatial_memory may contain simulator/base-odometry geometry that the
    # physical robot cannot expose with the same certainty. Keep it inside the
    # bridge/trace for evaluation, never in the actor-facing result/state.
    return out


def main(argv=None):
    import sys
    args = list(sys.argv[1:] if argv is None else argv)
    if not args or args[0] in {"-h", "--help"}:
        for key, value in ACTIONS.items():
            print(f"{key:14} {value}")
        return 0
    print(json.dumps(run(args[0]), ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
