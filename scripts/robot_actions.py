#!/usr/bin/env python3
"""실행 가능한 로봇 행동.

채팅 창이 호출할 수 있는 Python 도구는 이 파일의 ACTIONS 가 전부입니다.
파일을 통째로 바꾸거나 `--actions 다른파일.py` 로 바꿔 끼우면 됩니다.

기본 구현은 scripts/red_block/<이름>.py 를 그대로 호출합니다.
다른 로봇/다른 일을 쓰려면 ACTIONS 와 run() 만 맞추면 됩니다.

    python3 scripts/robot_actions.py approach
    python3 -m harness --chat --actions scripts/robot_actions.py
"""

from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
import threading
from pathlib import Path




_CURRENT_PROCESS_LOCK = threading.Lock()
_CURRENT_PROCESS: subprocess.Popen | None = None
CANCEL_GRACE_SECONDS = 1.5


def cancel_current() -> bool:
    """Terminate only the currently running action process group.

    The fresh skill Python process, its SSH client, and helper children inherit
    one local process group. The remote watchdog handles SSH hangup/termination
    by stopping its own skill process group, so cancelling a web turn cannot
    leave the MasterPi action running independently.
    """
    with _CURRENT_PROCESS_LOCK:
        proc = _CURRENT_PROCESS
    if proc is None or proc.poll() is not None:
        return False
    try:
        os.killpg(proc.pid, signal.SIGTERM)
    except ProcessLookupError:
        return False
    try:
        proc.wait(timeout=CANCEL_GRACE_SECONDS)
    except subprocess.TimeoutExpired:
        # This escalation is local to the action's private session/process group;
        # it cannot target another controller or an unrelated robot process.
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        try:
            proc.wait(timeout=0.5)
        except subprocess.TimeoutExpired:
            pass
    return True


ACTIONS = {
    "move_forward": "차체를 짧게 앞으로 이동합니다.",
    "move_backward": "차체를 짧게 뒤로 이동합니다.",
    "move_left": "수동/진단용 왼쪽 횡이동입니다. REAL에서는 yaw/전후 누설이 커 정밀 위치제어에는 쓰지 않습니다.",
    "move_right": "수동/진단용 오른쪽 횡이동입니다. REAL에서는 yaw/전후 누설이 커 정밀 위치제어에는 쓰지 않습니다.",
    "turn_left": "차체를 제자리에서 짧게 왼쪽으로 회전합니다.",
    "turn_right": "차체를 제자리에서 짧게 오른쪽으로 회전합니다.",
    "stop_motion": "현재 차체 모터를 정지시킵니다. 다른 controller가 actuator lease를 소유 중이면 간섭하지 않고 실패합니다.",
    "observe_scene": "실제 카메라와 마지막 명령 서보 자세만 읽어 색 물체와 가능한 metric 위치를 갱신합니다. 모터/서보는 움직이지 않습니다.",
    "search": "target_color로 지정한 블록을 바닥 우선·제한된 카메라 탐색으로 찾습니다.",
    "track": "target_color로 지정한 블록을 팔로 추적해 화면 중앙을 연속 확인합니다.",
    "approach": "target_color 블록을 약 26cm coarse arm-reach staging까지만 정지-재측정 방식으로 접근시킵니다. face 방향·최종 depth·IK/grasp는 pick이 담당합니다.",
    "pick": "approach의 fresh coarse handoff를 받아 근접 base creep, face/servo-6 정렬, 최종 capture depth, fresh IK와 grasp를 한 번에 수행합니다. approach의 최종 FK/IK 계획을 재사용하지 않습니다.",
    "put_down": "현재 들고 있는 블록을 precision pick의 원래 집기 위치로 되돌려 안전하게 내려놓고 집게를 엽니다.",
    "search_destination": "블록을 든 상태에서 집게/팔의 운반 자세를 보존하며 destination_color 블록을 안전하게 탐색합니다.",
    "place": "현재 들고 있는 target_color 블록을 destination_color 블록 위에 배치하고 시각적으로 적층을 검증합니다.",
    # Deprecated compatibility aliases. New planners must use ``place`` with
    # destination_color; aliases remain for saved plans and operators.
    "place_on_blue": "호환 alias: 빨간 블록을 파란 블록 위에 배치합니다.",
    "place_on_yellow": "호환 alias: 빨간 블록을 노란 블록 위에 배치합니다.",
    # Peer communication is a neutral message bus, not a central planner. Each
    # robot decides when and what to propose to its peers.
    "send_peer_message": "다른 로봇에게 관찰/의도/행동 제안을 자연어로 보냅니다. recipient은 r1|r2|r3|all 입니다.",
    "ack_peer_message": "받은 peer 메시지에 수락/거절 ack를 보냅니다.",
}

# Parameters shown to the LLM. The adapter and Pi driver enforce the same
# bounds again; prompt/schema validation is not the safety boundary.
ACTION_PARAMETERS = {
    name: {
        "speed": {"type": "int", "default": 35, "description": "31..40. 이 MasterPi는 30 이하에서 바퀴가 움직이지 않으므로 보통 35를 사용"},
        "duration": {"type": "float", "default": 0.30, "description": "0.10..0.80초"},
    }
    for name in (
        "move_forward", "move_backward",
        "turn_left", "turn_right"
    )
}
for _name in ("move_left", "move_right"):
    ACTION_PARAMETERS[_name] = {
        "speed": {"type": "int", "default": 65, "description": "31..70. 횡이동은 확실한 이동을 위해 기본 65; 70까지 허용"},
        "duration": {"type": "float", "default": 0.65, "description": "0.10..0.80초. 실제 횡이동은 최소 0.65초 펄스로 보정"},
    }
for _name in ("search", "track", "approach", "pick"):
    ACTION_PARAMETERS[_name] = {
        "target_color": {
            "type": "str", "default": "red",
            "description": "red | blue | yellow. 생략하면 기존 빨간 블록 동작과 동일",
        },
    }
ACTION_PARAMETERS["search_destination"] = {
    "target_color": {"type": "str", "default": "red", "description": "현재 들고 있는 블록: red | blue | yellow"},
    "destination_color": {"type": "str", "required": True, "description": "운반 자세를 유지한 채 찾을 목적 블록: red | blue | yellow"},
}
ACTION_PARAMETERS["place"] = {
    "target_color": {"type": "str", "default": "red", "description": "현재 들고 있는 블록: red | blue | yellow"},
    "destination_color": {"type": "str", "required": True, "description": "올려놓을 다른 블록: red | blue | yellow"},
}
ACTION_PARAMETERS["send_peer_message"] = {
    "recipient": {"type": "str", "required": True, "description": "r1 | r2 | r3 | all"},
    "message": {"type": "str", "required": True, "description": "상대에게 보낼 관찰/의도/요청"},
    "proposed_action": {"type": "str", "default": "", "description": "선택: 제안하는 다음 행동"},
    "required_partner": {"type": "str", "default": "", "description": "선택: 협력이 필요한 로봇 r1|r2|r3|all"},
}
ACTION_PARAMETERS["ack_peer_message"] = {
    "message_id": {"type": "str", "required": True, "description": "ack할 메시지 id"},
    "accepted": {"type": "bool", "default": True, "description": "제안을 수락하면 true"},
    "message": {"type": "str", "default": "", "description": "선택: ack와 함께 보낼 짧은 설명"},
}

CONTRACTS = {
    "move_forward": {"preconditions": [], "expected_postconditions": ["motion.command_completed=true"], "recoveries": {}},
    "move_backward": {"preconditions": [], "expected_postconditions": ["motion.command_completed=true"], "recoveries": {}},
    "move_left": {"preconditions": [], "expected_postconditions": ["motion.command_completed=true"], "recoveries": {}},
    "move_right": {"preconditions": [], "expected_postconditions": ["motion.command_completed=true"], "recoveries": {}},
    "turn_left": {"preconditions": [], "expected_postconditions": ["motion.command_completed=true"], "recoveries": {}},
    "turn_right": {"preconditions": [], "expected_postconditions": ["motion.command_completed=true"], "recoveries": {}},
    "stop_motion": {"preconditions": [], "expected_postconditions": ["motion.stopped=true"], "recoveries": {}},
    "observe_scene": {
        "preconditions": [],
        "expected_postconditions": ["spatial_memory.updated=true"],
        "recoveries": {},
    },
    "search": {
        "preconditions": [],
        "expected_postconditions": ["target.visible=true"],
        "recoveries": {},
    },
    "track": {
        "preconditions": ["target.visible=true"],
        "expected_postconditions": ["target.centered=true"],
        "recoveries": {"TARGET_NOT_VISIBLE": "search"},
    },
    "approach": {
        "preconditions": ["target.visible=true"],
        "expected_postconditions": ["target.range_class=PREGRASP"],
        "recoveries": {"TARGET_NOT_VISIBLE": "search"},
    },
    "pick": {
        "preconditions": ["task.phase=PREGRASP_READY", "target.visible=true", "target.centered=true", "target.range_class=PREGRASP"],
        "expected_postconditions": ["grasp.state=PROBABLE_HELD|HELD"],
        "recoveries": {
            "TARGET_NOT_VISIBLE": "search",
            "TARGET_NOT_CENTERED": "track",
            "TARGET_TOO_FAR": "approach",
        },
    },
    "put_down": {
        "preconditions": ["grasp.state=PROBABLE_HELD|HELD"],
        "expected_postconditions": ["grasp.state=EMPTY"],
        "recoveries": {},
    },
    "search_destination": {
        "preconditions": ["grasp.state=PROBABLE_HELD|HELD", "grasp.held_object_color=target_color"],
        "expected_postconditions": ["destination.visible=true"],
        "recoveries": {},
    },
    "place": {
        "preconditions": ["grasp.state=PROBABLE_HELD|HELD", "grasp.held_object_color=target_color"],
        "expected_postconditions": ["task.phase=PLACED", "grasp.state=EMPTY"],
        "recoveries": {},
    },
    "place_on_blue": {
        "preconditions": ["grasp.state=PROBABLE_HELD|HELD"],
        "expected_postconditions": ["task.phase=PLACED", "grasp.state=EMPTY"],
        "recoveries": {},
    },
    "place_on_yellow": {
        "preconditions": ["grasp.state=PROBABLE_HELD|HELD"],
        "expected_postconditions": ["task.phase=PLACED", "grasp.state=EMPTY"],
        "recoveries": {},
    },
    "send_peer_message": {"preconditions": [], "expected_postconditions": ["peer.message.sent=true"], "recoveries": {}},
    "ack_peer_message": {"preconditions": [], "expected_postconditions": ["peer.message.acknowledged=true"], "recoveries": {}},
}

AGENT_DEFAULT_ARGS = {
    "search": ["--seconds", "30.0"],
    "pick": ["--preserve-gaze"],
    "track": ["--seconds", "3.0"],
    "place_on_blue": ["--target-color", "red", "--destination-color", "blue"],
    "place_on_yellow": ["--target-color", "red", "--destination-color", "yellow"],
}

PRIMITIVE_MOTIONS = {
    "move_forward": "forward",
    "move_backward": "backward",
    "move_left": "left",
    "move_right": "right",
    "turn_left": "rotate-left",
    "turn_right": "rotate-right",
    "stop_motion": "stop",
}

SCRIPT_FOR = {
    "move_forward": "primitive",
    "move_backward": "primitive",
    "move_left": "primitive",
    "move_right": "primitive",
    "turn_left": "primitive",
    "turn_right": "primitive",
    "stop_motion": "primitive",
    "search": "search",
    "track": "track",
    "approach": "approach",
    "pick": "pick",
    "put_down": "put_down",
    "search_destination": "place",
    "place": "place",
    "place_on_blue": "place",
    "place_on_yellow": "place",
}


ROOT = Path(__file__).resolve().parent
PROJECT_ROOT = ROOT.parent
SKILL_PYTHON = PROJECT_ROOT / ".venv-sim" / "bin" / "python"
_ROBOT_ID = os.environ.get("UGRP_ROBOT_ID", "r1").strip().lower() or "r1"
_TRACE_SUFFIX = "" if "UGRP_ROBOT_ID" not in os.environ else f"-{_ROBOT_ID}"
LAST_REMOTE_ERROR_PATH = Path(f"/tmp/ugrp{_TRACE_SUFFIX}-last-remote-error.txt")
LAST_REAL_TRACE_PATH = Path(f"/tmp/ugrp{_TRACE_SUFFIX}-last-real-trace.json")


def _run_skill_process(script_name: str, script_args: list[str]) -> int:
    """Launch one isolated local skill process with the normal trace/cancel boundary."""
    script = ROOT / "red_block" / f"{script_name}.py"
    if not script.is_file():
        print(f"error: missing {script}", file=sys.stderr)
        return 2
    skill_python = SKILL_PYTHON if SKILL_PYTHON.is_file() else Path(sys.executable)
    argv = [str(skill_python), str(script), *script_args]
    child_env = os.environ.copy()
    try:
        from harness.execution_trace import current_run_id
        active_run_id = current_run_id()
    except Exception:
        active_run_id = None
    if active_run_id:
        child_env["UGRP_REAL_RUN_ID"] = active_run_id
        child_env["UGRP_REAL_TRACE_ENABLE"] = "1"
        child_env.setdefault("UGRP_REAL_TRACE_ALL_FRAMES", "1")
        child_env.setdefault("UGRP_REAL_TRACE_JPEG_QUALITY", "55")
    global _CURRENT_PROCESS
    with _CURRENT_PROCESS_LOCK:
        active = _CURRENT_PROCESS
        if active is not None and active.poll() is None:
            print("error: another robot action is already running", file=sys.stderr)
            return 3
        proc = subprocess.Popen(argv, start_new_session=True, env=child_env)
        _CURRENT_PROCESS = proc
    try:
        return int(proc.wait())
    finally:
        with _CURRENT_PROCESS_LOCK:
            if _CURRENT_PROCESS is proc:
                _CURRENT_PROCESS = None


def _trace_fields() -> dict:
    try:
        trace_meta = json.loads(LAST_REAL_TRACE_PATH.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}
    if not isinstance(trace_meta, dict):
        return {}
    out = {}
    for source, dest in (
        ("trace_id", "real_trace_id"),
        ("trace_dir", "real_trace_dir"),
        ("run_id", "real_run_id"),
        ("run_dir", "real_run_dir"),
    ):
        if trace_meta.get(source):
            out[dest] = trace_meta[source]
    return out


def _successful_program_stage(stage: str, target_color: str, trace: dict) -> dict:
    out = {
        "ok": True,
        "skill": stage,
        "command_status": "ACCEPTED",
        "execution_status": "COMPLETED",
        "outcome_status": "UNKNOWN" if stage == "pick" else "ACHIEVED",
        "exit_code": 0,
        "target_color": target_color,
        **trace,
    }
    if stage == "search":
        out["target_vision"] = {"visible": True}
        if target_color == "red": out["camera_red"] = {"visible": True}
        out["verification_source"] = f"pi_multiframe_{target_color}_search_single_process"
    elif stage == "track":
        out["target_vision"] = {"visible": True}
        if target_color == "red": out["camera_red"] = {"visible": True}
        out["center_verified"] = True
        out["verification_source"] = f"pi_multiframe_centered_{target_color}_track_single_process"
    elif stage == "approach":
        out["target_vision"] = {"visible": True}
        if target_color == "red": out["camera_red"] = {"visible": True}
        out["center_verified"] = True
        out["range_verified"] = True
        out["verification_source"] = f"camera_tight_{target_color}_pregrasp_single_process"
    elif stage == "pick":
        out["visual_hold"] = {
            "probable": True,
            "confidence": 0.65,
            "source": "whole_view_floor_clear_without_positive_gripper_sensor",
        }
        out["verification_source"] = "single_process_camera_floor_clear_probable_only"
    return out


def run_program(program: list[tuple[str, dict]]) -> list[dict]:
    """Hidden fastpath for a contiguous acquisition suffix ending in pick.

    The executive may skip already-proven search/track stages, but the executor
    still pays only one Python/SSH/camera startup boundary. ``approach`` remains
    mandatory because it establishes the coarse stopped arm-reach handoff; pick
    performs the final measurement and IK itself.
    """
    expected = ("search", "track", "approach", "pick")
    requested = tuple(name for name, _ in program)
    valid = tuple(expected[i:] for i in range(len(expected) - 1))
    if requested not in valid:
        raise ValueError("run_program accepts search/track/approach/pick or a contiguous suffix ending in pick")
    colors = []
    for name, args in program:
        if not isinstance(args, dict):
            raise ValueError(f"{name} args must be a dict")
        unknown = set(args) - {"target_color"}
        if unknown:
            raise ValueError(f"unexpected {name} args: {', '.join(sorted(unknown))}")
        color = str(args.get("target_color", "red")).strip().lower()
        if color not in {"red", "blue", "yellow"}:
            raise ValueError("target_color must be red, blue, or yellow")
        colors.append(color)
    if len(set(colors)) != 1:
        raise ValueError("all fused acquisition stages must use the same target_color")
    target_color = colors[0]

    for path in (LAST_REMOTE_ERROR_PATH, LAST_REAL_TRACE_PATH):
        try:
            path.unlink()
        except FileNotFoundError:
            pass
    code = _run_skill_process(
        "task_runner",
        [
            "--target-color", target_color,
            "--search-seconds", "30.0",
            "--track-seconds", "3.0",
            "--start-stage", requested[0],
        ],
    )
    trace = _trace_fields()
    reason = None
    if code != 0:
        try:
            reason = LAST_REMOTE_ERROR_PATH.read_text(encoding="utf-8").strip() or None
        except OSError:
            reason = None
    failed_stage = None
    detail = reason or f"single-process task exited {code}"
    if code != 0 and reason:
        lowered = reason.lower()
        for stage in requested:
            marker = f"task stage={stage}:"
            if marker in lowered:
                failed_stage = stage
                # Keep the original case/content after the marker when possible.
                idx = lowered.index(marker) + len(marker)
                detail = reason[idx:].strip() or reason
                break
    if code != 0 and failed_stage is None:
        failed_stage = requested[0]

    results = []
    for stage in requested:
        if code == 0 or stage != failed_stage:
            if code != 0 and requested.index(stage) > requested.index(failed_stage):
                break
            results.append(_successful_program_stage(stage, target_color, trace))
            continue
        failure_code, required_state, recovery = classify_failure(stage, detail)
        results.append({
            "ok": False,
            "skill": stage,
            "command_status": "ACCEPTED",
            "execution_status": "FAILED",
            "outcome_status": "NOT_ACHIEVED",
            "exit_code": code,
            "target_color": target_color,
            "failure_code": failure_code,
            "required_state": required_state,
            "recommended_recovery": recovery,
            "reason": detail,
            **trace,
        })
        break
    return results


def classify_failure(name: str, reason: str | None) -> tuple[str, str | None, str | None]:
    text = (reason or "").lower()
    if any(x in text for x in (
        "cannot reach ugrp1", "deployment sync timed out", "could not sync verified",
        "verified package deployment failed", "ssh transport exceeded",
        "connection timed out", "connection refused", "no route to host", "host is down",
    )):
        return "ROBOT_UNAVAILABLE", "robot.connection=READY", None
    if "remote skill safety timeout" in text:
        return "SKILL_TIMEOUT", "skill.progress=BOUNDED", None
    if any(x in text for x in (
        "actuator lease", "legacy raw-i2c controller is already active",
        "concurrent actuation", "controller is already active", "masterpi actuators are busy",
    )):
        return "CONTROLLER_BUSY", "actuator.owner=AVAILABLE", None
    if any(x in text for x in (
        "write timed out after", "failed to stop all motors", "i2c write",
        "motor transport", "servo transport",
    )):
        return "ACTUATOR_IO_FAILED", "actuator.io=READY", None
    if any(x in text for x in (
        "camera unavailable", "could not open camera", "camera stream", "snapshot failed",
        "failed to capture", "video capture", "mjpeg", "http stream ended",
    )):
        return "CAMERA_UNAVAILABLE", "camera.ready=true", None
    if name in {"put_down", "search_destination", "place", "place_on_blue", "place_on_yellow"} and "carry handoff" in text:
        return "CARRY_STATE_UNVERIFIED", "grasp.causal_handoff=fresh", None
    if name in {"search_destination", "place"} and "lost during delivery" in text:
        return "CARRY_LOST_DURING_DELIVERY", "grasp.state=HELD", None
    if name == "search_destination" and ("target not found" in text or "delivery search" in text):
        return "DESTINATION_NOT_VISIBLE", "destination.visible=true", "search_destination"
    if name == "approach" and "requires search/track camera geometry" in text:
        return "APPROACH_POSE_INVALID", "arm.pose=SEARCH_TRACK", "search"
    target_lost = (
        "target disappeared", "target not visible", "block not visible",
        "block not found", "locked block not found", "lost the block",
        "lost target", "target lost", "track handoff lost", "fixed-pose capture lost",
        "could not hold the red block",
    )
    if name in {"track", "approach", "pick"} and any(x in text for x in target_lost):
        return "TARGET_NOT_VISIBLE", "target.visible=true", "search"
    if name == "approach" and "capture target x drifted" in text:
        return "TARGET_NOT_CENTERED", "target.centered=true", "track"
    if name == "approach" and any(x in text for x in (
        "capture creep made insufficient visual progress",
        "pre-capture creep made insufficient visual progress",
        "could not bring red block into calibrated visual capture window",
        "face-route chassis turn exhausted bounded bearing pulses",
        "face-route straight translation exhausted bounded pulses",
        "face-route passed safe radial envelope",
        "face-route ended ",
        "face-route improved face error by only",
    )):
        # These failures can leave the arm/gaze outside the inherited
        # SEARCH_TRACK geometry. Search is the safe causal reset before another
        # metric approach; retrying track->approach can otherwise hit an invalid
        # capture-family pose with no fresh handoff.
        return "APPROACH_PROGRESS_STALLED", "arm.pose=SEARCH_TRACK", "search"
    if name == "approach" and any(x in text for x in (
        "pose telemetry remains stale after exact-pose reassertion",
        "pose telemetry is stale and this controller cannot reassert it",
    )):
        return "APPROACH_POSE_STALE", "arm.pose_trust=FRESH", None
    if name == "approach" and any(x in text for x in (
        "could not retreat capture target into the requested hand-eye window",
        "capture retreat made insufficient visual progress",
    )):
        return "TARGET_TOO_CLOSE", "target.capture_depth=SAFE", None
    if name == "pick":
        # ``pick`` consumes a short-lived coarse-approach handoff.  Keep the
        # historical v1 wording classified as PREGRASP_PLAN_MISSING too so
        # saved callers/tests and mixed-version workers recover through the
        # same causal action: run approach again, then pick immediately.
        handoff_invalid = (
            "precision pick plan is missing",
            "precision pick plan is stale",
            "coarse approach handoff is unreadable",
            "coarse approach handoff has an unsupported",
            "coarse approach handoff has no valid",
            "coarse approach handoff did not verify target visibility",
            "coarse approach handoff target is",
            "coarse approach handoff has no measurement pose",
            "arm/camera pose changed or is unknown since coarse approach",
            "arm/camera moved after coarse approach",
        )
        if any(x in text for x in handoff_invalid):
            return "PREGRASP_PLAN_MISSING", "task.phase=PREGRASP_READY", "approach"

        grasp_miss = (
            "red block is still visible after the grasp",
            "blue block is still visible on the floor after the arm-only grasp",
            "yellow block is still visible on the floor after the arm-only grasp",
            "red block is still visible on the floor after the arm-only grasp",
            "red block remains at pickup site",
            "grasp postcondition failed",
            "miss_red_still_visible",
        )
        if any(x in text for x in grasp_miss):
            return "GRASP_NOT_ACQUIRED", "grasp.state=PROBABLE_HELD|HELD", None
        if any(x in text for x in (
            "outside reachable block envelope",
            "lost calibrated reach camera pose",
            "outside calibrated visual reach window",
            "visual coordinate is unstable",
            "valid arm-only pick visual samples",
            "face error",
            "run approach to re-enter reach",
            "run approach to restage",
            "run approach to re-aim chassis",
        )):
            return "TARGET_TOO_FAR", "target.range_class=PREGRASP", "approach"
    return "SKILL_FAILED", None, None


def run(name: str, extra: list[str] | None = None, **params) -> dict:
    if name not in ACTIONS:
        raise ValueError(f"unknown action: {name}")
    if name in {"send_peer_message", "ack_peer_message"}:
        if extra:
            raise ValueError("peer tools do not accept CLI-style extra args")
        from harness import team_bus
        sender = os.environ.get("UGRP_ROBOT_ID", "r1")
        if name == "send_peer_message":
            item = team_bus.send_message(
                sender,
                str(params.get("recipient") or ""),
                str(params.get("message") or ""),
                proposed_action=str(params.get("proposed_action") or ""),
                required_partner=str(params.get("required_partner") or ""),
                namespace=os.environ.get("UGRP_TEAM_NAMESPACE") or os.environ.get("UGRP_UI_MODE"),
            )
            return {
                "ok": True, "skill": name, "command_status": "ACCEPTED",
                "execution_status": "COMPLETED", "outcome_status": "ACHIEVED",
                "peer_message": item, "reason": f"peer message {item['id']} sent",
            }
        item = team_bus.acknowledge(
            sender,
            str(params.get("message_id") or ""),
            bool(params.get("accepted", True)),
            str(params.get("message") or ""),
            namespace=os.environ.get("UGRP_TEAM_NAMESPACE") or os.environ.get("UGRP_UI_MODE"),
        )
        return {
            "ok": True, "skill": name, "command_status": "ACCEPTED",
            "execution_status": "COMPLETED", "outcome_status": "ACHIEVED",
            "peer_message": item, "reason": f"peer message {item['id']} acknowledged",
        }
    extra_args = list(extra or [])
    if name in PRIMITIVE_MOTIONS:
        unknown = sorted(set(params) - {"speed", "duration"})
        if unknown:
            raise ValueError(f"unexpected primitive args: {', '.join(unknown)}")
        if name != "stop_motion":
            lateral = name in {"move_left", "move_right"}
            speed = int(params.get("speed", 65 if lateral else 35))
            duration = float(params.get("duration", 0.65 if lateral else 0.30))
            max_speed = 70 if lateral else 40
            if not 35 <= speed <= max_speed:
                raise ValueError(f"primitive speed must be between 35 and {max_speed}; non-zero wheel commands below 35 are forbidden")
            if not 0.10 <= duration <= 0.80:
                raise ValueError("primitive duration must be between 0.10 and 0.80 seconds")
            extra_args.extend(["--speed", str(speed), "--duration", str(duration)])
    elif name in {"search", "track", "approach", "pick", "search_destination", "place"}:
        allowed = {"target_color"}
        if name in {"search_destination", "place"}:
            allowed.add("destination_color")
        unknown = sorted(set(params) - allowed)
        if unknown:
            raise ValueError(f"unexpected {name} args: {', '.join(unknown)}")
        target_color = str(params.get("target_color", "red")).strip().lower()
        if target_color not in {"red", "blue", "yellow"}:
            raise ValueError("target_color must be red, blue, or yellow")
        extra_args.extend(["--target-color", target_color])
        if name in {"search_destination", "place"}:
            destination_color = str(params.get("destination_color", "")).strip().lower()
            if destination_color not in {"red", "blue", "yellow"}:
                raise ValueError("destination_color must be red, blue, or yellow")
            if destination_color == target_color:
                raise ValueError("destination_color must differ from target_color")
            extra_args.extend(["--destination-color", destination_color])
    elif params:
        raise ValueError(f"action {name} does not accept parameters")
    argv = [name, *AGENT_DEFAULT_ARGS.get(name, []), *extra_args]
    if name == "observe_scene":
        from scripts.real_observe_scene import run as observe_run
        host = os.environ.get("UGRP_ROBOT_HOST", "ugrp1")
        if extra:
            for i, token in enumerate(extra):
                if token == "--host" and i + 1 < len(extra):
                    host = extra[i + 1]
        out = observe_run(host)
        out["argv"] = argv
        return out
    try:
        LAST_REMOTE_ERROR_PATH.unlink()
    except FileNotFoundError:
        pass
    try:
        LAST_REAL_TRACE_PATH.unlink()
    except FileNotFoundError:
        pass
    code = main(argv)
    ok = code == 0
    dry_run = "--dry-run" in argv
    detect_only = "--detect-only" in argv
    # Only live skills whose driver exits 0 *after* an explicit camera
    # postcondition may claim ACHIEVED. search.py returns 0 only after a
    # bounded multi-frame red-target confirmation; approach.py returns 0 only
    # after its live camera says the red block is in the grasp zone; pick/place
    # have their own stronger postcondition checks. Dry-run/detect-only modes
    # do not execute the physical postcondition and must remain UNKNOWN.
    verified_outcome = (
        ok
        and not dry_run
        and not detect_only
        and name in {"search", "track", "approach", "search_destination", "put_down", "place", "place_on_blue", "place_on_yellow"}
    )
    out = {
        "ok": ok,  # legacy compatibility only
        "skill": name,
        "command_status": "ACCEPTED",
        "execution_status": "COMPLETED" if ok else "FAILED",
        "outcome_status": "ACHIEVED" if verified_outcome else ("UNKNOWN" if ok else "NOT_ACHIEVED"),
        "exit_code": code,
        "argv": argv,
    }
    target_color = _argv_color(argv, "--target-color") or "red"
    destination_color = _argv_color(argv, "--destination-color")
    if name in {"search", "track", "approach", "pick", "search_destination", "place", "place_on_blue", "place_on_yellow"}:
        out["target_color"] = target_color
    if destination_color is not None:
        out["destination_color"] = destination_color
    out.update(_trace_fields())
    if name in PRIMITIVE_MOTIONS and ok and not dry_run:
        out["motion_executed"] = True
        out["motion"] = PRIMITIVE_MOTIONS[name]
        out["chassis_motion"] = name != "stop_motion"
        out["verification_source"] = "bounded_driver_completion_needs_post_action_vision"
    if name == "search" and verified_outcome:
        # search.py has already confirmed the same bounded external candidate
        # across multiple fresh Pi frames. Do not fabricate coordinates here;
        # track will reacquire its exact image position on the next skill.
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
        out["verification_source"] = (
            "camera_metric_arm_reach_controller"
            if target_color == "red" else f"camera_metric_{target_color}_arm_reach_controller"
        )
    elif name == "pick" and ok and not dry_run and not detect_only:
        # The eye-in-hand camera can prove a miss when a red cube remains, but
        # disappearance cannot prove the cube is inside the gripper.  Never
        # manufacture HELD from an exit code or pickup-site absence.
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
    elif name in {"place", "place_on_blue", "place_on_yellow"} and verified_outcome:
        out["place_verified"] = True
        out["verification_source"] = f"camera_{target_color}_over_{destination_color}_stack_geometry"
    if not ok:
        reason = None
        try:
            reason = LAST_REMOTE_ERROR_PATH.read_text(encoding="utf-8").strip() or None
        except OSError:
            pass
        failure_code, required_state, recommended_recovery = classify_failure(name, reason)
        out.update({
            "failure_code": failure_code,
            "required_state": required_state,
            "recommended_recovery": recommended_recovery,
        })
        if reason:
            out["reason"] = reason
    return out


def main(argv: list[str] | None = None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    if not args or args[0] in {"-h", "--help"}:
        print("사용법: python3 scripts/robot_actions.py <action> [옵션]")
        print()
        for name, help_text in ACTIONS.items():
            print(f"  {name:<10} {help_text}")
        print()
        print("예: python3 scripts/robot_actions.py approach")
        print("    python3 scripts/robot_actions.py pick")
        print("에이전트 호출은 track에 --seconds 3.0을 자동으로 붙입니다.")
        print("이 파일을 바꿔 끼우면 채팅 창의 실행 도구가 바뀝니다.")
        return 0
    skill = args[0]
    if skill not in ACTIONS:
        print(
            f"error: unknown action {skill!r}. choose from: {', '.join(ACTIONS)}",
            file=sys.stderr,
        )
        return 2
    if skill == "observe_scene":
        from scripts.real_observe_scene import main as observe_main
        old_argv = sys.argv
        try:
            sys.argv = [str(ROOT / "real_observe_scene.py"), *args[1:]]
            return observe_main()
        finally:
            sys.argv = old_argv
    script_name = SCRIPT_FOR.get(skill, skill)
    script = ROOT / "red_block" / f"{script_name}.py"
    if not script.is_file():
        print(f"error: missing {script}", file=sys.stderr)
        return 2
    script_args = list(args[1:])
    if skill == "search_destination" and "--locate-only" not in script_args:
        script_args.append("--locate-only")
    if skill in {"place_on_blue", "place_on_yellow"} and "--destination-color" not in script_args:
        script_args.extend(["--target-color", "red", "--destination-color", "blue" if skill == "place_on_blue" else "yellow"])
    if skill in PRIMITIVE_MOTIONS and "--motion" not in script_args:
        script_args.extend(["--motion", PRIMITIVE_MOTIONS[skill]])
    # Run each standalone public skill in its own coherent process. Deterministic
    # acquisition programs may use the hidden run_program() fastpath instead.
    return _run_skill_process(script_name, script_args)

def _argv_color(argv: list[str], flag: str) -> str | None:
    try:
        value = argv[argv.index(flag) + 1].strip().lower()
    except (ValueError, IndexError, AttributeError):
        return None
    return value if value in {"red", "blue", "yellow"} else None


if __name__ == "__main__":
    raise SystemExit(main())
