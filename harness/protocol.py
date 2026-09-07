"""Parse a model's JSON action. The model never writes Python."""

from __future__ import annotations

import json
import re
from dataclasses import dataclass
from typing import Any


class ProtocolError(ValueError):
    """Raised when the model reply is not a valid action."""


@dataclass(frozen=True)
class ToolCall:
    name: str
    args: dict[str, Any]
    say: str | None = None


@dataclass(frozen=True)
class FinalAnswer:
    text: str


@dataclass(frozen=True)
class Plan:
    steps: tuple[str, ...]
    say: str | None = None


@dataclass(frozen=True)
class Wait:
    seconds: float
    say: str | None = None


@dataclass(frozen=True)
class Look:
    note: str | None = None
    say: str | None = None


Action = ToolCall | FinalAnswer | Plan | Wait | Look


def parse_action(text: str) -> Action:
    obj = extract_json_object(text)
    if not isinstance(obj, dict):
        raise ProtocolError("action must be a JSON object")
    if "tool" in obj:
        name = obj["tool"]
        if not isinstance(name, str) or not name.strip():
            raise ProtocolError("tool must be a non-empty string")
        args = obj.get("args", {})
        if args is None:
            args = {}
        if not isinstance(args, dict):
            raise ProtocolError("args must be an object")
        extra = set(obj) - {"tool", "args", "say"}
        if extra:
            raise ProtocolError(f"unexpected keys: {', '.join(sorted(extra))}")
        say = _optional_say(obj)
        return ToolCall(name=name.strip(), args=args, say=say)
    if "final" in obj:
        message = obj["final"]
        if not isinstance(message, str):
            raise ProtocolError("final must be a string")
        extra = set(obj) - {"final", "say"}
        if extra:
            raise ProtocolError(f"unexpected keys: {', '.join(sorted(extra))}")
        return FinalAnswer(text=message)
    if "plan" in obj:
        return _parse_plan(obj)
    if "wait" in obj:
        return _parse_wait(obj)
    if "look" in obj:
        return _parse_look(obj)
    say = _optional_say(obj)
    if say is not None and set(obj) <= {"say"}:
        return FinalAnswer(text=say)
    # Models regularly collapse {"tool": name, "args": {...}} into
    # {name: {...}} (seen live with send_peer_message). The registry still
    # validates the name and args, so accept that shape instead of burning the
    # turn on protocol errors.
    keyed = [key for key in obj if key != "say"]
    if len(keyed) == 1 and isinstance(obj[keyed[0]], dict) and _looks_like_tool_name(keyed[0]):
        return ToolCall(name=keyed[0], args=dict(obj[keyed[0]]), say=say)
    raise ProtocolError('expected {"plan"|"tool"|"wait"|"look"|"final"}')


def _looks_like_tool_name(name: str) -> bool:
    return bool(re.fullmatch(r"[a-z][a-z0-9_]{1,63}", name))


def extract_json_object(text: str) -> Any:
    stripped = text.strip()
    if stripped.startswith("```"):
        stripped = _strip_fence(stripped)
    try:
        return json.loads(stripped)
    except json.JSONDecodeError:
        start = stripped.find("{")
        end = stripped.rfind("}")
        if start < 0 or end <= start:
            raise ProtocolError("no JSON object in model output") from None
        try:
            return json.loads(stripped[start : end + 1])
        except json.JSONDecodeError as exc:
            raise ProtocolError(f"invalid JSON: {exc.msg}") from exc


def _strip_fence(text: str) -> str:
    lines = text.splitlines()
    if lines and lines[0].startswith("```"):
        lines = lines[1:]
    if lines and lines[-1].strip() == "```":
        lines = lines[:-1]
    return "\n".join(lines).strip()


def _optional_say(obj: dict[str, Any]) -> str | None:
    if "say" not in obj:
        return None
    say = obj["say"]
    if not isinstance(say, str):
        raise ProtocolError("say must be a string")
    say = say.strip()
    return say or None


def _parse_plan(obj: dict[str, Any]) -> Plan:
    extra = set(obj) - {"plan", "say"}
    if extra:
        raise ProtocolError(f"unexpected keys: {', '.join(sorted(extra))}")
    raw = obj["plan"]
    if not isinstance(raw, list) or not raw:
        raise ProtocolError("plan must be a non-empty list of steps")
    if len(raw) > 8:
        raise ProtocolError("plan may have at most 8 steps")
    steps: list[str] = []
    for item in raw:
        if not isinstance(item, str) or not item.strip():
            raise ProtocolError("plan steps must be non-empty strings")
        steps.append(item.strip())
    return Plan(steps=tuple(steps), say=_optional_say(obj))


def _parse_wait(obj: dict[str, Any]) -> Wait:
    extra = set(obj) - {"wait", "say"}
    if extra:
        raise ProtocolError(f"unexpected keys: {', '.join(sorted(extra))}")
    value = obj["wait"]
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ProtocolError("wait must be a number of seconds")
    seconds = float(value)
    if not 0.2 <= seconds <= 5.0:
        raise ProtocolError("wait must be between 0.2 and 5.0 seconds")
    return Wait(seconds=seconds, say=_optional_say(obj))


def _parse_look(obj: dict[str, Any]) -> Look:
    extra = set(obj) - {"look", "say"}
    if extra:
        raise ProtocolError(f"unexpected keys: {', '.join(sorted(extra))}")
    value = obj["look"]
    note = None
    if isinstance(value, str):
        note = value.strip() or None
    elif value is not True:
        raise ProtocolError("look must be true or a short note")
    return Look(note=note, say=_optional_say(obj))


def team_prompt(self_id: str) -> str:
    """Collaboration rules for one robot in the three-robot team (D3 baseline).

    The peer bus only carries evidence; these rules tell the planner how to
    turn that evidence into a local decision without a central planner.
    """
    me = str(self_id or "r?").strip().lower()
    peers = ", ".join(p for p in ("r1", "r2", "r3") if p != me) or "the other robots"
    return (
        f"TEAM: you are robot {me}. Peers {peers} are equal robots, not the user. All three share one workspace and the same "
        "three blocks (red, blue, yellow); a block a peer is working on is not yours.\n"
        "A system message multi_robot_peer_context gives team_goal, peer_messages (to you or to all), peers (each peer's last "
        "tool, its target_color, whether it succeeded) and recent_team_events. Read it before every decision.\n"
        "Role rule: if team_goal, the user text, or a peer message assigns you a color or a step, do only that. If nothing assigns "
        "you a color and the goal needs one block per robot, choose a color no peer has announced or is working on "
        "(peers[*].target_color), announce it once with send_peer_message(recipient=\"all\") and then act. If two robots "
        "announced the same color, the lower robot number keeps it and the other picks another color.\n"
        "Use send_peer_message only for: announcing your role, handing a completed step to the next robot with the exact next "
        "step, asking a peer for a step you cannot do, or reporting a failure that blocks a peer's step. One short Korean message "
        "per event; no chat, no acknowledgements of acknowledgements, no replies to a peer's status report. Never message peers "
        "about infrastructure errors (bridge/worker timeouts, offline sim): finish your turn and tell the user instead. "
        "Do not wait for a reply unless the goal requires the peer's step first.\n"
        "Sequential team goals (for example stacking): do your own step, verify it from evidence, then hand off to the next robot "
        "with send_peer_message naming the next concrete action. Never start a step that depends on a peer's unfinished step.\n"
        "Space rule: if a tool fails with failure_code PEER_TOO_CLOSE, a peer robot is within the safety radius. Use wait and retry; "
        "never drive toward a peer.\n"
    )


def system_prompt(
    tool_schema: str,
    *,
    talk_only: bool = False,
    auto_observe: bool = False,
    conversation_only: bool = False,
    team_id: str | None = None,
) -> str:
    team = team_prompt(team_id) if team_id else ""
    if talk_only:
        return (
            "You are a coworker sharing a workspace with the user.\n"
            "The robot and camera are not connected. This is a talk-only demo.\n"
            "Reply in Korean with one JSON object: {\"final\": \"...\"}.\n"
            "Do not call tools. Do not send plan, wait, or look.\n"
            "You may converse about the work. Do not write Python.\n"
            "Reply with one JSON object and nothing else.\n"
            'Finish: {"final": "지금은 로봇이 없어서 대화만 할 수 있어."}\n'
        )
    if conversation_only:
        return (
            "You are a coworker sharing a workspace with the user.\n"
            "This is a conversation-only turn. The robot may be connected, but the user did not request robot motion.\n"
            "Do not call tools. Do not send plan, wait, or look.\n"
            "If a situation image is supplied, answer only from clearly visible evidence in that image. "
            "Never invent people, clothing, screens, objects, text, or actions that are not clearly visible; say that a detail is unclear when uncertain. "
            "If the user asks about the scene and no image is supplied, say that a current frame is unavailable.\n"
            "Reply naturally in Korean with one JSON object: {\"final\": \"...\"}.\n"
            "Reply with one JSON object and nothing else.\n"
        )
    if auto_observe:
        return (
            "You are a coworker operating a robot. Work until the goal is done or one question is required.\n"
            "For any goal needing two or more public skills, you MUST first send one Plan containing the intended allowlisted skill sequence instead of issuing the skills one-by-one. "
            "The queue rechecks each head before execution; you are called again only when the queue drains, blocks, or fails. After a block/failure, inspect fresh evidence and revise the Plan or send one explicit recovery tool.\n"
            "world_state is deterministic sensor evidence. UNKNOWN means unknown. goal_evidence and recent_action_evidence are observations only; use them to judge progress. "
            "action_contract_status with executable=false means that tool is currently invalid. Preconditions are validity/safety gates, not recovery policy. "
            "last_action.recommended_recovery, when present, is an advisory from the physical skill that just failed; treat it as evidence, not a mandatory command, and choose the next action yourself. "
            "Approach is coarse staging only. Before pick, task.phase=PREGRASP_READY and world_state.causal_handoffs.pregrasp_reach.available must be true. Pick then freezes the chassis, re-measures the target over multiple fresh frames, verifies face/bearing geometry and solves IK itself; no approach-created FK/IK file is required.\n"
            "world_state.action_queue is authoritative. Never manually resubmit a tool that is already pending.\n"
            "The initial decision receives the current camera image. After actions, fresh camera evidence updates world_state but raw pixels are not reattached automatically. "
            "The camera is eye-in-hand; visible arm/gripper geometry is SELF. If raw pixels are genuinely needed again use {\"look\": true}, never {\"tool\": \"look\"}. Do not claim physical success from a pre-action image.\n"
            "No situation-to-action policy is supplied: choose/replan from tool semantics, contracts, world_state and the image. "
            "Speak Korean. Do not invent tools or write Python. If image/request is insufficient, ask one short question.\n"
            "Reply with exactly one JSON object. Plan={\"plan\":[\"tool_name\"]}; Act={\"tool\":\"tool_name\",\"args\":{}}; Finish={\"final\":\"...\"}.\n"
            f"{team}"
            "Available tools:\n"
            f"{tool_schema}"
        )
    return (
            "You are a coworker sharing a workspace with the user.\n"
            "You are not a chatbot. Work as an agent until the goal is done or you must ask one question.\n"
            "For any goal that needs two or more public skills, you MUST first send one Plan containing the intended allowlisted skill sequence instead of issuing the skills one-by-one. "
            "The action queue executes that Plan in order and rechecks each head immediately before execution; you are called again only when the queue drains, blocks, or fails. "
            "After a queued tool fails or blocks, inspect the fresh post-action evidence and queue state, then send a revised Plan or one explicit recovery tool.\n"
            "The harness also supplies a machine-readable world_state built from deterministic sensor processing. "
            "Treat known world_state fields as current sensor estimates; UNKNOWN means the sensor layer does not know. "
            "world_state also includes goal_evidence and recent_action_evidence: recent tool outcomes, before/after state changes, measurement deltas, and consecutive same-action counts. "
            "Use those facts to judge whether an action actually changed the situation and whether another invocation is justified. They are observations only and do not prescribe or prohibit any next action. "
            "The robot camera is mounted on the arm/gripper, so orange/black fingers, wrist, and arm pieces visible near the camera are SELF geometry, not task objects.\n"
            "Skill preconditions are safety/validity gates only. If a requested action is rejected, use the rejection facts and current observation to choose the next action yourself; the executive does not choose a recovery action for you. A driver-provided recommended_recovery is advisory evidence only, not a forced policy.\n"
            "When world_state.action_contract_status contains a tool with executable=false, that tool is currently invalid and must not be submitted. This is only a validity constraint, not a recovery policy; choose another currently executable action yourself. "
            "For pick specifically, task.phase=PREGRASP_READY plus world_state.causal_handoffs.pregrasp_reach.available=true means successful coarse approach left the target in a fresh stopped arm-reach corridor. Pick performs its own multi-frame final measurement, face/bearing verification and IK; it does not consume an approach-created FK/IK plan.\n"
            "world_state.action_queue is the authoritative per-robot public-skill queue. Inspect its running, pending, paused, and recent history fields before planning. "
            "A Plan replaces not-yet-started queued work and the executor then runs its skills in order, rechecking each head immediately before execution. Do not manually resubmit steps that are already pending. "
            "If a queued head is BLOCKED or FAILED, later queued skills do not run; inspect the queue and current world state, then revise the plan or submit one explicit recovery tool. A direct recovery replaces stale later queued work and you replan after it completes.\n"
            "After a tool, the image attached to your next decision is the fresh post-action observation when available. "
            "If an exceptional extra observation is truly needed, use the protocol {\"look\": true}, never {\"tool\": \"look\"}.\n"
            "Do not claim physical success from a pre-action image. Objective tool results may establish completion; otherwise judge from the current evidence.\n"
            "No situation-to-action policy is supplied: action choice and replanning are your responsibility. "
            "Use only the action semantics, arguments, contracts, current world_state, and observations below.\n"
            "Speak Korean like a person coordinating work. Do not write Python or invent tools.\n"
            "If the image is missing or the request is unclear, ask one short question in {\"final\": \"...\"}.\n"
            "Reply with one JSON object and nothing else.\n"
            'Plan format: {"plan": ["tool_name", "tool_name"]}\n'
            'Act format: {"tool": "tool_name", "args": {}}\n'
            'Finish format: {"final": "..."}\n'
            f"{team}"
            "Available tools:\n"
            f"{tool_schema}"
        )
