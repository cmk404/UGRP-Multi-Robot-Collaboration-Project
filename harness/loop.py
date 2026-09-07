"""Ask a model for JSON actions, then run only allowlisted tools."""

from __future__ import annotations

import json
import os
import time
import traceback
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Protocol

from .catalog import resolve_plan_step
from .protocol import (
    Action,
    FinalAnswer,
    Look,
    Plan,
    ProtocolError,
    ToolCall,
    Wait,
    parse_action,
    system_prompt,
)
from .registry import Registry, ToolError, validate_args
from .executive import TaskExecutive
from .state import StateEstimator
from .perception import detect_scene_bytes, prepare_planner_image
from .goals import GoalSpec, final_is_non_success, goal_status, infer_goal
from .action_queue import RobotActionQueue
from .vlm import VlmError

MAX_CONSECUTIVE_PROTOCOL_ERRORS = 3
DEFAULT_ACTION_QUEUE_TTL_S = 300.0
DEFAULT_SKILL_RETRY_BUDGET = 2


def _action_queue_ttl_seconds() -> float:
    """Bound persisted queue work so a later wake cannot run an old plan."""
    try:
        return max(0.0, float(os.environ.get("UGRP_ACTION_QUEUE_TTL_S", str(DEFAULT_ACTION_QUEUE_TTL_S))))
    except (TypeError, ValueError):
        return DEFAULT_ACTION_QUEUE_TTL_S


def _skill_retry_budget(skill: str) -> int:
    """Return a bounded per-public-skill recovery budget.

    Existing deployments used a hard-coded two-retry limit. Preserve that
    default while allowing SIM experiments to tighten one skill without
    changing REAL behaviour or the global planner-call budget.
    """
    name = str(skill or "").strip().upper().replace("-", "_")
    raw = os.environ.get(f"UGRP_MAX_RETRIES_{name}", str(DEFAULT_SKILL_RETRY_BUDGET))
    try:
        return max(0, min(8, int(raw)))
    except (TypeError, ValueError):
        return DEFAULT_SKILL_RETRY_BUDGET


def _retry_state_signature(state) -> str:
    """Stable actor-visible state subset used to suppress no-progress retries."""
    public = state.public() if hasattr(state, "public") else {}
    target = public.get("target") or {}
    grasp = public.get("grasp") or {}
    task = public.get("task") or {}
    return json.dumps(
        {
            "target": {
                "visible": target.get("visible"),
                "centered": target.get("centered"),
                "range_class": target.get("range_class"),
                "selected_color": target.get("selected_color"),
            },
            "grasp": {
                "state": grasp.get("state"),
                "held_object_color": grasp.get("held_object_color"),
            },
            "task": {"phase": task.get("phase"), "destination_color": task.get("destination_color")},
        },
        sort_keys=True,
        separators=(",", ":"),
    )


class Completer(Protocol):
    def complete(self, messages: list[dict[str, Any]], image: str | None = None) -> str: ...


class ReplayCompleter:
    """Return scripted model replies. Used by tests and `python -m harness --replay`."""

    def __init__(self, replies: list[Any]):
        self.replies = [
            reply if isinstance(reply, str) else json.dumps(reply, ensure_ascii=False)
            for reply in replies
        ]
        self.index = 0

    def complete(self, messages: list[dict[str, Any]], image: str | None = None) -> str:
        del messages, image
        if self.index >= len(self.replies):
            return json.dumps({"final": "no more replay replies"}, ensure_ascii=False)
        reply = self.replies[self.index]
        self.index += 1
        return reply


@dataclass(frozen=True)
class Step:
    raw: str
    action: Action | None
    result: dict[str, Any] | None = None
    error: str | None = None


@dataclass
class LoopResult:
    final: str | None
    steps: list[Step] = field(default_factory=list)
    stopped: str = "max_steps"
    pending_plan: list[str] = field(default_factory=list)
    world_state: dict[str, Any] = field(default_factory=dict)
    action_queue: dict[str, Any] = field(default_factory=dict)


def _structured_red_block_goal(goal: GoalSpec, tool_names: list[str], executive: TaskExecutive, state=None) -> list[tuple[str, dict[str, str]]]:
    """Compatibility shim: the deterministic executive owns known skill programs."""
    return executive.plan_calls(goal, tool_names, state)


def _fusable_acquisition_prefix(queue: list[tuple[str, dict[str, str]]]) -> list[tuple[str, dict[str, str]]] | None:
    """Return a contiguous acquisition suffix eligible for hidden fusion."""
    expected = ("search", "track", "approach", "pick")
    for start in range(len(expected) - 1):
        suffix = expected[start:]
        if len(queue) < len(suffix):
            continue
        prefix = queue[: len(suffix)]
        if tuple(name for name, _ in prefix) != suffix:
            continue
        colors = []
        valid = True
        for _name, args in prefix:
            if not isinstance(args, dict) or set(args) - {"target_color"}:
                valid = False
                break
            color = args.get("target_color", "red")
            if color not in {"red", "blue", "yellow"}:
                valid = False
                break
            colors.append(color)
        if valid and len(set(colors)) == 1:
            return prefix
    return None


def _acquisition_recovery_chain(
    recovery: str, failed: str, args: dict[str, Any] | None
) -> list[tuple[str, dict[str, Any]]]:
    """Rebuild causal prerequisites instead of retrying a broken middle stage."""
    stages = ("search", "track", "approach", "pick")
    if recovery not in stages or failed not in stages:
        return [(recovery, dict(args or {})), (failed, dict(args or {}))]
    start = stages.index(recovery)
    stop = stages.index(failed)
    if start > stop:
        return [(recovery, dict(args or {})), (failed, dict(args or {}))]
    color_args: dict[str, Any] = {}
    color = (args or {}).get("target_color")
    if color in {"red", "blue", "yellow"}:
        color_args["target_color"] = color
    return [(stage, dict(color_args)) for stage in stages[start : stop + 1]]


def _queued_recovery_chain(
    registry: Registry, recovery: str, failed: str, args: dict[str, Any] | None
) -> list[tuple[str, dict[str, Any]]]:
    """Build a bounded driver-advised recovery chain for an existing queue.

    Acquisition stages must rebuild causal prerequisites (search -> track ->
    approach -> pick). Other skills keep only arguments accepted by the recovery
    tool, then retry the failed skill with its original arguments.
    """
    if recovery in {"search", "track", "approach", "pick"} and failed in {
        "search", "track", "approach", "pick"
    }:
        return _acquisition_recovery_chain(recovery, failed, args)
    recovery_tool = registry.get(recovery)
    accepted = {param.name for param in recovery_tool.parameters}
    recovery_args = {
        key: value for key, value in dict(args or {}).items() if key in accepted
    }
    return [(recovery, recovery_args), (failed, dict(args or {}))]


def _structured_goal_final(goal_kind: str, state, *, tool_count: int = 0) -> str:
    status = goal_status(GoalSpec(goal_kind, ""), state)
    if status == "ACHIEVED":
        if goal_kind == "SCAN":
            return "주변 공간 인식을 완료했고 Semantic Radar 3D 맵을 갱신했어."
        if goal_kind == "SEARCH":
            color = getattr(state.target, "selected_color", None)
            label = {"red": "빨간", "blue": "파란", "yellow": "노란"}.get(color, "목표")
            return f"{label} 블록을 찾았어."
        return "요청한 작업을 완료했어."
    if tool_count <= 0:
        return "로봇 동작을 실행하지 못했어."
    if goal_kind == "GRASP":
        return "집기 동작까지 수행했지만 현재 카메라 센서만으로 실제로 잡혔는지는 확정할 수 없어."
    if goal_kind == "LIFT":
        return "집고 들어 올리는 동작까지 수행했지만 현재 카메라 센서만으로 물체를 계속 들고 있는지는 확정할 수 없어."
    if goal_kind == "PLACE":
        return "지정한 목표 블록 위에 내려놓는 동작까지 수행했어. 카메라 기반 검증만으로 정확한 적층 성공 여부는 아직 확정하지 않았어."
    return "동작을 수행했지만 센서만으로 완료 여부를 확정할 수 없어."


def _planner_state_public(state) -> dict[str, Any]:
    """Compact sensor state for the LLM; UI/debug state remains full fidelity."""
    public = state.public()
    target = public.get("target") or {}
    grasp = public.get("grasp") or {}
    action = public.get("last_action") or {}
    task = public.get("task") or {}
    landmarks: dict[str, Any] = {}
    for color, entry in (public.get("spatial_memory") or {}).items():
        if not isinstance(entry, dict):
            continue
        if entry.get("visible") is not True and entry.get("metric_position_available") is not True:
            continue
        landmarks[str(color)] = {
            key: entry.get(key)
            for key in ("visible", "image_xy")
            if key in entry
        }
    return {
        "target": {
            key: target.get(key)
            for key in ("selected_color", "visible", "confidence", "cx", "cy", "area_ratio", "centered", "range_class")
        },
        "grasp": {
            key: grasp.get(key)
            for key in ("held_object_color", "state", "confidence", "visual_support")
        },
        "last_action": {
            key: action.get(key)
            for key in (
                "name", "command_status", "execution_status", "outcome_status",
                "failure_code", "required_state", "recommended_recovery",
            )
        },
        "task": {key: task.get(key) for key in ("phase", "destination_color")},
        "landmarks": landmarks,
    }


def _planner_tool_result(payload: dict[str, Any]) -> dict[str, Any]:
    """Return one environment-neutral tool-result surface to the LLM.

    SIM may have MuJoCo geometry and REAL may have transport/trace details, but
    neither implementation detail is part of the agent contract.  Keep only
    facts both environments can truthfully expose.  Pixel geometry comes from
    the same post-action image path instead of adapter-specific result payloads.
    """
    nested = payload.get("result") if isinstance(payload.get("result"), dict) else {}
    keep = (
        "skill", "command_status", "execution_status", "outcome_status",
        "failure_code", "required_state", "recommended_recovery", "target_color", "destination_color",
        "center_verified", "range_verified", "grasp_verified", "visual_hold",
        "place_verified", "task_complete", "motion", "chassis_motion",
        "carry_evidence",
    )
    result = {key: nested.get(key) for key in keep if key in nested}
    vision = nested.get("target_vision")
    if not isinstance(vision, dict):
        vision = nested.get("camera_red")
    if isinstance(vision, dict) and isinstance(vision.get("visible"), bool):
        result["target_vision"] = {"visible": bool(vision["visible"])}
    return {
        "ok": payload.get("ok") is not False,
        "tool": payload.get("tool"),
        "result": result,
    }


def _state_condition_status(public: dict[str, Any], condition: str) -> bool | None:
    """Evaluate simple actor-visible contract equalities without inventing policy."""
    text = str(condition).strip()
    if "=" not in text or "!=" in text:
        return None
    path, expected = (part.strip() for part in text.split("=", 1))
    node: Any = public
    for part in path.split("."):
        if not isinstance(node, dict) or part not in node:
            return None
        node = node[part]
    options = [item.strip() for item in expected.split("|")]
    if isinstance(node, bool):
        normalized = "true" if node else "false"
    elif node is None:
        normalized = "null"
    else:
        normalized = str(node)
    return normalized in options


def _action_contract_status(state, registry: Registry, executive: TaskExecutive) -> dict[str, Any]:
    """Expose feasibility/postcondition facts; never select a next action."""
    public = _planner_state_public(state)
    target_color = (public.get("target") or {}).get("selected_color") or "red"
    out: dict[str, Any] = {}
    planner_status_tools = {"search", "track", "approach", "pick", "observe_scene", "search_destination", "place", "put_down", "stop_motion"}
    for name in registry.names():
        if name not in planner_status_tools:
            continue
        tool = registry.get(name)
        contract = tool.contract or {}
        preconditions = [str(x) for x in (contract.get("preconditions") or [])]
        postconditions = [str(x) for x in (contract.get("expected_postconditions") or [])]
        if not preconditions and not postconditions:
            continue
        args: dict[str, Any] = {}
        unresolved_required = False
        for param in tool.parameters:
            if param.name == "target_color":
                args[param.name] = target_color
            elif not param.required:
                args[param.name] = param.default
            else:
                unresolved_required = True
        decision = None if unresolved_required else executive.check(name, state, args)
        post_status = [
            {"condition": condition, "satisfied": _state_condition_status(public, condition)}
            for condition in postconditions
        ]
        known = [item["satisfied"] for item in post_status if item["satisfied"] is not None]
        out[name] = {
            "executable": None if decision is None else bool(decision.allowed),
            "blocking_fact": None if decision is None or decision.allowed else {
                "failure_code": decision.failure_code,
                "required_state": decision.required_state,
            },
            # The condition strings themselves already live in the immutable
            # tool schema above. Repeating them in every world_state costs
            # hundreds of tokens per planner call; expose only current truth.
            "expected_postconditions_all_satisfied": (
                bool(known) and len(known) == len(post_status) and all(known)
            ),
        }
    return out


def _queue_snapshot(
    queue: RobotActionQueue,
    state,
    executive: TaskExecutive,
    *,
    compact: bool = False,
) -> dict[str, Any]:
    """Expose queue order plus current precondition truth for waiting skills."""
    snapshot = queue.snapshot()
    running = snapshot.get("running")
    if isinstance(running, dict):
        decision = executive.check(str(running.get("tool") or ""), state, dict(running.get("args") or {}))
        running["executable_now"] = bool(decision.allowed)
        running["blocking_fact"] = None if decision.allowed else {
            "failure_code": decision.failure_code,
            "required_state": decision.required_state,
        }
    for entry in snapshot.get("pending", []):
        if not isinstance(entry, dict):
            continue
        decision = executive.check(str(entry.get("tool") or ""), state, dict(entry.get("args") or {}))
        entry["executable_now"] = bool(decision.allowed)
        entry["blocking_fact"] = None if decision.allowed else {
            "failure_code": decision.failure_code,
            "required_state": decision.required_state,
        }
    if not compact:
        return snapshot

    def compact_entry(entry: Any) -> Any:
        if not isinstance(entry, dict):
            return entry
        keys = (
            "tool", "args", "status", "failure_code", "required_state",
            "reason", "outcome_status", "executable_now", "blocking_fact",
        )
        out = {key: entry.get(key) for key in keys if entry.get(key) is not None}
        if isinstance(out.get("reason"), str):
            out["reason"] = out["reason"][:240]
        return out

    pause_reason = str(snapshot.get("pause_reason") or "")[:240] or None
    return {
        "revision": snapshot.get("revision"),
        "paused": snapshot.get("paused"),
        "pause_reason": pause_reason,
        "running": compact_entry(snapshot.get("running")),
        "pending": [compact_entry(entry) for entry in snapshot.get("pending", [])],
        "history": [compact_entry(entry) for entry in snapshot.get("history", [])[-4:]],
    }


def _planner_queue_snapshot(queue: RobotActionQueue, state, executive: TaskExecutive) -> dict[str, Any]:
    """Bounded queue view used only in LLM context."""
    return _queue_snapshot(queue, state, executive, compact=True)


def _plan_queue_calls(
    steps: tuple[str, ...], registry: Registry, goal: GoalSpec, state
) -> list[tuple[str, dict[str, Any]]]:
    """Resolve string plan steps to executable calls using only explicit goal/state identities."""
    colors = {"RED_BLOCK": "red", "BLUE_BLOCK": "blue", "YELLOW_BLOCK": "yellow"}
    target_color = colors.get(goal.subject) or getattr(state.target, "selected_color", None)
    destination_color = colors.get(goal.target) or getattr(state.task, "destination_color", None)
    calls: list[tuple[str, dict[str, Any]]] = []
    for step in steps:
        name = resolve_plan_step(step, registry.names())
        if not name:
            continue
        tool = registry.get(name)
        args: dict[str, Any] = {}
        for param in tool.parameters:
            if param.name == "target_color" and target_color in {"red", "blue", "yellow"}:
                args[param.name] = target_color
            elif param.name == "destination_color" and destination_color in {"red", "blue", "yellow"}:
                args[param.name] = destination_color
            elif not param.required:
                args[param.name] = param.default
            else:
                raise ProtocolError(
                    f"plan step {name!r} needs explicit argument {param.name!r}; submit it as a tool call instead"
                )
        # Normalize defaults/ranges now. Queue entries must never contain calls
        # that would only fail schema validation after earlier physical skills run.
        args = validate_args(tool, args)
        calls.append((name, args))
    if not calls:
        raise ProtocolError("plan did not resolve to any allowlisted tools")
    return calls


def _planner_decision_context(
    state,
    goal: GoalSpec,
    recent_action_evidence: list[dict[str, Any]],
    *,
    registry: Registry | None = None,
    executive: TaskExecutive | None = None,
    action_queue: RobotActionQueue | None = None,
) -> dict[str, Any]:
    """Expose evidence for the VLM's own progress and replanning judgment.

    This reports goal status, causal handoffs, action-contract truth, and recent
    state transitions, but deliberately does not recommend or forbid any next
    action. Repetition remains the model's decision when the evidence justifies it.
    """
    public = _planner_state_public(state)
    public["observation_count"] = int(getattr(state, "observation_count", 0))
    public["goal_evidence"] = {
        "kind": goal.kind,
        "subject": goal.subject,
        "target": goal.target,
        "sensor_status": goal_status(goal, state),
    }
    task = public.get("task") or {}
    target = public.get("target") or {}
    pregrasp_reach_ready = (
        task.get("phase") == "PREGRASP_READY"
        and target.get("range_class") == "PREGRASP"
    )
    public["causal_handoffs"] = {
        "pregrasp_reach": {
            "available": pregrasp_reach_ready,
            "target_color": target.get("selected_color") if pregrasp_reach_ready else None,
            "evidence": (
                "successful coarse approach left a fresh stopped arm-reach handoff; "
                "pick will re-measure and solve IK"
                if pregrasp_reach_ready else None
            ),
        }
    }
    if registry is not None and executive is not None:
        public["action_contract_status"] = _action_contract_status(state, registry, executive)
    if action_queue is not None and executive is not None:
        public["action_queue"] = _planner_queue_snapshot(action_queue, state, executive)
    public["recent_action_evidence"] = list(recent_action_evidence[-3:])
    return public


def _progress_evidence(
    action_name: str,
    before: dict[str, Any],
    after: dict[str, Any],
    payload: dict[str, Any],
    *,
    same_action_streak: int,
) -> dict[str, Any]:
    """Summarize actor-visible before/after measurements without choosing policy."""
    changes: list[dict[str, Any]] = []
    for section, fields in (
        ("target", ("visible", "centered", "range_class", "cx", "cy", "area_ratio")),
        ("grasp", ("state", "held_object_color", "visual_support")),
        ("task", ("phase", "destination_color")),
    ):
        b = before.get(section) or {}
        a = after.get(section) or {}
        for field in fields:
            bv, av = b.get(field), a.get(field)
            if bv != av:
                changes.append({"field": f"{section}.{field}", "before": bv, "after": av})

    def number(value: Any) -> float | None:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            return None
        return float(value)

    bt, at = before.get("target") or {}, after.get("target") or {}
    deltas: dict[str, float] = {}
    bcx, acx = number(bt.get("cx")), number(at.get("cx"))
    if bcx is not None and acx is not None:
        # Positive means the target moved closer to the optical center.
        deltas["center_error_reduction"] = round(abs(bcx - 0.5) - abs(acx - 0.5), 4)
    bcy, acy = number(bt.get("cy")), number(at.get("cy"))
    if bcy is not None and acy is not None:
        deltas["target_cy_delta"] = round(acy - bcy, 4)
    ba, aa = number(bt.get("area_ratio")), number(at.get("area_ratio"))
    if ba is not None and aa is not None:
        deltas["target_area_ratio_delta"] = round(aa - ba, 6)

    nested = payload.get("result") if isinstance(payload.get("result"), dict) else {}
    return {
        "action": action_name,
        "executed": payload.get("dry_run") is False,
        "same_action_streak": int(same_action_streak),
        "outcome_status": nested.get("outcome_status") or (after.get("last_action") or {}).get("outcome_status"),
        "failure_code": nested.get("failure_code") or (after.get("last_action") or {}).get("failure_code"),
        "observation_count_before": int(before.get("observation_count") or 0),
        "observation_count_after": int(after.get("observation_count") or 0),
        "changed_fields": changes,
        "measurement_deltas": deltas,
    }


def _structured_failure_final(skill: str | None, reason: str | None) -> str:
    reason_l = (reason or "").lower()
    if "slipped" in reason_l:
        detail = "운반 중 블록이 집게에서 미끄러졌어."
    elif "not found" in reason_l or "not visible" in reason_l:
        detail = "카메라와 공간기억으로 목표를 다시 찾지 못했어."
    elif "track" in reason_l or "center" in reason_l or "align" in reason_l:
        detail = "목표를 안정적으로 시야 중앙에 맞추지 못했어."
    elif "stack postcondition" in reason_l:
        detail = "내려놓기는 했지만 목표 블록 위 적층 조건을 만족하지 못했어."
    elif reason:
        detail = str(reason)
    else:
        detail = "실행 중 한 단계의 성공 조건을 만족하지 못했어."
    where = f" ({skill})" if skill else ""
    return f"작업을 완료하지 못했어{where}: {detail}"


def _carry_loss_final(state) -> str:
    """Report a driver-confirmed delivery loss without inviting motion recovery."""
    color = getattr(state.target, "selected_color", None)
    label = {"red": "빨간", "blue": "파란", "yellow": "노란"}.get(color, "들고 있던")
    return f"{label} 블록의 운반 상태가 더 이상 확인되지 않아 안전을 위해 자동 목적지 탐색·이동·재집기를 중단했어."


def _planner_budget_final(state, queue: RobotActionQueue) -> str:
    """Report current progress/blocker without spending another LLM call."""
    last = getattr(state, "last_action", None)
    if getattr(last, "outcome_status", None) == "ACHIEVED":
        if getattr(getattr(state, "target", None), "range_class", None) == "PREGRASP":
            return "팔 reach 안까지 접근했지만 이번 턴의 tool 실행 한도에 도달해 pick 전에 중단했어."
        return "마지막 로봇 동작은 성공했지만 이번 턴의 tool 실행 한도에 도달해 다음 동작 전에 중단했어."
    snapshot = queue.snapshot()
    reason = str(snapshot.get("pause_reason") or "").strip()
    if not reason:
        for entry in reversed(snapshot.get("history", [])):
            if not isinstance(entry, dict):
                continue
            if entry.get("status") not in {"FAILED", "BLOCKED"}:
                continue
            reason = str(entry.get("reason") or entry.get("failure_code") or "").strip()
            if reason:
                break
    if not reason:
        reason = str(getattr(last, "failure_code", "") or "").strip()
    if reason:
        return f"여러 차례 재계획했지만 현재 실패 원인 때문에 작업을 완료하지 못했어: {reason}"
    return "여러 차례 재계획했지만 작업을 완료하지 못해 추가 자동 재시도를 중단했어."


def run_loop(
    completer: Completer,
    registry: Registry,
    user_text: str,
    *,
    image: str | None = None,
    history: list[dict[str, str]] | None = None,
    max_steps: int = 12,
    execute: bool = False,
    observe: Callable[[], str | None] | None = None,
    on_step: Callable[[Step], None] | None = None,
    on_activity: Callable[[str, str | None], None] | None = None,
    on_planner_input: Callable[[str | None, dict[str, Any]], None] | None = None,
    sleeper: Callable[[float], None] = time.sleep,
    should_cancel: Callable[[], bool] | None = None,
    talk_only: bool = False,
    conversation_only: bool = False,
    auto_observe: bool = False,
    verify_final: Callable[[str, str, str], tuple[bool, str]] | None = None,
    state_estimator: StateEstimator | None = None,
    executive: TaskExecutive | None = None,
    planner_context: dict[str, Any] | None = None,
    planner_context_provider: Callable[[], dict[str, Any] | None] | None = None,
    action_queue: RobotActionQueue | None = None,
    planner_image_max_edge: int | None = None,
    auto_recommended_recovery: bool = False,
    executive_fallback_on_planner_error: bool = False,
) -> LoopResult:
    if max_steps < 1:
        raise ValueError("max_steps must be >= 1")
    # The image is attached through the provider's multimodal field. Do not
    # duplicate a meaningless local filesystem path into the text prompt.
    user_content = user_text
    team_id: str | None = None
    if isinstance(planner_context, dict) and not (talk_only or conversation_only):
        team_id = str(planner_context.get("self_id") or "").strip().lower() or None
    messages: list[dict[str, Any]] = [
        {"role": "system", "content": system_prompt(
            registry.prompt_schema(), talk_only=talk_only, auto_observe=auto_observe,
            conversation_only=conversation_only, team_id=team_id,
        )},
    ]
    if planner_context and planner_context_provider is None:
        # Static peer context is retained for non-live callers. Browser robot
        # sessions use planner_context_provider so messages/goals can change
        # while this same autonomous turn is still running.
        messages.append({
            "role": "system",
            "content": "multi_robot_peer_context: " + json.dumps(planner_context, ensure_ascii=False),
        })
    if history:
        messages.extend(history)
    messages.append({"role": "user", "content": user_content})
    mission_index = len(messages) - 1
    estimator = state_estimator or StateEstimator()
    task_executive = executive or TaskExecutive()
    skill_queue = action_queue or RobotActionQueue()
    # A queue may outlive the HTTP turn that created it. Expire only pending
    # work; running hardware/SIM actions still use the explicit cancel path.
    skill_queue.invalidate_stale(_action_queue_ttl_seconds())
    # A queue can persist across HTTP turns, but stale work never auto-resumes
    # merely because a new user message arrived. The planner sees it first and
    # may replace/prepend work explicitly. Plans created in this turn run
    # automatically in order until a skill blocks/fails or the queue drains.
    if skill_queue.has_pending() and not skill_queue.paused():
        skill_queue.pause("new_turn_review")
    queue_run_enabled = False
    goal_spec = infer_goal(user_text)
    color_by_subject = {"RED_BLOCK": "red", "BLUE_BLOCK": "blue", "YELLOW_BLOCK": "yellow"}
    estimator.set_goal_context(
        target_color=color_by_subject.get(goal_spec.subject),
        destination_color=color_by_subject.get(goal_spec.target),
    )
    contract_mode = any(bool(registry.get(name).contract) for name in registry.names())
    def update_camera_state(image_path: str | None) -> None:
        if not image_path:
            return
        try:
            scene = detect_scene_bytes(Path(image_path).read_bytes())
        except OSError:
            scene = None
        if scene is not None:
            # Same camera-only detector and same state update for SIM and REAL.
            # Metric simulator truth is deliberately absent from this path.
            estimator.update_scene_memory(scene)

    update_camera_state(image)
    result = LoopResult(
        final=None,
        world_state=estimator.state.public(),
        pending_plan=skill_queue.pending_names(),
        action_queue=_queue_snapshot(skill_queue, estimator.state, task_executive),
    )
    pending: list[str] = []  # compatibility view; authoritative state is skill_queue
    tool_count = 0
    tools_exhausted = False
    team_handoff_grace_used = False
    team_staged_base = False
    zero_tool_final_rejected = False
    needs_observation = False
    fresh_observation = False
    image_pending = bool(image)
    authoritative_completion = False
    structured_outcome_seen = False
    consecutive_protocol_errors = 0
    max_iters = max_steps * 4 + 8
    try:
        configured_planner_budget = int(os.environ.get("UGRP_MAX_PLANNER_CALLS_PER_TURN", "0"))
    except ValueError:
        configured_planner_budget = 0
    provider_planner_budget = int(getattr(completer, "planner_call_budget", 0) or 0)
    max_planner_calls = (
        configured_planner_budget if configured_planner_budget > 0
        else provider_planner_budget if provider_planner_budget > 0
        else max_iters
    )
    max_planner_calls = max(1, max_planner_calls)
    planner_call_count = 0
    # Design intent: the default embodied-agent path leaves action selection to
    # the VLM.  The old deterministic goal->skill compiler remains an explicit
    # baseline/debug mode only; it must never silently replace planner decisions.
    structured_queue = (
        _structured_red_block_goal(goal_spec, registry.names(), task_executive, estimator.state)
        if execute
        and auto_observe
        and contract_mode
        and not talk_only
        and os.environ.get("UGRP_STRUCTURED_RED_FASTPATH", "0") == "1"
        else []
    )
    structured_active = bool(structured_queue)
    structured_failed = False
    structured_failure_skill: str | None = None
    structured_failure_reason: str | None = None
    structured_retries: dict[str, int] = {}
    structured_failure_signatures: set[tuple[str, str | None, str | None, str]] = set()
    queued_recovery_retries: dict[tuple[str, str], int] = {}
    queued_failure_signatures: set[tuple[str, str, str | None, str | None, str]] = set()
    # Autonomous-mode loop telemetry. This is deliberately policy-free: it
    # records only that the same proposed action hit the same unsatisfied
    # precondition without any relevant world-state change. The VLM still
    # decides what alternative action, if any, to take.
    last_rejection_signature: tuple[str, str | None, str | None, str] | None = None
    repeated_rejection_count = 0
    # Successful tool calls need the same visibility as rejected proposals. The
    # harness records what changed, but never converts this into an action rule.
    recent_action_evidence: list[dict[str, Any]] = []
    last_executed_action: str | None = None
    repeated_executed_action_count = 0

    def record_executed_action(
        action_name: str,
        before: dict[str, Any],
        payload: dict[str, Any],
    ) -> dict[str, Any]:
        nonlocal last_executed_action, repeated_executed_action_count
        if action_name == last_executed_action:
            repeated_executed_action_count += 1
        else:
            last_executed_action = action_name
            repeated_executed_action_count = 1
        after = _planner_decision_context(
            estimator.state, goal_spec, [], action_queue=skill_queue,
        )
        evidence = _progress_evidence(
            action_name,
            before,
            after,
            payload,
            same_action_streak=repeated_executed_action_count,
        )
        recent_action_evidence.append(evidence)
        del recent_action_evidence[:-4]
        return evidence

    def emit(step: Step) -> Step:
        result.steps.append(step)
        result.pending_plan = skill_queue.pending_names()
        result.world_state = estimator.state.public()
        result.action_queue = _queue_snapshot(skill_queue, estimator.state, task_executive)
        if on_step is not None:
            on_step(step)
        return step

    def refresh(*, attach_image: bool = False) -> str | None:
        """Take an ordered fresh camera observation.

        Every refresh updates deterministic perception/world_state from the
        original camera frame. Raw pixels are expensive on Groq (and are not
        required for every state transition), so only an explicit visual request
        marks the frame for attachment to the next VLM call.
        """
        nonlocal image, fresh_observation, image_pending
        if observe is None:
            return None
        seen = observe()
        if seen:
            image = seen
            update_camera_state(seen)
            fresh_observation = True
            image_pending = bool(attach_image)
        return seen

    def cancelled() -> bool:
        return bool(should_cancel and should_cancel())

    # Missing camera/robot evidence is a hard execution gate, not an LLM
    # recovery loop.  Returning deterministically prevents repeated
    # ``robot is offline`` errors and, critically, prevents a structured goal
    # from draining its queued tool names and later claiming those motions ran.
    if talk_only:
        text = "로봇/카메라 연결이 없어 동작을 실행하지 않았어."
        action = FinalAnswer(text=text)
        raw = json.dumps({"final": text}, ensure_ascii=False)
        emit(Step(raw=raw, action=action))
        result.final = text
        result.stopped = "robot_offline"
        return result

    for _ in range(max_iters):
        queue_entry_id: str | None = None
        if cancelled():
            result.stopped = "cancel"
            return result
        if estimator.state.task.phase == "CARRY_LOST":
            # The driver supplied an explicit carry-loss result. Stop before a
            # model can reinterpret stale plan state as permission to turn,
            # drive, search a destination, or retry a destructive grasp.
            text = _carry_loss_final(estimator.state)
            action = FinalAnswer(text=text)
            raw = json.dumps({"final": text}, ensure_ascii=False)
            emit(Step(raw=raw, action=action))
            result.final = text
            result.stopped = "carry_lost"
            return result
        if structured_active and not structured_queue:
            text = (
                _structured_failure_final(structured_failure_skill, structured_failure_reason)
                if structured_failed
                else _structured_goal_final(goal_spec.kind, estimator.state, tool_count=tool_count)
            )
            action = FinalAnswer(text=text)
            raw = json.dumps({"final": text}, ensure_ascii=False)
        elif structured_queue:
            fused_program = (
                _fusable_acquisition_prefix(structured_queue)
                if registry.program_runner is not None
                and os.environ.get("UGRP_FUSED_ACQUISITION", "1") != "0"
                and tool_count + 4 <= max_steps
                else None
            )
            if fused_program is not None:
                # Keep the public executive contract as four ordinary stages;
                # only the physical process/SSH/camera lifetime is fused. Search
                # is the first public gate and the internal stage functions keep
                # their own causal handoff/safety checks thereafter.
                first_name, first_args = fused_program[0]
                decision = task_executive.check(first_name, estimator.state, first_args)
                if not decision.allowed:
                    fused_program = None
                else:
                    if on_activity is not None:
                        on_activity("tool_start", first_name)
                    try:
                        stage_values = registry.program_runner(
                            [(name, dict(args)) for name, args in fused_program]
                        )
                    except Exception as exc:
                        traceback.print_exc()
                        stage_values = [{
                            "ok": False,
                            "skill": first_name,
                            "command_status": "ACCEPTED",
                            "execution_status": "FAILED",
                            "outcome_status": "NOT_ACHIEVED",
                            "failure_code": "TOOL_EXCEPTION",
                            "reason": f"{type(exc).__name__}: {exc}",
                            "recommended_recovery": None,
                        }]

                    # Remove the physical program once. If it failed part-way,
                    # only explicitly unexecuted public stages can be restored.
                    del structured_queue[: len(fused_program)]
                    remainder = list(structured_queue)
                    structured_queue.clear()
                    failed_index = None
                    for index, ((stage, stage_args), value) in enumerate(zip(fused_program, stage_values)):
                        call = ToolCall(name=stage, args=dict(stage_args), say=None)
                        ok = isinstance(value, dict) and value.get("ok") is not False
                        payload = {
                            "ok": ok,
                            "dry_run": False,
                            "tool": stage,
                            "result": value,
                            "fused_program": True,
                        }
                        nested = value if isinstance(value, dict) else {}
                        if any(k in nested for k in ("command_status", "execution_status", "outcome_status")):
                            structured_outcome_seen = True
                        estimator.update_tool_result(payload)
                        tool_count += 1
                        emit(Step(
                            raw=f"executive_goal:{stage}",
                            action=call,
                            result={**payload, "world_state": estimator.state.public()},
                        ))
                        if not ok:
                            failed_index = index
                            structured_failure_skill = stage
                            structured_failure_reason = nested.get("reason")
                            failure_code = nested.get("failure_code")
                            recovery = nested.get("recommended_recovery")
                            retries = structured_retries.get(stage, 0)
                            failure_signature = (
                                stage,
                                str(failure_code or "") or None,
                                str(nested.get("required_state") or "") or None,
                                _retry_state_signature(estimator.state),
                            )
                            unexecuted = [
                                (name, dict(args))
                                for name, args in fused_program[index + 1 :]
                            ]
                            if failure_code == "GRASP_NOT_ACQUIRED" and stage == "pick" and task_executive.strict_pick_preconditions:
                                structured_failed = True
                            elif (
                                recovery in registry.names()
                                and recovery != stage
                                and retries < _skill_retry_budget(stage)
                                and failure_signature not in structured_failure_signatures
                                and tool_count < max_steps
                            ):
                                structured_retries[stage] = retries + 1
                                structured_failure_signatures.add(failure_signature)
                                recovery_chain = _acquisition_recovery_chain(
                                    recovery, stage, dict(stage_args)
                                )
                                structured_queue[:] = [
                                    *recovery_chain,
                                    *unexecuted,
                                    *remainder,
                                ]
                                structured_failure_skill = None
                                structured_failure_reason = None
                            else:
                                structured_failed = True
                            break
                    if failed_index is None:
                        if len(stage_values) != len(fused_program):
                            structured_failed = True
                            structured_failure_skill = fused_program[len(stage_values)][0]
                            structured_failure_reason = "fused program returned incomplete stage results"
                        else:
                            structured_queue[:] = remainder
                    continue
            if fused_program is None:
                next_tool, next_args = structured_queue.pop(0)
                action = ToolCall(name=next_tool, args=next_args, say=None)
                raw = f"executive_goal:{next_tool}"
        elif queue_run_enabled and not tools_exhausted and skill_queue.has_pending() and not skill_queue.paused():
            queued = skill_queue.claim_next(expected_epoch=skill_queue.snapshot().get("plan_epoch"))
            if queued is None:
                queue_run_enabled = False
                continue
            queue_entry_id = queued.id
            action = ToolCall(name=queued.tool, args=dict(queued.args), say=queued.say)
            raw = f"action_queue:{queued.id}:{queued.tool}"
        else:
            sent_image = image if image_pending else None
            if sent_image:
                sent_image = prepare_planner_image(
                    sent_image,
                    mask_masterpi_self=(
                        os.environ.get("UGRP_CAMERA_SELF_MASK") == "masterpi_eye_in_hand"
                    ),
                    max_edge=planner_image_max_edge,
                )
            model_messages = messages
            if auto_observe:
                # Embodied planning is state-based, not transcript-based. The
                # current world_state already carries last_action, bounded recent
                # action evidence and the authoritative queue. Re-sending raw
                # queue_plan_accepted/tool_result text duplicates those facts and
                # makes prompt size grow after every recovery until Groq TPM is
                # exceeded. Keep only one prior conversational item plus the
                # immutable mission; current execution evidence is supplied below
                # as one bounded world_state snapshot.
                prior = messages[1:mission_index][-1:]
                model_messages = [messages[0], *prior, messages[mission_index]]
            if planner_context_provider is not None:
                # Multi-robot messages are live evidence. Read them immediately
                # before every model decision, after any embodied-context
                # compaction. Do not append this ephemeral system message to
                # persistent history, otherwise old peer evidence accumulates.
                live_planner_context = planner_context_provider()
                if live_planner_context:
                    if team_id is None and not (talk_only or conversation_only):
                        # The team rules need the robot's own id, which only the
                        # live provider knows. Install them on the first decision.
                        team_id = str(live_planner_context.get("self_id") or "").strip().lower() or None
                        if team_id:
                            messages[0] = {"role": "system", "content": system_prompt(
                                registry.prompt_schema(), talk_only=talk_only, auto_observe=auto_observe,
                                conversation_only=conversation_only, team_id=team_id,
                            )}
                            model_messages = [messages[0], *model_messages[1:]]
                    live_context_message = {
                        "role": "system",
                        "content": "multi_robot_peer_context: " + json.dumps(live_planner_context, ensure_ascii=False),
                    }
                    model_messages = [model_messages[0], live_context_message, *model_messages[1:]]
            planner_state = _planner_decision_context(
                estimator.state, goal_spec, recent_action_evidence,
                registry=registry, executive=task_executive, action_queue=skill_queue,
            )
            if not conversation_only:
                state_message = {
                    "role": "user",
                    "content": "world_state: " + json.dumps(planner_state, ensure_ascii=False),
                }
                model_messages = [*model_messages, state_message]
            if planner_call_count >= max_planner_calls:
                text = _planner_budget_final(estimator.state, skill_queue)
                action = FinalAnswer(text=text)
                raw = json.dumps({"final": text}, ensure_ascii=False)
                emit(Step(raw=raw, action=action))
                result.final = text
                result.stopped = "planner_budget"
                return result
            planner_call_count += 1
            if on_activity is not None:
                on_activity("planning", None)
            if on_planner_input is not None:
                on_planner_input(sent_image, planner_state)
            try:
                raw = completer.complete(model_messages, image=sent_image)
            except VlmError as exc:
                if executive_fallback_on_planner_error and execute and not conversation_only:
                    fallback_calls = task_executive.plan_calls(
                        goal_spec, registry.names(), estimator.state
                    )
                    if fallback_calls:
                        skill_queue.replace_pending(
                            fallback_calls, source="team_planner_fallback"
                        )
                        queue_run_enabled = True
                        image_pending = False
                        fallback_action = Plan(
                            steps=tuple(name for name, _args in fallback_calls), say=None
                        )
                        emit(Step(
                            raw=f"executive_planner_fallback:{type(exc).__name__}",
                            action=fallback_action,
                            result={
                                "ok": True,
                                "fallback": "executive_goal_compile",
                                "planner_error": str(exc),
                                "plan": [name for name, _args in fallback_calls],
                                "action_queue": _queue_snapshot(
                                    skill_queue, estimator.state, task_executive
                                ),
                            },
                        ))
                        continue
                raise
            image_pending = False
            messages.append({"role": "assistant", "content": raw})
            try:
                action = parse_action(raw)
            except ProtocolError as exc:
                if conversation_only and raw.strip():
                    # Conversation/VLM answers should never disappear merely
                    # because a model forgot the JSON envelope. Tool-mode keeps
                    # strict protocol parsing; no actuator command can enter here.
                    text = raw.strip()
                    if text.startswith("```") and text.endswith("```"):
                        lines = text.splitlines()
                        if len(lines) >= 3:
                            text = "\n".join(lines[1:-1]).strip()
                    action = FinalAnswer(text=text)
                else:
                    consecutive_protocol_errors += 1
                    emit(Step(raw=raw, action=None, error=str(exc)))
                    if consecutive_protocol_errors >= MAX_CONSECUTIVE_PROTOCOL_ERRORS:
                        # A model that cannot produce the JSON envelope three
                        # times in a row will not recover by being told again;
                        # each retry costs a planner call and camera time.
                        text = "모델 응답 형식 오류가 반복돼서 이 turn을 멈췄어. 다시 요청해 줘."
                        result.final = text
                        result.stopped = "protocol_error"
                        emit(Step(raw=json.dumps({"final": text}, ensure_ascii=False), action=FinalAnswer(text=text)))
                        return result
                    messages.append({"role": "user", "content": f"error: {exc}. Reply with one JSON object."})
                    continue
            consecutive_protocol_errors = 0
        if isinstance(action, FinalAnswer):
            if structured_active:
                emit(Step(raw=raw, action=action))
                result.final = action.text
                result.stopped = "final"
                return result
            if needs_observation:
                emit(
                    Step(
                        raw=raw,
                        action=action,
                        error="must wait/look and observe after the last tool before final",
                    )
                )
                messages.append(
                    {
                        "role": "user",
                        "content": (
                            "error: you have not observed the scene after the last tool. "
                            "Wait or look first, then judge whether the goal is complete."
                        ),
                    }
                )
                continue
            sensor_goal_status = goal_status(goal_spec, estimator.state)
            if (
                execute
                and tool_count == 0
                and not zero_tool_final_rejected
                and goal_spec.kind in {"SEARCH", "GRASP", "LIFT", "PLACE"}
                and sensor_goal_status != "ACHIEVED"
                and not final_is_non_success(action.text)
            ):
                # Live R3 trace 20260902T075829Z: the model opened the turn with
                # "배치 완료했습니다 … 넘겼습니다" without running any skill. A
                # success claim before the first tool is never sensor-backed;
                # push back once instead of letting the TEAM stage silently fail.
                zero_tool_final_rejected = True
                emit(Step(raw=raw, action=action, error="success claimed before any tool ran"))
                messages.append({
                    "role": "user",
                    "content": (
                        "executive_goal_gate: no tool has run in this turn, so nothing is done yet. "
                        "Run the required skill now, or report explicitly that you did not act."
                    ),
                })
                image_pending = False
                continue
            if (
                execute
                and contract_mode
                and structured_outcome_seen
                and not authoritative_completion
                and tool_count > 0
                and goal_spec.kind in {"GRASP", "LIFT", "PLACE"}
                and sensor_goal_status != "ACHIEVED"
                and not final_is_non_success(action.text)
            ):
                evidence = (
                    f"goal={goal_spec.kind}, sensor_goal_status={sensor_goal_status}, "
                    f"grasp_state={estimator.state.grasp.state}"
                )
                emit(Step(raw=raw, action=action, error=f"goal predicate not satisfied: {evidence}"))
                messages.append({
                    "role": "user",
                    "content": (
                        "executive_goal_gate: do not claim physical success. "
                        f"{evidence}. Use only non-destructive sensing/verification if available; "
                        "otherwise report that completion cannot be verified or that the task failed. "
                        "Do not repeat disruptive motion just to force a success claim."
                    ),
                })
                image_pending = False
                continue
            # A sensor-verified goal is the same truth that auto-finalizes a
            # queued goal; asking the visual verifier again only rejects a
            # correct report when the camera no longer frames the stack site.
            if (
                verify_final is not None and image and tool_count > 0
                and not authoritative_completion
                and not (execute and goal_spec.kind in {"GRASP", "LIFT", "PLACE"} and sensor_goal_status == "ACHIEVED")
            ):
                accepted, evidence = verify_final(user_text, action.text, image)
                if not accepted:
                    emit(
                        Step(
                            raw=raw,
                            action=action,
                            error=f"final verification failed: {evidence}",
                        )
                    )
                    messages.append(
                        {
                            "role": "user",
                            "content": (
                                "independent_verifier: completion is not visually verified. "
                                f"Evidence: {evidence}. Continue from the current scene, "
                                "diagnose the failure, and change course if needed."
                            ),
                        }
                    )
                    # The verifier already consumed the latest scene. Keep
                    # its evidence in text; the actor can request an explicit
                    # look if raw pixels are needed for another decision.
                    fresh_observation = True
                    image_pending = False
                    continue
            emit(Step(raw=raw, action=action))
            result.final = action.text
            result.stopped = "final"
            return result
        if talk_only:
            emit(
                Step(
                    raw=raw,
                    action=action,
                    error="robot is offline; reply with final",
                )
            )
            messages.append(
                {
                    "role": "user",
                    "content": (
                        'error: the robot is not connected. '
                        'Reply with {"final": "..."} only. Do not call tools.'
                    ),
                }
            )
            continue
        if conversation_only:
            emit(
                Step(
                    raw=raw,
                    action=action,
                    error="conversation-only turn; reply with final",
                )
            )
            messages.append(
                {
                    "role": "user",
                    "content": (
                        'error: this is a conversation-only turn. '
                        'Reply with {"final": "..."} only. Do not call tools.'
                    ),
                }
            )
            continue
        if isinstance(action, Plan):
            try:
                calls = _plan_queue_calls(action.steps, registry, goal_spec, estimator.state)
            except (ProtocolError, ToolError) as exc:
                emit(Step(raw=raw, action=action, error=str(exc)))
                messages.append({
                    "role": "user",
                    "content": f"queue_plan_rejected: {exc}. Submit a corrected plan or an explicit tool call with required args.",
                })
                continue
            skill_queue.replace_pending(calls, source="llm_plan")
            queue_run_enabled = True
            fresh_observation = False
            pending = skill_queue.pending_names()
            snapshot = _queue_snapshot(skill_queue, estimator.state, task_executive)
            planner_snapshot = _queue_snapshot(
                skill_queue, estimator.state, task_executive, compact=True
            )
            emit(Step(raw=raw, action=action, result={
                "ok": True, "plan": list(pending), "action_queue": snapshot,
            }))
            messages.append({
                "role": "user",
                "content": (
                    "queue_plan_accepted: " + json.dumps(planner_snapshot, ensure_ascii=False) +
                    ". The executor now owns queue order and will run only the head after rechecking its current preconditions. "
                    "Do not resubmit queued tools individually; you will be asked again only if the queue drains, blocks, or fails."
                ),
            })
            continue
        if isinstance(action, Wait):
            sleeper(action.seconds)
            observed = refresh(attach_image=False)
            if observed:
                needs_observation = False
            payload = {"ok": True, "waited": action.seconds, "observed": observed}
            emit(Step(raw=raw, action=action, result=payload))
            messages.append({"role": "user", "content": f"tool_result: {payload}"})
            continue
        if isinstance(action, Look):
            observed = refresh(attach_image=True)
            if observed:
                needs_observation = False
            payload = {"ok": True, "observed": observed, "note": action.note}
            emit(Step(raw=raw, action=action, result=payload))
            messages.append({"role": "user", "content": f"tool_result: {payload}"})
            continue
        if tools_exhausted:
            text = _planner_budget_final(estimator.state, skill_queue)
            final_action = FinalAnswer(text=text)
            emit(Step(raw=raw, action=action, error="tool limit reached; stopping without another tool"))
            emit(Step(
                raw=json.dumps({"final": text}, ensure_ascii=False),
                action=final_action,
            ))
            result.final = text
            result.stopped = "tool_budget"
            return result
        replanned_from: list[str] | None = None
        if isinstance(action, ToolCall) and not structured_active and queue_entry_id is None:
            # Every model-selected public skill enters the same authoritative
            # queue before execution. If a prior queued head FAILED/BLOCKED, its
            # untouched later steps were planned under invalidated assumptions.
            # A direct recovery therefore replaces those stale future steps; the
            # actor replans again after the recovery instead of auto-running an
            # old destructive action such as pick.
            try:
                tool = registry.get(action.name)
                normalized_args = validate_args(tool, action.args)
            except ToolError as exc:
                emit(Step(raw=raw, action=action, error=str(exc)))
                messages.append({"role": "user", "content": f"error: {exc}. Pick an allowlisted tool."})
                continue
            if skill_queue.paused():
                skill_queue.replace_pending(
                    [(action.name, normalized_args)], source="llm_recovery"
                )
            elif skill_queue.has_pending():
                skill_queue.enqueue_front(
                    action.name, normalized_args, source="llm_tool", say=action.say
                )
            else:
                skill_queue.enqueue(
                    action.name, normalized_args, source="llm_tool", say=action.say
                )
            queue_run_enabled = True
            pending = skill_queue.pending_names()
            messages.append({
                "role": "user",
                "content": "tool_queued: " + json.dumps(
                    _queue_snapshot(skill_queue, estimator.state, task_executive, compact=True), ensure_ascii=False
                ),
            })
            continue
        # Design intent: Executive is a safety/validity gate, not a planner.
        # In the default VLM mode it may reject an unsafe/impossible action, but
        # it must not turn the rejection into a hidden situation->action policy.
        # The legacy deterministic baseline can opt back into automatic recovery
        # only together with the explicit structured fastpath.
        recoveries: list[str] = []
        rejected_payload: dict[str, Any] | None = None
        while True:
            decision = task_executive.check(action.name, estimator.state, action.args)
            if decision.allowed:
                break
            if not structured_active:
                rejected_payload = task_executive.rejection_payload(action.name, decision)
                # Keep the internal recommendation available to deterministic
                # baselines/debuggers, but do not reveal it to the autonomous VLM.
                rejected_payload.pop("recommended_recovery", None)
                estimator.update_tool_result(rejected_payload)
                break
            recovery = decision.recovery
            if (
                not recovery
                or recovery not in registry.names()
                or recovery in recoveries
                or tool_count >= max_steps
            ):
                rejected_payload = task_executive.rejection_payload(action.name, decision)
                estimator.update_tool_result(rejected_payload)
                break

            recovery_args = {}
            if recovery in {"search", "track", "approach"} and isinstance(action.args, dict):
                color = action.args.get("target_color")
                if color in {"red", "blue", "yellow"}:
                    recovery_args = {"target_color": color}
            recovery_call = ToolCall(name=recovery, args=recovery_args, say=None)
            if on_activity is not None:
                on_activity("tool_start", recovery)
            try:
                recovery_payload = dispatch(registry, recovery_call, execute=execute)
            except ToolError as exc:
                rejected_payload = task_executive.rejection_payload(action.name, decision)
                rejected_payload["reason"] = f"recovery {recovery} could not run: {exc}"
                estimator.update_tool_result(rejected_payload)
                break
            recoveries.append(recovery)
            tool_count += 1
            nested_recovery = recovery_payload.get("result") if isinstance(recovery_payload.get("result"), dict) else recovery_payload
            if isinstance(nested_recovery, dict) and any(k in nested_recovery for k in ("command_status", "execution_status", "outcome_status")):
                structured_outcome_seen = True
            estimator.update_tool_result(recovery_payload)
            fresh_observation = False
            if observe is not None:
                needs_observation = True
            if auto_observe and observe is not None:
                observed = refresh(attach_image=False)
                recovery_payload = {**recovery_payload, "observed": observed}
                if observed:
                    needs_observation = False
            elif structured_active:
                needs_observation = False
            recovery_payload = {
                **recovery_payload,
                "executive_recovery_for": action.name,
                "world_state": estimator.state.public(),
            }
            emit(Step(raw=f"executive_recovery:{recovery}", action=recovery_call, result=recovery_payload))
            if recovery_payload.get("ok") is False:
                rejected_payload = task_executive.rejection_payload(action.name, decision)
                rejected_payload["reason"] = f"recovery {recovery} failed"
                estimator.update_tool_result(rejected_payload)
                break

        if rejected_payload is not None:
            if queue_entry_id is not None:
                skill_queue.block_running(
                    failure_code=rejected_payload.get("failure_code"),
                    required_state=rejected_payload.get("required_state"),
                    reason=rejected_payload.get("reason") or "skill precondition failed",
                )
                queue_run_enabled = False
            if structured_active:
                structured_queue.clear()
                structured_failed = True
                structured_failure_skill = action.name
                structured_failure_reason = rejected_payload.get("reason")
            payload = {
                "ok": False,
                "dry_run": not execute,
                "tool": action.name,
                "result": rejected_payload,
                "world_state": estimator.state.public(),
            }
            emit(Step(raw=raw, action=action, result=payload, error=f"precondition failed: {rejected_payload.get('failure_code')}"))
            # Planner gets facts needed to reason about feasibility, not an
            # executive-authored policy hint. Human/debug payloads still retain
            # the full reason/recovery fields outside the model context.
            planner_rejection = {
                key: value for key, value in rejected_payload.items()
                if key not in {"recommended_recovery", "reason"}
            }
            # Report repeated infeasible proposals as objective loop state, not
            # as an executive-selected recovery. We intentionally do NOT name
            # another tool here. This keeps action choice with the VLM while
            # making "same rejected proposal, same state" visible instead of
            # silently burning planner turns until max_iters.
            public_state = estimator.state.public()
            target = public_state.get("target") or {}
            grasp = public_state.get("grasp") or {}
            task = public_state.get("task") or {}
            feasibility_state = json.dumps(
                {
                    "target": {
                        "visible": target.get("visible"),
                        "centered": target.get("centered"),
                        "range_class": target.get("range_class"),
                    },
                    "grasp": {
                        "state": grasp.get("state"),
                        "held_object_color": grasp.get("held_object_color"),
                    },
                    "task": {"phase": task.get("phase")},
                },
                sort_keys=True,
                separators=(",", ":"),
            )
            rejection_signature = (
                action.name,
                rejected_payload.get("failure_code"),
                rejected_payload.get("required_state"),
                feasibility_state,
            )
            if rejection_signature == last_rejection_signature:
                repeated_rejection_count += 1
            else:
                last_rejection_signature = rejection_signature
                repeated_rejection_count = 1
            if repeated_rejection_count >= 2:
                planner_rejection["stagnation"] = {
                    "same_rejected_action_count": repeated_rejection_count,
                    "relevant_state_changed": False,
                    "fact": (
                        "The identical action has been rejected again under the "
                        "same relevant state and cannot execute while its required_state "
                        "remains unsatisfied."
                    ),
                }
            messages.append({
                "role": "user",
                "content": "executive_result: " + json.dumps(planner_rejection, ensure_ascii=False),
            })
            continue

        last_rejection_signature = None
        repeated_rejection_count = 0
        before_action = _planner_decision_context(
            estimator.state, goal_spec, [], action_queue=skill_queue,
        )
        if on_activity is not None:
            on_activity("tool_start", action.name)
        try:
            payload = dispatch(registry, action, execute=execute)
        except ToolError as exc:
            if queue_entry_id is not None:
                skill_queue.fail_running(failure_code="TOOL_ERROR", reason=str(exc))
                queue_run_enabled = False
            emit(Step(raw=raw, action=action, error=str(exc)))
            messages.append({"role": "user", "content": f"error: {exc}. Pick an allowlisted tool."})
            continue
        nested_payload = payload.get("result") if isinstance(payload.get("result"), dict) else payload
        if isinstance(nested_payload, dict) and any(k in nested_payload for k in ("command_status", "execution_status", "outcome_status")):
            structured_outcome_seen = True
        estimator.update_tool_result(payload)
        if queue_entry_id is not None:
            nested = nested_payload if isinstance(nested_payload, dict) else {}
            failed = (
                payload.get("ok") is False
                or nested.get("execution_status") == "FAILED"
                or nested.get("outcome_status") == "NOT_ACHIEVED"
            )
            if failed:
                skill_queue.fail_running(
                    failure_code=nested.get("failure_code"),
                    reason=nested.get("reason") or "queued skill did not achieve its postcondition",
                    outcome_status=nested.get("outcome_status"),
                )
                queue_run_enabled = False
                recovery = nested.get("recommended_recovery")
                target_color = str((action.args or {}).get("target_color") or "")
                retry_key = (action.name, target_color)
                retries = queued_recovery_retries.get(retry_key, 0)
                failure_signature = (
                    action.name,
                    target_color,
                    str(nested.get("failure_code") or "") or None,
                    str(nested.get("required_state") or "") or None,
                    _retry_state_signature(estimator.state),
                )
                same_failure_without_progress = failure_signature in queued_failure_signatures
                strict_failed_pick = (
                    action.name == "pick"
                    and nested.get("failure_code") == "GRASP_NOT_ACQUIRED"
                    and task_executive.strict_pick_preconditions
                )
                if (
                    auto_recommended_recovery
                    and isinstance(recovery, str)
                    and recovery in registry.names()
                    and recovery != action.name
                    and retries < _skill_retry_budget(action.name)
                    and not same_failure_without_progress
                    and not strict_failed_pick
                    and tool_count < max_steps
                ):
                    # fail_running archives the failed head and pauses the old
                    # tail. Rebuild a fresh causal prefix ahead of that tail so
                    # a known physical recovery does not require another LLM
                    # call merely to name the action the driver already advised.
                    tail_snapshot = skill_queue.snapshot().get("pending", [])
                    tail = [
                        (str(item.get("tool")), dict(item.get("args") or {}))
                        for item in tail_snapshot
                        if item.get("tool")
                    ]
                    recovery_chain = _queued_recovery_chain(
                        registry, recovery, action.name, dict(action.args or {})
                    )
                    queued_recovery_retries[retry_key] = retries + 1
                    queued_failure_signatures.add(failure_signature)
                    skill_queue.replace_pending(
                        [*recovery_chain, *tail], source="team_recovery"
                    )
                    queue_run_enabled = True
            else:
                skill_queue.complete_running(outcome_status=nested.get("outcome_status"))
        pending = skill_queue.pending_names()
        tool_count += 1
        fresh_observation = False
        if observe is not None:
            needs_observation = True
        if auto_observe and observe is not None:
            # SIM and REAL both take a fresh ordered camera observation after
            # every embodied action. The full-resolution frame updates the same
            # deterministic world_state in both modes; raw pixels are attached
            # to the LLM only for the initial decision or an explicit Look.
            observed = refresh(attach_image=False)
            payload = {**payload, "observed": observed}
            if observed:
                needs_observation = False
        elif structured_active:
            needs_observation = False
        payload = {**payload, "world_state": estimator.state.public()}
        progress_evidence = record_executed_action(action.name, before_action, payload)
        payload = {**payload, "progress_evidence": progress_evidence}
        authoritative_completion = _authoritative_completion(payload)
        if authoritative_completion:
            # Keep the freshly captured scene for UI/history, but do not resend
            # a redundant image to the VLM. Objective contact/height/stability
            # facts are already stronger completion evidence. The actor may use
            # an explicit look if a later subtask genuinely needs vision.
            image_pending = False
        if replanned_from:
            payload = {**payload, "replanned_from": replanned_from}
        if tool_count >= max_steps:
            tools_exhausted = True
        emit(Step(raw=raw, action=action, result=payload))
        # A sensor-satisfied queued goal is terminal without spending another
        # provider call merely to verbalize truth the executor already has. This
        # is especially important for TEAM turns during provider rate limits.
        if (
            execute
            and not structured_active
            and tool_count > 0
            and goal_spec.kind in {"SEARCH", "GRASP", "LIFT", "PLACE"}
            and not skill_queue.has_pending()
            and not skill_queue.paused()
            and goal_status(goal_spec, estimator.state) == "ACHIEVED"
        ):
            if (
                team_id
                and not team_handoff_grace_used
                and not tools_exhausted
                and action.name not in {"send_peer_message", "ack_peer_message"}
            ):
                # LLM-first collaboration: the peer handoff is the planner's
                # decision, so give it exactly one more step to compose
                # send_peer_message before the sensor-verified goal closes the
                # turn. Without this the turn ended here and every handoff
                # came from the router fallback.
                team_handoff_grace_used = True
                team_staged_base = action.name == "stage_base"
                messages.append({"role": "user", "content": "tool_result: " + json.dumps(_planner_tool_result(payload), ensure_ascii=False)})
                messages.append({
                    "role": "user",
                    "content": (
                        "goal_status: ACHIEVED (sensor-verified). Your own step is done. "
                        "If your mission hands the next step to a peer, call it now exactly as "
                        '{"tool": "send_peer_message", "args": {"recipient": "r1", "message": "..."}}. '
                        'Otherwise reply {"final": "..."}. Do not run another robot skill.'
                    ),
                })
                continue
            color = {
                "RED_BLOCK": "빨간", "BLUE_BLOCK": "파란", "YELLOW_BLOCK": "노란",
            }.get(goal_spec.subject, "목표")
            if goal_spec.kind == "SEARCH":
                text = f"{color} 블록을 찾았어."
            elif goal_spec.kind == "GRASP":
                text = f"{color} 블록 집기를 완료했어."
            elif goal_spec.kind == "LIFT":
                text = f"{color} 블록 들어 올리기를 완료했어."
            elif action.name == "stage_base" or team_staged_base:
                text = f"{color} 블록을 공용 스택 지점 바닥층에 놓았어."
            else:
                destination = {
                    "RED_BLOCK": "빨간", "BLUE_BLOCK": "파란", "YELLOW_BLOCK": "노란",
                }.get(goal_spec.target, "목표")
                text = f"{color} 블록을 {destination} 블록 위치에 배치했어."
            final_action = FinalAnswer(text=text)
            emit(Step(
                raw=json.dumps({"final": text}, ensure_ascii=False),
                action=final_action,
            ))
            result.final = text
            result.stopped = "goal_achieved"
            return result
        if structured_active and action.name == "carry" and estimator.state.grasp.state == "EMPTY":
            retries = structured_retries.get("visual_regrasp", 0)
            fresh_plan = task_executive.plan_calls(goal_spec, registry.names(), estimator.state)
            if retries < 1 and fresh_plan and tool_count + len(fresh_plan) <= max_steps:
                structured_retries["visual_regrasp"] = retries + 1
                structured_queue[:] = fresh_plan
            else:
                structured_queue.clear()
                structured_failed = True
        if structured_active and payload.get("ok") is False:
            nested = payload.get("result") if isinstance(payload.get("result"), dict) else {}
            structured_failure_skill = action.name
            structured_failure_reason = nested.get("reason") if isinstance(nested, dict) else None
            recovery = nested.get("recommended_recovery") if isinstance(nested, dict) else None
            failure_code = nested.get("failure_code") if isinstance(nested, dict) else None
            retries = structured_retries.get(action.name, 0)
            failure_signature = (
                action.name,
                str(failure_code or "") or None,
                str(nested.get("required_state") or "") or None,
                _retry_state_signature(estimator.state),
            )
            if failure_code == "SIM_EPISODE_RESET":
                episode_retries = structured_retries.get("__sim_episode_reset__", 0)
                fresh_plan = task_executive.plan_calls(goal_spec, registry.names(), estimator.state)
                if episode_retries < 1 and fresh_plan:
                    # A replacement GPU process owns a completely fresh MuJoCo
                    # world. Discard all temporal evidence and restart the known
                    # goal from step 1; never replay a middle primitive such as
                    # carry/place into the new episode.
                    structured_retries["__sim_episode_reset__"] = episode_retries + 1
                    estimator.reset()
                    structured_queue[:] = fresh_plan
                    pending.clear()
                    tool_count = 0
                    tools_exhausted = False
                    authoritative_completion = False
                    structured_outcome_seen = False
                    structured_failure_skill = None
                    structured_failure_reason = None
                    needs_observation = False
                    fresh_observation = False
                else:
                    structured_queue.clear()
                    structured_failed = True
            elif failure_code == "GRASP_NOT_ACQUIRED" and (action.name == "carry" or action.name == "pick" or action.name.startswith("pick_")):
                # A destructive grasp proven to miss may have pushed/reoriented
                # the object and leaves the eye-in-hand arm in a changed near-field
                # state. SIM must preserve the same task-level safety semantics as
                # REAL: never start a fresh autonomous acquisition/grasp pipeline
                # in the same user turn after that miss. Robustness experiments
                # should reset/replay an episode explicitly instead of silently
                # changing the physical initial condition via an automatic retry.
                structured_queue.clear()
                structured_failed = True
            elif (
                recovery in registry.names()
                and recovery != action.name
                and retries < _skill_retry_budget(action.name)
                and failure_signature not in structured_failure_signatures
                and tool_count < max_steps
            ):
                structured_retries[action.name] = retries + 1
                structured_failure_signatures.add(failure_signature)
                structured_queue[:0] = _acquisition_recovery_chain(
                    recovery, action.name, dict(action.args)
                )
            else:
                structured_queue.clear()
                structured_failed = True
        follow = "tool_result: " + json.dumps(_planner_tool_result(payload), ensure_ascii=False)
        pending = skill_queue.pending_names()
        if pending:
            follow += (
                " action_queue=" + json.dumps(
                    _queue_snapshot(skill_queue, estimator.state, task_executive, compact=True), ensure_ascii=False
                )
            )
            if queue_run_enabled and not skill_queue.paused():
                follow += " The executor will recheck and run the next queued head automatically."
            else:
                follow += " Queue execution is paused; inspect the failure/block and revise or prepend recovery work."
        elif tools_exhausted:
            follow += ' Tool limit reached. Reply with {"final": "..."}.'
        elif authoritative_completion:
            follow += (
                " Objective physical completion is authoritative. If these facts satisfy the user's requested goal, "
                "reply with final now and do not disturb the completed state. No extra image call is required; use look only if a different remaining subtask needs vision."
            )
        elif auto_observe:
            follow += (
                " Fresh post-action camera evidence has updated world_state. "
                "Raw pixels are not attached automatically; request {\"look\":true} only if they are genuinely needed."
            )
        else:
            follow += " Wait and look before the next tool if the robot moved."
        messages.append({"role": "user", "content": follow})
    result.pending_plan = skill_queue.pending_names()
    result.world_state = estimator.state.public()
    result.action_queue = _queue_snapshot(skill_queue, estimator.state, task_executive)
    result.stopped = "max_steps" if tools_exhausted else "max_iters"
    return result


def _authoritative_completion(payload: dict[str, Any]) -> bool:
    if payload.get("authoritative_completion") is True:
        return True
    nested = payload.get("result")
    return bool(isinstance(nested, dict) and nested.get("authoritative_completion") is True)


def _plan_tools(steps: tuple[str, ...], tool_names: list[str]) -> list[str]:
    pending: list[str] = []
    for step in steps:
        token = resolve_plan_step(step, tool_names)
        if token:
            pending.append(token)
    return pending


def dispatch(registry: Registry, call: ToolCall, *, execute: bool) -> dict[str, Any]:
    tool = registry.get(call.name)
    args = validate_args(tool, call.args)
    if not execute:
        return {"ok": True, "dry_run": True, "tool": tool.name, "args": args}
    try:
        value = tool.handler(**args)
    except ToolError:
        raise
    except Exception as exc:
        # A tool implementation bug/import failure must not tear down the HTTP
        # request thread and leave the UI stuck at "tool started" forever.
        # Keep the traceback in the service journal for diagnosis, but return a
        # bounded structured failure to the loop/UI.
        traceback.print_exc()
        failure = {
            "ok": False,
            "command_status": "ACCEPTED",
            "execution_status": "FAILED",
            "outcome_status": "NOT_ACHIEVED",
            "failure_code": "TOOL_EXCEPTION",
            "reason": f"{type(exc).__name__}: {exc}",
            "recommended_recovery": None,
        }
        return {"ok": False, "dry_run": False, "tool": tool.name, "result": failure}
    if isinstance(value, dict):
        ok = value.get("ok") is not False
        return {"ok": ok, "dry_run": False, "tool": tool.name, "result": value}
    return {"ok": True, "dry_run": False, "tool": tool.name, "result": {"value": value}}
