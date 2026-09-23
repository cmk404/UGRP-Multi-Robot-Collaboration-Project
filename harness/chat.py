"""Research turn handling over the allowlisted skill loop.

The model never writes Python. Each user goal runs run_loop until
final, with optional plan / tool / wait / look steps.
"""

from __future__ import annotations

from typing import Any, Callable

from .catalog import default_registry
from .loop import Completer, LoopResult, Step, run_loop
from .protocol import FinalAnswer, Look, Plan, ToolCall, Wait
from .registry import Registry
from .executive import TaskExecutive
from .state import StateEstimator
from .action_queue import RobotActionQueue


def spoken_text(result: LoopResult) -> str:
    parts: list[str] = []
    for step in result.steps:
        action = step.action
        if isinstance(action, (ToolCall, Plan, Wait, Look)) and action.say:
            parts.append(action.say)
        elif isinstance(action, FinalAnswer) and action.text:
            parts.append(action.text)
    if parts:
        return "\n".join(parts)
    return result.final or ""


def format_skill_log(result: LoopResult) -> str:
    lines: list[str] = []
    for step in result.steps:
        if step.error:
            lines.append(f"error: {step.error}")
            continue
        action = step.action
        payload = step.result or {}
        if isinstance(action, Plan):
            if action.say:
                lines.append(f"say: {action.say}")
            lines.append("plan: " + " → ".join(action.steps))
        elif isinstance(action, Wait):
            if action.say:
                lines.append(f"say: {action.say}")
            lines.append(f"wait: {action.seconds}s")
            if payload.get("observed"):
                lines.append(f"  observed: {payload['observed']}")
        elif isinstance(action, Look):
            if action.say:
                lines.append(f"say: {action.say}")
            lines.append("look")
            if payload.get("observed"):
                lines.append(f"  observed: {payload['observed']}")
        elif isinstance(action, ToolCall):
            mode = "execute" if payload.get("dry_run") is False else "dry-run"
            if action.say:
                lines.append(f"say: {action.say}")
            lines.append(f"[{mode}] tool: {action.name} args={action.args}")
            if payload.get("result") is not None:
                lines.append(f"  result: {payload['result']}")
            if payload.get("observed"):
                lines.append(f"  observed: {payload['observed']}")
        elif isinstance(action, FinalAnswer):
            lines.append(f"final: {action.text}")
    if result.final is None and result.stopped != "final":
        lines.append(f"stopped: {result.stopped}")
    if result.pending_plan:
        lines.append("pending_plan: " + " → ".join(result.pending_plan))
    return "\n".join(lines) if lines else "(empty)"


def history_text(result: LoopResult) -> str:
    """Keep only user-facing assistant speech in future chat context."""
    return spoken_text(result).strip()


def event_from_step(step: Step) -> dict[str, Any]:
    action = step.action
    payload = step.result or {}
    if step.error:
        return {
            "type": "error",
            "text": step.error,
            "name": action.name if isinstance(action, ToolCall) else None,
        }
    if isinstance(action, Plan):
        return {"type": "plan", "text": action.say or "", "steps": list(action.steps)}
    if isinstance(action, Wait):
        return {
            "type": "wait",
            "text": action.say or "",
            "seconds": action.seconds,
            "observed": payload.get("observed"),
        }
    if isinstance(action, Look):
        return {
            "type": "look",
            "text": action.say or action.note or "",
            "observed": payload.get("observed"),
        }
    if isinstance(action, ToolCall):
        return {
            "type": "tool",
            "text": action.say or "",
            "name": action.name,
            "args": action.args,
            "dry_run": payload.get("dry_run", True),
            "result": payload.get("result"),
            "observed": payload.get("observed"),
        }
    if isinstance(action, FinalAnswer):
        return {"type": "final", "text": action.text}
    return {"type": "error", "text": "unknown step"}


def result_payload(result: LoopResult) -> dict[str, Any]:
    events = [event_from_step(step) for step in result.steps]
    tools = [event for event in events if event.get("type") == "tool"]
    return {
        "text": spoken_text(result),
        "final": result.final,
        "stopped": result.stopped,
        "log": format_skill_log(result),
        "events": events,
        "tools": tools,
        "pending_plan": list(result.pending_plan),
        "action_queue": dict(result.action_queue),
        "world_state": result.world_state,
    }


def handle_turn(
    user_text: str,
    *,
    image: str | None = None,
    history: list[dict[str, str]] | None = None,
    completer: Completer,
    registry: Registry | None = None,
    execute: bool = False,
    max_steps: int = 12,
    observe: Callable[[], str | None] | None = None,
    on_step: Callable[[Step], None] | None = None,
    on_activity: Callable[[str, str | None], None] | None = None,
    on_planner_input: Callable[[str | None, dict[str, Any]], None] | None = None,
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
) -> tuple[str, LoopResult]:
    text = user_text.strip()
    if not text:
        empty = LoopResult(final=None, stopped="empty")
        return "error: empty message", empty
    result = run_loop(
        completer,
        registry or default_registry(),
        text,
        image=image,
        history=history,
        max_steps=max_steps,
        execute=execute,
        observe=observe,
        on_step=on_step,
        on_activity=on_activity,
        on_planner_input=on_planner_input,
        should_cancel=should_cancel,
        talk_only=talk_only,
        conversation_only=conversation_only,
        auto_observe=auto_observe,
        verify_final=verify_final,
        state_estimator=state_estimator,
        executive=executive,
        planner_context=planner_context,
        planner_context_provider=planner_context_provider,
        action_queue=action_queue,
        planner_image_max_edge=planner_image_max_edge,
        auto_recommended_recovery=auto_recommended_recovery,
        executive_fallback_on_planner_error=executive_fallback_on_planner_error,
    )
    return format_skill_log(result), result
