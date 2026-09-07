"""Small, semantic context for camera-only transport model calls."""
from __future__ import annotations

import copy
import json
from collections.abc import Mapping
from typing import Any


_PLACEMENT_KEYS = {"status", "reason", "stage", "source", "cargo_id", "destination_zone"}
_ACTION_KEYS = {"kind", "fwd", "turn", "duration"}
_CRITICAL_MARKERS = ("ERROR", "FAILED", "FAILURE", "BLOCKED", "UNCERTAIN", "LIMIT", "REJECT")
_SEMANTIC_FEEDBACK_MARKERS = ("NAVIGATION_INTERRUPTED", "OWN_RGB_STAGNATION",
                              "NEW_PEER_MESSAGE", "VISUAL_LOAD_")


def compact_own_memory(memory: Any) -> dict[str, Any]:
    """Keep bounded decision semantics, excluding frame metadata and geometry."""
    records = list(memory) if isinstance(memory, (list, tuple)) else [memory]
    placement: dict[str, Any] | None = None
    feedback: list[Any] = []
    actions: list[dict[str, Any]] = []

    for record in records:
        for node in _walk(record):
            if not isinstance(node, Mapping):
                continue
            candidate = node.get("placement", node.get("placement_evidence"))
            if isinstance(candidate, Mapping):
                placement = _pick_scalars(candidate, _PLACEMENT_KEYS)
                identity = candidate.get("identity")
                if isinstance(identity, Mapping):
                    confirmed = identity.get("confirmed")
                    if isinstance(confirmed, bool):
                        placement["identity"] = {"confirmed": confirmed}
            elif {"status", "stage"}.issubset(node) and "placement" in str(node.get("source", "")).lower():
                placement = _pick_scalars(node, _PLACEMENT_KEYS)

            action = node.get("action", node.get("own_action"))
            if isinstance(action, Mapping) and isinstance(action.get("kind"), str):
                actions.append(_pick_scalars(action, _ACTION_KEYS))
            elif isinstance(action, str) and action:
                actions.append({"kind": action[:64]})

            for key, value in node.items():
                if _is_critical(key, value):
                    item = _compact_feedback(key, value)
                    if item is not None:
                        feedback.append(item)

    result: dict[str, Any] = {}
    if placement:
        result["latest_placement"] = placement
    if feedback:
        result["critical_feedback"] = _latest_unique(feedback, 4)
    if actions:
        result["last_actions"] = actions[-3:]
    # A fixed limit guards unexpected but valid long reason strings.
    encoded = json.dumps(result, ensure_ascii=False, allow_nan=False)
    if len(encoded) > 3000:
        for item in result.get("critical_feedback", []):
            if isinstance(item, dict):
                for key, value in list(item.items()):
                    if isinstance(value, str):
                        item[key] = value[:240]
    return copy.deepcopy(result)


def memory_needs_nav_comparison(compact_memory: Mapping[str, Any]) -> bool:
    """Return true only for a current critical error that benefits from before/after RGB."""
    text = json.dumps(compact_memory.get("critical_feedback", []), ensure_ascii=False).upper()
    return any(marker in text for marker in ("OWN_RGB_DRIVE_BLOCKED", "STAGN", "NO_PROGRESS", "DRIVE_FAILED"))


def _walk(value: Any):
    yield value
    if isinstance(value, Mapping):
        for child in value.values():
            yield from _walk(child)
    elif isinstance(value, (list, tuple)):
        for child in value:
            yield from _walk(child)


def _pick_scalars(value: Mapping[str, Any], allowed: set[str]) -> dict[str, Any]:
    return {key: copy.deepcopy(item) for key, item in value.items()
            if key in allowed and isinstance(item, (str, int, float, bool, type(None)))}


def _is_critical(key: Any, value: Any) -> bool:
    label = str(key).upper()
    scalar = str(value).upper() if isinstance(value, (str, int, bool)) else ""
    if label == "FEEDBACK" and scalar in {"ACCEPTED", "OK", "SUCCESS"}:
        return False
    return (key in {"feedback", "visual_feedback", "error", "failure", "warning", "limit_feedback"}
            or any(marker in label or marker in scalar
                   for marker in _CRITICAL_MARKERS + _SEMANTIC_FEEDBACK_MARKERS))


def _compact_feedback(key: Any, value: Any) -> Any:
    if isinstance(value, Mapping):
        kept = {str(k): copy.deepcopy(v) for k, v in value.items()
                if str(k) in {"code", "status", "reason", "stage", "message", "remaining"}
                and isinstance(v, (str, int, float, bool, type(None)))}
        return {str(key): kept} if kept else None
    if isinstance(value, (str, int, float, bool)):
        return {str(key): value[:500] if isinstance(value, str) else value}
    return None


def _latest_unique(items: list[Any], limit: int) -> list[Any]:
    kept: list[Any] = []
    seen: set[str] = set()
    for item in reversed(items):
        signature = json.dumps(item, ensure_ascii=False, sort_keys=True, allow_nan=False)
        if signature not in seen:
            kept.append(item)
            seen.add(signature)
        if len(kept) == limit:
            break
    return list(reversed(kept))


__all__ = ["compact_own_memory", "memory_needs_nav_comparison"]
