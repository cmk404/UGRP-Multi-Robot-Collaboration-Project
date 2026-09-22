"""Independent RGB-agent runtime for matched communication conditions.

This module intentionally knows nothing about simulator truth.  An execution
port supplies only each actor's own RGB-facing observation and local status;
the optional evaluator port is a separate interface and is never accepted by
``run_rgb_communication``.

The three study conditions share the same planner, observation, action and
budget path.  Only :class:`MessageChannel` changes its accepted message
representation.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import base64
import binascii
import copy
import hashlib
import json
import math
from pathlib import Path
import threading
import time
from typing import TYPE_CHECKING, Any, Callable, Mapping, Protocol, Sequence
import uuid

from harness.rgb_communication_clock import ExecutionClock, sanitized_error

if TYPE_CHECKING:
    # B owns this public type.  The runtime remains structurally usable before
    # the dependent execution-port PR is merged.
    from harness.rgb_execution_contract import RGBExecutionPortProtocol


SCHEMA_VERSION = "rgb-communication-event.v1"
REQUEST_SCHEMA_VERSION = "rgb-agent-request.v1"
CONDITIONS = ("none", "structured", "natural")
DEFAULT_ROBOTS = ("r1", "r2", "r3")

# These are deliberately actor-facing allowlists matching the common RGB port.
# Unknown fields fail closed before memory, planning and trace production.
OBSERVATION_FIELDS = frozenset({
    "schema",
    "robot_id",
    "observation_id",
    "observed_at_s",
    "clock_domain",
    "images",
    "static_context",
    "static_context_sha256",
    "own_issued_commands",
    "own_revision",
})
LOCAL_STATUS_FIELDS = frozenset({
    "schema",
    "robot_id",
    "clock_domain",
    "timestamp_s",
    "active",
    "pending",
    "own_submission_history",
    "own_revision",
    "own_skill_status",
})


class RGBPlanner(Protocol):
    model_name: str
    evidence_kind: str

    def decide(self, request: Mapping[str, Any]) -> Mapping[str, Any]: ...


@dataclass(frozen=True)
class RuntimeLimits:
    max_ticks: int = 12
    max_calls_per_robot: int = 12
    max_actions_per_robot: int = 12
    max_messages_per_robot: int = 12
    max_message_bytes_per_robot: int = 12_000
    max_message_ttl_s: float = 12.0
    decision_timeout_s: float = 30.0
    wall_timeout_s: float = 150.0
    tick_period_s: float = 1.0

    def validate(self) -> None:
        integer_limits = (
            self.max_ticks,
            self.max_calls_per_robot,
            self.max_actions_per_robot,
            self.max_messages_per_robot,
            self.max_message_bytes_per_robot,
        )
        time_limits = (
            self.max_message_ttl_s,
            self.decision_timeout_s,
            self.wall_timeout_s,
            self.tick_period_s,
        )
        if (any(not isinstance(value, int) or isinstance(value, bool) or value < 1
                for value in integer_limits)
                or any(not isinstance(value, (int, float)) or isinstance(value, bool)
                       or not math.isfinite(value) or value <= 0 for value in time_limits)):
            raise ValueError("INVALID_RUNTIME_LIMITS")


@dataclass
class _ActorState:
    robot_id: str
    observations: list[dict[str, Any]] = field(default_factory=list)
    trace_observations: list[dict[str, Any]] = field(default_factory=list)
    statuses: list[dict[str, Any]] = field(default_factory=list)
    received_messages: list[dict[str, Any]] = field(default_factory=list)
    expired_messages: list[dict[str, Any]] = field(default_factory=list)
    decisions: list[dict[str, Any]] = field(default_factory=list)
    action_results: list[dict[str, Any]] = field(default_factory=list)
    calls: int = 0
    actions: int = 0
    sent_messages: int = 0
    sent_message_bytes: int = 0
    finished: bool = False
    finish_claim: str | None = None

    def memory_snapshot(self) -> dict[str, Any]:
        return copy.deepcopy({
            "own_observations": self.observations[-4:],
            "own_statuses": self.statuses[-4:],
            "received_messages": self.received_messages[-12:],
            "expired_peer_claims": self.expired_messages[-12:],
            "own_decisions": self.decisions[-8:],
            "own_action_results": self.action_results[-8:],
        })


class _IdFactory:
    def __init__(self) -> None:
        self._counts: dict[str, int] = {}

    def new(self, prefix: str) -> str:
        self._counts[prefix] = self._counts.get(prefix, 0) + 1
        return f"{prefix}-{self._counts[prefix]:04d}"


class TraceRecorder:
    """Thread-safe JSONL recorder using the evaluator's minimal event schema."""

    def __init__(
        self,
        *,
        run_id: str,
        condition: str,
        wall_clock: Callable[[], float],
        wall_started_s: float,
        path: str | Path | None,
        ids: _IdFactory,
    ) -> None:
        self.run_id = run_id
        self.condition = condition
        self.wall_clock = wall_clock
        self.wall_started_s = wall_started_s
        self.path = Path(path) if path is not None else None
        self.ids = ids
        self.events: list[dict[str, Any]] = []
        self._last_wall_time_s = 0.0
        self._lock = threading.RLock()
        if self.path is not None:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            if self.path.exists():
                raise FileExistsError(self.path)

    def emit(
        self,
        event_type: str,
        *,
        robot_id: str | None,
        sim_time_s: float | None,
        related_ids: Mapping[str, str] | None = None,
        payload: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        with self._lock:
            wall_time_s = max(
                self._last_wall_time_s,
                0.0,
                float(self.wall_clock() - self.wall_started_s),
            )
            self._last_wall_time_s = wall_time_s
            row = {
                "schema_version": SCHEMA_VERSION,
                "run_id": self.run_id,
                "condition": self.condition,
                "event_id": self.ids.new("event"),
                "event_type": event_type,
                "robot_id": robot_id,
                "sim_time_s": sim_time_s,
                "wall_time_s": wall_time_s,
                "related_ids": copy.deepcopy(dict(related_ids or {})),
                "payload": copy.deepcopy(dict(payload or {})),
            }
            self.events.append(row)
            if self.path is not None:
                with self.path.open("a", encoding="utf-8") as stream:
                    stream.write(json.dumps(row, ensure_ascii=False, sort_keys=True) + "\n")
        return copy.deepcopy(row)


@dataclass
class _QueuedMessage:
    event: dict[str, Any]
    available_tick: int


class MessageChannel:
    """Recipient-specific, expiring communication with matched budgets."""

    def __init__(
        self,
        condition: str,
        robot_ids: Sequence[str],
        limits: RuntimeLimits,
        trace: TraceRecorder,
        ids: _IdFactory,
    ) -> None:
        if condition not in CONDITIONS:
            raise ValueError("UNKNOWN_COMMUNICATION_CONDITION")
        self.condition = condition
        self.robot_ids = tuple(robot_ids)
        self.limits = limits
        self.trace = trace
        self.ids = ids
        self._queues = {robot_id: [] for robot_id in self.robot_ids}

    def send(
        self,
        sender_id: str,
        proposal: Mapping[str, Any] | None,
        *,
        decision_id: str,
        clock_time_s: float,
        trace_sim_time_s: float | None,
        tick_index: int,
        actor: _ActorState,
    ) -> dict[str, Any] | None:
        if proposal is None:
            return None
        if self.condition == "none":
            self.trace.emit(
                "message_rejected",
                robot_id=sender_id,
                sim_time_s=trace_sim_time_s,
                related_ids={"decision_id": decision_id},
                payload={"reason": "COMMUNICATION_DISABLED"},
            )
            return None
        if not isinstance(proposal, Mapping):
            return self._reject(sender_id, decision_id, trace_sim_time_s, "INVALID_MESSAGE")
        if set(proposal) != {"recipients", "content", "ttl_s"}:
            return self._reject(sender_id, decision_id, trace_sim_time_s, "INVALID_MESSAGE_FIELDS")

        recipients = proposal.get("recipients")
        if (
            not isinstance(recipients, list)
            or not recipients
            or any(not isinstance(recipient, str) for recipient in recipients)
            or len(recipients) != len(set(recipients))
            or sender_id in recipients
            or any(recipient not in self._queues for recipient in recipients)
        ):
            return self._reject(sender_id, decision_id, trace_sim_time_s, "INVALID_RECIPIENTS")
        content = proposal.get("content")
        if self.condition == "structured" and not isinstance(content, Mapping):
            return self._reject(sender_id, decision_id, trace_sim_time_s, "STRUCTURED_CONTENT_REQUIRED")
        if self.condition == "structured":
            allowed_fields = {
                "message_type", "task_id", "object_id", "skill", "participants",
                "stage", "action", "reason_code", "observation_ids",
            }
            if (set(content) - allowed_fields or "message_type" not in content
                    or not isinstance(content["message_type"], str)
                    or content["message_type"] not in {
                        "observation", "intent", "help_request", "accept", "reject",
                        "recovery", "completed", "retract", "partner_change",
                    }):
                return self._reject(
                    sender_id, decision_id, trace_sim_time_s, "INVALID_STRUCTURED_CONTENT")
            if "participants" in content and (
                    not isinstance(content["participants"], list)
                    or any(not isinstance(participant, str) for participant in content["participants"])
                    or any(participant not in self.robot_ids for participant in content["participants"])):
                return self._reject(
                    sender_id, decision_id, trace_sim_time_s, "INVALID_MESSAGE_PARTICIPANTS")
            if "observation_ids" in content and (
                    not isinstance(content["observation_ids"], list)
                    or any(not isinstance(item, str) or not item for item in content["observation_ids"])):
                return self._reject(
                    sender_id, decision_id, trace_sim_time_s, "INVALID_MESSAGE_OBSERVATIONS")
            if any(not isinstance(value, str) or not value.strip()
                   for key, value in content.items()
                   if key not in {"participants", "observation_ids"}):
                return self._reject(sender_id, decision_id, trace_sim_time_s,
                                    "INVALID_STRUCTURED_VALUE")
        if self.condition == "natural" and (not isinstance(content, str) or not content.strip()):
            return self._reject(sender_id, decision_id, trace_sim_time_s, "NATURAL_TEXT_REQUIRED")
        ttl_s = proposal.get("ttl_s", self.limits.max_message_ttl_s)
        if (
            not isinstance(ttl_s, (int, float))
            or isinstance(ttl_s, bool)
            or not math.isfinite(ttl_s)
            or ttl_s <= 0
            or ttl_s > self.limits.max_message_ttl_s
        ):
            return self._reject(sender_id, decision_id, trace_sim_time_s, "INVALID_MESSAGE_TTL")

        wire = {
            "sender_id": sender_id,
            "recipients": list(recipients),
            "content": copy.deepcopy(content),
            "sent_at_s": float(clock_time_s),
            "expires_at_s": float(clock_time_s + ttl_s),
        }
        message_id = self.ids.new("message")
        wire = {"message_id": message_id, "condition": self.condition, **wire}
        size = len(json.dumps(wire, ensure_ascii=False, sort_keys=True).encode("utf-8"))
        if actor.sent_messages >= self.limits.max_messages_per_robot:
            return self._reject(sender_id, decision_id, trace_sim_time_s, "MESSAGE_COUNT_BUDGET_EXHAUSTED")
        if actor.sent_message_bytes + size > self.limits.max_message_bytes_per_robot:
            return self._reject(sender_id, decision_id, trace_sim_time_s, "MESSAGE_BYTE_BUDGET_EXHAUSTED")

        event = {
            **wire,
            "bytes": size,
        }
        actor.sent_messages += 1
        actor.sent_message_bytes += size
        for recipient in recipients:
            self._queues[recipient].append(_QueuedMessage(copy.deepcopy(event), tick_index + 1))
        self.trace.emit(
            "message_sent",
            robot_id=sender_id,
            sim_time_s=trace_sim_time_s,
            related_ids={"decision_id": decision_id, "message_id": message_id},
            payload=event,
        )
        return copy.deepcopy(event)

    def receive(
        self,
        recipient_id: str,
        *,
        clock_time_s: float,
        trace_sim_time_s: float | None,
        tick_index: int,
    ) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
        delivered: list[dict[str, Any]] = []
        expired: list[dict[str, Any]] = []
        keep: list[_QueuedMessage] = []
        for queued in self._queues[recipient_id]:
            message = queued.event
            if queued.available_tick > tick_index:
                keep.append(queued)
                continue
            related = {"message_id": message["message_id"]}
            if clock_time_s >= message["expires_at_s"]:
                expired_claim = {
                    "message_id": message["message_id"],
                    "sender_id": message["sender_id"],
                    "expired_at_s": float(clock_time_s),
                    "status": "EXPIRED_PEER_CLAIM",
                }
                expired.append(expired_claim)
                self.trace.emit(
                    "message_expired",
                    robot_id=recipient_id,
                    sim_time_s=trace_sim_time_s,
                    related_ids=related,
                    payload={"sender_id": message["sender_id"], "reason": "TTL_EXPIRED"},
                )
                continue
            received = {**copy.deepcopy(message), "received_at_s": float(clock_time_s)}
            delivered.append(received)
            self.trace.emit(
                "message_received",
                robot_id=recipient_id,
                sim_time_s=trace_sim_time_s,
                related_ids=related,
                payload=received,
            )
        self._queues[recipient_id] = keep
        return delivered, expired

    def _reject(
        self, sender_id: str, decision_id: str, sim_time_s: float | None, reason: str
    ) -> None:
        self.trace.emit(
            "message_rejected",
            robot_id=sender_id,
            sim_time_s=sim_time_s,
            related_ids={"decision_id": decision_id},
            payload={"reason": reason},
        )
        return None


def _expire_actor_messages(
    actor: _ActorState,
    *,
    clock_time_s: float,
    trace_sim_time_s: float | None,
    trace: TraceRecorder,
) -> None:
    current: list[dict[str, Any]] = []
    expired_ids = {item["message_id"] for item in actor.expired_messages}
    for message in actor.received_messages:
        if clock_time_s < message["expires_at_s"]:
            current.append(message)
            continue
        if message["message_id"] not in expired_ids:
            claim = {
                "message_id": message["message_id"],
                "sender_id": message["sender_id"],
                "expired_at_s": float(clock_time_s),
                "status": "EXPIRED_PEER_CLAIM",
            }
            actor.expired_messages.append(claim)
            trace.emit(
                "message_expired",
                robot_id=actor.robot_id,
                sim_time_s=trace_sim_time_s,
                related_ids={"message_id": message["message_id"]},
                payload={"sender_id": message["sender_id"], "reason": "TTL_EXPIRED_AFTER_RECEIPT"},
            )
    actor.received_messages = current


def _copy_allowlisted(
    payload: Mapping[str, Any], fields: frozenset[str], required: Sequence[str]
) -> dict[str, Any]:
    if not isinstance(payload, Mapping):
        raise ValueError("PORT_PAYLOAD_MUST_BE_MAPPING")
    unexpected = set(payload) - fields
    if unexpected:
        raise ValueError("UNEXPECTED_PORT_FIELDS:" + ",".join(sorted(unexpected)))
    result = copy.deepcopy(dict(payload))
    missing = [key for key in required if key not in result]
    if missing:
        raise ValueError("MISSING_PORT_FIELDS:" + ",".join(missing))
    return result


def _clock_time(observation: Mapping[str, Any], status: Mapping[str, Any]) -> float:
    if observation["clock_domain"] != status["clock_domain"]:
        raise ValueError("MIXED_CLOCK_DOMAIN")
    for value in (observation["observed_at_s"], status["timestamp_s"]):
        if (not isinstance(value, (int, float)) or isinstance(value, bool)
                or not math.isfinite(value) or value < 0):
            raise ValueError("INVALID_ACTOR_CLOCK")
    observation_time = float(observation["observed_at_s"])
    status_time = float(status["timestamp_s"])
    if observation_time > status_time:
        raise ValueError("FUTURE_OBSERVATION")
    return status_time


def _trace_sim_time(clock_domain: str, clock_time_s: float) -> float | None:
    return clock_time_s if clock_domain == "sim" else None


def _is_sha256(value: Any) -> bool:
    return (isinstance(value, str) and len(value) == 64
            and not any(char not in "0123456789abcdef" for char in value))


def _exact_mapping(value: Any, fields: set[str], reason: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping) or set(value) != fields:
        raise ValueError(reason)
    return value


def _validate_static_context(value: Any) -> None:
    context = _exact_mapping(
        value, {"schema", "task", "static_map", "camera_calibration"},
        "INVALID_STATIC_CONTEXT_FIELDS")
    if context["schema"] != "ugrp.rgb_static_context.v1":
        raise ValueError("INVALID_STATIC_CONTEXT_SCHEMA")
    task = _exact_mapping(
        context["task"], {"description", "object_ids", "allowed_skills"},
        "INVALID_STATIC_TASK_FIELDS")
    if not isinstance(task["description"], str) or not task["description"].strip():
        raise ValueError("INVALID_STATIC_TASK_DESCRIPTION")
    for name in ("object_ids", "allowed_skills"):
        items = task[name]
        if (not isinstance(items, list) or not items
                or any(not isinstance(item, str) or not item.strip() for item in items)
                or len(items) != len(set(items))):
            raise ValueError("INVALID_STATIC_TASK_LIST")
    for name in ("static_map", "camera_calibration"):
        item = _exact_mapping(
            context[name], {"id", "version", "sha256", "description"},
            "INVALID_STATIC_REFERENCE_FIELDS")
        if (not _is_sha256(item["sha256"])
                or any(not isinstance(item[key], str) or not item[key].strip()
                       for key in ("id", "version", "description"))):
            raise ValueError("INVALID_STATIC_REFERENCE")


def _validate_observation(
    observation: Mapping[str, Any], *, robot_id: str, artifact_dir: Path | None
) -> dict[str, Any]:
    if observation["schema"] != "ugrp.rgb_actor_observation.v1":
        raise ValueError("INVALID_OBSERVATION_SCHEMA")
    if observation["robot_id"] != robot_id:
        raise ValueError("OBSERVATION_ROBOT_MISMATCH")
    observation_id = observation["observation_id"]
    if (not isinstance(observation_id, str) or not observation_id
            or len(observation_id) > 160
            or any(c not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_.-"
                   for c in observation_id) or observation_id in {".", ".."}):
        raise ValueError("INVALID_OBSERVATION_ID")
    if observation["clock_domain"] not in {"sim", "monotonic"}:
        raise ValueError("INVALID_CLOCK_DOMAIN")
    _validate_revision(observation)
    _validate_static_context(observation["static_context"])
    encoded = json.dumps(
        observation["static_context"], ensure_ascii=False, sort_keys=True,
        separators=(",", ":"),
    ).encode()
    if hashlib.sha256(encoded).hexdigest() != observation["static_context_sha256"]:
        raise ValueError("STATIC_CONTEXT_HASH_MISMATCH")
    if not isinstance(observation["own_issued_commands"], list):
        raise ValueError("INVALID_OWN_COMMAND_HISTORY")
    command_fields = {
        "task_id", "lease_id", "command_id", "stage", "action",
        "duration_s", "issued_at_s", "meaning",
    }
    for command in observation["own_issued_commands"]:
        correlation_fields = {"observation_id", "decision_id", "skill_decision_id", "consent_observation_id"}
        if (not isinstance(command, Mapping) or not command_fields <= set(command)
                or set(command) - command_fields - correlation_fields
                or any(not isinstance(command[name], str) or not command[name]
                       for name in correlation_fields if name in command)):
            raise ValueError("INVALID_OWN_COMMAND_FIELDS")
        if not isinstance(command["action"], Mapping):
            raise ValueError("INVALID_OWN_COMMAND_ACTION")

    images = _exact_mapping(
        observation["images"], {"own_rgb", "top_rgb"}, "INVALID_IMAGE_CHANNELS")
    trace_images: dict[str, Any] = {}
    for channel_name, image in images.items():
        image = _exact_mapping(
            image, {"ref", "sha256", "jpeg_base64"}, "INVALID_IMAGE_FIELDS")
        if not isinstance(image["ref"], str) or not image["ref"].strip() or not _is_sha256(image["sha256"]):
            raise ValueError("INVALID_IMAGE_REFERENCE")
        try:
            jpeg = base64.b64decode(image["jpeg_base64"], validate=True)
        except (binascii.Error, ValueError, TypeError) as exc:
            raise ValueError("INVALID_IMAGE_BASE64") from exc
        if (not jpeg.startswith(b"\xff\xd8") or not jpeg.endswith(b"\xff\xd9")
                or hashlib.sha256(jpeg).hexdigest() != image["sha256"]):
            raise ValueError("IMAGE_HASH_OR_JPEG_MISMATCH")
        trace_image = {"ref": image["ref"], "sha256": image["sha256"], "media_type": "image/jpeg"}
        if artifact_dir is not None:
            artifact_dir.mkdir(parents=True, exist_ok=True)
            path = artifact_dir / f"{observation['observation_id']}-{channel_name}.jpg"
            if path.exists() and path.read_bytes() != jpeg:
                raise ValueError("ARTIFACT_PATH_COLLISION")
            if not path.exists():
                path.write_bytes(jpeg)
            trace_image["path"] = str(path)
        trace_images[channel_name] = trace_image
    trace_observation = copy.deepcopy(dict(observation))
    trace_observation["images"] = trace_images
    return trace_observation


def _validate_local_status(status: Mapping[str, Any], *, robot_id: str) -> None:
    if status["schema"] != "ugrp.rgb_actor_status.v1":
        raise ValueError("INVALID_STATUS_SCHEMA")
    if status["robot_id"] != robot_id:
        raise ValueError("STATUS_ROBOT_MISMATCH")
    if status["clock_domain"] not in {"sim", "monotonic"}:
        raise ValueError("INVALID_CLOCK_DOMAIN")
    _validate_revision(status)
    if "own_skill_status" in status:
        skill = status["own_skill_status"]
        if (not isinstance(skill, Mapping) or set(skill) - {
                "task_id", "state", "phase", "reason", "meaning"}
                or (skill and (not {"task_id", "state", "meaning"} <= set(skill)
                    or skill["state"] not in {"RUNNING", "STOPPED", "FINISHED_UNVERIFIED"}))
                or any(not isinstance(value, str) for value in skill.values())):
            raise ValueError("INVALID_OWN_SKILL_STATUS")
    if not isinstance(status["active"], list) or not isinstance(status["pending"], list):
        raise ValueError("INVALID_LOCAL_STATUS_LISTS")
    active_fields = {
        "task_id", "object_id", "skill", "stage", "participants", "own_role",
        "lease_id", "expires_at_s", "own_command_pending",
    }
    pending_fields = {
        "request_id", "task_id", "object_id", "skill", "stage", "participants",
        "own_role", "expires_at_s", "status",
    }
    for item in status["active"]:
        if (not isinstance(item, Mapping) or not active_fields <= set(item)
                or set(item) - active_fields - {"paused", "own_resume_pending"}):
            raise ValueError("INVALID_ACTIVE_STATUS_FIELDS")
        for name in ("paused", "own_resume_pending"):
            if name in item and not isinstance(item[name], bool):
                raise ValueError("INVALID_ACTIVE_LIFECYCLE_STATE")
    for item in status["pending"]:
        _exact_mapping(item, pending_fields, "INVALID_PENDING_STATUS_FIELDS")
        if item["status"] != "PENDING_CONSENT":
            raise ValueError("INVALID_PENDING_STATUS")
    history = status["own_submission_history"]
    if not isinstance(history, list):
        raise ValueError("INVALID_SUBMISSION_HISTORY")
    required_result_fields = {"request_id", "status", "reason", "clock_domain", "timestamp_s"}
    optional_result_fields = {
        "task_id", "lease_id", "expires_at_s", "command_id", "hold_failed",
    }
    for item in history:
        _validate_submission_result(
            item, expected_clock_domain=status["clock_domain"],
            required_fields=required_result_fields, optional_fields=optional_result_fields)


def _validate_revision(payload: Mapping[str, Any]) -> None:
    if "own_revision" in payload and (type(payload["own_revision"]) is not int
                                      or payload["own_revision"] < 0):
        raise ValueError("INVALID_OWN_REVISION")


def _validate_submission_result(
    result: Any,
    *,
    expected_clock_domain: str,
    required_fields: set[str] | None = None,
    optional_fields: set[str] | None = None,
) -> dict[str, Any]:
    required_fields = required_fields or {
        "request_id", "status", "reason", "clock_domain", "timestamp_s",
    }
    optional_fields = optional_fields or {
        "task_id", "lease_id", "expires_at_s", "command_id", "hold_failed",
    }
    if (not isinstance(result, Mapping)
            or not required_fields <= set(result)
            or set(result) - required_fields - optional_fields):
        raise ValueError("INVALID_SUBMISSION_RESULT_FIELDS")
    if result["clock_domain"] != expected_clock_domain:
        raise ValueError("MIXED_SUBMISSION_RESULT_CLOCK")
    if "hold_failed" in result and not isinstance(result["hold_failed"], bool):
        raise ValueError("INVALID_HOLD_FAILED")
    return copy.deepcopy(dict(result))


def _normalize_action(
    proposal: Mapping[str, Any],
    *,
    robot_id: str,
    robot_ids: Sequence[str],
    observation_id: str,
    decision_id: str,
    action_id: str,
    now_s: float,
) -> dict[str, Any]:
    if not isinstance(proposal, Mapping):
        raise ValueError("INVALID_ACTION")
    if proposal.get("kind") == "wait":
        if set(proposal) != {"kind"}:
            raise ValueError("INVALID_WAIT_FIELDS")
        return {"kind": "wait"}
    if proposal.get("kind") == "finish":
        if (set(proposal) != {"kind", "claim"}
                or proposal.get("claim") not in {"mission_complete", "cannot_continue"}):
            raise ValueError("INVALID_FINISH_FIELDS")
        return copy.deepcopy(dict(proposal))
    kind = proposal.get("kind")
    fields_by_kind = {
        "task_request": {
            "kind", "task_id", "object_id", "skill", "participants", "resources",
            "stage", "own_role", "expires_at_s",
        },
        "command": {
            "kind", "task_id", "lease_id", "command_id", "stage", "action", "duration_s",
        },
        "interrupt": {"kind", "task_id", "lease_id", "reason"},
        "release": {"kind", "task_id", "lease_id", "reason"},
        "pause": {"kind", "task_id", "lease_id", "reason"},
        "resume": {"kind", "task_id", "lease_id", "reason"},
        "cancel_pending": {"kind", "task_id", "reason"},
    }
    if not isinstance(kind, str) or kind not in fields_by_kind or set(proposal) != fields_by_kind[kind]:
        raise ValueError("INCOMPLETE_ACTION_REQUEST")
    if kind == "task_request":
        participants = proposal["participants"]
        if (
            not isinstance(participants, list)
            or any(not isinstance(participant, str) for participant in participants)
            or robot_id not in participants
            or len(participants) != len(set(participants))
            or any(participant not in robot_ids for participant in participants)
        ):
            raise ValueError("INVALID_ACTION_PARTICIPANTS")
        expires_at_s = proposal["expires_at_s"]
        if (not isinstance(expires_at_s, (int, float)) or isinstance(expires_at_s, bool)
                or not math.isfinite(expires_at_s)):
            raise ValueError("INVALID_ACTION_EXPIRY")
        if expires_at_s <= now_s:
            raise ValueError("ACTION_ALREADY_EXPIRED")
    action = copy.deepcopy(dict(proposal))
    action.update({
        "request_id": action_id,
        "decision_id": decision_id,
        "observation_id": observation_id,
        "requested_at_s": float(now_s),
    })
    if kind == "task_request":
        action["expires_at_s"] = float(proposal["expires_at_s"])
    return action


def run_rgb_communication(
    port: "RGBExecutionPortProtocol",
    planners: Mapping[str, RGBPlanner],
    *,
    condition: str,
    common_task: Mapping[str, Any],
    run_id: str | None = None,
    robot_ids: Sequence[str] = DEFAULT_ROBOTS,
    limits: RuntimeLimits | None = None,
    trace_path: str | Path | None = None,
    artifact_dir: str | Path | None = None,
    wall_clock: Callable[[], float] = time.monotonic,
    clock_snapshot: Callable[[], dict] | None = None,
) -> dict[str, Any]:
    """Run one bounded, matched-condition fixture/replay/live-LLM episode.

    ``planners`` must contain three distinct planner objects. No evaluator
    snapshot is accepted. The optional clock-only supervisor callback
    supplies terminal/close evidence, never actor inputs or wake-up decisions.
    """

    limits = limits or RuntimeLimits()
    limits.validate()
    robot_ids = tuple(robot_ids)
    if condition not in CONDITIONS:
        raise ValueError("UNKNOWN_COMMUNICATION_CONDITION")
    if len(robot_ids) != 3 or len(set(robot_ids)) != 3:
        raise ValueError("THREE_UNIQUE_ROBOTS_REQUIRED")
    if set(planners) != set(robot_ids) or len({id(planners[rid]) for rid in robot_ids}) != 3:
        raise ValueError("THREE_INDEPENDENT_PLANNERS_REQUIRED")
    common_task = copy.deepcopy(dict(common_task))

    run_id = run_id or uuid.uuid4().hex
    artifact_dir = Path(artifact_dir) if artifact_dir is not None else None
    ids = _IdFactory()
    wall_started_s = wall_clock()
    trace = TraceRecorder(
        run_id=run_id,
        condition=condition,
        wall_clock=wall_clock,
        wall_started_s=wall_started_s,
        path=trace_path,
        ids=ids,
    )
    actors = {rid: _ActorState(rid) for rid in robot_ids}
    channel = MessageChannel(condition, robot_ids, limits, trace, ids)
    termination_reason = "MAX_TICKS"
    outcome = "aborted"
    runtime_error_type: str | None = None
    primary_error = close_error = None
    clock = ExecutionClock(getattr(port, "clock_domain", "monotonic"), clock_snapshot)
    tick_index = 0
    trace.emit(
        "run_started",
        robot_id=None,
        sim_time_s=None,
        payload={
            "robot_ids": list(robot_ids),
            "planner_models": {rid: planners[rid].model_name for rid in robot_ids},
            "evidence_kinds": {
                rid: getattr(planners[rid], "evidence_kind", "unknown") for rid in robot_ids
            },
            "limits": copy.deepcopy(limits.__dict__),
            "measured_metrics": [
                "high_level_actions", "messages", "message_bytes", "model_calls",
                "model_response_time_s",
            ],
        },
    )

    try:
        for tick_index in range(1, limits.max_ticks + 1):
            if wall_clock() - wall_started_s >= limits.wall_timeout_s:
                termination_reason = "WALL_TIMEOUT"
                break
            # Tick output is supervisor/audit material.  It is deliberately not
            # inspected for actor wake-up, planning input, or completion.
            requested_tick_s = (tick_index - 1) * limits.tick_period_s
            clock.tick(port, requested_tick_s)

            for robot_id in robot_ids:
                actor = actors[robot_id]
                if actor.finished or actor.calls >= limits.max_calls_per_robot:
                    continue
                observation = _copy_allowlisted(
                    port.observe(robot_id), OBSERVATION_FIELDS,
                    ("schema", "robot_id", "observation_id", "observed_at_s",
                     "clock_domain", "images", "static_context",
                     "static_context_sha256", "own_issued_commands")
                )
                trace_observation = _validate_observation(
                    observation, robot_id=robot_id, artifact_dir=artifact_dir)
                if common_task != observation["static_context"]["task"]:
                    raise ValueError("COMMON_TASK_MUST_MATCH_STATIC_CONTEXT")
                status = _copy_allowlisted(
                    port.local_status(robot_id), LOCAL_STATUS_FIELDS,
                    ("schema", "robot_id", "clock_domain", "timestamp_s",
                     "active", "pending", "own_submission_history")
                )
                _validate_local_status(status, robot_id=robot_id)
                clock_time_s = _clock_time(observation, status)
                sim_time_s = _trace_sim_time(status["clock_domain"], clock_time_s)
                actor.observations.append(copy.deepcopy(observation))
                actor.trace_observations.append(copy.deepcopy(trace_observation))
                actor.statuses.append(copy.deepcopy(status))
                observation_id = str(observation["observation_id"])
                status_id = ids.new("status")
                trace.emit(
                    "observation_captured",
                    robot_id=robot_id,
                    sim_time_s=sim_time_s,
                    related_ids={"observation_id": observation_id},
                    payload=trace_observation,
                )
                trace.emit(
                    "local_status_read",
                    robot_id=robot_id,
                    sim_time_s=sim_time_s,
                    related_ids={"status_id": status_id},
                    payload=status,
                )
                _expire_actor_messages(
                    actor,
                    clock_time_s=clock_time_s,
                    trace_sim_time_s=sim_time_s,
                    trace=trace,
                )
                received, expired = channel.receive(
                    robot_id,
                    clock_time_s=clock_time_s,
                    trace_sim_time_s=sim_time_s,
                    tick_index=tick_index,
                )
                actor.received_messages.extend(copy.deepcopy(received))
                actor.expired_messages.extend(copy.deepcopy(expired))

                request_id = ids.new("request")
                request = {
                    "schema_version": REQUEST_SCHEMA_VERSION,
                    "run_id": run_id,
                    "request_id": request_id,
                    "robot_id": robot_id,
                    "condition": condition,
                    "common_task": copy.deepcopy(common_task),
                    "observation": observation,
                    "local_status": status,
                    "memory": actor.memory_snapshot(),
                    "budget": {
                        "calls_remaining": limits.max_calls_per_robot - actor.calls,
                        "actions_remaining": limits.max_actions_per_robot - actor.actions,
                        "messages_remaining": limits.max_messages_per_robot - actor.sent_messages,
                        "message_bytes_remaining": (
                            limits.max_message_bytes_per_robot - actor.sent_message_bytes
                        ),
                    },
                }
                trace_request = copy.deepcopy(request)
                trace_request["observation"] = trace_observation
                trace_request["memory"]["own_observations"] = copy.deepcopy(
                    actor.trace_observations[-4:])
                trace.emit(
                    "planner_requested",
                    robot_id=robot_id,
                    sim_time_s=sim_time_s,
                    related_ids={
                        "request_id": request_id,
                        "observation_id": observation_id,
                        "status_id": status_id,
                    },
                    payload=trace_request,
                )
                actor.calls += 1
                call_started_s = wall_clock()
                try:
                    reply = planners[robot_id].decide(copy.deepcopy(request))
                except Exception as exc:
                    trace.emit(
                        "planner_response_rejected",
                        robot_id=robot_id,
                        sim_time_s=sim_time_s,
                        related_ids={"request_id": request_id},
                        payload={"reason": "PLANNER_ERROR", "error_type": type(exc).__name__},
                    )
                    continue
                call_finished_s = wall_clock()
                if not isinstance(reply, Mapping):
                    trace.emit(
                        "planner_response_rejected",
                        robot_id=robot_id,
                        sim_time_s=sim_time_s,
                        related_ids={"request_id": request_id},
                        payload={"reason": "INVALID_PLANNER_REPLY"},
                    )
                    continue
                if (
                    call_finished_s - call_started_s > limits.decision_timeout_s
                    or call_finished_s - wall_started_s >= limits.wall_timeout_s
                ):
                    trace.emit(
                        "planner_response_rejected",
                        robot_id=robot_id,
                        sim_time_s=sim_time_s,
                        related_ids={"request_id": request_id},
                        payload={"reason": "LATE_REPLY", "raw_text": reply.get("raw_text")},
                    )
                    continue

                latest_status = _copy_allowlisted(
                    port.local_status(robot_id), LOCAL_STATUS_FIELDS,
                    ("schema", "robot_id", "clock_domain", "timestamp_s",
                     "active", "pending", "own_submission_history")
                )
                _validate_local_status(latest_status, robot_id=robot_id)
                if latest_status != status:
                    trace.emit(
                        "planner_response_rejected",
                        robot_id=robot_id,
                        sim_time_s=sim_time_s,
                        related_ids={"request_id": request_id},
                        payload={
                            "reason": "OWN_STATE_CHANGED",
                            "raw_text": reply.get("raw_text"),
                        },
                    )
                    continue

                decision_id = ids.new("decision")
                latency_s = max(0.0, call_finished_s - call_started_s)
                usage = copy.deepcopy(reply.get("usage"))
                if usage is None:
                    usage = {}
                if not isinstance(usage, Mapping):
                    usage = {}
                usage = {
                    key: usage[key] for key in ("input_tokens", "output_tokens")
                    if isinstance(usage.get(key), int) and not isinstance(usage.get(key), bool)
                }
                usage["response_latency_s"] = latency_s
                raw_text = reply.get("raw_text")
                decision = {
                    "decision_id": decision_id,
                    "request_id": request_id,
                    "action": copy.deepcopy(reply.get("action")),
                    "message": copy.deepcopy(reply.get("message")),
                    "raw_text": raw_text,
                    "raw_text_sha256": (hashlib.sha256(raw_text.encode()).hexdigest()
                                            if isinstance(raw_text, str) else None),
                    **usage,
                }
                actor.decisions.append(copy.deepcopy(decision))
                trace.emit(
                    "planner_responded",
                    robot_id=robot_id,
                    sim_time_s=sim_time_s,
                    related_ids={"request_id": request_id, "decision_id": decision_id},
                    payload=decision,
                )
                channel.send(
                    robot_id,
                    reply.get("message"),
                    decision_id=decision_id,
                    clock_time_s=clock_time_s,
                    trace_sim_time_s=sim_time_s,
                    tick_index=tick_index,
                    actor=actor,
                )

                proposed_action = reply.get("action")
                if proposed_action is None:
                    continue
                action_id = ids.new("action")
                try:
                    action = _normalize_action(
                        proposed_action,
                        robot_id=robot_id,
                        robot_ids=robot_ids,
                        observation_id=observation_id,
                        decision_id=decision_id,
                        action_id=action_id,
                        now_s=clock_time_s,
                    )
                except ValueError as exc:
                    trace.emit(
                        "action_rejected",
                        robot_id=robot_id,
                        sim_time_s=sim_time_s,
                        related_ids={"decision_id": decision_id, "action_id": action_id},
                        payload={"reason": str(exc)},
                    )
                    continue
                if action.get("kind") == "wait":
                    continue
                if action.get("kind") == "finish":
                    actor.finished = True
                    actor.finish_claim = action["claim"]
                    trace.emit(
                        "actor_finished",
                        robot_id=robot_id,
                        sim_time_s=sim_time_s,
                        related_ids={
                            "decision_id": decision_id,
                            "action_id": action_id,
                            "observation_id": observation_id,
                        },
                        payload={"claim": action["claim"], "meaning": "unverified_actor_claim"},
                    )
                    continue
                if actor.actions >= limits.max_actions_per_robot:
                    trace.emit(
                        "budget_exhausted",
                        robot_id=robot_id,
                        sim_time_s=sim_time_s,
                        related_ids={"decision_id": decision_id, "action_id": action_id},
                        payload={"budget": "actions"},
                    )
                    continue
                trace.emit(
                    "action_submitted",
                    robot_id=robot_id,
                    sim_time_s=sim_time_s,
                    related_ids={
                        "decision_id": decision_id,
                        "action_id": action_id,
                        "observation_id": observation_id,
                    },
                    payload=action,
                )
                result = _validate_submission_result(
                    port.submit(robot_id, copy.deepcopy(action)),
                    expected_clock_domain=status["clock_domain"],
                )
                actor.actions += 1
                action_result = {"action_id": action_id, "result": result}
                actor.action_results.append(action_result)
                trace.emit(
                    "action_result",
                    robot_id=robot_id,
                    sim_time_s=sim_time_s,
                    related_ids={"decision_id": decision_id, "action_id": action_id},
                    payload=result,
                )

            if all(actor.finished for actor in actors.values()):
                termination_reason = "ALL_ACTORS_FINISHED"
                outcome = "completed"
                break
            if all(actor.finished or actor.calls >= limits.max_calls_per_robot
                   for actor in actors.values()):
                termination_reason = "CALL_BUDGET_EXHAUSTED"
                break
    except Exception as exc:
        termination_reason = "RUNTIME_ERROR"
        outcome = "aborted"
        primary_error = sanitized_error(exc)
        runtime_error_type = primary_error["error_type"]
        clock.refresh()
        trace.emit(
            "run_error",
            robot_id=None,
            sim_time_s=clock.sim_time,
            payload={"reason": termination_reason, "phase": "runtime",
                     **primary_error, "clock": clock.payload()},
        )
    finally:
        clock.refresh()
        try:
            port.close(clock.close_argument(failed=primary_error is not None))
        except Exception as exc:
            close_error = sanitized_error(exc)
            if primary_error is None:
                termination_reason = "CLOSE_ERROR"
                outcome = "aborted"
                runtime_error_type = close_error["error_type"]
            clock.refresh()
            trace.emit(
                "run_error",
                robot_id=None,
                sim_time_s=clock.sim_time,
                payload={"reason": "CLOSE_ERROR", "phase": "close",
                         **close_error, "clock": clock.payload()},
            )
        clock.refresh()

    if termination_reason == "WALL_TIMEOUT":
        outcome = "timeout"

    result = {
        "run_id": run_id,
        "condition": condition,
        "outcome": outcome,
        "termination_reason": termination_reason,
        "clock": clock.payload(),
        "primary_error": primary_error,
        "close_error": close_error,
        "actor_finish_claims": {rid: actors[rid].finish_claim == "mission_complete" for rid in robot_ids},
        "actor_finish_details": {rid: actors[rid].finish_claim for rid in robot_ids},
        "ticks": tick_index,
        "calls": {rid: actors[rid].calls for rid in robot_ids},
        "actions": {rid: actors[rid].actions for rid in robot_ids},
        "messages": {rid: actors[rid].sent_messages for rid in robot_ids},
        "message_bytes": {rid: actors[rid].sent_message_bytes for rid in robot_ids},
        "trace_events": len(trace.events) + 1,
    }
    if runtime_error_type is not None:
        result["error_type"] = runtime_error_type
    trace.emit(
        "run_finished",
        robot_id=None,
        sim_time_s=clock.sim_time,
        payload=result,
    )
    return {**result, "events": copy.deepcopy(trace.events)}


def run_rgb_communication_async(port, planners, **kwargs):
    """Asynchronous path; the sequential entry point remains a separate baseline.

    See ``rgb_communication_async.AsyncRuntimeLimits`` and the implementation's
    explicit signature for the scheduler clock and cancellation contract.
    """
    from harness.rgb_communication_async import run_rgb_communication_async as run
    return run(port, planners, **kwargs)
