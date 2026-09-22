"""Public contracts for the communication-study RGB execution boundary.

The actor-facing objects in this module intentionally contain no evaluator
state.  A caller may inject a separate evaluation source into
``RGBExecutionPort``, but that source is only reachable through the explicit
evaluation method.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Mapping, Protocol, runtime_checkable


REQUEST_KIND = "task_request"
COMMAND_KIND = "command"
INTERRUPT_KIND = "interrupt"
RELEASE_KIND = "release"
PAUSE_KIND = "pause"
RESUME_KIND = "resume"
CANCEL_PENDING_KIND = "cancel_pending"
SUBMISSION_KINDS = frozenset({REQUEST_KIND, COMMAND_KIND, INTERRUPT_KIND, RELEASE_KIND,
                              PAUSE_KIND, RESUME_KIND, CANCEL_PENDING_KIND})


def finite_number(value: Any, *, minimum: float | None = None) -> bool:
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(value) and (minimum is None or value >= minimum))


def nonempty_text(value: Any) -> bool:
    return isinstance(value, str) and bool(value.strip())


def validate_static_context(value: Any) -> None:
    """Validate immutable operator knowledge accepted by the actor boundary."""
    if not isinstance(value, dict) or set(value) != {"schema", "task", "static_map",
                                                    "camera_calibration"}:
        raise ValueError("invalid static context fields")
    if value["schema"] != "ugrp.rgb_static_context.v1":
        raise ValueError("invalid static context schema")
    task = value["task"]
    if (not isinstance(task, dict)
            or set(task) != {"description", "object_ids", "allowed_skills"}
            or not nonempty_text(task["description"])):
        raise ValueError("invalid static task description")
    for name in ("object_ids", "allowed_skills"):
        items = task[name]
        if (not isinstance(items, list) or not items
                or any(not nonempty_text(item) for item in items)
                or len(set(items)) != len(items)):
            raise ValueError(f"invalid static task {name}")
    static_map = value["static_map"]
    calibration = value["camera_calibration"]
    for name, item in (("static_map", static_map), ("camera_calibration", calibration)):
        if (not isinstance(item, dict)
                or set(item) != {"id", "version", "sha256", "description"}
                or not all(nonempty_text(item[field]) for field in ("id", "version", "description"))
                or not isinstance(item["sha256"], str) or len(item["sha256"]) != 64
                or any(char not in "0123456789abcdef" for char in item["sha256"])):
            raise ValueError(f"invalid {name}")


@dataclass(frozen=True)
class SkillCapability:
    """Caller-declared capability of the injected physical endpoint.

    Declaring a capability does not prove it works on a robot.  The declaration
    is an executable allowlist which must be paired with separate physical
    validation evidence.
    """

    skill: str
    stages: frozenset[str]
    team_sizes: frozenset[int]
    action_kinds: frozenset[str]
    max_duration_s: float
    distinct_roles: bool = False

    def __post_init__(self) -> None:
        if (not nonempty_text(self.skill) or not self.stages
                or any(not nonempty_text(stage) for stage in self.stages)):
            raise ValueError("skill and stages must be nonempty text")
        if (not self.team_sizes or any(not isinstance(size, int) or isinstance(size, bool)
                                       or size < 1 for size in self.team_sizes)):
            raise ValueError("team_sizes must contain positive integers")
        if not self.action_kinds or any(not nonempty_text(kind) for kind in self.action_kinds):
            raise ValueError("action_kinds must be nonempty text")
        if not finite_number(self.max_duration_s, minimum=0.) or self.max_duration_s == 0:
            raise ValueError("max_duration_s must be positive")


class MotionEndpoint(Protocol):
    """Narrow seam implemented by CameraRobotPort-like physical endpoints."""

    robot_id: str

    def validate_bounded(self, action: Mapping[str, Any], duration_s: float) -> None: ...
    def apply_bounded(self, action: Mapping[str, Any], now_s: float, duration_s: float) -> None: ...
    def hold(self, now_s: float) -> None: ...
    def tick(self, now_s: float) -> None: ...


class FrameSource(Protocol):
    """Return only the actor's own JPEG and the common top JPEG."""

    def __call__(self, robot_id: str, now_s: float) -> Mapping[str, bytes]: ...


class EvaluationSource(Protocol):
    """Trusted sink/source which is never consulted by actor operations."""

    def __call__(self) -> Mapping[str, Any]: ...


@runtime_checkable
class RGBExecutionPortProtocol(Protocol):
    """Runtime-facing API shared by all communication conditions.

    ``evaluation_snapshot`` is deliberately absent.  Evaluation consumers use
    a separate reference and must not pass its result into an actor runtime.
    """

    @property
    def clock_domain(self) -> str: ...
    def observe(self, robot_id: str) -> Mapping[str, Any]: ...
    def local_status(self, robot_id: str) -> Mapping[str, Any]: ...
    def submit(self, robot_id: str, submission: Mapping[str, Any]) -> Mapping[str, Any]: ...
    def tick(self, now_s: float) -> Mapping[str, Any]: ...
    def close(self, now_s: float) -> None: ...
