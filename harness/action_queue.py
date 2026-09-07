"""Per-robot authoritative queue for public LLM-selected skills.

The queue contains semantic/public skills only. Low-level motor/servo pulses remain
inside skill controllers and are intentionally invisible to the planner queue.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import threading
import time
from typing import Any, Iterable


@dataclass
class ActionQueueEntry:
    id: str
    tool: str
    args: dict[str, Any]
    source: str
    status: str = "QUEUED"  # QUEUED | RUNNING | COMPLETED | BLOCKED | FAILED | CANCELLED
    say: str | None = None
    failure_code: str | None = None
    required_state: str | None = None
    reason: str | None = None
    outcome_status: str | None = None
    # Monotonic plan generation prevents a command from an older replacement
    # plan being mistaken for current work by a long-lived robot session.
    plan_epoch: int = 0
    created_at: float = 0.0


class RobotActionQueue:
    """Thread-safe queue shared by one robot's HTTP turns and planner loop."""

    def __init__(self, *, history_limit: int = 24) -> None:
        self._lock = threading.RLock()
        self._next_id = 1
        self._pending: list[ActionQueueEntry] = []
        self._running: ActionQueueEntry | None = None
        self._history: list[ActionQueueEntry] = []
        self._history_limit = max(4, int(history_limit))
        self._revision = 0
        self._paused_reason: str | None = None
        self._plan_epoch = 0

    def _new(self, tool: str, args: dict[str, Any], *, source: str, say: str | None = None) -> ActionQueueEntry:
        entry = ActionQueueEntry(
            id=f"q{self._next_id:06d}",
            tool=str(tool),
            args=dict(args),
            source=str(source),
            say=say,
            plan_epoch=self._plan_epoch,
            created_at=time.monotonic(),
        )
        self._next_id += 1
        return entry

    def _touch(self) -> None:
        self._revision += 1

    def _archive(self, entry: ActionQueueEntry) -> None:
        self._history.append(entry)
        del self._history[:-self._history_limit]

    def replace_pending(self, calls: Iterable[tuple[str, dict[str, Any]]], *, source: str = "llm_plan") -> list[ActionQueueEntry]:
        """Replace only not-yet-started work; completed/failed history is retained."""
        with self._lock:
            if self._running is not None:
                raise RuntimeError("cannot replace queue while a skill is running")
            for old in self._pending:
                old.status = "CANCELLED"
                old.reason = "replanned"
                self._archive(old)
            self._plan_epoch += 1
            self._pending = [self._new(name, args, source=source) for name, args in calls]
            self._paused_reason = None
            self._touch()
            return list(self._pending)

    def enqueue_front(self, tool: str, args: dict[str, Any], *, source: str = "llm_tool", say: str | None = None) -> ActionQueueEntry:
        with self._lock:
            if self._running is not None:
                raise RuntimeError("cannot enqueue a new head while a skill is running")
            entry = self._new(tool, args, source=source, say=say)
            self._pending.insert(0, entry)
            self._paused_reason = None
            self._touch()
            return entry

    def enqueue(self, tool: str, args: dict[str, Any], *, source: str = "llm_tool", say: str | None = None) -> ActionQueueEntry:
        with self._lock:
            entry = self._new(tool, args, source=source, say=say)
            self._pending.append(entry)
            self._touch()
            return entry

    def claim_next(self, *, expected_epoch: int | None = None) -> ActionQueueEntry | None:
        with self._lock:
            if self._running is not None or not self._pending or self._paused_reason is not None:
                return None
            entry = self._pending.pop(0)
            if expected_epoch is not None and entry.plan_epoch != expected_epoch:
                entry.status = "CANCELLED"
                entry.reason = "stale_plan_epoch"
                self._archive(entry)
                self._touch()
                return None
            entry.status = "RUNNING"
            self._running = entry
            self._touch()
            return ActionQueueEntry(**asdict(entry))

    def _finish_running(
        self,
        status: str,
        *,
        failure_code: str | None = None,
        required_state: str | None = None,
        reason: str | None = None,
        outcome_status: str | None = None,
        pause: bool = False,
    ) -> None:
        with self._lock:
            if self._running is None:
                return
            entry = self._running
            entry.status = status
            entry.failure_code = failure_code
            entry.required_state = required_state
            entry.reason = reason
            entry.outcome_status = outcome_status
            self._archive(entry)
            self._running = None
            self._paused_reason = reason or failure_code or status if pause else None
            self._touch()

    def complete_running(self, *, outcome_status: str | None = None) -> None:
        self._finish_running("COMPLETED", outcome_status=outcome_status, pause=False)

    def block_running(self, *, failure_code: str | None, required_state: str | None, reason: str | None) -> None:
        self._finish_running(
            "BLOCKED",
            failure_code=failure_code,
            required_state=required_state,
            reason=reason,
            outcome_status="NOT_ACHIEVED",
            pause=True,
        )

    def fail_running(self, *, failure_code: str | None, reason: str | None, outcome_status: str | None = None) -> None:
        self._finish_running(
            "FAILED",
            failure_code=failure_code,
            reason=reason,
            outcome_status=outcome_status or "NOT_ACHIEVED",
            pause=True,
        )

    def pause(self, reason: str) -> None:
        with self._lock:
            if self._pending and self._running is None:
                self._paused_reason = str(reason)
                self._touch()

    def resume(self) -> None:
        with self._lock:
            if self._paused_reason is not None:
                self._paused_reason = None
                self._touch()

    def clear_pending(self, reason: str) -> None:
        with self._lock:
            for entry in self._pending:
                entry.status = "CANCELLED"
                entry.reason = str(reason)
                self._archive(entry)
            self._pending.clear()
            self._paused_reason = str(reason) if self._running is None else self._paused_reason
            self._touch()

    def invalidate_stale(self, ttl_seconds: float, *, now: float | None = None) -> int:
        """Cancel queued work that outlived its planning context.

        This only touches not-yet-started entries. A running physical skill is
        never interrupted here; callers use the normal cancellation path for
        that. A short-lived HTTP/planner turn may otherwise leave a queue that
        a later wake resumes after its world assumptions are no longer valid.
        """
        try:
            ttl = float(ttl_seconds)
        except (TypeError, ValueError):
            return 0
        if ttl <= 0:
            return 0
        now = time.monotonic() if now is None else float(now)
        with self._lock:
            kept: list[ActionQueueEntry] = []
            removed = 0
            for entry in self._pending:
                created = float(entry.created_at or 0.0)
                if created > 0.0 and now - created >= ttl:
                    entry.status = "CANCELLED"
                    entry.reason = "stale_pending_ttl"
                    self._archive(entry)
                    removed += 1
                else:
                    kept.append(entry)
            if removed:
                self._pending = kept
                if not self._pending and self._running is None:
                    self._paused_reason = "stale_pending_ttl"
                self._touch()
            return removed

    def reset(self) -> None:
        """Discard all queue state at a world/episode boundary."""
        with self._lock:
            self._pending.clear()
            self._running = None
            self._history.clear()
            self._paused_reason = None
            self._next_id = 1
            self._plan_epoch += 1
            self._touch()

    def has_pending(self) -> bool:
        with self._lock:
            return bool(self._pending)

    def pending_names(self) -> list[str]:
        with self._lock:
            return [entry.tool for entry in self._pending]

    def paused(self) -> bool:
        with self._lock:
            return self._paused_reason is not None

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {
                "revision": self._revision,
                "plan_epoch": self._plan_epoch,
                "paused": self._paused_reason is not None,
                "pause_reason": self._paused_reason,
                "running": asdict(self._running) if self._running is not None else None,
                "pending": [asdict(entry) for entry in self._pending],
                "history": [asdict(entry) for entry in self._history[-12:]],
            }
