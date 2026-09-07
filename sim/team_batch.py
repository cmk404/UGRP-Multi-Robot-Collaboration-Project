"""Per-turn metadata for synchronizing the first TEAM actuator command."""

from __future__ import annotations

from contextlib import contextmanager
from contextvars import ContextVar
from typing import Iterator


_ACTIVE_BATCH: ContextVar[tuple[str, int] | None] = ContextVar(
    "ugrp_sim_team_batch", default=None,
)
_COMMAND_INDEX: ContextVar[int] = ContextVar("ugrp_sim_team_command_index", default=0)


@contextmanager
def use_team_batch(batch_id: object, expected: object = 3) -> Iterator[None]:
    """Attach one consensus batch to the first actuator call in this turn."""
    identifier = str(batch_id or "").strip()
    try:
        member_count = int(expected)
    except (TypeError, ValueError):
        member_count = 3
    if not identifier or member_count < 2:
        yield
        return
    member_count = min(3, member_count)
    batch_token = _ACTIVE_BATCH.set((identifier, member_count))
    index_token = _COMMAND_INDEX.set(0)
    try:
        yield
    finally:
        _COMMAND_INDEX.reset(index_token)
        _ACTIVE_BATCH.reset(batch_token)


def first_command_metadata() -> dict[str, object]:
    """Return batch metadata once; recovery actions must not wait for peers."""
    active = _ACTIVE_BATCH.get()
    if active is None:
        return {}
    index = _COMMAND_INDEX.get()
    _COMMAND_INDEX.set(index + 1)
    if index != 0:
        return {}
    batch_id, expected = active
    return {
        "team_batch_id": batch_id,
        "team_batch_expected": expected,
    }
