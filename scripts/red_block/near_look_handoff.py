"""Short-lived causal handoff from search near-look to precision approach.

The close camera geometry is also used around grasp verification, so pose values
alone cannot prove that a close view came from a fresh SEARCH observation.  This
token is written only after search has multi-frame-confirmed red in near-look
geometry and is consumed by approach before any close-view manipulation.
"""
from __future__ import annotations

import json
import os
import time
from contextlib import contextmanager
from contextvars import ContextVar
from pathlib import Path
from typing import Any

PATH = Path('/tmp/ugrp-red-near-look-handoff.json')
_PATH_OVERRIDE: ContextVar[Path | None] = ContextVar('ugrp_near_look_path', default=None)
MAX_AGE_SECONDS = 30.0
SOURCE = 'search_near_look_multiframe_red'
FIXED_SERVOS = (4, 5)


def _path() -> Path:
    return _PATH_OVERRIDE.get() or PATH


@contextmanager
def use_near_look_path(path: Path):
    token = _PATH_OVERRIDE.set(Path(path))
    try:
        yield
    finally:
        _PATH_OVERRIDE.reset(token)


def invalidate_near_look_handoff() -> None:
    path = _path()
    try:
        path.unlink()
    except FileNotFoundError:
        pass


def save_near_look_handoff(*, robot_pose: dict[int, int]) -> None:
    path = _path()
    payload = {
        'version': 1,
        'created_at': time.time(),
        'source': SOURCE,
        'fixed_pose': {str(s): int(robot_pose[s]) for s in FIXED_SERVOS if s in robot_pose},
    }
    temporary = path.with_suffix(f'.tmp.{os.getpid()}')
    temporary.write_text(json.dumps(payload, sort_keys=True), encoding='utf-8')
    temporary.replace(path)


def require_near_look_handoff(
    *, current_pose: dict[int, int], now: float | None = None,
) -> dict[str, Any]:
    path = _path()
    if not path.is_file():
        raise RuntimeError('fresh close near-look search handoff is missing; run search again')
    try:
        payload = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as exc:
        invalidate_near_look_handoff()
        raise RuntimeError('close near-look search handoff is unreadable; run search again') from exc
    if payload.get('version') != 1 or payload.get('source') != SOURCE:
        invalidate_near_look_handoff()
        raise RuntimeError('close near-look search handoff has invalid provenance; run search again')
    created = payload.get('created_at')
    if isinstance(created, bool) or not isinstance(created, (int, float)):
        invalidate_near_look_handoff()
        raise RuntimeError('close near-look search handoff has no valid timestamp; run search again')
    age = (time.time() if now is None else float(now)) - float(created)
    if age < -2.0 or age > MAX_AGE_SECONDS:
        invalidate_near_look_handoff()
        raise RuntimeError(f'close near-look search handoff is stale ({age:.1f}s); run search again')
    saved = payload.get('fixed_pose')
    if not isinstance(saved, dict):
        invalidate_near_look_handoff()
        raise RuntimeError('close near-look search handoff has no fixed arm pose; run search again')
    for servo in FIXED_SERVOS:
        expected = saved.get(str(servo))
        current = current_pose.get(servo)
        if expected is None or current is None or int(expected) != int(current):
            invalidate_near_look_handoff()
            raise RuntimeError(
                f'close near-look arm geometry changed after search (servo {servo}); run search again'
            )
    return payload
