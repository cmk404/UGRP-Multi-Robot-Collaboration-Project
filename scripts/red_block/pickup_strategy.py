"""Short-lived adaptive pickup strategy state.

The arm-only path is an optimization, never a replacement for the previously
validated precision face/capture path.  A confirmed arm-only grasp miss requests
one precision fallback on the next approach.  The marker is short-lived and
color-scoped so unrelated tasks cannot inherit it indefinitely.
"""
from __future__ import annotations

import json
import os
import time
from pathlib import Path

FALLBACK_PATH = Path("/tmp/ugrp-pick-precision-fallback.json")
FALLBACK_MAX_AGE_SECONDS = 120.0


def clear_precision_fallback(target_color: str | None = None) -> None:
    if target_color is not None and FALLBACK_PATH.is_file():
        try:
            payload = json.loads(FALLBACK_PATH.read_text(encoding="utf-8"))
            if payload.get("target_color") != target_color:
                return
        except Exception:
            pass
    try:
        FALLBACK_PATH.unlink()
    except FileNotFoundError:
        pass


def request_precision_fallback(target_color: str, reason: str) -> None:
    if target_color not in {"red", "blue", "yellow"}:
        raise ValueError("target_color must be red, blue, or yellow")
    payload = {
        "version": 1,
        "created_at": time.time(),
        "target_color": target_color,
        "reason": str(reason)[:500],
    }
    temporary = FALLBACK_PATH.with_suffix(f".tmp.{os.getpid()}")
    temporary.write_text(json.dumps(payload, sort_keys=True), encoding="utf-8")
    temporary.replace(FALLBACK_PATH)


def precision_fallback_reason(target_color: str, *, now: float | None = None) -> str | None:
    if not FALLBACK_PATH.is_file():
        return None
    try:
        payload = json.loads(FALLBACK_PATH.read_text(encoding="utf-8"))
        created = float(payload["created_at"])
    except Exception:
        clear_precision_fallback()
        return None
    age = (time.time() if now is None else float(now)) - created
    if age < -2.0 or age > FALLBACK_MAX_AGE_SECONDS:
        clear_precision_fallback()
        return None
    if payload.get("target_color") != target_color:
        return None
    return str(payload.get("reason") or "arm-only pickup requested precision fallback")


def should_use_precision_fallback(target_color: str, *, now: float | None = None) -> bool:
    return precision_fallback_reason(target_color, now=now) is not None
