"""Shared client helpers for talking to the local simulation bridge.

The bridge binds to loopback, but loopback is not an authentication boundary:
any process on the host could otherwise revoke GPU authority or enqueue robot
actions. Control-plane callers (harness, sim_actions, failover) therefore send
the same bearer token that workers use. The token is read from the environment
first so deployed services can inject it, then from the repo-local token file.
"""
from __future__ import annotations

import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TOKEN_FILE = ROOT / ".sim_bridge_token"
TOKEN_HEADER = "X-UGRP-Sim-Token"


def bridge_token() -> str:
    env = os.environ.get("UGRP_SIM_TOKEN", "").strip()
    if env:
        return env
    try:
        return TOKEN_FILE.read_text(encoding="utf-8").strip()
    except OSError:
        return ""


def bridge_headers(extra: dict | None = None) -> dict:
    headers = {"Content-Type": "application/json"}
    token = bridge_token()
    if token:
        headers[TOKEN_HEADER] = token
    if extra:
        headers.update(extra)
    return headers
