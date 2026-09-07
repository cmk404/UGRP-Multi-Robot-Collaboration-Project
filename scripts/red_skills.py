#!/usr/bin/env python3
"""Compatibility CLI. The action list lives in robot_actions.py."""

from __future__ import annotations

import importlib.util
from pathlib import Path


_PATH = Path(__file__).with_name("robot_actions.py")
_SPEC = importlib.util.spec_from_file_location("ugrp_robot_actions", _PATH)
if _SPEC is None or _SPEC.loader is None:
    raise FileNotFoundError(_PATH)
_MOD = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_MOD)

ACTIONS = _MOD.ACTIONS
SKILLS = ACTIONS
main = _MOD.main
run = _MOD.run


if __name__ == "__main__":
    raise SystemExit(main())
