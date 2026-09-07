#!/usr/bin/env python3
"""Launcher for approaching a red block with the chassis."""

from __future__ import annotations

import runpy
from pathlib import Path

runpy.run_path(
    str(Path(__file__).resolve().parent / "red_block" / "approach.py"),
    run_name="__main__",
)
