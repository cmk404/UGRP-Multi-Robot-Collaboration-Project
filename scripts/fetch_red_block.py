#!/usr/bin/env python3
"""Launcher: approach a red block, then pick it up and hold."""

from __future__ import annotations

import runpy
from pathlib import Path

runpy.run_path(
    str(Path(__file__).resolve().parent / "red_block" / "fetch.py"),
    run_name="__main__",
)
