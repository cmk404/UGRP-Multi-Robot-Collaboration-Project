#!/usr/bin/env python3
"""Launcher for the red-block pickup package.

구현은 scripts/red_block/ 아래에 있습니다.
"""

from __future__ import annotations

import runpy
from pathlib import Path

runpy.run_path(
    str(Path(__file__).resolve().parent / "red_block" / "pick.py"),
    run_name="__main__",
)
