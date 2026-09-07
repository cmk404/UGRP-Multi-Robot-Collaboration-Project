#!/usr/bin/env python3
"""Launcher for red-block gaze tracking.

From any Mac that can SSH to the robot as ugrp1:

    python3 scripts/track_red_block.py
    python3 scripts/track_red_block.py --host 100.119.44.65
"""

from __future__ import annotations

import runpy
from pathlib import Path

runpy.run_path(
    str(Path(__file__).resolve().parent / "red_block" / "track.py"),
    run_name="__main__",
)
