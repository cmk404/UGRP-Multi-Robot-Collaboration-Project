#!/usr/bin/env python3
"""Read-only UGRP evidence → TensorBoard events (optional dependencies)."""
import sys
from pathlib import Path

if __name__ == '__main__':
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from scripts.tensorboard_tools.export import main
    raise SystemExit(main())
