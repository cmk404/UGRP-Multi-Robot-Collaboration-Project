#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from harness.execution_trace import trace_root
from harness.trace_analysis import analyze_run


def resolve_run(value: str) -> Path:
    root = trace_root()
    if value == "latest":
        dirs = [p for p in root.iterdir() if p.is_dir()] if root.exists() else []
        if not dirs:
            raise FileNotFoundError(f"no REAL traces under {root}")
        return max(dirs, key=lambda p: p.stat().st_mtime)
    p = Path(value)
    return p if p.is_dir() else root / value


def main() -> int:
    parser = argparse.ArgumentParser(description="Analyze one durable REAL MasterPi execution trace")
    parser.add_argument("run", nargs="?", default="latest", help="run id, run directory, or 'latest'")
    args = parser.parse_args()
    run_dir = resolve_run(args.run)
    if not run_dir.is_dir():
        raise FileNotFoundError(run_dir)
    summary = analyze_run(run_dir, write=True)
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
