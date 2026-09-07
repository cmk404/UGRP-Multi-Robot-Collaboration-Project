#!/usr/bin/env python3
"""Create and inspect a physically executable MasterPi digital-twin calibration plan.

The plan intentionally uses only the live-safe chassis command range (31..40)
and multiple pulse durations so motor lag/force/damping can be identified rather
than collapsed into one endpoint gain.  Ground truth must be measured from an
external reference (floor grid, overhead camera, motion capture, etc.).
"""
from __future__ import annotations

import argparse
import json
import random
from pathlib import Path

DIRECTIONS = ("forward", "backward", "left", "right", "rotate-left", "rotate-right")
COMMANDS = (31, 35, 40)
DRIVE_DURATIONS_S = (0.20, 0.55)
COAST_S = 0.50
FIT_REPEATS = 2
HOLDOUT_REPEATS = 1


def make_plan(seed: int = 20260829) -> list[dict]:
    rows: list[dict] = []
    index = 0
    for direction in DIRECTIONS:
        for command in COMMANDS:
            for drive_s in DRIVE_DURATIONS_S:
                for rep in range(FIT_REPEATS + HOLDOUT_REPEATS):
                    index += 1
                    split = "holdout" if rep >= FIT_REPEATS else "fit"
                    rows.append({
                        "trial_id": f"chassis-{index:03d}",
                        "split": split,
                        "direction": direction,
                        "command": command,
                        "drive_s": drive_s,
                        "coast_s": COAST_S,
                        "repeat": rep + 1,
                        "dx_m": None,
                        "dy_m": None,
                        "dyaw_deg": None,
                        "peak_speed_mps": None,
                        "stop_distance_m": None,
                        "measurement_source": None,
                        "notes": None,
                    })
    rng = random.Random(seed)
    rng.shuffle(rows)
    return rows


def write_plan(path: Path, seed: int) -> None:
    rows = make_plan(seed)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as fh:
        for row in rows:
            fh.write(json.dumps(row, ensure_ascii=False, sort_keys=True) + "\n")


def load(path: Path) -> list[dict]:
    rows = []
    for raw in path.read_text(encoding="utf-8").splitlines():
        raw = raw.strip()
        if raw:
            rows.append(json.loads(raw))
    return rows


def status(rows: list[dict]) -> dict:
    complete = [r for r in rows if all(r.get(k) is not None for k in ("dx_m", "dy_m", "dyaw_deg"))]
    fit = [r for r in complete if r.get("split") == "fit"]
    hold = [r for r in complete if r.get("split") == "holdout"]
    return {
        "total_trials": len(rows),
        "completed_trials": len(complete),
        "fit_completed": len(fit),
        "holdout_completed": len(hold),
        "required_for_fitter": len(fit) >= 6,
        "held_out_gate_count_met": len(hold) >= 20,
        "next_trial": next((r for r in rows if r not in complete), None),
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="command", required=True)
    make = sub.add_parser("create")
    make.add_argument("output", type=Path)
    make.add_argument("--seed", type=int, default=20260829)
    show = sub.add_parser("status")
    show.add_argument("plan", type=Path)
    args = ap.parse_args()
    if args.command == "create":
        write_plan(args.output, args.seed)
        print(json.dumps(status(load(args.output)), ensure_ascii=False, indent=2))
        return 0
    print(json.dumps(status(load(args.plan)), ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
