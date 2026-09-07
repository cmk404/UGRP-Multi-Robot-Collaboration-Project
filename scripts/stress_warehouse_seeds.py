#!/usr/bin/env python3
"""Run the physical TEAM warehouse mission across deterministic layout seeds."""

from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
import json
from pathlib import Path
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from sim.multi_masterpi_production import MultiMasterPiProductionV2


def run_seed(
    seed: int,
    routes: tuple[tuple[str, str], ...],
    retarget_at_phase: str | None = None,
    retarget_destination: str | None = None,
) -> dict[str, object]:
    world = MultiMasterPiProductionV2(seed=int(seed), render=False)
    try:
        retargeted = False

        def maybe_retarget() -> None:
            nonlocal retargeted
            if retargeted or not retarget_at_phase or not retarget_destination:
                return
            if any(event.get("phase") == retarget_at_phase for event in world.warehouse_trace):
                world.request_warehouse_goal_update(
                    retarget_destination,
                    revision=world._warehouse_goal_snapshot()[1] + 1,
                    reason="stress_injected_moving_goal",
                )
                retargeted = True

        if retarget_at_phase and retarget_destination:
            world.frame_callback = maybe_retarget
        starts = {
            spec.cargo_id: [round(float(v), 4) for v in spec.start_xyz[:2]]
            for spec in world.warehouse_specs
        }
        terrain = [
            {
                "id": item.terrain_id,
                "kind": item.kind,
                "center": [round(float(v), 4) for v in item.center_xy],
                "half_extents": [round(float(v), 4) for v in item.half_extents_xy],
                "height_m": round(float(item.height_m), 4),
                "traversable": bool(item.traversable),
            }
            for item in world.warehouse_terrain_specs
        ]
        goals = {
            spec.cargo_id: [
                round(float(v), 4)
                for v in world.warehouse_zone_positions[spec.cargo_id][routes[-1][1]][:2]
            ]
            for spec in world.warehouse_specs
        }
        started = time.monotonic()
        route_results = []
        results = None
        for source, destination in routes:
            commands = [
                {
                    "robot_id": rid,
                    "action": "team_zone_transfer",
                    "mission_id": f"warehouse_seed_stress_{seed}_{source}_{destination}",
                    "source_zone": source,
                    "destination_zone": destination,
                    "selector": "all",
                }
                for rid in world.robot_ids
            ]
            results = world.act_parallel(commands)
            first = next(iter(results.values()))
            route_results.append({
                "source": source,
                "destination": destination,
                "ok": all(result.ok for result in results.values()),
                "reason": first.reason,
            })
            if not route_results[-1]["ok"]:
                break
        state = world.warehouse_state()
        first = next(iter(results.values())) if results else None
        return {
            "seed": int(seed),
            "ok": len(route_results) == len(routes) and all(
                bool(result["ok"]) for result in route_results
            ),
            "status": state["status"],
            "moved_count": state["moved_count"],
            "remaining_ids": state["remaining_ids"],
            "wall_s": round(time.monotonic() - started, 3),
            "sim_s": round(float(world.data.time), 3),
            "starts": starts,
            "terrain": terrain,
            "goals": goals,
            "reason": first.reason if first else "no route executed",
            "routes": route_results,
            "retargeted": retargeted,
            "final_goal": state.get("current_goal"),
            "motion_metrics": state.get("motion_metrics"),
        }
    finally:
        world.close()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--start", type=int, default=0)
    parser.add_argument("--count", type=int, default=32)
    parser.add_argument("--jobs", type=int, default=4)
    parser.add_argument(
        "--sequence", default="A:B",
        help="comma-separated stateful legs, for example A:B,B:C,C:B,B:A",
    )
    parser.add_argument(
        "--retarget-at-phase",
        help="inject one goal revision after this trace phase, e.g. warehouse_route_leg_started",
    )
    parser.add_argument(
        "--retarget-destination",
        choices=("A", "B", "C"),
        help="new destination for the injected goal revision",
    )
    args = parser.parse_args()
    seeds = list(range(args.start, args.start + args.count))
    routes = tuple(
        tuple(part.strip().upper() for part in leg.split(":", 1))
        for leg in args.sequence.split(",") if leg.strip()
    )
    if not routes or any(len(route) != 2 for route in routes):
        raise SystemExit("--sequence must contain SOURCE:DESTINATION legs")
    rows: list[dict[str, object]] = []
    with ProcessPoolExecutor(max_workers=max(1, args.jobs)) as pool:
        futures = {
            pool.submit(
                run_seed, seed, routes,
                args.retarget_at_phase, args.retarget_destination,
            ): seed
            for seed in seeds
        }
        for future in as_completed(futures):
            rows.append(future.result())
    rows.sort(key=lambda item: int(item["seed"]))
    failures = [row for row in rows if not bool(row["ok"])]
    summary = {
        "ok": not failures,
        "seed_start": args.start,
        "seed_count": len(rows),
        "passed": len(rows) - len(failures),
        "failed": len(failures),
        "unique_start_layouts": len({
            json.dumps(row["starts"], sort_keys=True) for row in rows
        }),
        "unique_terrain_layouts": len({
            json.dumps(row["terrain"], sort_keys=True) for row in rows
        }),
        "max_wall_s": max((float(row["wall_s"]) for row in rows), default=0.0),
        "max_sim_s": max((float(row["sim_s"]) for row in rows), default=0.0),
        "sequence": [list(route) for route in routes],
        "retarget": {
            "phase": args.retarget_at_phase,
            "destination": args.retarget_destination,
            "applied": sum(bool(row.get("retargeted")) for row in rows),
        },
        "max_loaded_lateral_ratio": max(
            (float((row.get("motion_metrics") or {}).get("loaded_lateral_ratio") or 0.0) for row in rows),
            default=0.0,
        ),
        "max_empty_lateral_segments": max(
            (int((row.get("motion_metrics") or {}).get("empty_lateral_segments") or 0) for row in rows),
            default=0,
        ),
        "failures": failures,
    }
    print(json.dumps(summary, ensure_ascii=False, sort_keys=True))
    return 0 if not failures else 1


if __name__ == "__main__":
    raise SystemExit(main())
