#!/usr/bin/env python3
"""Run protocol fixtures with two fake local ports. No RGB inference or robot motion."""

from __future__ import annotations

import argparse
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import platform
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from harness.task_stage_sync import DONE_CHECKS, READY_CHECKS, STAGES, StageReport, TaskPlan, TaskStageSync

CASES = ("normal", "delayed_ready", "delayed_done", "hold_reobserve", "report_gap",
         "missing_support", "command_only", "old_stage_permission", "plan_change")


class FixtureRobotPort:
    """Example interface only: all observations are synthetic fixture reports."""

    def __init__(self, robot_id: str, case: str) -> None:
        self.robot_id = robot_id
        self.case = case
        self.sequence = 0
        self.issued: list[str] = []

    def report(self, sync, now, status="READY", *, checks=None, command_id=None):
        self.sequence += 1
        if checks is None:
            checks = DONE_CHECKS[sync.stage] if status == "DONE" else READY_CHECKS[sync.stage]
        prefix = f"fixture://{self.case}/{self.robot_id}/{self.sequence}"
        return StageReport(sync.plan.task_id, sync.run_id, sync.plan.plan_version, sync.stage,
                           sync.epoch, self.robot_id, self.sequence, status, now, now, now,
                           prefix, prefix + "/own", prefix + "/top", .95,
                           tuple(sorted(checks)), command_id, "SYNTHETIC protocol fixture").to_dict()

    def submit(self, command_id):
        self.issued.append(command_id)  # An ACK/record only, never a DONE report.


def run_case(case: str, plan: TaskPlan) -> dict:
    if case not in CASES:
        raise ValueError("unknown fixture")
    started = time.monotonic()
    sync = TaskStageSync(plan)
    ports = {r: FixtureRobotPort(r, case) for r in plan.participants}
    first, second = plan.participants
    now = 0.0
    assertions = 0

    def check(condition):
        nonlocal assertions
        assertions += 1
        if not condition:
            raise AssertionError(f"fixture {case}: assertion {assertions} at {sync.stage}")

    def send(robot, status="READY", **kw):
        return sync.receive(robot, ports[robot].report(sync, now, status, **kw), now_s=now)

    def ready_pair():
        for robot in ports:
            check(send(robot))
        decision = sync.authorize(now_s=now)
        check(decision["phase"] == "GO" and decision["permission"] is not None)
        return decision["permission"]

    def dispatch(robot, permission, command_id):
        return sync.dispatch(robot, permission, command_id, now_s=now, duration_s=.05,
                             submit=lambda: ports[robot].submit(command_id))

    error = None
    try:
        for stage_index, stage in enumerate(STAGES):
            now = float(stage_index * 3 + 1)
            check(sync.stage == stage)
            if case == "delayed_ready":
                for tick in range(8):
                    now = stage_index * 3 + 1 + tick / 10
                    check(send(first))
                    check(sync.authorize(now_s=now)["permission"] is None)
                    check(not dispatch(first, None, stage + "-too-early"))
                now += .1
            if case == "missing_support" and stage == "RELEASE":
                check(send(first))
                check(send(second, checks={"stopped"}))
                check(sync.authorize(now_s=now)["phase"] == "HOLD")
                check(not dispatch(first, None, "premature-release"))
                break
            permission = ready_pair()
            if case == "report_gap" and stage == "LIFT":
                now += .7
                check(sync.authorize(now_s=now)["phase"] == "HOLD")
                check(not dispatch(first, permission, "expired"))
                now += .01
                check(send(first))
                check(sync.authorize(now_s=now)["permission"] is None)
                now += .01
                permission = ready_pair()
            if case == "plan_change" and stage == "CARRY":
                now += .01
                sync.update_plan(replace(sync.plan, plan_version=2, goal_region="alternate-goal"), now_s=now)
                check(not dispatch(first, permission, "old-plan"))
                now += .01
                permission = ready_pair()
            commands = {r: stage + "-" + r for r in ports}
            for robot, command in commands.items():
                check(dispatch(robot, permission, command))
            check(not sync.advance(now_s=now))
            if case == "command_only":
                now += .7
                check(sync.authorize(now_s=now)["phase"] == "HOLD")
                check(sync.stage == "GRASP")
                break
            now += .08
            if case == "hold_reobserve":
                sync.hold("fixture_uncertainty", now_s=now)
                check(not dispatch(first, permission, "old-permission"))
                now += .01
            check(send(first, "DONE", command_id=commands[first]))
            check(not sync.advance(now_s=now))
            if case == "delayed_done":
                now += .2
                check(not dispatch(first, permission, "repeat-after-done"))
                check(not sync.advance(now_s=now))
            check(send(second, "DONE", command_id=commands[second]))
            check(sync.advance(now_s=now))
            if case == "old_stage_permission":
                check(not dispatch(first, permission, "replay-previous-stage"))
        final = sync.authorize(now_s=now)
        expected = "HOLD" if case in ("missing_support", "command_only") else "FINISH"
        check(final["phase"] == expected)
    except Exception as exc:
        error = f"{type(exc).__name__}: {exc}"
        final = sync.authorize(now_s=now)
    return {"case": case, "passed": error is None, "error": error,
            "scope": "synthetic protocol fixture; no image inference, physics, network or hardware",
            "phase": final["phase"], "stage": final["stage"], "assertions": assertions,
            "issued_commands": {r: p.issued for r, p in ports.items()},
            "command_count": sum(len(p.issued) for p in ports.values()),
            "report_count": sum(e["event"] == "REPORT" for e in sync.events),
            "hold_count": sum(e["event"] == "HOLD" for e in sync.events),
            "fixture_clock_s": now, "wall_time_s": time.monotonic()-started,
            "model_calls": 0, "cost_usd": 0, "physical_success_evaluated": False,
            "events": sync.events}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    source = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    dirty = subprocess.check_output(["git", "status", "--porcelain"], cwd=ROOT, text=True)
    if dirty:
        parser.error("commit source and clear untracked work before recording a cohort")
    map_path = ROOT / "maps/navigation/open.json"
    fixture_plan = TaskPlan.from_dict(json.loads((ROOT / "examples/task_stage_sync/plan.json").read_text()))
    if hashlib.sha256(map_path.read_bytes()).hexdigest() != fixture_plan.map_sha256:
        parser.error("example plan/static map hash mismatch")
    args.output.mkdir(parents=True, exist_ok=False)
    results = [run_case(case, fixture_plan) for case in CASES]
    evidence = []
    for result in results:
        path = args.output / (result["case"] + ".json")
        path.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
        evidence.append({"path": path.name, "sha256": hashlib.sha256(path.read_bytes()).hexdigest()})
    summary = {"schema": "ugrp.task_stage_sync_fixture.v1", "source_sha": source,
               "environment": {"python": sys.version, "platform": platform.platform()},
               "scope": "protocol fixture cohort; not physical or independent robot validation",
               "plan": fixture_plan.to_dict(), "evidence": evidence,
               "results": [{k: v for k, v in r.items() if k != "events"} for r in results]}
    (args.output / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n")
    for result in results:
        print(f"{result['case']}: {'PASS' if result['passed'] else 'FAIL'} / {result['phase']} / {result['command_count']} commands")
    return 0 if all(r["passed"] for r in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
