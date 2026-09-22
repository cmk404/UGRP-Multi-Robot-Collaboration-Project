#!/usr/bin/env python3
"""Prepare and audit the finite RGB communication connection pilot.

This CLI is offline-only.  It does not import a simulator or model client and
does not submit jobs.  ``admit`` is the mandatory go/no-go check a future
single submitter must pass before using the ordered schedule.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from harness.rgb_communication_evaluation import (  # noqa: E402
    ContractError,
    build_manifest,
    evaluate_manifest,
    execution_blockers,
    finite_schedule,
    validate_manifest,
)


def read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        raise ContractError(f"cannot read JSON object: {path}") from exc
    if not isinstance(value, dict):
        raise ContractError(f"expected JSON object: {path}")
    return value


def write_new_json(path: Path, value: dict[str, Any]) -> None:
    if path.exists():
        raise ContractError(f"refusing to overwrite: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n")


def git_source() -> tuple[str, bool]:
    sha = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    dirty = subprocess.check_output(["git", "status", "--porcelain"], cwd=ROOT, text=True).strip()
    return sha, not bool(dirty)


def prepare(args: argparse.Namespace) -> int:
    source_sha, source_clean = git_source()
    manifest = build_manifest(read_json(args.protocol), source_sha=source_sha, source_clean=source_clean)
    write_new_json(args.output, manifest)
    print(json.dumps({
        "manifest": str(args.output),
        "planned_denominator": manifest["planned_denominator"],
        "source_sha": source_sha,
        "source_clean": source_clean,
        "execution_ready": not execution_blockers(manifest),
        "execution_blockers": execution_blockers(manifest),
    }, ensure_ascii=False, indent=2))
    return 0


def validate(args: argparse.Namespace) -> int:
    manifest = read_json(args.manifest)
    validate_manifest(manifest)
    blockers = execution_blockers(manifest)
    print(json.dumps({
        "valid": True,
        "planned_denominator": manifest["planned_denominator"],
        "execution_ready": not blockers,
        "execution_blockers": blockers,
    }, ensure_ascii=False, indent=2))
    return 0


def schedule(args: argparse.Namespace) -> int:
    manifest = read_json(args.manifest)
    rows = finite_schedule(manifest, require_ready=args.require_ready)
    print(json.dumps({
        "strategy": manifest["execution"],
        "schedule": rows,
    }, ensure_ascii=False, indent=2))
    return 0


def admit(args: argparse.Namespace) -> int:
    manifest = read_json(args.manifest)
    blockers = execution_blockers(manifest)
    result = {
        "go": not blockers,
        "blockers": blockers,
        "next_step": (
            "single named submitter may consume the finite schedule"
            if not blockers else
            "do not submit simulation/model/remote work; resolve every blocker and regenerate the manifest"
        ),
    }
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if not blockers else 3


def evaluate(args: argparse.Namespace) -> int:
    manifest = read_json(args.manifest)
    report = evaluate_manifest(manifest, args.artifacts)
    write_new_json(args.output, report)
    print(json.dumps({
        "report": str(args.output),
        "planned_denominator": report["planned_denominator"],
        "mission_complete_numerator": report["mission_complete_numerator"],
        "outcome_counts": report["outcome_counts"],
    }, ensure_ascii=False, indent=2))
    return 0


def parser() -> argparse.ArgumentParser:
    value = argparse.ArgumentParser(description=__doc__)
    commands = value.add_subparsers(dest="command", required=True)

    command = commands.add_parser("prepare", help="freeze a six-run provisional manifest")
    command.add_argument("--protocol", type=Path, required=True)
    command.add_argument("--output", type=Path, required=True)
    command.set_defaults(handler=prepare)

    command = commands.add_parser("validate", help="validate structure and show execution blockers")
    command.add_argument("--manifest", type=Path, required=True)
    command.set_defaults(handler=validate)

    command = commands.add_parser("schedule", help="print the finite randomized schedule")
    command.add_argument("--manifest", type=Path, required=True)
    command.add_argument("--require-ready", action="store_true",
                         help="fail unless every go/no-go gate and budget is fixed")
    command.set_defaults(handler=schedule)

    command = commands.add_parser("admit", help="mandatory no-side-effect execution go/no-go")
    command.add_argument("--manifest", type=Path, required=True)
    command.set_defaults(handler=admit)

    command = commands.add_parser("evaluate", help="audit runtime and separate evaluator artifacts")
    command.add_argument("--manifest", type=Path, required=True)
    command.add_argument("--artifacts", type=Path, required=True)
    command.add_argument("--output", type=Path, required=True)
    command.set_defaults(handler=evaluate)
    return value


def main() -> int:
    args = parser().parse_args()
    try:
        return args.handler(args)
    except ContractError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
