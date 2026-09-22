"""Prepare/check offline, or execute a finite communication study on Colab.

No provisioning, retry, model choice, or gate override is implicit. Existing
scripts.colab_simulation_cli transports committed source and verifies recovery.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import platform

from harness.rgb_communication_study import (
    ContractError, prepare_manifest, preflight, read_json, run_study, run_trial, write_new_json,
)

ROOT = Path(__file__).resolve().parents[1]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    prepare = sub.add_parser("prepare")
    prepare.add_argument("--config", type=Path, required=True)
    prepare.add_argument("--output", type=Path, required=True)
    for name in ("check", "run", "trial"):
        action = sub.add_parser(name)
        action.add_argument("--manifest", type=Path, required=True)
        action.add_argument("--evidence-root", type=Path, required=True)
        if name != "check":
            action.add_argument("--output", type=Path, required=True)
        if name == "trial":
            action.add_argument("--run-id", required=True)
        if name == "run":
            action.add_argument("--allocation-wall-s", type=float)
    args = parser.parse_args(argv)
    try:
        if args.command == "prepare":
            result = prepare_manifest(read_json(args.config), ROOT)
            write_new_json(args.output, result)
        else:
            manifest = read_json(args.manifest)
            if args.command == "check":
                result = preflight(manifest, root=ROOT, evidence_root=args.evidence_root)
            else:
                if platform.system() != "Linux" or not Path("/content").is_dir():
                    raise ContractError("actual trials require the authorized Colab execution host")
                common = {"root": ROOT, "evidence_root": args.evidence_root,
                          "output": args.output}
                result = run_study(manifest, allocation_wall_s=args.allocation_wall_s, **common) if args.command == "run" else run_trial(
                    manifest, run_id=args.run_id, **common)
        print(json.dumps(result, ensure_ascii=False, allow_nan=False))
        return 2 if result.get("ready") is False else 0
    except (ContractError, OSError, KeyError, TypeError) as exc:
        # Provider exception strings can contain URLs/tokens. No raw traceback.
        parser.exit(2, f"study blocked: {type(exc).__name__}: {exc if isinstance(exc, ContractError) else 'invalid or inaccessible evidence'}\n")


if __name__ == "__main__":
    raise SystemExit(main())
