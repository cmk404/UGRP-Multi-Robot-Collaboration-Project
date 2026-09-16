#!/usr/bin/env python3
"""RGB-only JSON-lines ACT worker; model and data loading stay separate."""
from __future__ import annotations

import argparse
import base64
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def decode_request(request):
    if not isinstance(request, dict) or set(request) != {"own_rgb", "top_rgb"}:
        raise ValueError("only own_rgb and top_rgb are allowed")
    if any(not isinstance(value, str) or len(value) > 8_000_000 for value in request.values()):
        raise ValueError("invalid RGB payload")
    return tuple(base64.b64decode(request[key], validate=True) for key in ("own_rgb", "top_rgb"))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--model-dir", required=True, type=Path)
    args = p.parse_args()
    import torch
    from harness.recovery_act import RecoveryAct
    torch.set_num_threads(2)
    actor = RecoveryAct.load(args.model_dir.resolve())
    print(json.dumps({"ready": True, "runtime_inputs": ["own_rgb", "top_rgb"]}), flush=True)
    for line in sys.stdin:
        try:
            own, top = decode_request(json.loads(line))
            result = actor.predict(own, top)
        except Exception as exc:
            print(json.dumps({"error": f"{type(exc).__name__}: {exc}"}), flush=True)
            return 1
        print(json.dumps(result, allow_nan=False), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
