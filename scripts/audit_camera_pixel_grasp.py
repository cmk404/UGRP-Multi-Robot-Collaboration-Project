#!/usr/bin/env python3
"""Replay a pixel-grasp run solely from recorded RGB and own-command history."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


EXPECTED_INPUT_BOUNDARY = (
    "current/prior own RGB, current/prior shared top RGB, and this robot's issued command "
    "history only; deterministic pixel-servo; no simulator state, geometry, "
    "evaluator feedback, peer commands, or model calls"
)


def _canonical(value: object) -> object:
    return json.loads(json.dumps(value, sort_keys=True))


def _digest(value: object) -> str:
    encoded = json.dumps(_canonical(value), sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(encoded).hexdigest()


def _read_image(root: Path, record: dict) -> bytes:
    path = (root / record["path"]).resolve()
    path.relative_to(root.resolve())
    data = path.read_bytes()
    assert hashlib.sha256(data).hexdigest() == record["sha256"], f"image hash mismatch: {path}"
    return data


def audit(root: Path | str) -> dict:
    from harness.camera_grasp_controller import STARTUP_COMMANDS
    from harness.camera_pixel_grasp import PixelGraspController

    root = Path(root).resolve()
    # Deliberately load no evaluation-only JSONL, simulator, or truth-derived data.
    report = json.loads((root / "result.json").read_text())
    startup = report["startup_commands"]
    config = report["config"]
    assert startup == _canonical(STARTUP_COMMANDS), "startup commands differ from static policy"
    assert report["config_sha256"] == _digest(config), "config hash mismatch"
    boundary = report["input_boundary"]
    assert config["policy"] == "pixel-servo", "unexpected policy"
    assert boundary == EXPECTED_INPUT_BOUNDARY, "unexpected input boundary"
    assert report["input_boundary_sha256"] == hashlib.sha256(boundary.encode()).hexdigest(), (
        "input boundary hash mismatch"
    )
    calls = report["calls"]
    robot_ids = ("r1", "r3") if config["active_robot"] == "both" else (config["active_robot"],)
    completed = int(report["rounds_completed"])
    assert completed == int(config["rounds"]), "run did not complete its fixed round budget"
    controllers = {
        rid: PixelGraspController(rid, startup_commands=startup) for rid in robot_ids
    }
    for rid, controller in controllers.items():
        assert _canonical(controller.history) == report["startup_history"][rid], (
            f"startup history mismatch: {rid}"
        )

    seen = set()
    rows = []
    top_by_round = {}
    for call in calls:
        rid, index = call["robot_id"], int(call["round"])
        assert rid in controllers, f"unexpected robot: {rid}"
        assert (rid, index) not in seen, "duplicate round/robot call"
        expected_active = (
            True
            if config["active_robot"] != "both"
            else rid == ("r1", "r3")[index % 2]
        )
        assert bool(call["active"]) == expected_active, f"active schedule mismatch: {rid} round {index}"
        seen.add((rid, index))
        own = _read_image(root, call["images"]["own"])
        top = _read_image(root, call["images"]["top"])
        top_hash = hashlib.sha256(top).hexdigest()
        if index in top_by_round:
            assert top_by_round[index] == top_hash, f"different shared top image in round {index}"
        top_by_round[index] = top_hash
        before_history = _canonical(controllers[rid].history)
        assert call["input_sha256"] == _digest(
            {"images": call["images"], "history": before_history, "active": call["active"]}
        ), f"input boundary mismatch: {rid} round {index}"
        action = _canonical(controllers[rid].step(own, top, active=bool(call["active"])))
        assert action == call["action"], f"action replay mismatch: {rid} round {index}"
        assert _canonical(controllers[rid].last_observation) == call["observation"], (
            f"observation replay mismatch: {rid} round {index}"
        )
        assert _canonical(controllers[rid].last_decision) == call["decision"], (
            f"decision replay mismatch: {rid} round {index}"
        )
        assert _canonical(controllers[rid].history) == call["history"], (
            f"history replay mismatch: {rid} round {index}"
        )
        rows.append({"round": index, "robot_id": rid, "active": bool(call["active"]),
                     "own_sha256": call["images"]["own"]["sha256"], "top_sha256": top_hash})

    for rid in robot_ids:
        indices = sorted(index for seen_rid, index in seen if seen_rid == rid)
        assert indices == list(range(completed)), f"incomplete round coverage: {rid}"
    assert len(top_by_round) == completed, "shared-top round count mismatch"
    assert len(rows) == completed * len(robot_ids), "actor/round call count mismatch"
    assert rows, "no calls to audit"
    return {
        "ok": True,
        "policy": config["policy"],
        "calls": len(rows),
        "rounds": len(top_by_round),
        "source_sha": report["source_sha"],
        "replay": rows,
        "checked": (
            "image hashes, shared top RGB, startup history, input-boundary hashes, and exact "
            "JSON-normalized action/observation/decision/history replay; evaluator never read"
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    args = parser.parse_args()
    result = audit(args.run_dir)
    output = args.run_dir.resolve() / "pixel-input-audit.json"
    output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({key: value for key, value in result.items() if key != "replay"}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
