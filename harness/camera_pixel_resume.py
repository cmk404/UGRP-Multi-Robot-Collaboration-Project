"""Input-only validation and policy replay for pixel-servo continuation."""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import shutil


def canonical(value: object) -> object:
    return json.loads(json.dumps(value, sort_keys=True))


def digest(value: object) -> str:
    encoded = json.dumps(canonical(value), sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(encoded).hexdigest()


def file_sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _contained(root: Path, relative: str) -> Path:
    path = (root / relative).resolve()
    path.relative_to(root.resolve())
    return path


def read_verified_image(root: Path, record: dict) -> bytes:
    path = _contained(root, record["path"])
    payload = path.read_bytes()
    if hashlib.sha256(payload).hexdigest() != record["sha256"]:
        raise ValueError(f"parent image hash mismatch: {record['path']}")
    return payload


def materialize_verified_image(parent: Path, output: Path, record: dict) -> None:
    source = _contained(parent, record["path"])
    destination = _contained(output, record["path"])
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists():
        raise ValueError(f"resume image destination already exists: {record['path']}")
    try:
        os.link(source, destination)
    except OSError:
        shutil.copy2(source, destination)
    if file_sha256(destination) != record["sha256"]:
        destination.unlink(missing_ok=True)
        raise ValueError(f"materialized image hash mismatch: {record['path']}")


def load_resume_parent(parent: Path, *, startup_commands: list, seed: int,
                       active_robot: str, render_width: int, render_height: int,
                       total_rounds: int, input_boundary: str) -> tuple[dict, list[list[dict]]]:
    """Validate a completed parent without reading its evaluator artifacts."""
    parent = parent.expanduser().resolve()
    result_path = parent / "result.json"
    report = json.loads(result_path.read_text())
    config = report["config"]
    completed = int(report["rounds_completed"])
    if report.get("error") is not None or completed != int(config["rounds"]):
        raise ValueError("resume parent is not a successful fixed-budget run")
    if not 0 < completed < total_rounds:
        raise ValueError("total rounds must exceed the completed parent rounds")
    expected = {
        "seed": seed, "active_robot": active_robot,
        "render_width": render_width, "render_height": render_height,
        "settle_seconds": 1.0, "weld": False, "policy": "pixel-servo",
    }
    for key, value in expected.items():
        if config.get(key) != value:
            raise ValueError(f"resume parent config mismatch: {key}")
    if report.get("config_sha256") != digest(config):
        raise ValueError("resume parent config hash mismatch")
    if report.get("input_boundary") != input_boundary:
        raise ValueError("resume parent input boundary mismatch")
    if report.get("input_boundary_sha256") != hashlib.sha256(input_boundary.encode()).hexdigest():
        raise ValueError("resume parent input boundary hash mismatch")
    if report.get("startup_commands") != canonical(startup_commands):
        raise ValueError("resume parent startup commands mismatch")

    robot_ids = ("r1", "r3") if active_robot == "both" else (active_robot,)
    grouped = [[] for _ in range(completed)]
    seen = set()
    for call in report["calls"]:
        rid, index = call["robot_id"], int(call["round"])
        if rid not in robot_ids or not 0 <= index < completed or (rid, index) in seen:
            raise ValueError("resume parent call coverage is invalid")
        expected_active = active_robot != "both" or rid == ("r1", "r3")[index % 2]
        if bool(call["active"]) != expected_active:
            raise ValueError(f"resume parent active schedule mismatch: {rid} round {index}")
        if set(call.get("images", {})) != {"own", "top"}:
            raise ValueError(f"resume parent image keys mismatch: {rid} round {index}")
        for record in call["images"].values():
            read_verified_image(parent, record)
        grouped[index].append(call)
        seen.add((rid, index))
    if seen != {(rid, index) for rid in robot_ids for index in range(completed)}:
        raise ValueError("resume parent call coverage is incomplete")
    for calls in grouped:
        calls.sort(key=lambda call: robot_ids.index(call["robot_id"]))
        top_hashes = {call["images"]["top"]["sha256"] for call in calls}
        if len(top_hashes) != 1:
            raise ValueError("resume parent shared top image mismatch")
    return report, grouped


def verify_live_last_input(live_own: dict[str, bytes], live_top: bytes,
                           calls: list[dict], index: int) -> dict:
    """Require live pre-action camera bytes to match the final parent inputs."""
    expected_ids = {call["robot_id"] for call in calls}
    if set(live_own) != expected_ids:
        raise ValueError(f"resume live own image coverage mismatch: round {index}")
    top_hash = hashlib.sha256(live_top).hexdigest()
    own_hashes = {}
    for call in calls:
        rid = call["robot_id"]
        own_hashes[rid] = hashlib.sha256(live_own[rid]).hexdigest()
        if own_hashes[rid] != call["images"]["own"]["sha256"]:
            raise ValueError(f"resume live own image mismatch: {rid} round {index}")
        if top_hash != call["images"]["top"]["sha256"]:
            raise ValueError(f"resume live top image mismatch: {rid} round {index}")
    return {"round": index, "top_sha256": top_hash, "own_sha256": own_hashes}


def replay_recorded_call(controller, call: dict, parent: Path, output: Path) -> dict:
    """Replay one saved RGB input and require every policy output to match."""
    before_history = canonical(controller.history)
    expected_input = digest({"images": call["images"], "history": before_history,
                             "active": bool(call["active"])})
    if call.get("input_sha256") != expected_input:
        raise ValueError(f"parent input hash mismatch: {call['robot_id']} round {call['round']}")
    own = read_verified_image(parent, call["images"]["own"])
    top = read_verified_image(parent, call["images"]["top"])
    action = canonical(controller.step(own, top, active=bool(call["active"])))
    comparisons = {
        "action": action,
        "observation": canonical(controller.last_observation),
        "decision": canonical(controller.last_decision),
        "history": canonical(controller.history),
    }
    for field, actual in comparisons.items():
        if actual != call[field]:
            raise ValueError(
                f"parent {field} replay mismatch: {call['robot_id']} round {call['round']}"
            )
    materialize_verified_image(parent, output, call["images"]["own"])
    materialize_verified_image(parent, output, call["images"]["top"])
    return action
