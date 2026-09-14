#!/usr/bin/env python3
"""Audit and deterministically replay a saved known-map navigation run."""
from __future__ import annotations

import argparse
import ast
import hashlib
import inspect
import json
from pathlib import Path
import sys
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from harness.known_map_navigation import KnownMapNavigator
from harness.heading_map_navigation import HeadingMapNavigator
from scripts.run_known_map_navigation import navigator_class
from sim.authored_navigation_map import map_sha256, validate_map


_ROW_FIELDS = {"frame_id", "map_sha256", "images", "issued_history", "decision"}
_IMAGE_FIELDS = {"path", "sha256"}


def _sha(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def _json(path: Path) -> Any:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"cannot read JSON artifact: {path.name}") from exc


def _inside(root: Path, relative: Any) -> Path:
    if not isinstance(relative, str) or not relative or Path(relative).is_absolute():
        raise ValueError("manifest paths must be non-empty relative paths")
    path = (root / relative).resolve()
    if not path.is_relative_to(root):
        raise ValueError(f"artifact path escapes run directory: {relative}")
    return path


def _canonical(value: Any) -> Any:
    return json.loads(json.dumps(value, sort_keys=True, allow_nan=False))


def _verify_manifest(root: Path) -> dict[str, str]:
    manifest = _json(root / "manifest.json")
    if not isinstance(manifest, dict) or any(not isinstance(k, str) or not isinstance(v, str)
                                             for k, v in manifest.items()):
        raise ValueError("manifest must map paths to SHA-256 strings")
    actual = {str(path.relative_to(root)) for path in root.rglob("*")
              if path.is_file() and path.name != "manifest.json"}
    if set(manifest) != actual:
        raise ValueError("manifest does not exactly cover run files")
    for relative, expected in manifest.items():
        if len(expected) != 64 or any(ch not in "0123456789abcdef" for ch in expected):
            raise ValueError(f"invalid manifest digest: {relative}")
        path = _inside(root, relative)
        if path.name == "manifest.json" or not path.is_file() or _sha(path.read_bytes()) != expected:
            raise ValueError(f"manifest hash mismatch: {relative}")
    return manifest


def _read_image(root: Path, record: Any, expected_path: str) -> bytes:
    if not isinstance(record, dict) or set(record) != _IMAGE_FIELDS:
        raise ValueError("image record has unexpected fields")
    if record["path"] != expected_path:
        raise ValueError(f"unexpected sequential image path: {record.get('path')}")
    path = _inside(root, record["path"])
    value = path.read_bytes()
    if not isinstance(record["sha256"], str) or _sha(value) != record["sha256"]:
        raise ValueError(f"actor input image hash mismatch: {record['path']}")
    return value


def _inspect_actor_source(motion_style='holonomic') -> dict[str, Any]:
    classes = [KnownMapNavigator] + ([HeadingMapNavigator] if motion_style == 'heading' else [])
    imports: list[str] = []
    for cls in classes:
        tree = ast.parse(inspect.getsource(sys.modules[cls.__module__]))
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imports.extend(alias.name for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imports.append(node.module)
    forbidden = [name for name in imports if name == "mujoco" or name.startswith("mujoco.") or
                 (name.startswith("sim.") and name != "sim.authored_navigation_map")]
    if forbidden:
        raise ValueError(f"navigator imports simulator runtime: {forbidden}")

    runner_tree = ast.parse((ROOT / "scripts/run_known_map_navigation.py").read_text())
    calls = [node for node in ast.walk(runner_tree) if isinstance(node, ast.Call) and
             isinstance(node.func, ast.Attribute) and node.func.attr == "decide"]
    if len(calls) != 1 or len(calls[0].args) != 3 or calls[0].keywords:
        raise ValueError("runner must call navigator.decide with exactly own, top, frame")
    return {"forbidden_runtime_imports": [], "runner_decide_positional_inputs": 3}


def audit_run(path: str | Path) -> dict[str, Any]:
    """Verify artifact integrity and replay actor decisions without referee data."""
    root = Path(path).resolve()
    if not root.is_dir():
        raise ValueError("run path must be a directory")
    manifest = _verify_manifest(root)
    metadata = _json(root / "run.json")
    result = _json(root / "result.json")
    before = _json(root / "invariants-before.json")
    after = _json(root / "invariants-after.json")
    authored_map = validate_map(_json(root / "actor-map.json"))
    digest = map_sha256(authored_map)
    if metadata.get("schema") != "ugrp.known_map_run.v1":
        raise ValueError("unsupported run metadata")
    if any(value.get("map_sha256") != digest for value in (metadata, result, before, after)):
        raise ValueError("map hash differs across actor metadata and scene invariants")
    if before != after or before.get('added_navigation_camera') is not False:
        raise ValueError('camera/geometry invariants changed or extra navigation camera present')
    if metadata.get('source_sha') != result.get('source_sha'):
        raise ValueError('run/result source SHA mismatch')
    robot_id, condition = metadata.get("robot_id"), metadata.get("condition")
    if result.get("condition") != condition or condition not in {"map", "direct"}:
        raise ValueError("saved condition mismatch")
    if robot_id not in {"r1", "r3"}:
        raise ValueError("invalid saved robot id")
    # Records predating the motion-style option contain the unchanged holonomic baseline.
    motion_style = metadata.get('motion_style', 'holonomic')
    if result.get('motion_style', 'holonomic') != motion_style:
        raise ValueError('saved motion style mismatch')
    controller_class = navigator_class(motion_style)

    ledger_path = root / "actor-decisions.jsonl"
    rows = []
    for line_number, text in enumerate(ledger_path.read_text(encoding="utf-8").splitlines(), 1):
        try:
            row = json.loads(text)
        except json.JSONDecodeError as exc:
            raise ValueError(f"invalid actor row {line_number}") from exc
        if not isinstance(row, dict) or set(row) != _ROW_FIELDS:
            raise ValueError(f"actor row {line_number} has unexpected input fields")
        rows.append(row)
    if not rows:
        raise ValueError("actor decision ledger is empty")
    if result.get("decisions") != len(rows):
        raise ValueError("result decision count differs from ledger")

    actor = controller_class(authored_map, robot_id, condition)
    history: list[dict[str, Any]] = []
    replay = []
    terminal_seen = False
    for index, row in enumerate(rows):
        if row["frame_id"] != index or row["map_sha256"] != digest:
            raise ValueError(f"non-sequential frame or map hash at actor row {index}")
        if row["issued_history"] != history:
            raise ValueError(f"own issued_history chain mismatch at actor row {index}")
        if terminal_seen:
            raise ValueError("actor ledger continues after terminal decision")
        images = row["images"]
        if not isinstance(images, dict) or set(images) != {"own", "top"}:
            raise ValueError(f"actor row {index} has unexpected image inputs")
        own = _read_image(root, images["own"], f"rgb/{index:04d}-own.jpg")
        top = _read_image(root, images["top"], f"rgb/{index:04d}-top.jpg")
        replayed = actor.decide(own, top, index)
        if _canonical(replayed) != _canonical(row["decision"]):
            raise ValueError(f"deterministic actor replay mismatch at frame {index}")
        history.append(replayed["action"])
        terminal_seen = bool(replayed["done"])
        replay.append({"frame_id": index, "own_sha256": images["own"]["sha256"],
                       "top_sha256": images["top"]["sha256"], "status": replayed["status"]})

    expected_status = rows[-1]["decision"]["status"] if terminal_seen else "budget_exhausted"
    if result.get("actor_status") != expected_status:
        raise ValueError("result actor status differs from replay")
    source_check = _inspect_actor_source(motion_style)
    navigation_success = bool(isinstance(result.get("evaluation"), dict) and
                              result["evaluation"].get("success") is True)
    return {"audit_pass": True, "navigation_success": navigation_success,
            "condition": condition, "motion_style": motion_style, "robot_id": robot_id, "frames": len(rows),
            "map_sha256": digest, "manifest_files": len(manifest), "replay": replay,
            "source_check": source_check,
            "claim_limit": "artifact replay checks saved inputs and code boundaries; input logging does not cryptographically prove hidden state was absent at runtime"}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", type=Path)
    args = parser.parse_args()
    print(json.dumps(audit_run(args.path), sort_keys=True, indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
