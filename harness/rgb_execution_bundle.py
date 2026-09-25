"""Versioned, immutable RGB execution contracts; stdlib-only offline checks.

The registry pins source files and declared behavior. A live backend must also
compare its applied scene/model/camera and measured macro schedule before any
actor may observe or issue a task command. A historical bundle is never runnable.
"""
from __future__ import annotations

import argparse
import ast
import hashlib
from importlib import metadata
import json
from pathlib import Path
import platform
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
REGISTRY = Path("config/rgb_execution_bundles")
SCHEMA = "ugrp.rgb_execution_bundle.v1"
RUNNABLE_ID = "rgb-standard-dispatch-v59"
RETIRED_IDS = frozenset({"rgb-adapter-contact-fine-boundary-identity-v5", "rgb-adapter-contact-fine-component-identity-v4", "rgb-adapter-contact-fine-progress-budget-v7", "rgb-adapter-contact-fine-replay-v2", "rgb-adapter-contact-fine-role-binding-v6", "rgb-adapter-contact-fine-solo-parity-v1", "rgb-adapter-contact-fine-visual-recovery-v3", "rgb-adapter-contact-fine-yaw-wheel-anchor-v8", "rgb-adapter-legacy-v1", "rgb-standard-dispatch-v10", "rgb-standard-dispatch-v11", "rgb-standard-dispatch-v12", "rgb-standard-dispatch-v13", "rgb-standard-dispatch-v14", "rgb-standard-dispatch-v15", "rgb-standard-dispatch-v16", "rgb-standard-dispatch-v17", "rgb-standard-dispatch-v18", "rgb-standard-dispatch-v19", "rgb-standard-dispatch-v2", "rgb-standard-dispatch-v20", "rgb-standard-dispatch-v21", "rgb-standard-dispatch-v22", "rgb-standard-dispatch-v23", "rgb-standard-dispatch-v24", "rgb-standard-dispatch-v25", "rgb-standard-dispatch-v26", "rgb-standard-dispatch-v27", "rgb-standard-dispatch-v28", "rgb-standard-dispatch-v29", "rgb-standard-dispatch-v3", "rgb-standard-dispatch-v30", "rgb-standard-dispatch-v31", "rgb-standard-dispatch-v32", "rgb-standard-dispatch-v33", "rgb-standard-dispatch-v4", "rgb-standard-dispatch-v40", "rgb-standard-dispatch-v41", "rgb-standard-dispatch-v42", "rgb-standard-dispatch-v43", "rgb-standard-dispatch-v44", "rgb-standard-dispatch-v46", "rgb-standard-dispatch-v47", "rgb-standard-dispatch-v48", "rgb-standard-dispatch-v49", "rgb-standard-dispatch-v5", "rgb-standard-dispatch-v50", "rgb-standard-dispatch-v53", "rgb-standard-dispatch-v54", "rgb-standard-dispatch-v56", "rgb-standard-dispatch-v6", "rgb-standard-dispatch-v7", "rgb-standard-dispatch-v8", "rgb-standard-dispatch-v9", "rgb-standard-dispatch-v45", "rgb-standard-dispatch-v51", "rgb-standard-dispatch-v55", "rgb-standard-dispatch-v57", "rgb-standard-dispatch-v58"})
BASELINE_ID = "dispatch-f1-local-contact-fine-v1"
REQUIRED_SOURCE_PATHS = frozenset({
    "harness/rgb_execution_bundle.py", "harness/rgb_skill_execution.py",
    "harness/rgb_execution_port.py", "harness/rgb_execution_contract.py",
    "harness/rgb_communication_study.py", "harness/rgb_communication_evaluation.py",
    "harness/rgb_communication_runtime.py", "harness/rgb_communication_async.py",
    "harness/solo_box_transport.py", "harness/visual_box_skill.py",
    "harness/camera_goal_transport.py", "harness/visual_attachment.py",
    "harness/visual_macro_runtime.py",
    "harness/camera_varied_start_student.py", "harness/grasp_student_inference.py",
    "harness/three_robot_mission.py", "harness/rgb_communication_planner.py",
    "harness/dispatch_skill_binding.py", "harness/camera_motion_identity.py",
    "scripts/research_dispatch_scene.py", "scripts/run_dispatch_e2e.py",
    "scripts/probe_dual_grasp_sync.py", "scripts/prepare_rgb_communication_replay.py",
    "scripts/run_rgb_communication_study.py", "scripts/submit_rgb_communication_study.py",
    # These subprocess entry points are launched by file path, so an import
    # closure alone cannot discover their inference/model implementation.
    "scripts/carry_input_worker.py", "scripts/pair_carry_act_worker.py",
    "sim/research_dispatch_arena.py", "sim/multi_masterpi_production.py",
    "sim/dispatch_contact_profile.py", "sim/camera_robot_port.py", "sim/act_map_suite.py",
})
RELEVANT_PACKAGES = ("mujoco", "numpy", "opencv-python", "opencv-python-headless",
                     "opencv-contrib-python", "Pillow")
REQUIRED_IDENTITY_BINDING = frozenset({"source.git_sha", "backend_descriptor.input_hashes",
    "backend_descriptor.map_instance_sha256", "backend_descriptor.reset_sha256",
    "backend_provenance.map_to_scene.scene_xml_sha256"})


def canonical_digest(value: object) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                     ensure_ascii=False, allow_nan=False).encode()).hexdigest()


def file_digest(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def environment_fingerprint() -> dict:
    """Small runtime identity; no environment variables or credential capture."""
    packages = {}
    for name in RELEVANT_PACKAGES:
        try:
            packages[name] = metadata.version(name)
        except metadata.PackageNotFoundError:
            packages[name] = None
    value = {"python": sys.version, "platform": platform.platform(), "packages": packages}
    return {**value, "sha256": canonical_digest(value)}


def source_identity(*, root: Path = ROOT) -> dict:
    sha = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip()
    dirty = bool(subprocess.check_output(["git", "status", "--porcelain"], cwd=root).strip())
    return {"git_sha": sha, "dirty": dirty}


def source_closure(*, root: Path = ROOT) -> frozenset[str]:
    """Pin transitive local Python imports of the execution entry points."""
    found, pending = set(REQUIRED_SOURCE_PATHS), list(REQUIRED_SOURCE_PATHS)
    def add_module(module: str) -> None:
        parts = module.split(".")
        if parts[0] not in {"harness", "scripts", "sim"}:
            return
        # Package initializer code can alter imported attributes or defaults.
        for index in range(1, len(parts) + 1):
            initializer = "/".join(parts[:index]) + "/__init__.py"
            if (root / initializer).is_file() and initializer not in found:
                found.add(initializer)
                pending.append(initializer)
        for candidate in ("/".join(parts) + ".py", "/".join(parts) + "/__init__.py"):
            if (root / candidate).is_file() and candidate not in found:
                found.add(candidate)
                pending.append(candidate)

    while pending:
        name = pending.pop()
        tree = ast.parse((root / name).read_text(), filename=name)
        package = name.removesuffix("/__init__.py").removesuffix(".py").split("/")
        if not name.endswith("/__init__.py"):
            package = package[:-1]
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                for item in node.names:
                    add_module(item.name)
            elif isinstance(node, ast.ImportFrom):
                if node.level:
                    if node.level > len(package):
                        continue
                    base = package[:len(package) - node.level + 1]
                    if node.module:
                        base += node.module.split(".")
                    module = ".".join(base)
                else:
                    module = node.module or ""
                add_module(module)
                for item in node.names:
                    if item.name != "*":
                        add_module(module + "." + item.name)
    return frozenset(found)


def load_bundle(bundle_id: str, *, root: Path = ROOT, require_runnable: bool = True) -> tuple[dict, str]:
    if bundle_id not in {RUNNABLE_ID, BASELINE_ID, *RETIRED_IDS}:
        raise ValueError("unknown RGB execution bundle ID")
    path = root / REGISTRY / (bundle_id + ".json")
    raw = path.read_bytes()
    value = json.loads(raw)
    if not isinstance(value, dict) or value.get("schema") != SCHEMA or value.get("id") != bundle_id:
        raise ValueError("RGB execution bundle schema or ID mismatch")
    if value.get("status") not in {"experimental_unqualified", "historical_success_original_executor"}:
        raise ValueError("RGB execution bundle status missing")
    if require_runnable and (bundle_id != RUNNABLE_ID or value["status"] != "experimental_unqualified"):
        if bundle_id in RETIRED_IDS:
            raise ValueError("retired execution bundle requires its original source checkout")
        raise ValueError("historical success bundle cannot run in this adapter")
    if set(value.get("effective", {})) != {"physics", "camera", "execution"}:
        raise ValueError("RGB execution effective contract incomplete")
    if bundle_id == RUNNABLE_ID and set(value.get("identity_binding", [])) != REQUIRED_IDENTITY_BINDING:
        raise ValueError("RGB execution identity binding incomplete")
    hashes = value.get("source_files_sha256")
    if bundle_id == RUNNABLE_ID:
        if not isinstance(hashes, dict) or set(hashes) != source_closure(root=root):
            raise ValueError("RGB execution required source set mismatch")
        for name, expected in hashes.items():
            relative = Path(name)
            if relative.is_absolute() or ".." in relative.parts or not name.endswith(".py"):
                raise ValueError("RGB execution source path invalid")
            actual = file_digest(root / relative)
            if actual != expected:
                raise ValueError(f"RGB execution source drift: {name}")
    return value, hashlib.sha256(raw).hexdigest()


def compare_effective(expected: dict, actual: dict) -> dict:
    """Exact nested comparison; missing/extra fields are failures, not defaults."""
    differences = {}
    def walk(path, left, right):
        if isinstance(left, dict) and isinstance(right, dict):
            for key in sorted(set(left) | set(right)):
                if key not in left or key not in right:
                    differences[".".join((*path, key))] = {"expected": left.get(key, "<missing>"),
                                                               "actual": right.get(key, "<missing>")}
                else:
                    walk((*path, key), left[key], right[key])
        elif left != right or type(left) is not type(right):
            differences[".".join(path)] = {"expected": left, "actual": right}
    walk((), expected, actual)
    return differences


def require_effective(bundle: dict, actual: dict) -> None:
    differences = compare_effective(bundle["effective"], actual)
    if differences:
        raise ValueError("RGB execution effective mismatch: " + ", ".join(differences))


def baseline_diff(actual: dict, *, root: Path = ROOT) -> dict:
    baseline, _ = load_bundle(BASELINE_ID, root=root, require_runnable=False)
    return compare_effective(baseline["effective"], actual)


def verify_registry_immutable(base_ref: str, *, root: Path = ROOT) -> None:
    """On PR CI, existing main bundle bytes cannot be rewritten under one ID."""
    subprocess.run(["git", "rev-parse", "--verify", base_ref + "^{commit}"], cwd=root,
                   check=True, stdout=subprocess.DEVNULL)
    for path in sorted((root / REGISTRY).glob("*.json")):
        relative = path.relative_to(root).as_posix()
        old = subprocess.run(["git", "show", f"{base_ref}:{relative}"], cwd=root,
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        if old.returncode == 0 and old.stdout != path.read_bytes():
            raise ValueError(f"existing RGB execution bundle changed; create a new ID: {relative}")
        if old.returncode not in (0, 128):
            raise ValueError(f"cannot inspect base bundle: {relative}")
    old_names = subprocess.check_output(["git", "ls-tree", "-r", "--name-only", base_ref, "--", str(REGISTRY)], cwd=root).decode().splitlines()
    for name in old_names:
        if name.endswith(".json") and not (root / name).is_file():
            raise ValueError(f"existing RGB execution bundle removed: {name}")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Validate RGB execution bundle without simulator or model calls")
    sub = parser.add_subparsers(dest="command", required=True)
    current = sub.add_parser("verify-current")
    current.add_argument("--id", required=True)
    registry = sub.add_parser("verify-registry")
    registry.add_argument("--base", required=True)
    args = parser.parse_args(argv)
    if args.command == "verify-current":
        bundle, sha = load_bundle(args.id)
        print(json.dumps({"id": bundle["id"], "status": bundle["status"], "sha256": sha,
                          "scope": "static manifest and source hashes only; live applied values require backend startup"}))
    else:
        verify_registry_immutable(args.base)
        print(json.dumps({"registry_immutable_against": args.base}))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        print(f"RGB execution bundle invalid: {error}", file=sys.stderr)
        raise SystemExit(1)
