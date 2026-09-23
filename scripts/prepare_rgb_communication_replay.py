"""Prepare immutable diagnostic inputs and static evidence, never run physics.

The output is deliberately NOT admitted until A supplies an independent offline
boundary review for the exact source/configuration. No live readiness is issued.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

from harness.rgb_communication_study import (
    PHYSICAL_BUDGETS, SCHEDULER_ID, ContractError, digest_file, digest_json, source_state,
    source_file_hashes, read_json, write_new_json,
)

ROOT = Path(__file__).resolve().parents[1]


def prepare_assets(root: Path, assets: Path, output: Path, *, mode: str) -> dict:
    """Copy a portable capsule, or pin existing local assets without copying them."""
    if mode not in {"copy", "reference"}:
        raise ContractError("asset mode must be copy or reference")
    files, evidence, model_roots = {}, {"data": [], "checkpoint": []}, {}
    for folder, manifest_name, staged in (("grasp", "student-skill.json", False),
                                          ("varied", "varied-start-skill.json", True)):
        src = (assets / folder).resolve()
        manifest = read_json(src / manifest_name)
        names = {manifest_name, "training-report.json"}
        for entry in manifest["models"].values():
            for model in entry.values() if staged else [entry]:
                name = Path(model["path"])
                if name.is_absolute() or ".." in name.parts or (src / name).is_symlink() \
                        or not (src / name).resolve().is_relative_to(src):
                    raise ContractError("asset path escapes model root")
                if digest_file(src / name) != model["sha256"]:
                    raise ContractError("existing saved model hash mismatch")
                names.add(str(name))
        for name in sorted(names):
            source = src / name
            if source.is_symlink() or not source.is_file():
                raise ContractError("asset must be an existing regular file")
            files[str(source)] = digest_file(source)
            if mode == "copy":
                destination = output / "assets" / folder / name
                destination.parent.mkdir(parents=True, exist_ok=True)
                with destination.open("xb") as target, source.open("rb") as original:
                    shutil.copyfileobj(original, target)
        model_roots[folder] = str(src if mode == "reference" else
                                  (output / "assets" / folder).relative_to(root))
        for kind, name in (("data", "training-report.json"), ("checkpoint", manifest_name)):
            relative = "assets/" + folder + "/" + name
            if mode == "reference":
                relative = "asset-references/" + folder + "/" + name
                write_new_json(output / relative, {"schema": "rgb-local-asset-reference.v1",
                    "path": str(src / name), "sha256": files[str(src / name)],
                    "scope": "existing local artifact, read-only reuse; no asset copy"})
            evidence[kind].append({"path": relative, "sha256": digest_file(output / relative)})
    reference = (root / "tests/fixtures/camera_goal_transport/reference-top.jpg").resolve()
    if mode == "copy":
        shutil.copyfile(reference, output / "assets/reference-top.jpg")
        reference_path = str((output / "assets/reference-top.jpg").relative_to(root))
    else:
        reference_path = str(reference)
        files[reference_path] = digest_file(reference)
    catalog = None
    if mode == "reference":
        write_new_json(output / "local-assets.json", {
            "schema": "rgb-local-assets.v1", "mode": "read_only_reference", "files": files})
        catalog = {"path": "local-assets.json", "sha256": digest_file(output / "local-assets.json")}
    return {"grasp_model_dir": model_roots["grasp"], "stage_model_dir": model_roots["varied"],
            "reference_top": reference_path, "provenance": evidence, "local_assets": catalog}


def prepare(root: Path, assets: Path, *, output: Path | None = None,
            asset_mode: str = "copy", run_prefix: str = "", submitter: str = "D2") -> dict:
    state = source_state(root)
    if not state["clean"]:
        raise ContractError("commit preparation source before preparing evidence")
    if submitter == "D3" and asset_mode != "reference":
        raise ContractError("D3 requires existing read-only local assets, not a copied capsule")
    root = root.resolve()
    output = (output or root / "outputs/study-inputs").resolve()
    if not output.is_relative_to(root / "outputs"):
        raise ContractError("preparation output must be a new directory under project outputs")
    if run_prefix and (not run_prefix.isascii() or
                       any(not (c.isalnum() or c in "-_") for c in run_prefix)):
        raise ContractError("run prefix must be path-safe ASCII")
    output.mkdir(parents=True, exist_ok=False)

    def record(name, value):
        path = output / name
        write_new_json(path, value)
        return {"path": name, "sha256": digest_file(path)}

    def ref(name):
        return {"path": name, "sha256": digest_file(output / name)}

    asset_info = prepare_assets(root, assets, output, mode=asset_mode)
    from harness.rgb_skill_execution import backend_descriptor, public_static_context
    backend = {"schema": "ugrp.rgb_skill_backend.v1", "map_id": "dispatch_open", "seed": 11,
               "output_dir": "outputs/not-yet-submitted-rgb-replay", "max_sim_s": 180,
               "max_commands": 6000,
               **{key: asset_info[key] for key in ("grasp_model_dir", "stage_model_dir", "reference_top")}}
    descriptor = backend_descriptor(backend)
    public = public_static_context(backend)
    descriptor_ref = record("static/backend-descriptor.json", descriptor)
    source_ref = record("static/source.json", {**state, "source_files": source_file_hashes(root)})

    # Build the same XML as DispatchScene without constructing a world, calling
    # mj_step, rendering, evaluating a policy, or launching a simulator.
    from sim.research_dispatch_arena import episode, build_scene_xml, FIXED_TOP
    from sim.session_config import validate_config
    from sim.session_scenes import Scene
    from sim.multi_masterpi_production import build_multi_robot_xml
    from scripts.probe_dual_grasp_sync import _plain_beam_xml
    scene_selection = validate_config({"version": 1, "scene": {"layout": "dispatch/open", "seed": 11}})
    selected_scene = Scene(scene_selection["scene"], root)
    config = selected_scene.config
    assert config == episode("open", 11)
    selection_ref = record("static/scene-selection.json", {
        "selector": selected_scene.selection, "backend_map_id": descriptor["map_id"],
        "seed": 11, "reset_sha256": digest_json(config["setup_only"]),
        "map_instance_sha256": digest_json(config["static_map"]),
        "scope": "registry authored map/setup identity only; backend camera dimensions and physics remain separate"})
    xml, xml_manifest = build_scene_xml(_plain_beam_xml(build_multi_robot_xml)(navigation_camera=False), config)
    tree = ET.fromstring(xml)
    assert all(w.get("active") == "false" for w in tree.findall("equality/weld"))
    camera = tree.find(".//camera[@name='cctv_top']")
    assert float(camera.get("fovy")) == FIXED_TOP["fov_y_deg"]
    assert [float(v) for v in camera.get("pos").split()] == FIXED_TOP["position_m"]
    assert xml_manifest["static_map_sha256"] == descriptor["map_instance_sha256"]
    with (output / "static/scene.xml").open("x") as stream:
        stream.write(xml)
    scene_ref = ref("static/scene.xml")
    camera_ref = record("static/camera.json", public["camera_calibration"])
    reset_ref = record("static/reset.json", {"seed": 11, "setup_only": config["setup_only"],
        "actual_physical_reset_verified": False, "scope": "deterministic authored setup only"})
    evaluator_ref = record("static/evaluator.json", {"goal_frame": descriptor["goal_frame"],
        "docks": config["static_map"]["docks"], "source_file": "scripts/run_dispatch_e2e.py",
        "sha256": digest_file(root / "scripts/run_dispatch_e2e.py"), "actual_physics_verified": False})
    static_build = record("static/build.json", {"source_sha": state["git_sha"],
        "builder_manifest": xml_manifest, "descriptor_sha256": descriptor_ref["sha256"],
        "scope": "XML and authored setup checks only; no simulator or model was run"})
    selectors = ["tests/test_dispatch_research.py::test_environment_builder_retains_robots_camera_and_real_collision_geometry",
                 "tests/test_rgb_execution_skills.py::test_readonly_descriptor_binds_models_map_camera_goal_and_reset",
                 "tests/test_rgb_execution_skills.py::test_clock_jump_close_and_raw_budget_fail_closed"]
    with (output / "static/tests.log").open("x") as stream:
        completed = subprocess.run([sys.executable, "-m", "pytest", "-q", *selectors], cwd=root,
                                   stdout=stream, stderr=subprocess.STDOUT, timeout=60)
    if completed.returncode:
        raise ContractError("static contract tests failed; preserve preparation and do not submit")
    tests_ref = ref("static/tests.log")
    maps = [{"map_id": "dispatch_open", "map_sha256": descriptor["map_instance_sha256"],
             "layout_sha256": descriptor["map_group_sha256"], "scene": scene_ref, "camera": camera_ref,
             "reset": reset_ref, "evaluator_config": evaluator_ref, "capability": "transport"}]
    from harness.rgb_communication_scenarios import ENVIRONMENT_PREFLIGHT_CHECKS
    checks = {name: record("static/check-" + name + ".json", {
        "check_id": name, "verdict": "pass", "source_sha": state["git_sha"],
        "backend_id": descriptor["backend_id"], "maps_sha256": digest_json(maps),
        "evidence_kind": "static_contract_check", "artifacts": [source_ref, static_build, tests_ref,
                                                                   descriptor_ref, scene_ref]})
        for name in ENVIRONMENT_PREFLIGHT_CHECKS}
    review_ref = record("environment-review.json", {
        "schema_version": "rgb-environment-readiness.v1", "source_sha": state["git_sha"],
        "backend_id": descriptor["backend_id"], "scope": "static_reset_preflight", "maps": maps,
        "checks": checks})
    map_record = {k: descriptor[k] for k in ("map_id", "map_version", "map_instance_sha256", "map_group_sha256")}
    map_record["stratum"] = "regression"
    actions = {}
    for kind in ("solo", "joint"):
        participants = ["r2"] if kind == "solo" else ["r1", "r3"]
        skill = "solo_transport_A" if kind == "solo" else "pair_transport_A"
        capability = descriptor["capabilities"][skill]
        actions[kind] = {rid: [{"kind": "task_request", "task_id": kind + "-diagnostic",
            "object_id": capability["object_id"], "skill": skill, "participants": participants,
            "resources": capability["resources"], "stage": "RUN",
            "own_role": "solo" if kind == "solo" else "lower" if rid == "r1" else "upper",
            "expires_at_s": 180}] if rid in participants else [{"kind": "finish", "claim": "cannot_continue"}]
            for rid in ("r1", "r2", "r3")}
    prompt_ref = record("replay-actions.json", {"evidence_kind": "deterministic_physical_replay",
        "fixed_roles_not_negotiation": True, "actions": actions})
    # Missing historical map lineage is a real unknown, not a fabricated link
    # from the old grasp/alignment model to the current dispatch scene.
    provenance = {"schema_version": "rgb-map-exposure.v1", "records": [
        {"id": name, "kind": "dataset", "parents": [], "map_refs": [],
         "declared_splits": [], "origin": "unknown"} for name in ("legacy-grasp", "legacy-alignment")],
        "exposures": [], "freeze_order": 0, "final_test_ids": []}
    for kind, parent in (("grasp", "legacy-grasp"), ("varied", "legacy-alignment")):
        provenance["records"].append({"id": kind + "-checkpoint", "kind": "checkpoint",
            "parents": [parent], "map_refs": [], "declared_splits": [], "origin": "unknown"})
    provenance["records"].extend([
        {"id": "diagnostic-scene", "kind": "scenario", "parents": [], "origin": "known",
         "declared_splits": ["regression"], "map_refs": [{"map_id": "dispatch_open",
          "map_sha256": descriptor["map_instance_sha256"], "layout_sha256": descriptor["map_group_sha256"]}]},
        {"id": "fixed-replay-prompt", "kind": "prompt", "parents": ["diagnostic-scene"],
         "map_refs": [], "declared_splits": ["regression"], "origin": "known"}])
    from sim.act_map_suite import load_suite
    _, cases = load_suite()
    provenance["final_test_ids"] = [c["id"] for c in cases if c["split"] in {"test_a", "test_b"}]
    provenance_ref = record("exposure.json", provenance)
    run_ids = {kind: run_prefix + "-" + kind if run_prefix else kind + "-physical-replay"
               for kind in ("solo", "joint")}
    assignments = [{"scenario_id": run_ids[kind], "parent_map_id": "dispatch_open",
        "map_sha256": map_record["map_instance_sha256"], "layout_sha256": map_record["map_group_sha256"],
        "split": "regression"} for kind in ("solo", "joint")]
    environment_ref = record("environment.json", {
        "schema_version": "rgb-study-environment.v1", "source_sha": state["git_sha"],
        "suite": {"path": "maps/act_generalization/suite_v1.json",
                  "sha256": digest_file(root / "maps/act_generalization/suite_v1.json")},
        "provenance": provenance_ref, "scenario_assignments": assignments, "map": map_record,
        "review": review_ref, "scene_selection": selection_ref, "provenance_artifacts": {
            **asset_info["provenance"],
            "prompt": [prompt_ref]}})
    from harness.rgb_communication_async import AsyncRuntimeLimits
    limits = AsyncRuntimeLimits(max_ticks=3600, max_calls_per_robot=181, max_actions_per_robot=181,
        max_messages_per_robot=1, max_message_bytes_per_robot=1, wall_timeout_s=595,
        decision_period_s=1., max_input_tokens=1, max_output_tokens=1,
        max_input_tokens_per_call=1, max_output_tokens_per_call=1)
    fixed = {"map": "maps/research/dispatch_v1.json", "physics": "sim/research_dispatch_arena.py",
             "camera": "sim/multi_masterpi_production.py", "calibration": "tests/fixtures/camera_goal_transport/reference-top.jpg"}
    final_config = {"stage": "physical_replay", "claim_scope": "development_connection_smoke",
        "submitter": submitter, "backend_id": descriptor["backend_id"], "backend": backend,
        # Absolute descriptor paths intentionally bind a local host. A portable
        # D2 capsule keeps its previous relative-path contract after relocation.
        **({"backend_descriptor_evidence": descriptor_ref,
            "local_assets": asset_info["local_assets"]} if asset_mode == "reference" else {}),
        "components": {"serializer_id": "rgb-agent-request.v1", "backend_id": descriptor["backend_id"],
                       "scheduler_id": SCHEDULER_ID},
        "budgets": dict(PHYSICAL_BUDGETS), "runtime_limits": limits.__dict__,
        "inputs": {key: {"path": path, "sha256": digest_file(root / path)} for key, path in fixed.items()},
        "environment_evidence": environment_ref, "offline_boundary_evidence": None,
        "trials": [{"run_id": run_ids[kind], "scenario_id": run_ids[kind],
            "replay_kind": kind, "condition": "none", "seed": 11, "object_ids": ["box", "beam"],
            "replay_target_objects": ["box" if kind == "solo" else "beam"],
            "replay_actions": actions[kind], "common_task": public["task"], **map_record}
            for kind in ("solo", "joint")]}
    record("config-for-audit.json", final_config)
    return {"source_sha": state["git_sha"], "output": str(output), "ready": False,
            "asset_mode": asset_mode,
            "requires": ["independent A offline boundary review", "source-bound E0 preflight"],
            "physical_simulations": 0, "external_model_calls": 0}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--assets-root", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--asset-mode", choices=("copy", "reference"), default="copy")
    parser.add_argument("--run-prefix", default="")
    parser.add_argument("--submitter", default="D2")
    args = parser.parse_args()
    print(json.dumps(prepare(ROOT, args.assets_root, output=args.output,
                            asset_mode=args.asset_mode, run_prefix=args.run_prefix,
                            submitter=args.submitter), ensure_ascii=False))


if __name__ == "__main__":
    main()
