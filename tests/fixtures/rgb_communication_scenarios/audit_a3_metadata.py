"""Read-only A3 scene/ACT lineage packet; never imports a physics/model runtime.

Writes only the requested report. Existing model/data/episode files are hashed
in place, not copied. No image decoding, training, inference or physical replay.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from harness.rgb_communication_scenarios import scene_support_matrix
from sim.research_dispatch_arena import VARIANTS, authored_map, digest


def sha(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def audit_act(qualification, dataset_path, report_path, checkpoint):
    sources = {}

    def read(path):
        path = Path(path)
        raw = path.read_bytes()
        sources[str(path.resolve())] = hashlib.sha256(raw).hexdigest()
        return json.loads(raw)

    protocol = read(qualification/"acceptance-v3-full-protocol.json")
    runtime = read(qualification/"runtime-v3-manifest.json")
    terminal = read(qualification/"terminal-candidate-development-audit.json")
    sources[str((qualification/"README.md").resolve())] = sha(qualification/"README.md")
    dataset = read(dataset_path)
    training = read(report_path)
    issues = []
    assets = []
    for path, expected in protocol["asset_sha256"].items():
        actual = sha(path) if Path(path).is_file() else None
        matches = actual is not None and actual == expected
        assets.append({"path": path, "expected_sha256": expected, "actual_sha256": actual, "matches": matches})
        if not matches:
            issues.append("qualification asset missing or changed: "+path)
    if sha(dataset_path) != training["dataset_sha256"]:
        issues.append("training report dataset hash mismatch")
    checkpoint_sha = sha(checkpoint)
    if checkpoint_sha != training["model_sha256"] or checkpoint_sha != protocol["asset_sha256"].get(str(checkpoint)):
        issues.append("retained checkpoint binding mismatch")
    if runtime["protocol_sha256"] != sha(qualification/"acceptance-v3-full-protocol.json"):
        issues.append("runtime protocol binding mismatch")
    parents = {digest(authored_map(v)): v for v in VARIANTS if v != "north_blocked"}
    episodes = []
    for split in ("train", "development"):
        if sorted(training[split+"_episodes"]) != sorted(e["root"] for e in dataset[split]):
            issues.append("training report episode membership mismatch: "+split)
        for entry in dataset[split]:
            folder = Path(entry["root"])
            checked = {}
            required = {"result.json", "pair-decisions.json", "issued-commands.json",
                        "skill-bindings.json", "committed-plan.json", "episode-setup-only.json"}
            for name in sorted(required - set(entry["files"])):
                checked[name] = False
                issues.append("dataset required metadata binding absent: "+str(folder/name))
            for name, expected in entry["files"].items():
                path = folder/name
                if not path.resolve().is_relative_to(folder.resolve()):
                    checked[name] = False
                    issues.append("dataset metadata reference escapes episode: "+name)
                    continue
                actual = sha(path) if path.is_file() else None
                checked[name] = (actual is not None and isinstance(expected, str)
                                 and bool(re.fullmatch(r"[0-9a-f]{64}", expected)) and actual == expected)
                if not checked[name]:
                    issues.append("dataset metadata missing or changed: "+str(path))
            setup = read(folder/"episode-setup-only.json")
            static = setup["static_map"]
            parent_hash = digest(static)
            variant = parents.get(parent_hash)
            if variant is None:
                issues.append("unknown current authored parent definition: "+str(folder))
            episodes.append({"root": str(folder), "dataset_role": split,
                "source_sha": entry["source_sha"], "parent_variant": variant,
                "parent_map_id": static["map_id"], "parent_map_sha256": parent_hash,
                "parent_geometry_sha256": digest({k: static.get(k, []) for k in ("bounds_m", "obstacles", "terrain")}),
                "reset_sha256": digest(setup["setup_only"]),
                "effective_parent_split": "regression" if variant else "unknown",
                "metadata_hashes_match": all(checked.values()), "metadata_checks": checked,
                "recorded_image_hash_count": len(entry["image_hashes"]), "image_bytes_rehashed": False,
                "plan_sha256": entry["files"].get("committed-plan.json")})
    known_maps = sorted({e["parent_map_sha256"] for e in episodes})
    plans = sorted({e["plan_sha256"] for e in episodes if e["plan_sha256"] is not None})
    return {"schema_version": "rgb-a3-act-lineage.v1", "source_artifacts": sources,
        "asset_checks": assets, "integrity_blockers": issues,
        "checked_metadata_integrity": not issues, "episodes": episodes,
        "lineage": {
            "dataset": {"sha256": sha(dataset_path), "parent_maps": known_maps,
                        "roles": {k: len(dataset[k]) for k in ("train", "development")},
                        "effective_splits": sorted({e["effective_parent_split"] for e in episodes})},
            "checkpoint": {"sha256": checkpoint_sha, "dataset_sha256": training["dataset_sha256"],
                           "selection": training["selection"], "development_parent_maps": sorted({
                               e["parent_map_sha256"] for e in episodes if e["dataset_role"] == "development"})},
            "replayed_plans": {"sha256_values": plans, "meaning": "committed plan assets, not new independent LLM decisions"},
        },
        "qualification_cases": [{"id": c["id"], "variant": c["variant"], "seed": c["seed"],
            "offset": c["offset"], "dock": c["dock"], "effective_parent_split": "regression",
            "parent_map_sha256": digest(authored_map(c["variant"]))} for c in protocol["test"]],
        "qualification_scope": protocol["scope"],
        "retained_model_not_terminal_candidate": terminal["eligible_for_default"] is False,
        "foundation_and_historical_prompt_lineage_complete": False,
        "T1_T2_admission": False, "heldout_claim_ready": False, "execution_admission": False,
        "remaining": ["legacy train/development labels are episode roles, not disjoint structural map splits",
                      "original RGB bytes and historical prompt exposure are not exhaustively re-audited here",
                      "new suite training needs complete parent lineage, equal data/updates, fixed tests and repeated seeds",
                      "running qualification results are not read or marked complete"]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--qualification-dir", type=Path)
    parser.add_argument("--dataset", type=Path)
    parser.add_argument("--training-report", type=Path)
    parser.add_argument("--checkpoint", type=Path)
    args = parser.parse_args()
    if any((args.qualification_dir, args.dataset, args.training_report, args.checkpoint)) and not all((
            args.qualification_dir, args.dataset, args.training_report, args.checkpoint)):
        parser.error("ACT audit needs all four explicit paths")
    if args.output.exists():
        parser.error("report exists; use a new path to preserve prior evidence")
    value = {"schema_version": "rgb-a3-metadata-audit.v1",
             "source_sha": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
             "source_worktree_clean": not subprocess.check_output(
                 ["git", "status", "--porcelain"], cwd=ROOT, text=True).strip(),
             "audit_script_sha256": sha(__file__), "scene_support": scene_support_matrix(),
             "scope": "metadata/integrity only; not a new experimental result or execution certificate"}
    if args.qualification_dir:
        value["act_lineage"] = audit_act(args.qualification_dir, args.dataset, args.training_report, args.checkpoint)
    value["heavy_runtimes_imported"] = sorted(set(sys.modules).intersection({"mujoco", "torch"}))
    if value["heavy_runtimes_imported"]:
        raise RuntimeError("metadata audit unexpectedly imported heavy runtime")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(value, indent=2, ensure_ascii=False)+"\n")
    print(json.dumps({"report": str(args.output), "sha256": sha(args.output),
                      "scene_entries": value["scene_support"]["entry_count"],
                      "act_integrity_blockers": value.get("act_lineage", {}).get("integrity_blockers", [])}))


if __name__ == "__main__":
    main()
