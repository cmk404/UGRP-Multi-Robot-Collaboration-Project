#!/usr/bin/env python3
"""Summarize the 2026-09-26 disk apply receipts into committed records (read-only on raw).

Reads the raw receipts under /Users/changmin/projects/ugrp/outputs/disk-apply-20260926/
(item1/exec-*.json, item2/exec-*.json, item3/exec-*.json, outputs/archive/codex-worktrees/*.json,
item4/dedupe-summary.json) and writes compact JSON next to this script. Each summary
lists the raw file it came from with its sha256, so the committed record can be
checked against the local raw.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

RAW = Path("/Users/changmin/projects/ugrp/outputs/disk-apply-20260926")
ARCHIVE = Path("/Users/changmin/projects/ugrp/outputs/archive/codex-worktrees")
HERE = Path(__file__).resolve().parent
GIB = 2**30
# Worktrees left untouched, with the reason observed at the time of each item (census 16:51, items 18:24-18:36).
SKIPPED = [
    {"worktree": "~/projects/ugrp-worktrees/zone-hard-routes", "items": [1], "merged": "PR #173",
     "reason": "in use: Claude shell processes 53386/53824 (and 74139 at census) have cwd inside its outputs/"},
    {"worktree": "~/projects/ugrp-worktrees/zone-team-a2", "items": [1, 2], "merged": "PR #169",
     "reason": "in use: running kiro-teacher-fix job (PR #207) scripts read its outputs/zone-team-a2-20260925 by absolute path"},
    {"worktree": "~/projects/ugrp-wt/kiro-vision-loc", "items": [1], "merged": "ancestor of main (PR #210 head)",
     "reason": "running Kiro job (vision-loc); untracked files; modified within 60 min"},
    {"worktree": "~/projects/ugrp-wt/kiro-memory-literature", "items": [1], "merged": "ancestor of main",
     "reason": "running Kiro job named by the user; modified within 60 min; already sparse (0.13 GiB)"},
    {"worktree": "~/projects/ugrp-wt/kiro-references-0926", "items": [1], "merged": "ancestor of main",
     "reason": "running Kiro job (references); modified within 60 min; already sparse (0.13 GiB)"},
    {"worktree": "~/projects/ugrp-wt/kiro-study-core", "items": [2], "merged": None,
     "reason": "PR #194: modified within 60 min (git index/logs, AGENTS.md, maps/zones files); sparsify refused"},
    {"worktree": "~/projects/ugrp-wt/kiro-map-v3", "items": [2], "merged": None,
     "reason": "PR #208: running Kiro job (zone-map-v3); untracked files; modified within 60 min"},
    {"worktree": "~/projects/ugrp-wt/kiro-owncam-memory", "items": [2], "merged": None,
     "reason": "PR #211: running Kiro job (memory); untracked files; modified within 60 min"},
    {"worktree": "~/projects/ugrp-wt/kiro-sim-speed", "items": [2], "merged": None,
     "reason": "PR #209: running sim_profile processes 67205/67209 with cwd inside"},
    {"worktree": "~/projects/ugrp-wt/kiro-teacher-fix", "items": [2], "merged": None,
     "reason": "PR #207: running Kiro job (teacher-fix); tracked changes; modified within 60 min"},
    {"worktree": "~/projects/ugrp-wt/kiro-vision-loc-v3src", "items": [2], "merged": None,
     "reason": "detached source of PR #208; already sparse (0.01 GiB); modified within 60 min"},
    {"worktree": "~/projects/ugrp-wt/kiro-disk-rules", "items": [2], "merged": None,
     "reason": "this job's own worktree; already sparse"},
]


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rel_home(path: str) -> str:
    return path.replace("/Users/changmin/", "~/")


def retire_rows(folder: Path) -> list[dict]:
    rows = []
    for path in sorted(folder.glob("exec-*.json")):
        r = json.loads(path.read_text())
        manifest = Path(r["receipt"]) / "MANIFEST.tsv"
        rows.append({
            "worktree": rel_home(r["worktree"]), "owner": r["owner"], "branch": r["branch"], "head": r["head"],
            "evidence": r["merged_evidence"], "verified": r.get("verified"),
            "worktree_remove_exit": r.get("worktree_remove_exit"),
            "checkout_bytes_freed_est": r["checkout_bytes_freed_est"],
            "moved": [{"path": m["path"], "destination": rel_home(m["destination"]), "placement": m["placement"],
                       "files": m.get("files"), "links": m.get("links"), "bytes": m.get("bytes"),
                       "manifest_sha256": m.get("manifest_sha256")} for m in r["moved"]],
            "moved_manifest_tsv": rel_home(str(manifest)) if manifest.exists() else None,
            "moved_manifest_tsv_sha256": sha(manifest) if manifest.exists() else None,
            "deleted_caches": r["deleted_caches"],
            "receipt": rel_home(str(path)), "receipt_sha256": sha(path),
        })
    return rows


def item_totals(rows: list[dict]) -> dict:
    return {"worktrees": len(rows), "all_verified": all(r["verified"] and r["worktree_remove_exit"] == 0 for r in rows),
            "checkout_gib_freed_est": round(sum(r["checkout_bytes_freed_est"] for r in rows) / GIB, 2),
            "moved_files": sum((m["files"] or 0) for r in rows for m in r["moved"]),
            "moved_gib": round(sum((m["bytes"] or 0) for r in rows for m in r["moved"]) / GIB, 3)}


def sparse_rows() -> list[dict]:
    rows = []
    for path in sorted((RAW / "item2").glob("exec-*.json")):
        r = json.loads(path.read_text())
        name = path.stem.removeprefix("exec-")
        before = RAW / "item2" / f"status-before-{name}.txt"
        after = RAW / "item2" / f"status-after-{name}.txt"
        override = RAW / "item2" / f"override-{name}.txt"
        rows.append({"worktree": rel_home(r["path"]), "head": r["head"], "before_bytes": r["before_bytes"],
                     "after_bytes": r["after_bytes"], "before_files": r["before_files"], "after_files": r["after_files"],
                     "status_ignored_unchanged": before.read_bytes() == after.read_bytes(),
                     "idle_override": override.read_text().strip() if override.exists() else None,
                     "receipt_sha256": sha(path)})
    return rows


def archive_rows() -> list[dict]:
    rows = []
    for path in sorted(ARCHIVE.glob("*-0926.json")):
        r = json.loads(path.read_text())
        rows.append({"name": r["name"], "branch": r["branch"], "head": r["head"], "archive_ref": r["archive_ref"],
                     "remote_sha": r["remote_sha"], "remote_matches_head": r["remote_matches_head"],
                     "uncommitted_patch": r["patch"], "status_porcelain_ignored": r["status_porcelain_ignored"],
                     "bundle": {k: (rel_home(v) if k == "path" else v) for k, v in r["bundle"].items()},
                     "record": rel_home(str(path)), "record_sha256": sha(path)})
    return rows


def main() -> None:
    item1 = retire_rows(RAW / "item1")
    item3 = retire_rows(RAW / "item3")
    item2 = sparse_rows()
    out = {
        "item1_retire_merged": {"totals": item_totals(item1), "rows": item1},
        "item2_sparsify": {"totals": {"worktrees": len(item2),
                                      "before_gib": round(sum(r["before_bytes"] for r in item2) / GIB, 2),
                                      "after_gib": round(sum(r["after_bytes"] for r in item2) / GIB, 2),
                                      "freed_gib": round(sum(r["before_bytes"] - r["after_bytes"] for r in item2) / GIB, 2),
                                      "all_status_unchanged": all(r["status_ignored_unchanged"] for r in item2)},
                           "rows": item2},
        "item3_codex_unmerged": {"totals": {**item_totals(item3),
                                            "archived_refs_match": all(r["remote_matches_head"] for r in archive_rows())},
                                 "archive": archive_rows(), "rows": item3},
    }
    dedupe = RAW / "item4" / "dedupe-summary.json"
    if dedupe.exists():
        out["item4_dedupe"] = {**json.loads(dedupe.read_text()), "raw_sha256": sha(dedupe)}
    out["skipped"] = SKIPPED
    census = RAW / "census-before.json"
    out["raw_inputs"] = {rel_home(str(p)): sha(p) for p in (census, RAW / "disk-report-before.json",
                                                           RAW / "moves-check.json") if p.exists()}
    (HERE / "applied.json").write_text(json.dumps(out, ensure_ascii=False, indent=1) + "\n")
    print(json.dumps({k: v["totals"] for k, v in out.items() if isinstance(v, dict) and "totals" in v},
                     ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
