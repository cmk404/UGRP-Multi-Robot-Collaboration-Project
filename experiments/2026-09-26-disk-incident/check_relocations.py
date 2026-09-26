#!/usr/bin/env python3
"""Check Git references into retired worktrees before and after a move (read-only).

For each retired worktree root, every text blob on every local/remote ref is
scanned for ``<root>/<path>``. References into ignored data (outputs/,
MUJOCO_LOG.TXT) are resolved at the old location and at the retirement
destination ``<primary>/outputs/retired-worktrees/<label>/<path>``.

Usage (2026-09-26, the 14 merged Codex-app worktrees):
    python3 experiments/2026-09-26-disk-incident/check_relocations.py \
        --repo /Users/changmin/projects/ugrp --output /tmp/relocations.json \
        --root /Users/changmin/.codex/worktrees/<name>/ugrp=codex-<name> ...
"""

from __future__ import annotations

import argparse
import collections
import json
from pathlib import Path
import re
import subprocess

TEXT_SUFFIXES = {".md", ".json", ".jsonl", ".py", ".txt", ".csv", ".sh", ".yml", ".yaml", ".toml", ".log", ""}
PATH_CHARS = r"[A-Za-z0-9._\-/+=@%~*]*"
IGNORED = ("outputs/", "MUJOCO_LOG.TXT")


def git(repo: Path, *args: str) -> str:
    return subprocess.run(["git", "-C", str(repo), *args], check=True, capture_output=True, text=True).stdout


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    parser.add_argument("--repo", type=Path, required=True)
    parser.add_argument("--root", action="append", required=True, help="OLD_ROOT=LABEL")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    roots = dict(item.split("=", 1) for item in args.root)
    pattern = re.compile("(" + "|".join(re.escape(r) for r in roots) + ")(/" + PATH_CHARS + ")?")
    refs = [r for r in git(args.repo, "for-each-ref", "--format=%(refname)", "refs/remotes/origin",
                           "refs/heads").split() if not r.endswith("/HEAD")]
    blob_paths: dict[str, set[str]] = collections.defaultdict(set)
    blob_refs: dict[str, set[str]] = collections.defaultdict(set)
    for ref in refs:
        for line in git(args.repo, "ls-tree", "-r", ref).splitlines():
            meta, path = line.split("\t", 1)
            _mode, kind, blob = meta.split()
            if kind == "blob" and Path(path).suffix.lower() in TEXT_SUFFIXES:
                blob_paths[blob].add(path)
                blob_refs[blob].add(ref)
    proc = subprocess.Popen(["git", "-C", str(args.repo), "cat-file", "--batch"], stdin=subprocess.PIPE,
                            stdout=subprocess.PIPE)
    assert proc.stdin and proc.stdout
    hits: dict[str, dict] = {}
    needles = [r.encode() for r in roots]
    for blob in sorted(blob_paths):
        proc.stdin.write((blob + "\n").encode())
        proc.stdin.flush()
        size = int(proc.stdout.readline().split()[2])
        data = proc.stdout.read(size)
        proc.stdout.read(1)
        if not any(n in data for n in needles):
            continue
        for match in pattern.finditer(data.decode("utf-8", errors="replace")):
            root, rest = match.group(1), (match.group(2) or "").rstrip(".")
            row = hits.setdefault(root + rest, {"root": root, "rest": rest, "files": set(), "refs": set()})
            row["files"].update(blob_paths[blob])
            row["refs"].update(blob_refs[blob])
    proc.stdin.close()
    proc.wait()
    retired = args.repo / "outputs" / "retired-worktrees"
    rows = []
    for full, row in sorted(hits.items()):
        rel = row["rest"].lstrip("/")
        ignored = rel.startswith(IGNORED)
        new = retired / roots[row["root"]] / rel if ignored else None
        rows.append({
            "reference": full, "ignored_data": ignored, "old_exists": Path(full).exists(),
            "new_path": str(new) if new else None, "new_exists": new.exists() if new else None,
            "on_origin_main": "refs/remotes/origin/main" in row["refs"],
            "files": sorted(row["files"])[:5], "ref_count": len(row["refs"]),
        })
    data_rows = [r for r in rows if r["ignored_data"]]
    summary = {
        "references": len(rows), "into_ignored_data": len(data_rows),
        "old_exists": sum(r["old_exists"] for r in data_rows),
        "new_exists": sum(bool(r["new_exists"]) for r in data_rows),
        "resolvable": sum(r["old_exists"] or bool(r["new_exists"]) for r in data_rows),
    }
    args.output.write_text(json.dumps({"roots": roots, "refs_scanned": len(refs), "summary": summary,
                                       "rows": rows}, ensure_ascii=False, indent=1) + "\n")
    print(json.dumps(summary))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
