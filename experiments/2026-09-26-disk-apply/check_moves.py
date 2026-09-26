#!/usr/bin/env python3
"""Resolve Git references into worktrees retired on 2026-09-26 (disk apply, read-only).

Adapts experiments/2026-09-26-disk-incident/check_relocations.py to the v2
retirement receipts, where ignored `outputs/<name>` may keep its relative path in
the primary checkout. Every text blob on every local/remote ref is scanned for
`<old worktree>/<path>`; a reference into moved data is resolved through the
receipt's `moved[].path -> destination` (longest prefix) and checked on disk.

    python3 experiments/2026-09-26-disk-apply/check_moves.py --repo /Users/changmin/projects/ugrp \
        --receipts /Users/changmin/projects/ugrp/outputs/disk-apply-20260926/item1 \
        --receipts /Users/changmin/projects/ugrp/outputs/disk-apply-20260926/item3 --output moves.json
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


def git(repo: Path, *args: str) -> str:
    return subprocess.run(["git", "-C", str(repo), *args], check=True, capture_output=True, text=True).stdout


def load_receipts(folders: list[Path]) -> dict[str, dict]:
    roots = {}
    for folder in folders:
        for path in sorted(folder.glob("exec-*.json")):
            receipt = json.loads(path.read_text())
            if receipt.get("verified") and receipt.get("worktree_remove_exit") == 0:
                roots[receipt["worktree"]] = receipt
    return roots


def resolve(receipt: dict, rel: str) -> tuple[str | None, str | None]:
    best = None
    for item in receipt["moved"]:
        if rel == item["path"] or rel.startswith(item["path"] + "/"):
            if best is None or len(item["path"]) > len(best["path"]):
                best = item
    if best is None:
        return None, None
    return best["destination"] + rel[len(best["path"]):], best["placement"]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    parser.add_argument("--repo", type=Path, required=True)
    parser.add_argument("--receipts", type=Path, action="append", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    roots = load_receipts(args.receipts)
    pattern = re.compile("(" + "|".join(re.escape(r) for r in sorted(roots, key=len, reverse=True)) + ")(/"
                         + PATH_CHARS + ")?")
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
            if rest and not rest.startswith("/"):
                continue  # a longer sibling name (e.g. zone-team-a2-dev vs zone-team-a2)
            row = hits.setdefault(root + rest, {"root": root, "rest": rest, "files": set(), "refs": set()})
            row["files"].update(blob_paths[blob])
            row["refs"].update(blob_refs[blob])
    proc.stdin.close()
    proc.wait()
    rows = []
    for full, row in sorted(hits.items()):
        rel = row["rest"].lstrip("/")
        new, placement = resolve(roots[row["root"]], rel) if rel else (None, None)
        rows.append({"reference": full, "moved_data": new is not None, "new_path": new, "placement": placement,
                     "new_exists": Path(new).exists() if new else None,
                     "on_origin_main": "refs/remotes/origin/main" in row["refs"],
                     "files": sorted(row["files"])[:5], "ref_count": len(row["refs"])})
    moved = [r for r in rows if r["moved_data"]]
    summary = {"worktrees": len(roots), "refs_scanned": len(refs), "references": len(rows),
               "into_moved_data": len(moved), "resolvable": sum(bool(r["new_exists"]) for r in moved),
               "into_tracked_or_root_only": len(rows) - len(moved)}
    args.output.write_text(json.dumps({"summary": summary, "rows": rows}, ensure_ascii=False, indent=1) + "\n")
    print(json.dumps(summary))
    return 0 if summary["resolvable"] == summary["into_moved_data"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
