#!/usr/bin/env python3
"""APFS-clone dedupe of identical files in the primary outputs/ (2026-09-26 disk apply, item 4).

Glue around fclones 0.35.0 (https://github.com/pkolaczk/fclones, MIT):
`fclones group ... --hash-fn sha256 -f json` finds identical files and
`fclones dedupe` replaces redundant copies with APFS clones (`cp -c`: every path
keeps its bytes, mode, mtime and xattrs; nothing is deleted).

    plan    filter the fclones groups and write the pre-dedupe manifest
            (group, path, bytes, sha256, mode, mtime_ns). Dropped from dedupe:
            files modified in the last --idle-minutes, files under a folder with
            such recent activity (a running job), files with more than one hard
            link (a clone would split the link), and files open in any process.
    verify  re-hash every planned path with Python hashlib and compare bytes,
            sha256, mode and mtime with the pre-dedupe manifest.

The dedupe itself is `fclones dedupe --modified-before <plan time - idle> < plan.json`.
"""

from __future__ import annotations

import argparse
import collections
from concurrent.futures import ThreadPoolExecutor
import datetime as dt
import json
import os
from pathlib import Path
import subprocess
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from scripts.tree_manifest import file_sha256  # noqa: E402

OUTPUTS = Path("/Users/changmin/projects/ugrp/outputs")
# Folders whose second level is checked for activity (their top level holds this job's receipts).
SPLIT_TOP = {"retired-worktrees", "archive"}


def activity_key(path: str) -> str:
    rel = Path(path).relative_to(OUTPUTS).parts
    return "/".join(rel[:2]) if rel[0] in SPLIT_TOP and len(rel) > 2 else rel[0]


def active_folders(cutoff: float) -> set[str]:
    active = set()
    for top in sorted(OUTPUTS.iterdir()):
        if top.is_symlink() or not top.is_dir():
            continue
        subs = [top / c for c in os.listdir(top)] if top.name in SPLIT_TOP else [top]
        for folder in subs:
            if not folder.is_dir() or folder.is_symlink():
                continue
            for dirpath, _dirs, files in os.walk(folder):
                if any(os.lstat(os.path.join(dirpath, f)).st_mtime > cutoff for f in files):
                    active.add(str(folder.relative_to(OUTPUTS)))
                    break
    return active


def open_files() -> set[str]:
    result = subprocess.run(["lsof", "-n", "-P", "-w", "-F", "n"], capture_output=True, text=True)
    if result.returncode not in (0, 1) or not result.stdout:
        raise SystemExit("lsof failed; refusing to plan")
    prefix = str(OUTPUTS.resolve()) + os.sep
    return {line[1:] for line in result.stdout.splitlines() if line.startswith("n" + prefix)}


def cmd_plan(args: argparse.Namespace) -> int:
    groups = json.loads(args.groups.read_text())
    now = time.time()
    cutoff = now - args.idle_minutes * 60
    active = active_folders(cutoff)
    opened = open_files()
    dropped = collections.Counter()
    kept_groups, rows = [], []
    for index, group in enumerate(groups["groups"]):
        keep = []
        for path in group["files"]:
            st = os.lstat(path)
            if st.st_mtime > cutoff:
                dropped["recent_file"] += 1
            elif activity_key(path) in active:
                dropped["active_folder"] += 1
            elif st.st_nlink > 1:
                dropped["hard_link"] += 1
            elif path in opened:
                dropped["open_file"] += 1
            elif st.st_size != group["file_len"]:
                dropped["size_changed"] += 1
            else:
                keep.append((path, st))
        if len(keep) < 2:
            dropped["group_too_small"] += 1
            continue
        kept_groups.append({**group, "files": [p for p, _ in keep]})
        for path, st in keep:
            rows.append((index, path, st.st_size, group["file_hash"], oct(st.st_mode & 0o7777), st.st_mtime_ns))
    plan = {"header": {**groups["header"], "filtered_by": "experiments/2026-09-26-disk-apply/dedupe_outputs.py",
                       "planned_at": dt.datetime.fromtimestamp(now).astimezone().isoformat(),
                       "modified_before": dt.datetime.fromtimestamp(cutoff).astimezone().isoformat()},
            "groups": kept_groups}
    for path in (args.plan, args.manifest):
        if path.exists():
            raise SystemExit(f"refusing to overwrite {path}")
    args.plan.write_text(json.dumps(plan) + "\n")
    with open(args.manifest, "w", encoding="utf-8") as out:
        out.write("group\tpath\tbytes\tsha256\tmode\tmtime_ns\n")
        for row in rows:
            out.write("\t".join(map(str, row)) + "\n")
    redundant = sum(g["file_len"] * (len(g["files"]) - 1) for g in kept_groups)
    summary = {"groups_in": len(groups["groups"]), "groups_planned": len(kept_groups), "files_planned": len(rows),
               "redundant_bytes_planned": redundant, "dropped": dict(dropped), "active_folders": sorted(active),
               "open_files_under_outputs": len(opened), "modified_before": plan["header"]["modified_before"]}
    print(json.dumps(summary, indent=1))
    args.plan.with_suffix(".summary.json").write_text(json.dumps(summary, indent=1) + "\n")
    return 0


def check(row: list[str]) -> str | None:
    _group, path, size, digest, mode, mtime_ns = row
    try:
        st = os.lstat(path)
        if st.st_size != int(size) or oct(st.st_mode & 0o7777) != mode or st.st_mtime_ns != int(mtime_ns):
            return f"stat changed: {path}"
        if file_sha256(Path(path)) != digest:
            return f"sha256 changed: {path}"
    except OSError as exc:
        return f"unreadable: {path}: {exc}"
    return None


def cmd_verify(args: argparse.Namespace) -> int:
    with open(args.manifest, encoding="utf-8") as handle:
        next(handle)
        rows = [line.rstrip("\n").split("\t") for line in handle]
    started = time.time()
    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        problems = [p for p in pool.map(check, rows, chunksize=256) if p]
    result = {"files_checked": len(rows), "problems": len(problems), "examples": problems[:20],
              "seconds": round(time.time() - started, 1), "checked": "bytes, sha256 (hashlib), mode, mtime_ns"}
    print(json.dumps(result, indent=1))
    if args.out:
        args.out.write_text(json.dumps(result, indent=1) + "\n")
    return 0 if not problems else 1


def df_avail_kib(path: Path) -> tuple[str, int]:
    lines = path.read_text().splitlines()
    return lines[0], int(lines[2].split()[3])


def cmd_summary(args: argparse.Namespace) -> int:
    folder = args.folder
    plan = json.loads((folder / "dedupe-plan.summary.json").read_text())
    verify = json.loads((folder / "dedupe-verify.json").read_text())
    log = (folder / "fclones-dedupe.log").read_text()
    t0, before = df_avail_kib(folder / "df-before-dedupe.txt")
    t1, after = df_avail_kib(folder / "df-after-dedupe.txt")
    sha = {name: file_sha256(folder / name) for name in
           ("fclones-group.json", "dedupe-plan.json", "dedupe-manifest-before.tsv", "fclones-dedupe.log")}
    summary = {
        "tool": "fclones 0.35.0 (Homebrew bottle, MIT) group --min 4KiB --hash-fn sha256; dedupe --modified-before",
        "plan": plan, "fclones_log_tail": log.strip().splitlines()[-1],
        "fclones_log_errors": sum(w in line.lower() for line in log.splitlines() for w in ("error", "warn")),
        "df_before": {"at": t0, "avail_gib": round(before / 2**20, 2)},
        "df_after": {"at": t1, "avail_gib": round(after / 2**20, 2)},
        "df_gain_gib": round((after - before) / 2**20, 2),
        "planned_redundant_gib": round(plan["redundant_bytes_planned"] / 2**30, 2),
        "verify": verify, "raw_sha256": sha,
    }
    out = folder / "dedupe-summary.json"
    if out.exists():
        raise SystemExit(f"refusing to overwrite {out}")
    out.write_text(json.dumps(summary, indent=1) + "\n")
    print(json.dumps({k: summary[k] for k in ("df_gain_gib", "planned_redundant_gib", "fclones_log_errors")}
                     | {"verify_problems": verify["problems"], "files_checked": verify["files_checked"]}))
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n", 1)[0])
    sub = parser.add_subparsers(dest="action", required=True)
    plan = sub.add_parser("plan")
    plan.add_argument("--groups", type=Path, required=True, help="fclones group -f json output")
    plan.add_argument("--plan", type=Path, required=True, help="filtered groups for fclones dedupe")
    plan.add_argument("--manifest", type=Path, required=True, help="pre-dedupe TSV manifest")
    plan.add_argument("--idle-minutes", type=float, default=60)
    verify = sub.add_parser("verify")
    verify.add_argument("--manifest", type=Path, required=True)
    verify.add_argument("--workers", type=int, default=3)
    verify.add_argument("--out", type=Path)
    summary = sub.add_parser("summary", help="combine plan, fclones log, df and verify into dedupe-summary.json")
    summary.add_argument("--folder", type=Path, required=True)
    args = parser.parse_args()
    return {"plan": cmd_plan, "verify": cmd_verify, "summary": cmd_summary}[args.action](args)


if __name__ == "__main__":
    raise SystemExit(main())
