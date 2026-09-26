#!/usr/bin/env python3
"""Read-only census of every registered UGRP worktree before the 2026-09-26 disk apply.

For each worktree it records: owner, branch/HEAD, merged evidence (ancestor of
origin/main or a MERGED PR with the same head), open PRs, sparse state, tracked
changes, untracked files, ignored non-cache entries, allocated size, the newest
mtime outside .git (and of its git index/HEAD), and live processes whose cwd is
inside it or whose command line names it. The caller's own process ancestry is
excluded because this job's prompt names other worktrees.

Nothing is changed. Output: JSON on stdout (or --out).
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time

HERE = Path(__file__).resolve()
sys.path.insert(0, str(HERE.parents[2] / "scripts"))
import agent_worktree as aw  # noqa: E402  (reuse owner inference, status parsing, sizing)

RECENT_SECONDS = 60 * 60


def ancestry(pid: int) -> set[int]:
    out = subprocess.run(["ps", "-Ao", "pid=,ppid="], capture_output=True, text=True).stdout
    parent = {}
    for line in out.splitlines():
        a, b = line.split()
        parent[int(a)] = int(b)
    seen = set()
    while pid and pid not in seen:
        seen.add(pid)
        pid = parent.get(pid, 0)
    return seen


def process_table(exclude: set[int]) -> tuple[list[dict], dict[int, str]]:
    out = subprocess.run(["ps", "-Ao", "pid=,ppid=,command="], capture_output=True, text=True).stdout
    rows = []
    for line in out.splitlines():
        parts = line.strip().split(None, 2)
        if len(parts) < 3:
            continue
        pid, ppid, command = int(parts[0]), int(parts[1]), parts[2]
        if pid in exclude:
            continue
        rows.append({"pid": pid, "ppid": ppid, "command": command})
    # cwd of every process of this user (lsof -d cwd over all pids is fast)
    cwd: dict[int, str] = {}
    res = subprocess.run(["lsof", "-a", "-d", "cwd", "-n", "-P", "-w", "-F", "pn", "-u", str(os.getuid())],
                         capture_output=True, text=True)
    pid = None
    for line in res.stdout.splitlines():
        if line.startswith("p"):
            pid = int(line[1:])
        elif line.startswith("n") and pid is not None:
            cwd[pid] = line[1:]
    return rows, cwd


def inside(value: str, root: str) -> bool:
    return value == root or value.startswith(root + os.sep)


def newest_mtime(path: Path) -> tuple[float, str]:
    best, where = 0.0, ""
    for dirpath, dirnames, filenames in os.walk(path):
        dirnames[:] = [d for d in dirnames if d != ".git"]
        for name in filenames:
            full = os.path.join(dirpath, name)
            try:
                m = os.lstat(full).st_mtime
            except OSError:
                continue
            if m > best:
                best, where = m, full
    return best, where


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n", 1)[0])
    parser.add_argument("--primary", type=Path, default=Path("/Users/changmin/projects/ugrp"))
    parser.add_argument("--base", default="origin/main")
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    primary = args.primary.resolve()
    now = time.time()
    me = ancestry(os.getpid())
    procs, cwds = process_table(me)
    prs = json.loads(subprocess.run(
        ["gh", "pr", "list", "--state", "open", "--limit", "300", "--json", "number,headRefName,headRefOid"],
        cwd=primary, capture_output=True, text=True, check=True).stdout)
    merged_prs = json.loads(subprocess.run(
        ["gh", "pr", "list", "--state", "merged", "--limit", "400", "--json", "number,headRefName,headRefOid"],
        cwd=primary, capture_output=True, text=True, check=True).stdout)
    lock = subprocess.run([sys.executable, str(primary / "scripts" / "agent_lock.py"), "status"],
                          cwd=primary, capture_output=True, text=True).stdout.strip()
    rows = []
    for row in aw.worktrees(primary):
        path = row["path"]
        rec = {
            "path": str(path), "branch": row["branch"], "head": row.get("head"), "detached": row["detached"],
            "owner": row["owner"], "owner_source": row["owner_source"], "primary": row["primary"],
            "exists": path.is_dir(),
        }
        if row["primary"] or not path.is_dir():
            rows.append(rec)
            continue
        real = os.path.realpath(path)
        head = row["head"]
        rec["ancestor_of_base"] = aw.is_ancestor(primary, head, args.base)
        rec["merged_pr"] = [p["number"] for p in merged_prs if p["headRefOid"] == head]
        rec["open_pr"] = [p["number"] for p in prs if row["branch"] and p["headRefName"] == row["branch"]]
        rec["open_pr_same_head"] = [p["number"] for p in prs if p["headRefOid"] == head]
        rec["merged"] = rec["ancestor_of_base"] or bool(rec["merged_pr"])
        rec["sparse"] = aw.sparse_enabled(path)
        tracked = aw.status_entries(path, "--untracked-files=no")
        untracked = [r for c, r in aw.status_entries(path, "--untracked-files=all") if c == "??"]
        ignored = [r for c, r in aw.status_entries(path, "--ignored=traditional", "--untracked-files=normal")
                   if c == "!!"]
        rec["tracked_changes"] = [r for _, r in tracked]
        rec["untracked"] = untracked[:50]
        rec["untracked_count"] = len(untracked)
        rec["ignored_keep"] = [r.rstrip("/") for r in ignored if not aw.is_cache(r)]
        rec["ignored_cache"] = [r.rstrip("/") for r in ignored if aw.is_cache(r)]
        files, alloc = aw.tree_usage(path)
        rec["files"], rec["allocated_bytes"] = files, alloc
        rec["ignored_keep_bytes"] = sum(aw.logical_usage(path / r)[1] for r in rec["ignored_keep"])
        m, where = newest_mtime(path)
        gitdir = aw.worktree_git_dir(path)
        gm = max((os.stat(gitdir / n).st_mtime for n in ("index", "HEAD", "logs/HEAD")
                  if gitdir and (gitdir / n).exists()), default=0.0)
        rec["newest_mtime_age_min"] = round((now - m) / 60, 1) if m else None
        rec["newest_mtime_file"] = where
        rec["gitdir_mtime_age_min"] = round((now - gm) / 60, 1) if gm else None
        rec["recent_60min"] = bool((m and now - m < RECENT_SECONDS) or (gm and now - gm < RECENT_SECONDS))
        users = []
        for p in procs:
            c = cwds.get(p["pid"], "")
            hit_cwd = c and (inside(c, real) or inside(c, str(path)))
            hit_cmd = str(path) in p["command"] or real in p["command"]
            if hit_cwd or hit_cmd:
                users.append({"pid": p["pid"], "cwd": c, "via": "cwd" if hit_cwd else "cmdline",
                              "command": p["command"][:160]})
        rec["processes"] = users
        rows.append(rec)
    report = {"schema": "ugrp.disk-apply.worktree-census.v1", "taken_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
              "base": args.base, "base_sha": aw.git(primary, "rev-parse", args.base).strip(),
              "agent_lock_status": lock, "excluded_own_pids": sorted(me), "worktrees": rows}
    text = json.dumps(report, ensure_ascii=False, indent=2)
    if args.out:
        args.out.write_text(text + "\n")
    else:
        print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
