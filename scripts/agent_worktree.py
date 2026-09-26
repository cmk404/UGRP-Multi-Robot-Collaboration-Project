#!/usr/bin/env python3
"""Create, slim and retire UGRP agent worktrees without losing ignored raw data.

Subcommands (see docs/disk_management.md):

new       Create a worktree from origin/main whose sparse checkout omits heavy
          media under experiments/ (archives, videos, images). Enforces a
          per-agent cap on registered worktrees.
sparsify  Apply the same sparse profile to an existing clean worktree.
retire    After a merge, move every ignored non-cache entry (outputs/, logs, ...)
          into the primary checkout's outputs/retired-worktrees/<label>/, verify
          file count and bytes, and only then run `git worktree remove`
          (never --force). Dry run unless --execute is given.

This tool never deletes raw data, never runs `git worktree remove --force`, and
never touches a worktree that has uncommitted changes, untracked files or a
process whose working directory or open files are inside it.
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
AGENTS = ("kiro", "claude", "codex")
DEFAULT_CAP = 8
SPARSE_PROFILE = "agent-media-v1"
# Heavy file kinds under experiments/: archives, videos, images, rendered reports and
# arrays. JSON/JSONL/XML/Markdown/Python records stay checked out.
HEAVY_SUFFIXES = (
    "zip", "gz", "tgz", "xz", "bz2", "7z", "tar",
    "mp4", "mov", "m4v", "avi", "webm", "mkv",
    "gif", "jpg", "jpeg", "png", "webp", "bmp", "tif", "tiff",
    "pdf", "html", "npz", "npy",
)
# Tracked archives that code reads at run time (sim/workflow_manager.py,
# scripts/sim_dispatch.py, scripts/build_pair_terrain_gallery.py).
RUNTIME_KEEP = (
    "experiments/dispatch-skill-integration-20260917/models.zip",
    "experiments/2026-09-10-rgb-varied-start/models.zip",
)
CACHE_PARTS = {"__pycache__", ".pytest_cache", ".mypy_cache", ".ruff_cache"}
CACHE_SUFFIXES = (".pyc", ".pyo")
CACHE_NAMES = {".DS_Store"}
MARKER = "ugrp-worktree.json"
NAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]*$")


class Refused(RuntimeError):
    """A safety check failed; nothing was changed."""


def sparse_patterns() -> list[str]:
    lines = ["/*"]
    lines += [f"!/experiments/**/*.{suffix}" for suffix in HEAVY_SUFFIXES]
    lines += [f"/{path}" for path in RUNTIME_KEEP]
    return lines


def patterns_sha256() -> str:
    return hashlib.sha256("\n".join(sparse_patterns()).encode()).hexdigest()


def git(cwd: Path, *args: str, check: bool = True, input_text: str | None = None) -> str:
    result = subprocess.run(
        ["git", "--no-optional-locks", "-C", str(cwd), *args],
        capture_output=True, text=True, input=input_text,
    )
    if check and result.returncode != 0:
        raise RuntimeError(f"git {' '.join(args)} failed ({result.returncode}): {result.stderr.strip()}")
    return result.stdout


def primary_checkout(explicit: Path | None) -> Path:
    if explicit:
        return explicit.resolve()
    common = Path(git(ROOT, "rev-parse", "--path-format=absolute", "--git-common-dir").strip())
    return common.parent.resolve()


def worktrees(primary: Path) -> list[dict]:
    rows: list[dict] = []
    for block in git(primary, "worktree", "list", "--porcelain").strip().split("\n\n"):
        row: dict = {"branch": None, "detached": False, "locked": False, "prunable": False}
        for line in block.splitlines():
            key, _, value = line.partition(" ")
            if key == "worktree":
                row["path"] = Path(value)
            elif key == "HEAD":
                row["head"] = value
            elif key == "branch":
                row["branch"] = value.removeprefix("refs/heads/")
            elif key in ("detached", "locked", "prunable"):
                row[key] = True
        if "path" in row:
            rows.append(row)
    for row in rows:
        row["primary"] = row["path"].resolve() == primary
        row["marker"] = read_marker(row["path"]) if row["path"].exists() else None
        row["owner"], row["owner_source"] = infer_owner(row)
    return rows


def worktree_git_dir(path: Path) -> Path | None:
    try:
        return Path(git(path, "rev-parse", "--absolute-git-dir").strip())
    except RuntimeError:
        return None


def read_marker(path: Path) -> dict | None:
    gitdir = worktree_git_dir(path)
    if gitdir is None or not (gitdir / MARKER).is_file():
        return None
    try:
        return json.loads((gitdir / MARKER).read_text())
    except (OSError, json.JSONDecodeError):
        return None


def infer_owner(row: dict) -> tuple[str, str]:
    marker = row.get("marker") or {}
    if marker.get("owner") in AGENTS:
        return marker["owner"], "marker"
    branch = row.get("branch") or ""
    prefix = branch.split("/", 1)[0]
    if prefix in AGENTS:
        return prefix, "branch"
    path = str(row["path"])
    if "/.codex/worktrees/" in path:
        return "codex", "path"
    if row["path"].name.startswith("kiro-"):
        return "kiro", "path"
    if "/ugrp-wt/" in path or "/ugrp-worktrees/" in path:
        return "claude", "path"
    return "unknown", "none"


def is_ancestor(cwd: Path, commit: str, ref: str) -> bool:
    return subprocess.run(["git", "-C", str(cwd), "merge-base", "--is-ancestor", commit, ref],
                          capture_output=True).returncode == 0


def sparse_enabled(path: Path) -> bool:
    value = git(path, "config", "--get", "core.sparseCheckout", check=False).strip().lower()
    return value == "true"


def tree_usage(path: Path) -> tuple[int, int]:
    """Return (files, allocated bytes) under path without following symlinks."""
    files = size = 0
    seen: set[tuple[int, int]] = set()
    if path.is_symlink() or path.is_file():
        st = path.lstat()
        return 1, st.st_blocks * 512
    for dirpath, dirnames, filenames in os.walk(path):
        for name in filenames + [d for d in dirnames if os.path.islink(os.path.join(dirpath, d))]:
            try:
                st = os.lstat(os.path.join(dirpath, name))
            except OSError:
                continue
            key = (st.st_dev, st.st_ino)
            if key in seen:
                continue
            seen.add(key)
            files += 1
            size += st.st_blocks * 512
    return files, size


def logical_usage(path: Path) -> tuple[int, int]:
    """Return (entries, logical bytes); symlinks count as one entry of their own size."""
    if path.is_symlink() or not path.is_dir():
        return 1, path.lstat().st_size
    count = total = 0
    for dirpath, dirnames, filenames in os.walk(path):
        for name in filenames + [d for d in dirnames if os.path.islink(os.path.join(dirpath, d))]:
            count += 1
            total += os.lstat(os.path.join(dirpath, name)).st_size
    return count, total


def gib(value: int) -> str:
    return f"{value / 2**30:.2f} GiB"


def processes_using(path: Path) -> list[dict]:
    """Processes of this user whose cwd or open files are inside path (via lsof)."""
    # lsof reports kernel (resolved) paths; also accept the path as given.
    targets = {os.path.realpath(path), os.path.abspath(path)}
    result = subprocess.run(["lsof", "-n", "-P", "-w", "-F", "pcfn"], capture_output=True, text=True)
    if result.returncode not in (0, 1) or not result.stdout:
        raise Refused(f"cannot inspect open files with lsof (exit {result.returncode}); refusing")
    found: dict[int, dict] = {}
    pid = None
    command = ""
    fd = ""
    for line in result.stdout.splitlines():
        tag, value = line[:1], line[1:]
        if tag == "p":
            pid, command = int(value), ""
        elif tag == "c":
            command = value
        elif tag == "f":
            fd = value
        elif tag == "n" and pid is not None and pid != os.getpid():
            if any(value == t or value.startswith(t + os.sep) for t in targets):
                entry = found.setdefault(pid, {"pid": pid, "command": command, "files": []})
                if len(entry["files"]) < 5:
                    entry["files"].append(f"{fd}:{value}")
    return sorted(found.values(), key=lambda row: row["pid"])


def status_entries(path: Path, *extra: str) -> list[tuple[str, str]]:
    raw = git(path, "status", "--porcelain=v1", "-z", *extra)
    entries = []
    items = raw.split("\0")
    index = 0
    while index < len(items):
        item = items[index]
        index += 1
        if not item:
            continue
        code, rel = item[:2], item[3:]
        if code[0] in "RC":  # rename/copy: the next item is the source path
            index += 1
        entries.append((code, rel))
    return entries


def is_cache(rel: str) -> bool:
    parts = Path(rel.rstrip("/")).parts
    return bool(CACHE_PARTS.intersection(parts)) or rel.endswith(CACHE_SUFFIXES) or (
        bool(parts) and parts[-1] in CACHE_NAMES)


def default_label(path: Path, owner: str) -> str:
    name = path.parent.name if path.name == "ugrp" and "/.codex/worktrees/" in str(path) else path.name
    if name.startswith(f"{owner}-"):
        return name
    return f"{owner}-{name}" if owner != "unknown" else name


def find_row(primary: Path, path: Path) -> dict:
    target = path.resolve()
    for row in worktrees(primary):
        if row["path"].resolve() == target:
            return row
    raise Refused(f"not a registered worktree of {primary}: {path}")


def merged_evidence(primary: Path, row: dict, base: str, pr: int | None) -> str | None:
    head = row["head"]
    if is_ancestor(primary, head, base):
        return f"HEAD {head[:12]} is an ancestor of {base}"
    if not shutil.which("gh"):
        return None
    query = ["gh", "pr", "view", str(pr), "--json", "number,state,headRefOid"] if pr else (
        ["gh", "pr", "list", "--head", row["branch"], "--state", "merged", "--json", "number,state,headRefOid"]
        if row.get("branch") else None)
    if query is None:
        return None
    result = subprocess.run(query, cwd=primary, capture_output=True, text=True)
    if result.returncode != 0:
        return None
    data = json.loads(result.stdout or "null")
    for item in data if isinstance(data, list) else [data]:
        if item and item.get("state") == "MERGED" and item.get("headRefOid") == head:
            return f"PR #{item['number']} MERGED with head {head[:12]} (squash or rebase merge)"
    return None


def open_pr_numbers(primary: Path, branch: str | None) -> list[int] | None:
    """Open PRs whose head is branch (read-only gh call); None when gh cannot answer."""
    if not branch:
        return []
    if not shutil.which("gh"):
        return None
    try:
        result = subprocess.run(["gh", "pr", "list", "--head", branch, "--state", "open", "--json", "number"],
                                cwd=primary, capture_output=True, text=True, timeout=60)
    except (OSError, subprocess.TimeoutExpired):
        return None
    if result.returncode != 0:
        return None
    return [item["number"] for item in json.loads(result.stdout or "[]")]


def write_marker(path: Path, record: dict) -> None:
    gitdir = worktree_git_dir(path)
    if gitdir is not None:
        (gitdir / MARKER).write_text(json.dumps(record, indent=2, ensure_ascii=False) + "\n")


def apply_sparse(path: Path) -> None:
    git(path, "sparse-checkout", "set", "--no-cone", "--stdin", input_text="\n".join(sparse_patterns()) + "\n")


def cmd_new(args: argparse.Namespace) -> int:
    primary = primary_checkout(args.primary)
    if not NAME_RE.match(args.name):
        raise Refused("name must use letters, digits, '.', '-' or '_'")
    if args.detach:
        owner = args.owner
        if owner not in AGENTS:
            raise Refused(f"--detach needs --owner {'|'.join(AGENTS)}")
    else:
        if not args.branch:
            raise Refused("give --branch <agent>/<topic> or --detach")
        owner = args.branch.split("/", 1)[0]
        if owner not in AGENTS or "/" not in args.branch:
            raise Refused(f"branch must start with one of {', '.join(a + '/' for a in AGENTS)}")
    parent = (args.parent or primary.parent / "ugrp-wt").resolve()
    path = parent / args.name
    if path.exists():
        raise Refused(f"path already exists: {path}")
    if not args.no_fetch:
        git(primary, "fetch", "origin")
    base_sha = git(primary, "rev-parse", "--verify", f"{args.base}^{{commit}}").strip()
    rows = [row for row in worktrees(primary) if not row["primary"] and row["owner"] == owner]
    if len(rows) >= args.max_per_agent and not args.allow_over_cap:
        print(f"{owner} already has {len(rows)} registered worktrees (cap {args.max_per_agent}).", file=sys.stderr)
        for row in rows:
            merged = is_ancestor(primary, row["head"], args.base)
            print(f"  {'merged' if merged else 'open  '} {row['path']} ({row['branch'] or 'detached'})",
                  file=sys.stderr)
        print("Retire merged ones with `retire`, or pass --allow-over-cap with a reason.", file=sys.stderr)
        return 3
    if args.allow_over_cap and len(rows) >= args.max_per_agent and not args.reason:
        raise Refused("--allow-over-cap needs --reason")
    command = ["worktree", "add"]
    if not args.full:
        command.append("--no-checkout")
    if args.detach:
        command += ["--detach", str(path), base_sha]
    else:
        exists = git(primary, "rev-parse", "--verify", "--quiet", f"refs/heads/{args.branch}", check=False).strip()
        if exists and not args.existing_branch:
            raise Refused(f"branch {args.branch} exists; pass --existing-branch to check it out")
        command += [str(path), args.branch] if exists else ["--no-track", "-b", args.branch, str(path), base_sha]
    parent.mkdir(parents=True, exist_ok=True)
    git(primary, *command)
    if not args.full:
        apply_sparse(path)
        git(path, "read-tree", "-mu", "HEAD")
    hooks = False
    if not args.no_hooks and git(path, "config", "--get", "extensions.worktreeConfig", check=False).strip() == "true":
        git(path, "config", "--worktree", "core.hooksPath", ".githooks")
        hooks = True
    record = {
        "schema": "ugrp.agent-worktree.v1", "owner": owner, "created_at": dt.datetime.now().astimezone().isoformat(),
        "branch": None if args.detach else args.branch, "base": args.base, "base_sha": base_sha,
        "sparse_profile": None if args.full else SPARSE_PROFILE,
        "sparse_patterns_sha256": None if args.full else patterns_sha256(),
        "hooks_path": ".githooks" if hooks else None, "over_cap_reason": args.reason,
    }
    write_marker(path, record)
    files, size = tree_usage(path)
    print(json.dumps({"path": str(path), **record, "files": files, "allocated_bytes": size}, ensure_ascii=False, indent=2))
    return 0


def cmd_sparsify(args: argparse.Namespace) -> int:
    primary = primary_checkout(args.primary)
    row = find_row(primary, args.path)
    if row["primary"]:
        raise Refused("the primary checkout stays a full checkout")
    path = row["path"]
    changed = [rel for code, rel in status_entries(path, "--untracked-files=no")]
    if changed:
        raise Refused(f"tracked changes present ({len(changed)}), e.g. {changed[:3]}; commit or ask the owner")
    busy = processes_using(path)
    if busy and not args.allow_busy:
        raise Refused("worktree in use: " + "; ".join(f"{p['pid']} {p['command']}" for p in busy))
    before = tree_usage(path)
    if args.dry_run:
        print(f"would sparsify {path} (now {gib(before[1])}, {before[0]} files)")
        return 0
    apply_sparse(path)
    after = tree_usage(path)
    marker = row.get("marker") or {"schema": "ugrp.agent-worktree.v1", "owner": row["owner"]}
    marker.update({"sparse_profile": SPARSE_PROFILE, "sparse_patterns_sha256": patterns_sha256(),
                   "sparsified_at": dt.datetime.now().astimezone().isoformat()})
    write_marker(path, marker)
    print(json.dumps({"path": str(path), "before_bytes": before[1], "after_bytes": after[1],
                      "before_files": before[0], "after_files": after[0]}, indent=2))
    return 0


def cmd_retire(args: argparse.Namespace) -> int:
    primary = primary_checkout(args.primary)
    if not args.no_fetch:
        git(primary, "fetch", "origin")
    row = find_row(primary, args.path)
    if row["primary"]:
        raise Refused("refusing to retire the primary checkout")
    path = row["path"]
    if not path.is_dir():
        raise Refused(f"worktree directory is missing: {path} (use `git worktree prune` after checking)")
    evidence = merged_evidence(primary, row, args.base, args.pr)
    if evidence is None:
        raise Refused(f"not merged: HEAD {row['head'][:12]} is not in {args.base} and no merged PR has this head")
    if not args.no_pr_check:
        open_prs = open_pr_numbers(primary, row["branch"])
        if open_prs is None:
            raise Refused("cannot read PR state with gh; check by hand and pass --no-pr-check")
        if open_prs and not args.allow_open_pr:
            raise Refused(f"branch {row['branch']} still has open PR(s) {open_prs}; its owner decides")
    dirty = status_entries(path, "--untracked-files=all")
    if dirty:
        raise Refused(f"uncommitted or untracked files ({len(dirty)}), e.g. {[r for _, r in dirty[:3]]}")
    busy = processes_using(path)
    if busy:
        raise Refused("worktree in use: " + "; ".join(f"{p['pid']} {p['command']} {p['files'][:2]}" for p in busy))
    ignored = [rel for code, rel in status_entries(path, "--ignored=traditional", "--untracked-files=normal")
               if code == "!!"]
    keep = [rel.rstrip("/") for rel in ignored if not is_cache(rel)]
    caches = [rel.rstrip("/") for rel in ignored if is_cache(rel)]
    label = args.label or default_label(path, row["owner"])
    if not NAME_RE.match(label):
        raise Refused(f"invalid label: {label}")
    retired_root = primary / "outputs" / "retired-worktrees"
    dest = retired_root / label
    plan = []
    for rel in keep:
        count, size = logical_usage(path / rel)
        plan.append({"path": rel, "entries": count, "bytes": size})
    summary = {
        "schema": "ugrp.worktree-retirement.v1", "worktree": str(path), "branch": row["branch"],
        "head": row["head"], "owner": row["owner"], "merged_evidence": evidence,
        "destination": str(dest) if plan else None, "moved": plan,
        "moved_entries": sum(p["entries"] for p in plan), "moved_bytes": sum(p["bytes"] for p in plan),
        "deleted_caches": caches, "checkout_bytes_freed_est": None,
    }
    if plan and (dest.exists() or dest.is_symlink()):
        raise Refused(f"destination exists: {dest}; pass a new --label")
    if plan:
        retired_root.mkdir(parents=True, exist_ok=True)
        if path.stat().st_dev != retired_root.stat().st_dev:
            raise Refused("worktree and primary outputs are on different filesystems; move manually")
    total_files, total_bytes = tree_usage(path)
    moved_alloc = sum(tree_usage(path / p["path"])[1] for p in plan)
    summary["checkout_bytes_freed_est"] = total_bytes - moved_alloc
    if not args.execute:
        print(json.dumps({"dry_run": True, **summary}, ensure_ascii=False, indent=2))
        return 0
    moved = []
    for item in plan:
        source, target = path / item["path"], dest / item["path"]
        target.parent.mkdir(parents=True, exist_ok=True)
        os.rename(source, target)
        moved.append(item["path"])
        count, size = logical_usage(target)
        if (count, size) != (item["entries"], item["bytes"]) or source.exists() or source.is_symlink():
            raise RuntimeError(f"verification failed for {item['path']}: before {item['entries']}/{item['bytes']}"
                               f" after {count}/{size}; worktree NOT removed, moved so far: {moved}")
    summary["verified"] = True
    summary["retired_at"] = dt.datetime.now().astimezone().isoformat()
    if plan:
        (dest / "RETIRED.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n")
    remove = subprocess.run(["git", "-C", str(primary), "worktree", "remove", str(path)], capture_output=True, text=True)
    summary["worktree_remove_exit"] = remove.returncode
    summary["worktree_remove_stderr"] = remove.stderr.strip()
    retired_root.mkdir(parents=True, exist_ok=True)
    with (retired_root / "retirements.jsonl").open("a") as log:
        log.write(json.dumps(summary, ensure_ascii=False) + "\n")
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    if remove.returncode != 0:
        print(f"git worktree remove failed; data already moved to {dest}", file=sys.stderr)
        return 1
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n", 1)[0])
    parser.add_argument("--primary", type=Path, help="primary checkout (default: this repository's)")
    sub = parser.add_subparsers(dest="action", required=True)
    new = sub.add_parser("new", help="create a sparse agent worktree")
    new.add_argument("name")
    new.add_argument("--branch", help="new branch <kiro|claude|codex>/<topic>")
    new.add_argument("--detach", action="store_true", help="detached HEAD at --base (frozen source)")
    new.add_argument("--owner", choices=AGENTS, help="owner for --detach")
    new.add_argument("--existing-branch", action="store_true", help="check out an existing local branch")
    new.add_argument("--base", default="origin/main")
    new.add_argument("--parent", type=Path, help="parent directory (default: <primary>/../ugrp-wt)")
    new.add_argument("--full", action="store_true", help="full checkout, including experiments/ media")
    new.add_argument("--no-fetch", action="store_true")
    new.add_argument("--no-hooks", action="store_true", help="do not set core.hooksPath=.githooks")
    new.add_argument("--max-per-agent", type=int, default=DEFAULT_CAP)
    new.add_argument("--allow-over-cap", action="store_true")
    new.add_argument("--reason", help="required with --allow-over-cap when over the cap")
    sparsify = sub.add_parser("sparsify", help="apply the sparse profile to an existing clean worktree")
    sparsify.add_argument("path", type=Path)
    sparsify.add_argument("--dry-run", action="store_true")
    sparsify.add_argument("--allow-busy", action="store_true", help="owner confirmed running processes are safe")
    retire = sub.add_parser("retire", help="move ignored data to retired-worktrees, then remove the worktree")
    retire.add_argument("path", type=Path)
    retire.add_argument("--execute", action="store_true", help="act; default is a dry run")
    retire.add_argument("--label", help="folder name under outputs/retired-worktrees/")
    retire.add_argument("--base", default="origin/main")
    retire.add_argument("--pr", type=int, help="merged PR number for squash-merged branches")
    retire.add_argument("--no-fetch", action="store_true")
    retire.add_argument("--no-pr-check", action="store_true", help="skip the gh open-PR check (checked by hand)")
    retire.add_argument("--allow-open-pr", action="store_true", help="owner confirmed the open PR needs no worktree")
    args = parser.parse_args(argv)
    handler = {"new": cmd_new, "sparsify": cmd_sparsify, "retire": cmd_retire}[args.action]
    try:
        return handler(args)
    except Refused as exc:
        print(f"refused: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
