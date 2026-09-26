#!/usr/bin/env python3
"""Create, slim and retire UGRP agent worktrees without losing ignored raw data.

Subcommands (see docs/disk_management.md):

new       Create a worktree from origin/main whose sparse checkout omits heavy
          media under experiments/ (archives, videos, images). Enforces a
          per-agent cap on registered worktrees.
sparsify  Apply the same sparse profile to an existing clean, idle worktree.
retire    After a merge (or after its HEAD was pushed to an archive branch), move
          every ignored non-cache entry out of the worktree, verify count, bytes
          and sha256 of every file, and only then run `git worktree remove`
          (never --force). Ignored `outputs/<name>` goes to the same relative
          path in the primary checkout when that path is free; everything else
          (and colliding names) goes to outputs/retired-worktrees/<label>/.
          Dry run unless --execute is given.

This tool never deletes raw data, never runs `git worktree remove --force`, and
never touches a worktree that has uncommitted changes, untracked files, recent
modifications, or a process whose working directory or open files are inside
it or whose command line names it.
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
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from scripts import tree_manifest  # noqa: E402
from scripts.worktree_guard import Refused, refuse_if_in_use  # noqa: E402
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
DEFAULT_IDLE_MINUTES = 60


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


def archived_evidence(primary: Path, row: dict, archive_ref: str, remote: str) -> str:
    """Unmerged work may be retired once its exact HEAD is on the remote under archive_ref."""
    ref = archive_ref if archive_ref.startswith("refs/") else f"refs/heads/{archive_ref}"
    out = git(primary, "ls-remote", remote, ref, check=False).split()
    if not out or out[0] != row["head"]:
        raise Refused(f"{remote} {ref} is {out[0][:12] if out else 'missing'}, not HEAD {row['head'][:12]}")
    return f"HEAD {row['head'][:12]} archived at {remote} {ref} (ls-remote)"


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
    if not args.allow_busy:
        refuse_if_in_use(path, args.idle_minutes, worktree_git_dir(path))
    before = tree_usage(path)
    if args.dry_run:
        print(f"would sparsify {path} (now {gib(before[1])}, {before[0]} files)")
        return 0
    apply_sparse(path)
    after = tree_usage(path)
    still_changed = [rel for code, rel in status_entries(path, "--untracked-files=no")]
    head_after = git(path, "rev-parse", "HEAD").strip()
    if still_changed or head_after != row["head"] or sparse_enabled(primary):
        raise RuntimeError(f"post-check failed: changes {still_changed[:3]}, head {head_after[:12]},"
                           f" primary sparse {sparse_enabled(primary)}")
    marker = row.get("marker") or {"schema": "ugrp.agent-worktree.v1", "owner": row["owner"]}
    marker.update({"sparse_profile": SPARSE_PROFILE, "sparse_patterns_sha256": patterns_sha256(),
                   "sparsified_at": dt.datetime.now().astimezone().isoformat()})
    write_marker(path, marker)
    print(json.dumps({"path": str(path), "head": head_after, "before_bytes": before[1], "after_bytes": after[1],
                      "before_files": before[0], "after_files": after[0], "tracked_clean_after": True,
                      "primary_sparse": False}, indent=2))
    return 0


def retire_checks(primary: Path, args: argparse.Namespace) -> tuple[dict, str]:
    """All refusals happen here, before anything is moved."""
    if not args.no_fetch:
        git(primary, "fetch", "origin")
    row = find_row(primary, args.path)
    if row["primary"]:
        raise Refused("refusing to retire the primary checkout")
    path = row["path"]
    if not path.is_dir():
        raise Refused(f"worktree directory is missing: {path} (use `git worktree prune` after checking)")
    if args.archive_ref:
        evidence = archived_evidence(primary, row, args.archive_ref, args.remote)
    else:
        evidence = merged_evidence(primary, row, args.base, args.pr)
    if evidence is None:
        raise Refused(f"not merged: HEAD {row['head'][:12]} is not in {args.base} and no merged PR has this head;"
                      " push it to an archive branch and pass --archive-ref")
    if not args.no_pr_check:
        open_prs = open_pr_numbers(primary, row["branch"])
        if open_prs is None:
            raise Refused("cannot read PR state with gh; check by hand and pass --no-pr-check")
        if open_prs and not args.allow_open_pr:
            raise Refused(f"branch {row['branch']} still has open PR(s) {open_prs}; its owner decides")
    dirty = status_entries(path, "--untracked-files=all")
    if dirty:
        raise Refused(f"uncommitted or untracked files ({len(dirty)}), e.g. {[r for _, r in dirty[:3]]}")
    refuse_if_in_use(path, args.idle_minutes, worktree_git_dir(path))
    return row, evidence


def retire_plan(primary: Path, path: Path, keep: list[str], label: str, same_path: bool) -> list[dict]:
    """Where each ignored entry goes: free outputs/<name> keeps its relative path in the primary."""
    items = []
    for rel in keep:
        source = path / rel
        if same_path and rel == "outputs" and source.is_dir() and not source.is_symlink():
            items += [f"outputs/{name}" for name in sorted(os.listdir(source))]
        else:
            items.append(rel)
    archive = primary / "outputs" / "retired-worktrees" / label
    plan, taken = [], set()
    for rel in items:
        target = primary / rel
        parts = Path(rel).parts
        free = not (target.exists() or target.is_symlink()) and target not in taken
        if not (same_path and len(parts) == 2 and parts[0] == "outputs" and free):
            target = archive / rel
        taken.add(target)
        count, size = logical_usage(path / rel)
        plan.append({"path": rel, "destination": str(target), "entries": count, "bytes": size,
                     "placement": "same-path" if target == primary / rel else "retired-worktrees"})
    return plan


def move_verified(path: Path, plan: list[dict], manifest_tsv: Path) -> list[dict]:
    """Hash, rename, hash again; stop at the first difference (the worktree is then kept)."""
    done = []
    with open(manifest_tsv, "w", encoding="utf-8") as out:
        out.write("item\tdestination\tpath\ttype\tsize\tsha256_or_target\n")
        for item in plan:
            source, target = path / item["path"], Path(item["destination"])
            before = tree_manifest.build(source)
            if target.exists() or target.is_symlink():
                raise RuntimeError(f"destination appeared during retirement: {target}")
            target.parent.mkdir(parents=True, exist_ok=True)
            os.rename(source, target)
            after = tree_manifest.build(target)
            problems = tree_manifest.compare(before, after)
            if problems or source.exists() or source.is_symlink():
                raise RuntimeError(f"verification failed for {item['path']}: {problems[:3]}")
            for rel in sorted(after):
                e = after[rel]
                out.write("\t".join([item["path"], str(target), rel, e["type"], str(e.get("size", "")),
                                      e.get("sha256") or e.get("target") or ""]) + "\n")
            done.append({**item, **tree_manifest.totals(after), "manifest_sha256": tree_manifest.digest(after)})
    return done


def cmd_retire(args: argparse.Namespace) -> int:
    primary = primary_checkout(args.primary)
    row, evidence = retire_checks(primary, args)
    path = row["path"]
    ignored = [rel for code, rel in status_entries(path, "--ignored=traditional", "--untracked-files=normal")
               if code == "!!"]
    keep = [rel.rstrip("/") for rel in ignored if not is_cache(rel)]
    caches = [rel.rstrip("/") for rel in ignored if is_cache(rel)]
    label = args.label or default_label(path, row["owner"])
    if not NAME_RE.match(label):
        raise Refused(f"invalid label: {label}")
    retired_root = primary / "outputs" / "retired-worktrees"
    receipt_dir = retired_root / label
    if receipt_dir.exists() or receipt_dir.is_symlink():
        raise Refused(f"destination exists: {receipt_dir}; pass a new --label")
    plan = retire_plan(primary, path, keep, label, not args.archive_layout)
    retired_root.mkdir(parents=True, exist_ok=True)
    if plan and path.stat().st_dev != retired_root.stat().st_dev:
        raise Refused("worktree and primary outputs are on different filesystems; move manually")
    summary = {
        "schema": "ugrp.worktree-retirement.v2", "worktree": str(path), "branch": row["branch"],
        "head": row["head"], "owner": row["owner"], "merged_evidence": evidence, "receipt": str(receipt_dir),
        "moved": plan, "moved_entries": sum(p["entries"] for p in plan), "moved_bytes": sum(p["bytes"] for p in plan),
        "deleted_caches": caches,
        "checkout_bytes_freed_est": tree_usage(path)[1] - sum(tree_usage(path / p["path"])[1] for p in plan),
    }
    if not args.execute:
        print(json.dumps({"dry_run": True, **summary}, ensure_ascii=False, indent=2))
        return 0
    receipt_dir.mkdir(parents=True)
    try:
        summary["moved"] = move_verified(path, plan, receipt_dir / "MANIFEST.tsv")
    except Exception as exc:  # keep a receipt of what already moved; never remove the worktree
        summary.update({"verified": False, "error": f"{type(exc).__name__}: {exc}"})
        write_receipt(retired_root, receipt_dir, summary)
        print(json.dumps(summary, ensure_ascii=False, indent=2))
        print(f"retirement stopped; worktree NOT removed: {exc}", file=sys.stderr)
        return 1
    summary.update({"verified": True, "verification": "file count, bytes and sha256 of every file equal after move",
                    "retired_at": dt.datetime.now().astimezone().isoformat()})
    remove = subprocess.run(["git", "-C", str(primary), "worktree", "remove", str(path)], capture_output=True, text=True)
    summary["worktree_remove_exit"] = remove.returncode
    summary["worktree_remove_stderr"] = remove.stderr.strip()
    write_receipt(retired_root, receipt_dir, summary)
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    if remove.returncode != 0:
        print(f"git worktree remove failed; data already moved (see {receipt_dir})", file=sys.stderr)
        return 1
    return 0


def write_receipt(retired_root: Path, receipt_dir: Path, summary: dict) -> None:
    (receipt_dir / "RETIRED.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n")
    with (retired_root / "retirements.jsonl").open("a") as log:
        log.write(json.dumps(summary, ensure_ascii=False) + "\n")


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
    sparsify.add_argument("--idle-minutes", type=float, default=DEFAULT_IDLE_MINUTES,
                          help="refuse when anything changed this recently (0 disables)")
    retire = sub.add_parser("retire", help="move ignored data to retired-worktrees, then remove the worktree")
    retire.add_argument("path", type=Path)
    retire.add_argument("--execute", action="store_true", help="act; default is a dry run")
    retire.add_argument("--label", help="folder name under outputs/retired-worktrees/")
    retire.add_argument("--base", default="origin/main")
    retire.add_argument("--pr", type=int, help="merged PR number for squash-merged branches")
    retire.add_argument("--no-fetch", action="store_true")
    retire.add_argument("--no-pr-check", action="store_true", help="skip the gh open-PR check (checked by hand)")
    retire.add_argument("--allow-open-pr", action="store_true", help="owner confirmed the open PR needs no worktree")
    retire.add_argument("--archive-ref", help="unmerged work: remote branch that already holds exactly this HEAD")
    retire.add_argument("--remote", default="origin", help="remote checked with ls-remote for --archive-ref")
    retire.add_argument("--archive-layout", action="store_true",
                        help="put everything under outputs/retired-worktrees/<label>/ (no same-path placement)")
    retire.add_argument("--idle-minutes", type=float, default=DEFAULT_IDLE_MINUTES,
                        help="refuse when anything changed this recently (0 disables)")
    args = parser.parse_args(argv)
    handler = {"new": cmd_new, "sparsify": cmd_sparsify, "retire": cmd_retire}[args.action]
    try:
        return handler(args)
    except Refused as exc:
        print(f"refused: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
