#!/usr/bin/env python3
"""Preserve an unmerged Codex worktree before retirement (2026-09-26 disk apply, item 3).

For one worktree this records `git status --porcelain --ignored`, saves the
uncommitted diff (tracked + untracked) as a patch when there is one, writes a
git bundle of HEAD (prerequisite: its merge base with origin/main) and pushes
HEAD to refs/heads/codex/archive-<name>-0926 without touching the worktree.
It then checks with ls-remote that the remote ref equals HEAD. Nothing is
committed into the worktree; an existing archive ref with another SHA is an error.

Retirement itself is `scripts/agent_worktree.py retire <path> --archive-ref ...`.
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ARCHIVE_DIR = Path("/Users/changmin/projects/ugrp/outputs/archive/codex-worktrees")


def git(cwd: Path, *args: str, check: bool = True) -> subprocess.CompletedProcess:
    result = subprocess.run(["git", "--no-optional-locks", "-C", str(cwd), *args], capture_output=True)
    if check and result.returncode != 0:
        raise SystemExit(f"git {' '.join(args)} failed: {result.stderr.decode(errors='replace').strip()}")
    return result


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n", 1)[0])
    parser.add_argument("worktree", type=Path)
    parser.add_argument("--name", required=True, help="short name used in the archive ref and file names")
    parser.add_argument("--base", default="origin/main")
    parser.add_argument("--remote", default="origin")
    parser.add_argument("--out-dir", type=Path, default=ARCHIVE_DIR)
    args = parser.parse_args()
    path = args.worktree
    ref = f"refs/heads/codex/archive-{args.name}-0926"
    head = git(path, "rev-parse", "HEAD").stdout.decode().strip()
    branch = git(path, "symbolic-ref", "-q", "--short", "HEAD", check=False).stdout.decode().strip() or None
    status = git(path, "status", "--porcelain=v1", "--ignored=traditional").stdout.decode()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    record = {"schema": "ugrp.codex-worktree-archive.v1", "worktree": str(path), "name": args.name,
              "branch": branch, "head": head, "archive_ref": ref,
              "taken_at": dt.datetime.now().astimezone().isoformat(), "status_porcelain_ignored": status}
    # Uncommitted work: tracked diff against HEAD plus untracked (not ignored) files.
    tracked = git(path, "diff", "--binary", "HEAD").stdout
    untracked = [line[3:] for line in status.splitlines() if line.startswith("?? ")]
    if tracked or untracked:
        patch = args.out_dir / f"{args.name}-uncommitted-0926.patch"
        if patch.exists():
            raise SystemExit(f"refusing to overwrite {patch}")
        extra = b""
        for rel in untracked:  # --no-index diff against /dev/null gives an applicable patch
            extra += subprocess.run(["git", "-C", str(path), "diff", "--binary", "--no-index", "/dev/null", rel],
                                    capture_output=True).stdout
        patch.write_bytes(tracked + extra)
        record["patch"] = {"path": str(patch), "bytes": patch.stat().st_size, "sha256": sha256(patch),
                           "untracked_files": untracked}
    else:
        record["patch"] = None
    base = git(path, "merge-base", args.base, head).stdout.decode().strip()
    bundle = args.out_dir / f"{args.name}-0926.bundle"
    if bundle.exists():
        raise SystemExit(f"refusing to overwrite {bundle}")
    git(path, "bundle", "create", str(bundle), "HEAD", f"^{base}")
    verify = git(path, "bundle", "verify", str(bundle), check=False)
    heads = git(path, "bundle", "list-heads", str(bundle)).stdout.decode().split()
    record["bundle"] = {"path": str(bundle), "bytes": bundle.stat().st_size, "sha256": sha256(bundle),
                        "prerequisite": base, "verify_exit": verify.returncode, "head_in_bundle": heads[:1]}
    if verify.returncode != 0 or not heads or heads[0] != head:
        raise SystemExit(f"bundle check failed: {verify.stderr.decode(errors='replace')}")
    remote_now = git(path, "ls-remote", args.remote, ref).stdout.decode().split()
    if remote_now and remote_now[0] != head:
        raise SystemExit(f"{ref} already exists on {args.remote} with {remote_now[0]}; not overwriting")
    if not remote_now:
        push = git(path, "push", "--quiet", args.remote, f"{head}:{ref}", check=False)
        record["push_exit"] = push.returncode
        record["push_stderr"] = push.stderr.decode(errors="replace").strip()[-400:]
    else:
        record["push_exit"] = None  # already archived with the same SHA
    after = git(path, "ls-remote", args.remote, ref).stdout.decode().split()
    record["remote_sha"] = after[0] if after else None
    record["remote_matches_head"] = bool(after) and after[0] == head
    out = args.out_dir / f"{args.name}-0926.json"
    if out.exists():
        raise SystemExit(f"refusing to overwrite {out}")
    out.write_text(json.dumps(record, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({k: record[k] for k in ("name", "head", "archive_ref", "remote_sha", "remote_matches_head")}
                     | {"patch": bool(record["patch"]), "bundle_bytes": record["bundle"]["bytes"]}))
    return 0 if record["remote_matches_head"] else 1


if __name__ == "__main__":
    sys.exit(main())
