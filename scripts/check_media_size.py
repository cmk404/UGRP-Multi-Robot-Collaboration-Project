#!/usr/bin/env python3
"""Warn when a change adds heavy media to Git (limits in docs/disk_management.md).

Checks the files that a change adds or modifies:

    --staged      the index (used by .githooks/pre-commit)
    --base REV    the commits in REV...HEAD (used by CI for pull requests)

Rules, reported as warnings:

1. a media/archive file under experiments/ is larger than 1 MiB;
2. an experiments/<ID>/ directory holds more than 5 MiB of media after the change
   (reported only when the change adds or grows media there);
3. any file is larger than 5 MiB.

Raw frames, videos and large logs belong in outputs/ with their hashes recorded
in the experiment. The exit status is 0 unless --strict is given (1 on any
warning); the existing hook still rejects single files over 20 MiB.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from scripts.agent_worktree import HEAVY_SUFFIXES  # noqa: E402  (same media kinds as the sparse profile)

MIB = 1 << 20
EXPERIMENT_FILE_LIMIT = 1 * MIB
EXPERIMENT_MEDIA_LIMIT = 5 * MIB
ANY_FILE_LIMIT = 5 * MIB
DOC = "docs/disk_management.md"


def git(*args: str, check: bool = True) -> str:
    result = subprocess.run(["git", *args], capture_output=True, text=True)
    if check and result.returncode != 0:
        raise RuntimeError(f"git {' '.join(args)}: {result.stderr.strip()}")
    return result.stdout


def is_media(path: str) -> bool:
    name = path.rsplit("/", 1)[-1].lower()
    return "." in name and name.rsplit(".", 1)[-1] in HEAVY_SUFFIXES


def experiment_of(path: str) -> str | None:
    parts = path.split("/")
    return "/".join(parts[:2]) if len(parts) >= 3 and parts[0] == "experiments" else None


def object_sizes(names: list[str]) -> dict[str, int]:
    """Sizes for object names such as ':path', 'HEAD:path' or blob SHAs."""
    if not names:
        return {}
    result = subprocess.run(["git", "cat-file", "--batch-check=%(objectsize)"], input="\n".join(names) + "\n",
                            capture_output=True, text=True, check=True)
    sizes = {}
    for name, line in zip(names, result.stdout.splitlines()):
        sizes[name] = int(line) if line.strip().isdigit() else 0
    return sizes


def tree_media(treeish: str | None, folder: str) -> int:
    """Total media bytes under folder in a commit tree (None: nothing)."""
    if treeish is None:
        return 0
    total = 0
    for record in git("ls-tree", "-r", "-l", "-z", treeish, "--", folder, check=False).split("\0"):
        if not record:
            continue
        meta, path = record.split("\t", 1)
        size = meta.split()[3]
        if size.isdigit() and is_media(path):
            total += int(size)
    return total


def index_media(folder: str) -> int:
    shas = []
    for row in git("ls-files", "-s", "-z", "--", folder).split("\0"):
        if row:
            meta, path = row.split("\t", 1)
            if is_media(path):
                shas.append(meta.split()[1])
    sizes = object_sizes(shas)
    return sum(sizes.get(sha, 0) for sha in shas)


def head_or_none() -> str | None:
    return "HEAD" if subprocess.run(["git", "rev-parse", "--verify", "--quiet", "HEAD"],
                                    capture_output=True).returncode == 0 else None


def check(changed: dict[str, int], media_after, media_before) -> list[tuple[str, str]]:
    warnings = []
    folders: dict[str, int] = {}
    for path, size in sorted(changed.items()):
        folder = experiment_of(path)
        if folder and is_media(path):
            folders[folder] = folders.get(folder, 0) + size
            if size > EXPERIMENT_FILE_LIMIT:
                warnings.append((path, f"{size / MIB:.1f} MiB media file in {folder}/ exceeds 1 MiB; keep it in "
                                       f"outputs/ and record its sha256 ({DOC})"))
                continue
        if size > ANY_FILE_LIMIT:
            warnings.append((path, f"{size / MIB:.1f} MiB file exceeds 5 MiB ({DOC})"))
    for folder in sorted(folders):
        after, before = media_after(folder), media_before(folder)
        if after > EXPERIMENT_MEDIA_LIMIT and after > before:
            warnings.append((folder, f"{folder}/ holds {after / MIB:.1f} MiB of media after this change "
                                     f"(was {before / MIB:.1f} MiB; budget 5 MiB per experiment, {DOC})"))
    return warnings


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n", 1)[0])
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--staged", action="store_true", help="check the index (pre-commit)")
    mode.add_argument("--base", help="check REV...HEAD (pull request)")
    parser.add_argument("--strict", action="store_true", help="exit 1 when there is any warning")
    parser.add_argument("--github-annotations", action="store_true", help="also print ::warning lines")
    args = parser.parse_args(argv)
    if args.staged:
        paths = [p for p in git("diff", "--cached", "--name-only", "--diff-filter=ACMR", "-z").split("\0") if p]
        sizes = object_sizes([f":{p}" for p in paths])
        changed = {p: sizes.get(f":{p}", 0) for p in paths}
        head = head_or_none()
        warnings = check(changed, index_media, lambda folder: tree_media(head, folder))
    else:
        merge_base = git("merge-base", args.base, "HEAD").strip()
        paths = [p for p in git("diff", "--name-only", "--diff-filter=ACMR", "-z", f"{merge_base}", "HEAD").split("\0")
                 if p]
        sizes = object_sizes([f"HEAD:{p}" for p in paths])
        changed = {p: sizes.get(f"HEAD:{p}", 0) for p in paths}
        warnings = check(changed, lambda folder: tree_media("HEAD", folder),
                         lambda folder: tree_media(merge_base, folder))
    for path, message in warnings:
        print(f"WARNING (media budget): {path}: {message}", file=sys.stderr)
        if args.github_annotations:
            print(f"::warning file={path}::{message}")
    if warnings:
        print(f"media budget: {len(warnings)} warning(s); this check does not block the commit", file=sys.stderr)
    return 1 if (warnings and args.strict) else 0


if __name__ == "__main__":
    raise SystemExit(main())
