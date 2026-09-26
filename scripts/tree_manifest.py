#!/usr/bin/env python3
"""Content manifests for moving, thinning and deduplicating raw data safely.

A manifest maps each relative path under a root to its type, size and content:
regular files carry a sha256, symlinks carry their target (never followed).
`compare` reports every path whose presence, type, size or content differs, so a
move is verified by building the manifest before and after and requiring no
difference. Used by scripts/agent_worktree.py (retire) and scripts/disk_dedupe.py.
"""

from __future__ import annotations

import hashlib
import os
from pathlib import Path
import stat

CHUNK = 1 << 20


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for block in iter(lambda: handle.read(CHUNK), b""):
            digest.update(block)
    return digest.hexdigest()


def entry(path: Path) -> dict:
    st = os.lstat(path)
    if stat.S_ISLNK(st.st_mode):
        return {"type": "link", "size": st.st_size, "target": os.readlink(path)}
    if stat.S_ISREG(st.st_mode):
        return {"type": "file", "size": st.st_size, "sha256": file_sha256(path)}
    if stat.S_ISDIR(st.st_mode):
        return {"type": "dir"}
    return {"type": "other", "size": st.st_size}


def build(root: Path) -> dict[str, dict]:
    """Manifest of root: a file/symlink gives {'.': ...}; a directory lists everything below it.

    Empty directories are recorded (type 'dir') so that a move that drops them is detected.
    """
    root = Path(root)
    if root.is_symlink() or not root.is_dir():
        return {".": entry(root)}
    result: dict[str, dict] = {}
    for dirpath, dirnames, filenames in os.walk(root):
        base = Path(dirpath)
        for name in sorted(dirnames):
            full = base / name
            if full.is_symlink():
                result[str(full.relative_to(root))] = entry(full)
            elif not os.listdir(full):
                result[str(full.relative_to(root))] = {"type": "dir"}
        for name in sorted(filenames):
            full = base / name
            result[str(full.relative_to(root))] = entry(full)
    return result


def totals(manifest: dict[str, dict]) -> dict:
    files = [e for e in manifest.values() if e["type"] == "file"]
    return {"entries": len(manifest), "files": len(files),
            "links": sum(e["type"] == "link" for e in manifest.values()),
            "bytes": sum(e.get("size", 0) for e in manifest.values() if e["type"] in ("file", "link", "other"))}


def digest(manifest: dict[str, dict]) -> str:
    """One sha256 over the sorted manifest lines (path, type, size, content)."""
    lines = []
    for rel in sorted(manifest):
        e = manifest[rel]
        lines.append("\t".join([rel, e["type"], str(e.get("size", "")), e.get("sha256") or e.get("target") or ""]))
    return hashlib.sha256("\n".join(lines).encode()).hexdigest()


def compare(before: dict[str, dict], after: dict[str, dict]) -> list[str]:
    problems = []
    for rel in sorted(set(before) | set(after)):
        if rel not in after:
            problems.append(f"missing after: {rel}")
        elif rel not in before:
            problems.append(f"unexpected after: {rel}")
        elif before[rel] != after[rel]:
            problems.append(f"changed: {rel}: {before[rel]} != {after[rel]}")
    return problems


def write_tsv(manifest: dict[str, dict], path: Path, prefix: str = "") -> None:
    with open(path, "w", encoding="utf-8") as out:
        out.write("path\ttype\tsize\tsha256_or_target\n")
        for rel in sorted(manifest):
            e = manifest[rel]
            name = prefix if rel == "." else (f"{prefix}/{rel}" if prefix else rel)
            out.write("\t".join([name, e["type"], str(e.get("size", "")),
                                 e.get("sha256") or e.get("target") or ""]) + "\n")
