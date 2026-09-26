#!/usr/bin/env python3
"""In-use guards for touching an agent worktree (scripts/agent_worktree.py).

A worktree counts as in use when a process has its cwd or an open file inside it
(lsof), when another process's command line names it as a whole path (the
caller's own process ancestry is excluded, because an agent's prompt may name
other worktrees), or when a file outside .git or the git index/HEAD changed
within the idle window. Every check fails closed: a tool that cannot answer
raises Refused.
"""

from __future__ import annotations

import math
import os
from pathlib import Path
import re
import subprocess
import time

PATH_CHARS = re.compile(r"[A-Za-z0-9._-]")


class Refused(RuntimeError):
    """A safety check failed; nothing was changed."""


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


def own_ancestry() -> set[int]:
    """This process and its ancestors (an agent's own prompt may name other worktrees)."""
    out = subprocess.run(["ps", "-Ao", "pid=,ppid="], capture_output=True, text=True).stdout
    parent = {}
    for line in out.splitlines():
        fields = line.split()
        if len(fields) == 2:
            parent[int(fields[0])] = int(fields[1])
    seen: set[int] = set()
    pid = os.getpid()
    while pid and pid not in seen:
        seen.add(pid)
        pid = parent.get(pid, 0)
    return seen


def names_path(command: str, target: str) -> bool:
    """True when command contains target as a whole path (not as a prefix of a longer name)."""
    start = command.find(target)
    while start >= 0:
        end = start + len(target)
        if end >= len(command) or not PATH_CHARS.match(command[end]):
            return True
        start = command.find(target, start + 1)
    return False


def processes_naming(path: Path) -> list[dict]:
    """Other processes whose command line names path (e.g. a job started with that worktree)."""
    # -ww: unlimited width. procps (Linux) otherwise cuts `command` to 80 columns when
    # stdout is not a terminal, which would hide a worktree path late in the command line.
    result = subprocess.run(["ps", "-ww", "-Ao", "pid=,command="], capture_output=True, text=True)
    if result.returncode != 0:
        raise Refused("cannot list processes with ps; refusing")
    targets = {os.path.realpath(path), os.path.abspath(path)}
    mine = own_ancestry()
    found = []
    for line in result.stdout.splitlines():
        fields = line.strip().split(None, 1)
        if len(fields) != 2 or int(fields[0]) in mine:
            continue
        if any(names_path(fields[1], t) for t in targets):
            found.append({"pid": int(fields[0]), "command": fields[1][:120]})
    return found


def recent_activity(path: Path, minutes: float, gitdir: Path | None) -> list[str]:
    """Files (outside .git) and git index/HEAD of this worktree modified in the last `minutes`."""
    if minutes <= 0:
        return []
    cutoff = time.time() - minutes * 60
    hits = []
    for name in ("index", "HEAD", "logs/HEAD"):
        if gitdir and (gitdir / name).exists() and (gitdir / name).stat().st_mtime > cutoff:
            hits.append(f"git:{name}")
    for dirpath, dirnames, filenames in os.walk(path):
        dirnames[:] = [d for d in dirnames if d != ".git"]
        for name in filenames:
            try:
                if os.lstat(os.path.join(dirpath, name)).st_mtime > cutoff:
                    hits.append(os.path.relpath(os.path.join(dirpath, name), path))
            except OSError:
                continue
            if len(hits) >= 5:
                return hits
    return hits


def refuse_if_in_use(path: Path, idle_minutes: float, gitdir: Path | None) -> None:
    if isinstance(idle_minutes, bool) or not isinstance(idle_minutes, (int, float)) \
            or not math.isfinite(idle_minutes) or idle_minutes < 0:
        raise Refused(f"--idle-minutes must be a finite number >= 0, got {idle_minutes!r}")
    busy = processes_using(path)
    if busy:
        raise Refused("worktree in use: " + "; ".join(f"{p['pid']} {p['command']} {p['files'][:2]}" for p in busy))
    named = processes_naming(path)
    if named:
        raise Refused("worktree named by a running command: " + "; ".join(f"{p['pid']} {p['command']}" for p in named))
    recent = recent_activity(path, idle_minutes, gitdir)
    if recent:
        raise Refused(f"modified within {idle_minutes:g} min: {recent}")
