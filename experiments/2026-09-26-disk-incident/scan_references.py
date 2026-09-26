#!/usr/bin/env python3
"""Scan Git refs for references to worktrees that were removed on 2026-09-26.

Read-only: lists refs, reads blobs through ``git cat-file --batch`` and writes
one JSON report. Each unique text blob is scanned once even when many branches
contain it. Paths under ignored locations (``outputs/``, ``MUJOCO_LOG.TXT``) are
classified as data that ``git worktree remove`` deleted; other paths are tracked
files that remain recoverable from Git.

Usage:
    python3 experiments/2026-09-26-disk-incident/scan_references.py \
        --repo /Users/changmin/projects/ugrp --output /tmp/refs.json
"""

from __future__ import annotations

import argparse
import collections
import json
import re
import subprocess
from pathlib import Path

# Removed by the manager session with `git worktree remove` (2026-09-26 13:50 KST),
# from wt-remove-ugrp.tsv. Names are directory names under the given parent.
REMOVED_UGRP = {
    'ugrp-worktrees': [
        'agent-coordination', 'beam-regrasp', 'beam-regrasp-coarse', 'current-status',
        'dynamic-coordination', 'fine-gain-schedule', 'plan-objective-preview',
        'post-run-replay', 'r3-pose-record', 'realtime-stop-gap', 'records-0925',
        'team-recovery-fix', 'zc3-prereg', 'zone-cargo', 'zone-cargo-perception',
        'zone-cargo-perception-v2', 'zone-comm-audit', 'zone-communication',
        'zone-dialogue-ko', 'zone-dispatch', 'zone-rgb-color', 'zone-rgb-outcome',
        'zone-team-jobs', 'zone-wide-arena',
    ],
    'ugrp-wt': ['records-0925b', 'zone-owncam-loc', 'zone-owncam-skill'],
}
REMOVED_TOP = ['ugrp-wt-goto']
# Codex-app worktrees that were already gone before this job (inventory 2026-09-25
# recorded no outputs/ for them). Scanned for completeness.
REMOVED_CODEX = [
    'act-route-coverage', 'act-teacher-load-policy', 'action-act-followup-results',
    'action-act-model-release', 'action-act-results', 'local-suite-host-lock',
]
TEXT_SUFFIXES = {
    '.md', '.json', '.jsonl', '.py', '.txt', '.csv', '.sh', '.yml', '.yaml', '.toml',
    '.log', '.stdout', '.command', '.html', '.tsv', '.cfg', '.ini', '',
}
# ASCII path characters only, so Korean text directly after a path is not captured.
PATH_CHARS = r"[A-Za-z0-9._\-/+=@%~*]*"
PATTERN = re.compile(
    r"(?P<root>(?:ugrp-worktrees|ugrp-wt)/(?P<name>[A-Za-z0-9._-]+)"
    r"|ugrp-wt-goto"
    r"|\.codex/worktrees/(?P<codex>[A-Za-z0-9._-]+)/ugrp)"
    r"(?P<rest>/" + PATH_CHARS + r")?"
)
IGNORED_PREFIXES = ('outputs/', 'outputs', 'MUJOCO_LOG.TXT', 'stress_logs/', 'work/', 'tmp/')


def git(repo: Path, *args: str) -> str:
    return subprocess.run(['git', '-C', str(repo), *args], check=True, capture_output=True,
                          text=True).stdout


def removed_root(match: re.Match) -> str | None:
    root = match.group('root')
    if root == 'ugrp-wt-goto':
        return 'ugrp-wt-goto'
    if match.group('codex'):
        return f".codex/worktrees/{match.group('codex')}/ugrp" if match.group('codex') in REMOVED_CODEX else None
    parent, name = root.split('/', 1)
    return root if name in REMOVED_UGRP.get(parent, []) else None


def classify(rest: str) -> str:
    rel = rest.lstrip('/').rstrip('.')
    if not rel:
        return 'worktree_root'
    if rel.startswith(IGNORED_PREFIXES):
        return 'ignored_deleted'
    return 'tracked_or_other'


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split('\n', 1)[0])
    parser.add_argument('--repo', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    refs = [r for r in git(args.repo, 'for-each-ref', '--format=%(refname)', 'refs/remotes/origin',
                           'refs/heads').split() if not r.endswith('/HEAD')]
    blob_paths: dict[str, set[str]] = collections.defaultdict(set)
    blob_refs: dict[str, set[str]] = collections.defaultdict(set)
    for ref in refs:
        for line in git(args.repo, 'ls-tree', '-r', ref).splitlines():
            meta, path = line.split('\t', 1)
            _mode, kind, blob = meta.split()
            if kind != 'blob' or Path(path).suffix.lower() not in TEXT_SUFFIXES:
                continue
            blob_paths[blob].add(path)
            blob_refs[blob].add(ref)
    blobs = sorted(blob_paths)
    proc = subprocess.Popen(['git', '-C', str(args.repo), 'cat-file', '--batch'], stdin=subprocess.PIPE,
                            stdout=subprocess.PIPE)
    assert proc.stdin and proc.stdout
    hits: dict[str, dict] = {}
    for blob in blobs:
        proc.stdin.write((blob + '\n').encode())
        proc.stdin.flush()
        header = proc.stdout.readline().split()
        size = int(header[2])
        data = proc.stdout.read(size)
        proc.stdout.read(1)
        if b'ugrp-w' not in data and b'.codex/worktrees' not in data:
            continue
        text = data.decode('utf-8', errors='replace')
        for match in PATTERN.finditer(text):
            root = removed_root(match)
            if root is None:
                continue
            rest = (match.group('rest') or '').rstrip('.')
            full = root + rest
            row = hits.setdefault(full, {
                'reference': full, 'removed_root': root, 'class': classify(rest),
                'files': set(), 'refs': set(),
            })
            row['files'].update(blob_paths[blob])
            row['refs'].update(blob_refs[blob])
    proc.stdin.close()
    proc.wait()
    rows = []
    for full, row in sorted(hits.items()):
        refs_sorted = sorted(row['refs'])
        rows.append({
            'reference': full, 'removed_root': row['removed_root'], 'class': row['class'],
            'files': sorted(row['files']),
            'on_origin_main': 'refs/remotes/origin/main' in row['refs'],
            'refs': refs_sorted,
        })
    summary = collections.Counter((r['removed_root'], r['class']) for r in rows)
    report = {
        'repo': str(args.repo),
        'head_origin_main': git(args.repo, 'rev-parse', 'refs/remotes/origin/main').strip(),
        'refs_scanned': len(refs),
        'unique_text_blobs': len(blobs),
        'references': rows,
        'summary': [{'removed_root': k[0], 'class': k[1], 'references': v}
                    for k, v in sorted(summary.items())],
    }
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=1) + '\n')
    print(f"refs={len(refs)} blobs={len(blobs)} references={len(rows)} -> {args.output}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
