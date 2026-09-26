#!/usr/bin/env python3
"""List ``outputs/...`` references in origin/main records that no local copy satisfies.

Read-only helper for the 2026-09-26 disk incident. A record can name its raw
folder relatively (``outputs/zone-cargo/final-d81514a``) even when the raw lived in
a worktree. This checks each referenced ``outputs/<a>/<b>`` against the primary
checkout, every registered worktree, and ``outputs/retired-worktrees/*/``. Missing
references are candidates only: CI paths, remote (Colab/Kaggle) paths, examples
and data deleted by earlier approved cleanups also appear here.

Usage:
    python3 experiments/2026-09-26-disk-incident/scan_missing_outputs.py \
        --repo /Users/changmin/projects/ugrp --output /tmp/missing.json
"""

from __future__ import annotations

import argparse
import collections
import json
import re
import subprocess
from pathlib import Path

REF = re.compile(r"(?<![A-Za-z0-9_.-])outputs/([A-Za-z0-9._-]+)(?:/([A-Za-z0-9._-]+))?")
PLACEHOLDER = re.compile(r"NEW|<|ID$|-ci\b|^ci-|example|\*", re.IGNORECASE)
SCAN_PREFIXES = ('experiments/', 'docs/', 'configs/', 'README.md', 'CONTRIBUTING.md')


def git(repo: Path, *args: str) -> str:
    return subprocess.run(['git', '-C', str(repo), *args], check=True, capture_output=True,
                          text=True).stdout


def roots(repo: Path) -> list[Path]:
    found = []
    for block in git(repo, 'worktree', 'list', '--porcelain').split('\n\n'):
        for line in block.splitlines():
            if line.startswith('worktree '):
                found.append(Path(line[9:]) / 'outputs')
    retired = repo / 'outputs' / 'retired-worktrees'
    if retired.is_dir():
        found.extend(p / 'outputs' for p in retired.iterdir() if (p / 'outputs').is_dir())
    return [p for p in found if p.is_dir()]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split('\n', 1)[0])
    parser.add_argument('--repo', type=Path, required=True)
    parser.add_argument('--ref', default='refs/remotes/origin/main')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    bases = roots(args.repo)
    refs: dict[str, set[str]] = collections.defaultdict(set)
    for line in git(args.repo, 'ls-tree', '-r', '--name-only', args.ref).splitlines():
        if not line.startswith(SCAN_PREFIXES) or not line.endswith(('.md', '.json', '.jsonl', '.py', '.txt')):
            continue
        text = git(args.repo, 'show', f'{args.ref}:{line}')
        for match in REF.finditer(text):
            first, second = match.group(1), match.group(2)
            key = first if not second else f'{first}/{second}'
            refs[key].add(line)
    missing = []
    for key, files in sorted(refs.items()):
        if PLACEHOLDER.search(key):
            continue
        first = key.split('/', 1)[0]
        if any((base / key).exists() or (base / first).exists() and '/' not in key for base in bases):
            continue
        partial = [str(base / first) for base in bases if (base / first).exists()]
        missing.append({'reference': f'outputs/{key}', 'top_level_exists_at': partial,
                        'files': sorted(files)})
    report = {'ref': git(args.repo, 'rev-parse', args.ref).strip(), 'search_roots': [str(b) for b in bases],
              'references_checked': len(refs), 'missing': missing}
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=1) + '\n')
    print(f'checked={len(refs)} missing={len(missing)} roots={len(bases)} -> {args.output}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
