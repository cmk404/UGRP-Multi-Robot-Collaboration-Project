"""Compare two M1 run directories (or sim_profile directories) for exact equivalence (dev tool).

Checks, in order: trajectory checkpoints (qpos/qvel/act SHA-256 every N mj_step,
from ``scripts/sim_profile.py``), issued commands, control-input frame rows and
the saved JPEG bytes, controller/skill/macro logs, eval-only logs and every
``result.json`` field. ``--until-sim-s T`` compares only rows before SIM time T
(a truncated run against a full one; result.json is then skipped because the
truncated run ends with SIM_LIMIT). manifest.json differences are listed but not
judged: code SHA, wall time, load and the recorded speedup set are expected to
differ.

    python3 scripts/sim_equivalence.py A_DIR B_DIR [--until-sim-s 120] [--json out.json]

``A_DIR``/``B_DIR`` may be a run directory (with result.json) or a sim_profile
output directory (with run/ and qpos_checkpoints.jsonl).
"""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path

LOGS = ('inputs/commands.jsonl', 'inputs/frames.jsonl', 'controller_events.jsonl', 'skill_events.jsonl',
        'macros.jsonl', 'eval_only/frames_eval.jsonl', 'eval_only/gt_trajectory.jsonl', 'eval_only/contacts.jsonl',
        'eval_only/retention.jsonl')
MANIFEST_EXPECTED = ('code', 'wall_s', 'load_average', 'speedups', 'env', 'files')


def split_dirs(path: Path) -> tuple[Path, Path | None]:
    """(run_dir, profile_dir or None)."""
    if (path/'run'/'attempt_started.json').exists():
        return path/'run', path
    return path, (path.parent if (path.parent/'qpos_checkpoints.jsonl').exists() else None)


def rows(path: Path) -> list[dict]:
    if not path.exists():
        return []
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def row_time(row: dict):
    for key in ('t', 'sim_t'):
        if isinstance(row.get(key), (int, float)):
            return float(row[key])
    return None


def cut(items: list[dict], until: float | None) -> list[dict]:
    if until is None:
        return items
    return [r for r in items if row_time(r) is None or row_time(r) < until]


def compare_lists(a: list, b: list) -> dict:
    n = min(len(a), len(b))
    first = next((i for i in range(n) if a[i] != b[i]), None)
    same = first is None and len(a) == len(b)
    out = {'identical': same, 'len_a': len(a), 'len_b': len(b)}
    if not same:
        out['first_diff_index'] = first if first is not None else n
        if first is not None:
            out['a'] = json.dumps(a[first], ensure_ascii=False)[:400]
            out['b'] = json.dumps(b[first], ensure_ascii=False)[:400]
    return out


def compare(a_path: Path, b_path: Path, until: float | None = None) -> dict:
    a_run, a_prof = split_dirs(a_path)
    b_run, b_prof = split_dirs(b_path)
    report: dict = {'a': str(a_path), 'b': str(b_path), 'until_sim_s': until, 'checks': {}}
    checks = report['checks']
    if a_prof and b_prof:
        ca, cb = rows(a_prof/'qpos_checkpoints.jsonl'), rows(b_prof/'qpos_checkpoints.jsonl')
        checks['qpos_checkpoints'] = compare_lists(cut(ca, until), cut(cb, until))
        if until is None and len(ca) != len(cb):        # different lengths of full runs: compare the common prefix
            n = min(len(ca), len(cb))
            checks['qpos_checkpoints']['common_prefix_identical'] = ca[:n] == cb[:n]
    untimed = []
    for name in LOGS:
        la, lb = rows(a_run/name), rows(b_run/name)
        if not la and not lb:
            continue
        if until is not None and any(row_time(r) is None for r in la + lb):
            untimed.append(name)        # e.g. skill_events (no SIM time): a truncated run cannot be cut
            continue
        checks[name] = compare_lists(cut(la, until), cut(lb, until))
    if untimed:
        report['not_compared_untimed_rows'] = untimed
    fa, fb = cut(rows(a_run/'inputs/frames.jsonl'), until), cut(rows(b_run/'inputs/frames.jsonl'), until)
    n = min(len(fa), len(fb))
    bad = []
    for i in range(n):
        ha = hashlib.sha256((a_run/fa[i]['file']).read_bytes()).hexdigest()
        hb = hashlib.sha256((b_run/fb[i]['file']).read_bytes()).hexdigest()
        if ha != hb or ha != fa[i]['sha256'] or hb != fb[i]['sha256']:
            bad.append(i)
    checks['frame_jpeg_bytes'] = {'identical': not bad and len(fa) == len(fb), 'compared': n,
                                  'mismatched': bad[:20], 'n_mismatched': len(bad)}
    if until is None:
        ra = json.loads((a_run/'result.json').read_text())
        rb = json.loads((b_run/'result.json').read_text())
        diff = sorted(k for k in set(ra) | set(rb) if ra.get(k) != rb.get(k))
        checks['result.json'] = {'identical': not diff, 'fields': len(ra), 'differing_fields': diff}
        ma, mb = (json.loads((r/'manifest.json').read_text()) for r in (a_run, b_run))
        mdiff = sorted(k for k in set(ma) | set(mb) if ma.get(k) != mb.get(k))
        report['manifest_differences'] = {'expected': [k for k in mdiff if k in MANIFEST_EXPECTED],
                                          'unexpected': [k for k in mdiff if k not in MANIFEST_EXPECTED]}
        report['speedups'] = {'a': ma.get('speedups', 'none (field absent)'), 'b': mb.get('speedups', 'none (field absent)')}
    report['equivalent'] = all(c['identical'] for c in checks.values()) and not (
        report.get('manifest_differences', {}).get('unexpected'))
    return report


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('a', type=Path)
    p.add_argument('b', type=Path)
    p.add_argument('--until-sim-s', type=float, default=None)
    p.add_argument('--json', type=Path, default=None, help='also write the report here')
    args = p.parse_args(argv)
    report = compare(args.a, args.b, args.until_sim_s)
    text = json.dumps(report, indent=2, ensure_ascii=False)
    if args.json:
        args.json.write_text(text + '\n')
    print(text)
    return 0 if report['equivalent'] else 1


if __name__ == '__main__':
    sys.exit(main())
