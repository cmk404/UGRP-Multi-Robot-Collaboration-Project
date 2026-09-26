"""Compare two M1 run directories (or sim_profile directories) for exact equivalence (dev tool).

Checks, in order: trajectory checkpoints (qpos/qvel/act SHA-256 every N mj_step,
from ``scripts/sim_profile.py``), issued commands, control-input frame rows and
the saved JPEG bytes, controller/skill/macro logs, eval-only logs and every
``result.json`` field. ``--until-sim-s T`` compares only rows before SIM time T
(a truncated run against a full one; result.json is then skipped because the
truncated run ends with SIM_LIMIT). manifest.json differences are listed but not
judged: code SHA, wall time, load and the recorded speedup set are expected to
differ.

Missing evidence is a failure, never a pass (Codex review of PR #209): both
sides must be directories with ``inputs/commands.jsonl`` and
``inputs/frames.jsonl`` (plus ``result.json``/``manifest.json`` for a full
comparison), every row must parse as a JSON object, both sides must reach the
compared window (last control frame at or after T), and the compared command,
frame, JPEG and (when both sides are sim_profile directories) checkpoint counts
must be nonzero. Exit code 0 = equivalent, 1 = different, 2 = insufficient evidence.

    python3 scripts/sim_equivalence.py A_DIR B_DIR [--until-sim-s 120] [--json out.json]

``A_DIR``/``B_DIR`` may be a run directory (with result.json) or a sim_profile
output directory (with run/ and qpos_checkpoints.jsonl).
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

LOGS = ('inputs/commands.jsonl', 'inputs/frames.jsonl', 'controller_events.jsonl', 'skill_events.jsonl',
        'macros.jsonl', 'eval_only/frames_eval.jsonl', 'eval_only/gt_trajectory.jsonl', 'eval_only/contacts.jsonl',
        'eval_only/retention.jsonl')
REQUIRED_LOGS = ('inputs/commands.jsonl', 'inputs/frames.jsonl')
REQUIRED_FULL = ('result.json', 'manifest.json')
CHECKPOINTS = 'qpos_checkpoints.jsonl'
MANIFEST_EXPECTED = ('code', 'wall_s', 'load_average', 'speedups', 'env', 'files')
EXIT_EQUIVALENT, EXIT_DIFFERENT, EXIT_INSUFFICIENT = 0, 1, 2


class Evidence:
    """Collects evidence gaps; any gap makes the verdict ``insufficient_evidence``."""

    def __init__(self) -> None:
        self.errors: list[str] = []

    def rows(self, side: str, path: Path, *, required: bool) -> list[dict]:
        if not path.is_file():
            if required:
                self.errors.append(f'{side}: missing {path}')
            return []
        out = []
        for number, line in enumerate(path.read_text().splitlines(), 1):
            if not line.strip():
                continue
            try:
                row = json.loads(line)
            except ValueError:
                self.errors.append(f'{side}: {path} line {number} is not JSON')
                continue
            if not isinstance(row, dict):
                self.errors.append(f'{side}: {path} line {number} is not a JSON object')
                continue
            out.append(row)
        return out

    def document(self, side: str, path: Path) -> dict:
        try:
            value = json.loads(path.read_text())
        except OSError:
            self.errors.append(f'{side}: missing {path}')
            return {}
        except ValueError:
            self.errors.append(f'{side}: {path} is not JSON')
            return {}
        if not isinstance(value, dict):
            self.errors.append(f'{side}: {path} is not a JSON object')
            return {}
        return value


def split_dirs(path: Path) -> tuple[Path, Path | None]:
    """(run_dir, profile_dir or None)."""
    if (path/'run'/'attempt_started.json').exists():
        return path/'run', path
    return path, (path.parent if (path.parent/CHECKPOINTS).exists() else None)


def row_time(row: dict):
    for key in ('t', 'sim_t'):
        value = row.get(key)
        if isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value):
            return float(value)
    return None


def cut(items: list[dict], until: float | None) -> list[dict]:
    if until is None:
        return items
    return [r for r in items if row_time(r) is None or row_time(r) < until]


def check_until(until) -> float | None:
    if until is None:
        return None
    if isinstance(until, bool) or not isinstance(until, (int, float)) or not math.isfinite(until) or until <= 0:
        raise ValueError(f'until must be a finite SIM time > 0 or None, got {until!r}')
    return float(until)


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


def frame_digest(side: str, run: Path, row: dict, ev: Evidence) -> str | None:
    name, recorded = row.get('file'), row.get('sha256')
    if not isinstance(name, str) or not isinstance(recorded, str):
        ev.errors.append(f'{side}: frame row {row.get("frame")!r} lacks file/sha256')
        return None
    try:
        digest = hashlib.sha256((run/name).read_bytes()).hexdigest()
    except OSError:
        ev.errors.append(f'{side}: missing frame file {run/name}')
        return None
    return digest if digest == recorded else None       # a JPEG that disagrees with its own log row never matches


def compare(a_path: Path, b_path: Path, until: float | None = None) -> dict:
    until = check_until(until)
    a_path, b_path = Path(a_path), Path(b_path)
    report: dict = {'a': str(a_path), 'b': str(b_path), 'until_sim_s': until, 'checks': {}, 'not_compared': []}
    checks, ev = report['checks'], Evidence()
    for side, path in (('A', a_path), ('B', b_path)):
        if not path.is_dir():
            ev.errors.append(f'{side}: {path} is not a directory')
    if ev.errors:
        return finish(report, ev)
    a_run, a_prof = split_dirs(a_path)
    b_run, b_prof = split_dirs(b_path)
    if a_prof and b_prof:
        ca = ev.rows('A', a_prof/CHECKPOINTS, required=True)
        cb = ev.rows('B', b_prof/CHECKPOINTS, required=True)
        checks['qpos_checkpoints'] = compare_lists(cut(ca, until), cut(cb, until))
        if until is None and len(ca) != len(cb):        # different lengths of full runs: compare the common prefix
            n = min(len(ca), len(cb))
            checks['qpos_checkpoints']['common_prefix_identical'] = ca[:n] == cb[:n]
        if not (cut(ca, until) and cut(cb, until)):
            ev.errors.append('qpos checkpoints: nothing to compare on at least one side')
    else:
        only = 'A' if a_prof else 'B' if b_prof else None
        report['not_compared'].append('qpos_checkpoints: ' + (f'only {only} is a sim_profile directory' if only
                                                              else 'neither side is a sim_profile directory'))
    untimed, logs = [], {}
    for name in LOGS:
        required = name in REQUIRED_LOGS
        la = ev.rows('A', a_run/name, required=required)
        lb = ev.rows('B', b_run/name, required=required)
        logs[name] = (la, lb)
        if not la and not lb and not required:
            continue
        if until is not None and any(row_time(r) is None for r in la + lb):
            if required:
                ev.errors.append(f'{name}: rows without SIM time cannot be cut at {until}')
            untimed.append(name)        # e.g. skill_events (no SIM time): a truncated run cannot be cut
            continue
        checks[name] = compare_lists(cut(la, until), cut(lb, until))
        if required and not (cut(la, until) and cut(lb, until)):
            ev.errors.append(f'{name}: no rows to compare on at least one side')
    if untimed:
        report['not_compared_untimed_rows'] = untimed
    if until is not None:
        reach = {side: max((t for t in map(row_time, rows) if t is not None), default=None)
                 for side, rows in zip('AB', logs['inputs/frames.jsonl'])}
        report['last_frame_sim_s'] = reach
        for side, last in reach.items():
            if last is None or last < until:
                ev.errors.append(f'{side}: control frames end at {last} s, before the compared window {until} s')
    fa, fb = (cut(rows, until) for rows in logs['inputs/frames.jsonl'])
    n = min(len(fa), len(fb))
    bad = []
    for i in range(n):
        ha, hb = frame_digest('A', a_run, fa[i], ev), frame_digest('B', b_run, fb[i], ev)
        if ha is None or ha != hb:
            bad.append(i)
    checks['frame_jpeg_bytes'] = {'identical': not bad and len(fa) == len(fb), 'compared': n,
                                  'mismatched': bad[:20], 'n_mismatched': len(bad)}
    if n == 0:
        ev.errors.append('frame JPEG bytes: nothing compared')
    if until is None:
        ra, rb = (ev.document(side, run/'result.json') for side, run in (('A', a_run), ('B', b_run)))
        diff = sorted(k for k in set(ra) | set(rb) if ra.get(k) != rb.get(k))
        checks['result.json'] = {'identical': bool(ra) and not diff, 'fields': len(ra), 'differing_fields': diff}
        ma, mb = (ev.document(side, run/'manifest.json') for side, run in (('A', a_run), ('B', b_run)))
        mdiff = sorted(k for k in set(ma) | set(mb) if ma.get(k) != mb.get(k))
        report['manifest_differences'] = {'expected': [k for k in mdiff if k in MANIFEST_EXPECTED],
                                          'unexpected': [k for k in mdiff if k not in MANIFEST_EXPECTED]}
        report['speedups'] = {'a': ma.get('speedups', 'none (field absent)'), 'b': mb.get('speedups', 'none (field absent)')}
    return finish(report, ev)


def finish(report: dict, ev: Evidence) -> dict:
    same = all(c['identical'] for c in report['checks'].values()) and not (
        report.get('manifest_differences', {}).get('unexpected'))
    report['evidence_errors'] = ev.errors
    report['verdict'] = 'insufficient_evidence' if ev.errors else 'equivalent' if same else 'different'
    report['equivalent'] = report['verdict'] == 'equivalent'
    return report


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('a', type=Path)
    p.add_argument('b', type=Path)
    p.add_argument('--until-sim-s', type=float, default=None)
    p.add_argument('--json', type=Path, default=None, help='also write the report here')
    args = p.parse_args(argv)
    try:
        report = compare(args.a, args.b, args.until_sim_s)
    except ValueError as exc:
        p.error(str(exc))
    text = json.dumps(report, indent=2, ensure_ascii=False)
    if args.json:
        args.json.write_text(text + '\n')
    print(text)
    return {'equivalent': EXIT_EQUIVALENT, 'different': EXIT_DIFFERENT}.get(report['verdict'], EXIT_INSUFFICIENT)


if __name__ == '__main__':
    sys.exit(main())
