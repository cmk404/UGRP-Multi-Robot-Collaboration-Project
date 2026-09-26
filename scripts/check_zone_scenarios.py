#!/usr/bin/env python3
"""Offline feasibility check of zone study scenarios: refuse impossible scenarios before they ship.

Reads scenario configs (``configs/zone_study_scenarios/*.json`` or any file in the
same layout) plus the authored maps under ``maps/zones/`` and prints a Korean
report per scenario. No MuJoCo, no scene, no model call, no simulator state, and
no scenario file is written: a check is read-only.

    python3 scripts/check_zone_scenarios.py --scenario-dir configs/zone_study_scenarios
    python3 scripts/check_zone_scenarios.py --scenario /tmp/s6.json --json out.json

Exit code 1 when any scenario carries an ``infeasible`` finding, 0 otherwise
(``--strict`` also fails on a ``warning``, e.g. a forced clearing order).
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from harness.zone_scenario_feasibility import (GRID_M, SCHEMA, TEAM_MARGIN_M, YAW_STEPS,  # noqa: E402
                                               evaluate, report_ko)
from sim.zone_arena import MAP_DIR  # noqa: E402

DEFAULT_DIR = ROOT / 'configs' / 'zone_study_scenarios'


def scenario_paths(args) -> list[Path]:
    paths = [Path(p) for p in args.scenario]
    for directory in args.scenario_dir or ([DEFAULT_DIR] if not paths else []):
        paths += sorted(Path(directory).glob('*.json'))
    if not paths:
        raise SystemExit('no scenario config given (--scenario / --scenario-dir)')
    return paths


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--scenario', action='append', default=[], metavar='FILE',
                        help='a scenario config; repeatable')
    parser.add_argument('--scenario-dir', action='append', metavar='DIR',
                        help='every *.json in this directory; repeatable')
    parser.add_argument('--maps-dir', default=str(MAP_DIR), help='authored zone maps (read-only)')
    parser.add_argument('--margin-m', type=float, default=TEAM_MARGIN_M,
                        help='planning footprint growth of the carrying formation')
    parser.add_argument('--grid-m', type=float, default=GRID_M, help='pose lattice step')
    parser.add_argument('--yaw-steps', type=int, default=YAW_STEPS, help='pose lattice yaw steps')
    parser.add_argument('--no-swept', action='store_true',
                        help='skip the per-segment swept re-check of an accepted route')
    parser.add_argument('--json', metavar='FILE', help='write every report as JSON')
    parser.add_argument('--markdown', metavar='FILE', help='write the Korean reports as Markdown')
    parser.add_argument('--strict', action='store_true', help='also fail on a warning finding')
    parser.add_argument('--quiet', action='store_true', help='print only the verdict lines')
    args = parser.parse_args(argv)

    reports, records, texts = [], [], []
    for path in scenario_paths(args):
        scenario = json.loads(Path(path).read_text())
        report = evaluate(scenario, maps_dir=args.maps_dir, margin=args.margin_m, grid=args.grid_m,
                          yaw_steps=args.yaw_steps, verify_swept=not args.no_swept)
        reports.append(report)
        record = report.record()
        record['scenario_file'] = str(path)
        records.append(record)
        texts.append(report_ko(report))
        if not args.quiet:
            print(texts[-1])
    print('## 판정 요약\n')
    for report in reports:
        infeasible = [f.code for f in report.findings if f.severity == 'infeasible']
        warnings = [f.code for f in report.findings if f.severity == 'warning']
        print('- %-24s %s%s%s' % (report.scenario_id, report.verdict,
                                  ' / 불가 근거: ' + ', '.join(infeasible) if infeasible else '',
                                  ' / 경고: ' + ', '.join(warnings) if warnings else ''))
    if args.json:
        Path(args.json).write_text(json.dumps({'schema': SCHEMA, 'scenarios': records},
                                              ensure_ascii=False, indent=2, sort_keys=True) + '\n')
        print('\nJSON: %s' % args.json)
    if args.markdown:
        Path(args.markdown).write_text('\n'.join(texts))
        print('Markdown: %s' % args.markdown)
    bad = [r for r in reports if not r.ok or (args.strict and r.verdict != 'feasible')]
    return 1 if bad else 0


if __name__ == '__main__':
    raise SystemExit(main())
