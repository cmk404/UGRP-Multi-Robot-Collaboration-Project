#!/usr/bin/env python3
"""no-LLM offline smoke of the Korean-dialogue zone study (stored RGB + fake clock).

Runs every scenario x (four main conditions + the reference ceiling R) with one
seed each, entirely offline: no model call, no physics, no simulator import. The
loop is ``harness.zone_study_offline``; this script only drives it, writes the
result bundle and prints the checks.

    PYTHONPATH=. python3 scripts/run_zone_study_offline_smoke.py \
        --output outputs/zone-study-offline-smoke

What a pass means: the protocol, the input boundary, the SIM cost model, the log
schema and the evaluation are WIRED correctly. It says nothing about language
understanding, about any communication effect, or about physical transport.
"""
from __future__ import annotations

import argparse
import json
import os
import platform
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from harness import zone_study_eval as ev                                    # noqa: E402
from harness import zone_study_offline as off                                # noqa: E402
from harness.zone_study_contract import MAIN_CONDITIONS, digest             # noqa: E402
from harness.zone_study_scenarios import scenario_ids                         # noqa: E402

CONDITIONS = tuple(MAIN_CONDITIONS) + ('reference_R',)


def code_sha() -> str:
    try:
        return subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=ROOT, capture_output=True,
                              text=True, check=True).stdout.strip()
    except Exception:                                    # noqa: BLE001 - provenance, not control flow
        return 'unknown'


def environment() -> dict:
    load = os.getloadavg()
    return {'python': sys.version.split()[0], 'platform': platform.platform(),
            'machine': platform.machine(), 'cpu_count': os.cpu_count(),
            'loadavg_1_5_15': [round(v, 2) for v in load],
            'threads': {name: os.environ.get(name) for name in
                        ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'VECLIB_MAXIMUM_THREADS',
                         'MKL_NUM_THREADS')}}


def summarise(bundle) -> dict:
    """Package I cohort summary over the offline trial records."""
    trials = [ev.parse_trial(record) for record in bundle['trial_records']]
    summary = ev.summarise(trials)
    return {'conditions': {name: {k: row[k] for k in ('label_ko', 'is_reference', 'trials',
                                                      'successes', 'success_rate', 'end_reasons',
                                                      'censored_trials', 'seeds', 'leader_ids',
                                                      'metrics', 'dialogue',
                                                      'boundary_clean_trials',
                                                      'boundary_violation_trials',
                                                      'usage_unknown_calls',
                                                      'tokens_incomplete_trials',
                                                      'cohort_tokens_total',
                                                      'cohort_tokens_total_lower_bound')}
                          for name, row in summary['conditions'].items()},
            'scenarios': summary['scenarios'], 'trials': summary['trials'],
            'provenance': summary['provenance']}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--output', required=True, help='directory for the result bundle')
    parser.add_argument('--scenarios', nargs='*', default=list(scenario_ids()))
    parser.add_argument('--conditions', nargs='*', default=list(CONDITIONS))
    parser.add_argument('--horizon-s', type=float, default=off.DEFAULT_HORIZON_S)
    parser.add_argument('--no-probe', action='store_true',
                        help='skip the private-section backflow probe (halves the run)')
    args = parser.parse_args(argv)

    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=True)
    bundle = off.run_smoke(args.scenarios, args.conditions, code_sha=code_sha(),
                           horizon_s=args.horizon_s, probe=not args.no_probe)
    summary = summarise(bundle)
    env = environment()

    records = out / 'trial_records'
    records.mkdir(exist_ok=True)
    for record in bundle['trial_records']:
        (records / f'{record["trial_id"]}.json').write_text(
            json.dumps(record, ensure_ascii=False, indent=2, sort_keys=True) + '\n')
    # third review, finding 1: every SAVED record is read back from disk and
    # each archived final request re-hashed; the run fails if any does not.
    reopened = [off.reopen_trial_record(records / f'{record["trial_id"]}.json')
                for record in bundle['trial_records']]
    disk = {'ok': all(r['ok'] for r in reopened), 'records': len(reopened),
            'calls': sum(r['calls'] for r in reopened),
            'rehashed_requests': sum(r['rehashed'] for r in reopened),
            'problems': [p for r in reopened for p in r['problems']][:50],
            'files': {Path(r['path']).name: r['file_sha256'] for r in reopened}}
    bundle['disk_rehash'] = {k: v for k, v in disk.items() if k != 'files'}
    bundle['ok'] = bool(bundle['ok'] and disk['ok'])
    trials = {k: v for k, v in bundle.items() if k != 'trial_records'}
    (out / 'smoke.json').write_text(json.dumps(trials, ensure_ascii=False, indent=2,
                                               sort_keys=True) + '\n')
    (out / 'cohort_summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2,
                                                        sort_keys=True) + '\n')
    (out / 'environment.json').write_text(json.dumps(env, ensure_ascii=False, indent=2,
                                                     sort_keys=True) + '\n')
    hashes = {p.relative_to(out).as_posix(): digest_file(p)
              for p in sorted(out.rglob('*.json')) if p.name != 'sha256.json'}
    (out / 'sha256.json').write_text(json.dumps(hashes, ensure_ascii=False, indent=2,
                                                sort_keys=True) + '\n')

    print(f'code_sha={code_sha()}  loadavg={env["loadavg_1_5_15"]}  threads={env["threads"]}')
    print(f'trials={len(bundle["trials"])}  probes={len(bundle["backflow_probes"])}  '
          f'ok={bundle["ok"]}')
    print(f'actor_isolation={bundle["actor_isolation"]}')
    print(f'disk_rehash: ok={disk["ok"]} records={disk["records"]} calls={disk["calls"]} '
          f'rehashed_requests={disk["rehashed_requests"]}')
    for problem in disk['problems']:
        print(f'    ! {problem}')
    header = f'{"run_id":34} {"calls":>5} {"msgs":>4} {"sent":>4} {"free":>4} {"f2f":>3} ' \
             f'{"lead":>5} {"think_s":>8} {"talk_s":>7} chan cost'
    print(header)
    for row in bundle['trials']:
        channel, cost = row['channel'], row['cost']
        print(f'{row["run_id"]:34} {row["calls"]:5d} {row["messages"]:4d} {channel["sent"]:4d} '
              f'{channel["free_text_messages"]:4d} {channel["follower_to_follower"]:3d} '
              f'{str(row["leader_id"] or "-"):>5} {cost["charged_sim_s"]:8.1f} '
              f'{cost["talk_sim_s"]:7.1f} {"ok" if channel["ok"] else "FAIL":>4} '
              f'{"ok" if cost["ok"] else "FAIL":>4}')
        for problem in channel['problems'] + cost['problems']:
            print(f'    ! {problem}')
    bad = [p for p in bundle['backflow_probes'] if not p['ok']]
    print(f'backflow: {len(bundle["backflow_probes"]) - len(bad)}/{len(bundle["backflow_probes"])} '
          'identical SIM trace with the private section changed')
    for problem in bad:
        print(f'    ! {problem["condition"]} {problem["scenario"]}: {problem["problems"]}')
    print(f'wrote {out}/smoke.json, cohort_summary.json, environment.json, sha256.json, '
          f'trial_records/ ({len(bundle["trial_records"])} files)')
    print('범위: 프로토콜·비용·로그·평가 배선만 검증했다. 언어 이해, 통신 효과, '
          '물리 운반 성공은 입증하지 않는다.')
    return 0 if bundle['ok'] else 1


def digest_file(path: Path) -> str:
    import hashlib

    return hashlib.sha256(path.read_bytes()).hexdigest()


if __name__ == '__main__':
    raise SystemExit(main())
