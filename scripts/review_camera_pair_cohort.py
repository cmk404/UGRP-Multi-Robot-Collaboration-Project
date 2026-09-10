"""Summarize bounded camera-policy comparisons without feeding evaluation to actors."""
from __future__ import annotations
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import statistics


def summarize(root: Path) -> dict:
    report = json.loads((root / 'result.json').read_text())
    calls = report.get('calls', [])
    usage = [row.get('usage') or {} for row in calls]
    latencies = [row['model_latency_ms'] for row in calls if row.get('model_latency_ms') is not None]
    robots = {}
    for rid in ('r1', 'r3'):
        own_calls = [row for row in calls if row['robot_id'] == rid]
        keys = [json.dumps(row['action'], sort_keys=True) for row in own_calls]
        longest, streak, previous = 0, 0, None
        for key in keys:
            streak = streak + 1 if key == previous else 1
            longest, previous = max(longest, streak), key
        counts = Counter()
        for row in own_calls:
            a = row['action']
            counts[a['kind']] += 1
            if a['kind'] == 'arm':
                counts[f"servo_{a['servo_id']}"] += 1
                if a['servo_id'] == 1:
                    counts['open' if a['pulse'] == 2000 else 'close' if a['pulse'] == 1500 else 'other_grip'] += 1
        frames = sorted((root / rid).glob('*-own.jpg'))
        hashes = [hashlib.sha256(p.read_bytes()).hexdigest() for p in frames]
        robots[rid] = {'actions': dict(counts), 'longest_identical_command_streak': longest,
                       'unique_commands': len(set(keys)),
                       'identical_consecutive_own_frames': sum(a == b for a, b in zip(hashes, hashes[1:])),
                       'aba_command_patterns': sum(keys[i] == keys[i-2] and keys[i] != keys[i-1]
                                                   for i in range(2, len(keys))),
                       'last_learning': own_calls[-1].get('learning') if own_calls else None}
    return {'run': str(root), 'sha': report['git_sha'], 'mode': report['config'].get('mode', 'baseline'),
            'seed': report['config']['seed'], 'calls': len(calls), 'error': report.get('error'),
            'rounds': report['rounds_completed'], 'grasp_success': report.get('grasp_success'),
            'max_lift_m': report.get('max_lift_m'),
            'hold_s': report.get('longest_qualifying_duration_s'),
            'input_tokens': sum(item.get('prompt_tokens', 0) for item in usage),
            'output_tokens': sum(item.get('completion_tokens', 0) for item in usage),
            'usage_records': sum(bool(item) for item in usage),
            'median_model_latency_ms': statistics.median(latencies) if latencies else None,
            'wall_elapsed_s': report.get('wall_elapsed_s'), 'robots': robots}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('runs', type=Path, nargs='+')
    args = parser.parse_args()
    rows = [summarize(p.resolve()) for p in args.runs]
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / 'comparison.json').write_text(json.dumps(rows, indent=2) + '\n')
    lines = ['# Camera-policy comparison', '',
             '| Mode / seed | Calls | Grasp | Lift m | Hold s | Input tokens | Median call ms | Open commands | Longest repeat r1/r3 |',
             '|---|---:|---|---:|---:|---:|---:|---:|---|']
    for r in rows:
        robots = r['robots']
        opens = sum(v['actions'].get('open', 0) for v in robots.values())
        repeats = '/'.join(str(robots[rid]['longest_identical_command_streak']) for rid in ('r1', 'r3'))
        latency = r['median_model_latency_ms']
        lines.append(f"| {r['mode']} / {r['seed']} | {r['calls']} | {r['grasp_success']} | {r['max_lift_m']} | "
                     f"{r['hold_s']} | {r['input_tokens']} | {round(latency) if latency is not None else 'unknown'} | {opens} | {repeats} |")
    lines += ['', 'Exploratory bounded cohort: no statistical significance claim. Actual billing is unknown.',
              'Identical frames and repeated commands are symptoms, not causal proof. Physics pauses during inference;',
              'SIM success and call latency do not establish real-time hardware performance. Raw artifacts remain local.']
    (args.out / 'comparison.md').write_text('\n'.join(lines) + '\n')
    print('\n'.join(lines))


if __name__ == '__main__':
    main()
