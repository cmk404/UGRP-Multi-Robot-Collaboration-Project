#!/usr/bin/env python3
"""Offline survey: realtime lease stop gaps vs paired fine RGB alignment time.

Reads existing local run records only (pair-decisions.json, issued-commands.json);
no simulation, model call or ground-truth control input. SIM times and image
errors are post-hoc measurements of what the controller already logged.

A "stop gap" is a moving->moving transition of the same pair robot whose next
command is issued after the previous lease expired. CameraRobotPort.tick() then
holds the robot (motor command 0), which switches MasterPi dynamics from moving
damping (1.4 N/(m/s)) to stop damping (18 N/(m/s)) and discards momentum.
"""
import glob, hashlib, json, os, statistics, sys
from pathlib import Path

ROOT = Path('/Users/changmin/projects/ugrp/outputs')
EPS = 1e-9


def mag(action):
    return max(abs(action.get(k, 0) or 0) for k in ('forward', 'left', 'turn'))


def lease_end(command):
    return command.get('valid_until_s', command['issued_at_s'] + command['action'].get('duration_s', 0))


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def spearman(a, b):
    def rank(xs):
        order = sorted(range(len(xs)), key=xs.__getitem__)
        out = [0] * len(xs)
        for k, i in enumerate(order):
            out[i] = k
        return out
    ra, rb = rank(a), rank(b)
    ma, mb = statistics.mean(ra), statistics.mean(rb)
    num = sum((x - ma) * (y - mb) for x, y in zip(ra, rb))
    return num / (sum((x - ma) ** 2 for x in ra) * sum((y - mb) ** 2 for y in rb)) ** .5


def survey_run(run):
    decisions_path, commands_path = run / 'pair-decisions.json', run / 'issued-commands.json'
    try:
        decisions = json.loads(decisions_path.read_text())
        issued = json.loads(commands_path.read_text())
    except (OSError, ValueError):
        return None
    learned = [r for r in decisions if isinstance(r, dict) and r.get('kind') == 'learned_approach']
    if not learned or not learned[0]['report'].get('approach_ok') or 'r3' not in issued:
        return None
    report = learned[0]['report']
    fine_calls = len([c for c in report['approach_calls'] if c['robot_id'] == 'r3'])
    fine = [c for c in issued['r3'] if c.get('stage') == 'APPROACH'][-fine_calls:]
    if len(fine) < 10:
        return None
    realtime = any('valid_until_s' in c for c in fine)
    moving = [(a, b) for a, b in zip(fine, fine[1:]) if mag(a['action']) > EPS and mag(b['action']) > EPS]
    gaps = [b['issued_at_s'] - lease_end(a) for a, b in moving]
    gapped = [g for g in gaps if g > 1e-6]
    per_unit = []
    for rid in ('r1', 'r3'):
        calls = [c for c in report['approach_calls'] if c['phase_index'] == 3 and c['robot_id'] == rid]
        errors = [c['decision'].get('diagnostics', {}).get('image_derived_error') for c in calls]
        commanded = sum(c['action']['forward'] for c in calls if c['action']['forward'] > 0)
        if errors and errors[0] is not None and errors[-1] is not None and commanded > 0:
            per_unit.append((errors[0] - errors[-1]) / commanded)
    return {'run': str(run.relative_to(ROOT)), 'mode': 'realtime' if realtime else 'synchronous',
            'fine_alignment_sim_s': round(report.get('approach_elapsed_sim_s', 0.), 3),
            'movement_slices': sum(s.get('movement_slices', 0) for s in report['stage_results']),
            'r3_moving_transitions': len(moving), 'r3_stop_gap_transitions': len(gapped),
            'r3_stop_gap_fraction': round(len(gapped) / len(moving), 4) if moving else None,
            'r3_stop_gap_sum_sim_s': round(sum(gapped), 3),
            'forward_image_error_per_unit_command': round(statistics.mean(per_unit), 4) if per_unit else None,
            'pair_decisions_sha256': sha(decisions_path), 'issued_commands_sha256': sha(commands_path)}


def main(out):
    runs = sorted({Path(p).parent for p in glob.glob(str(ROOT / '**/pair-decisions.json'), recursive=True)})
    rows = [row for row in map(survey_run, runs) if row]
    summary = {}
    for mode in ('realtime', 'synchronous'):
        sel = [r for r in rows if r['mode'] == mode]
        summary[mode] = {
            'runs': len(sel),
            'fine_alignment_sim_s_median': statistics.median(r['fine_alignment_sim_s'] for r in sel),
            'fine_alignment_sim_s_range': [min(r['fine_alignment_sim_s'] for r in sel), max(r['fine_alignment_sim_s'] for r in sel)],
            'stop_gap_fraction_range': [min(r['r3_stop_gap_fraction'] for r in sel), max(r['r3_stop_gap_fraction'] for r in sel)],
            'forward_error_per_unit_command_median': statistics.median(r['forward_image_error_per_unit_command'] for r in sel)}
    rt = [r for r in rows if r['mode'] == 'realtime']
    gap = [r['r3_stop_gap_fraction'] for r in rt]
    fine = [r['fine_alignment_sim_s'] for r in rt]
    unit = [r['forward_image_error_per_unit_command'] for r in rt]
    mx, my = statistics.mean(gap), statistics.mean(fine)
    slope = sum((x - mx) * (y - my) for x, y in zip(gap, fine)) / sum((x - mx) ** 2 for x in gap)
    summary['realtime_correlation'] = {
        'spearman_stop_gap_fraction_vs_error_per_unit_command': round(spearman(gap, unit), 3),
        'spearman_stop_gap_fraction_vs_fine_sim_s': round(spearman(gap, fine), 3),
        'linear_fit_fine_sim_s': {'intercept_at_zero_gap': round(my - slope * mx, 2), 'slope_per_gap_fraction': round(slope, 2)},
        'note': 'Observational across heterogeneous source versions, maps and host load; not a controlled A/B.'}
    result = {'schema': 'ugrp.realtime_stop_gap_survey.v1', 'root': str(ROOT), 'summary': summary, 'runs': rows}
    Path(out).write_text(json.dumps(result, indent=1) + '\n')
    print(json.dumps(summary, indent=1))


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else Path(__file__).with_name('survey.json'))
