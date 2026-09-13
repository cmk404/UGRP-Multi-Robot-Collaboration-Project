"""Post-run motion metrics. Referee pose/yaw never feed the actor."""
from __future__ import annotations

from collections import Counter
import argparse
import json
from pathlib import Path

import numpy as np

from scripts.audit_known_map_navigation import audit_run
from scripts.report_known_map_cohort import localization_errors


def motion_metrics(root):
    actor = [json.loads(line)['decision'] for line in (root/'actor-decisions.jsonl').read_text().splitlines()]
    referee = [json.loads(line) for line in (root/'evaluation-only.jsonl').read_text().splitlines()]
    xyz = np.asarray([r['robot_xyz_m'] for r in referee])
    yaw = np.unwrap([r['robot_rpy_rad'][2] for r in referee])
    times = np.asarray([r['sim_time_s'] for r in referee])
    delta = np.diff(xyz[:, :2], axis=0)
    distance = np.linalg.norm(delta, axis=1)
    angle = np.arctan2(delta[:, 1], delta[:, 0])
    midpoint_yaw = (yaw[:-1] + yaw[1:]) / 2
    error = np.abs((angle - midpoint_yaw + np.pi) % (2*np.pi) - np.pi)
    # Exclude near-zero displacement where a direction is undefined. Include
    # sideways and reverse movements in both conditions, not just forward commands.
    moving = distance >= .0002
    commands = [d['action'] for d in actor]
    counts = Counter(d['status'] for d in actor)
    action_ends = times[0] + np.cumsum([a['duration_s'] for a in commands])
    frame_times = np.r_[times[0], action_ends[:-1]]
    heading_errors = []
    for frame_time, decision in zip(frame_times, actor):
        estimate = decision['diagnostics'].get('heading', {}).get('estimate_rad')
        if estimate is not None:
            truth = np.interp(frame_time, times, yaw)
            heading_errors.append(float(np.degrees(abs((estimate-truth+np.pi) % (2*np.pi)-np.pi))))
    indices = np.minimum(np.searchsorted(action_ends, (times[:-1]+times[1:])/2), len(commands)-1)
    forward = np.array([commands[i]['forward'] > 0 and commands[i]['turn'] == 0 for i in indices]) & moving
    def angular_summary(mask):
        length = float(distance[mask].sum())
        return {'distance_m': length,
                'distance_weighted_heading_error_deg': float(np.degrees(np.average(error[mask], weights=distance[mask]))) if length else None,
                'distance_fraction_within_20deg': float(distance[mask & (error <= np.deg2rad(20))].sum()/length) if length else None}
    return {'status_counts': dict(counts), 'lateral_commands': sum(a['left'] != 0 for a in commands),
            'reverse_commands': sum(a['forward'] < 0 for a in commands),
            'turn_commands': sum(a['turn'] != 0 for a in commands),
            'forward_commands': sum(a['forward'] > 0 for a in commands),
            'stop_commands': sum(a['forward'] == a['left'] == a['turn'] == 0 for a in commands),
            'absolute_yaw_travel_deg': float(np.degrees(np.abs(np.diff(yaw)).sum())),
            'heading_estimate_error_deg': {'frames': len(heading_errors),
                'mean': float(np.mean(heading_errors)) if heading_errors else None,
                'max': max(heading_errors) if heading_errors else None},
            'all_moving_samples': angular_summary(moving), 'forward_only_commands': angular_summary(forward),
            'reference': 'output-only ~0.1 s pose samples; >=0.2 mm translation; time interpolation only after run'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('cohort', type=Path)
    args = parser.parse_args()
    summaries = []
    for result in json.loads((args.cohort/'results.json').read_text()):
        root = args.cohort/result['name']
        try:
            audit = audit_run(root)
            entry = {k:v for k,v in audit.items() if k != 'replay'}
            entry.update(motion=motion_metrics(root), localization_error=localization_errors(root))
        except Exception as exc:
            entry = {'audit_pass': False, 'error': str(exc)}
        entry.update(name=result['name'])
        summaries.append(entry)
    (args.cohort/'audits.json').write_text(json.dumps(summaries, indent=2)+'\n')
    print(json.dumps(summaries, indent=2))


if __name__ == '__main__':
    main()
