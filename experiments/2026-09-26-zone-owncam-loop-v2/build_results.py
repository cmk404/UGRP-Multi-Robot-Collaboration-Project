"""Collect every closed-loop attempt (dev a1-a4, test) into results.json and raw_index.json.

Raw runs stay local under outputs/owncam-loop-20260925/ (not a remote backup);
raw_index.json pins each run directory by per-file SHA-256.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
RAW = Path('/Users/changmin/projects/ugrp/outputs/owncam-loop-v2-20260926')
ATTEMPTS = {
    'dev-a1': 'v2 dev (97ee2b0, frozen source): stop-lag motion, loaded camera correction + azimuth scale, loaded look hysteresis',
    'test': 'v2 test split, once, frozen source 97ee2b0 (records-only head b3edddf), NEW seeds 61-66',
}


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def row(attempt: str, run: Path) -> dict:
    r = json.loads((run/'result.json').read_text())
    m = json.loads((run/'manifest.json').read_text())
    g = r['gates']
    return {
        'attempt': attempt, 'episode': r['episode'], 'split': r['split'], 'condition': r['condition'],
        'code_sha': m['code']['sha'], 'code_dirty': m['code']['dirty'], 'outcome': r['outcome'],
        'pass': r['episode_pass'],
        'R1_gt_distance_m': g['R1_exit_waypoint']['gt_distance_m'],
        'R1_declared': g['R1_exit_waypoint']['declared_arrival'],
        'R2_wall_contacts': g['R2_no_wall_contact']['wall_contacts'],
        'R3_p90_pos_m': g['R3_estimate_near_door']['p90_pos_m'],
        'R3_p90_yaw_deg': g['R3_estimate_near_door']['p90_yaw_deg'],
        'R3_frames': g['R3_estimate_near_door']['frames'],
        'R3_uninitialized': g['R3_estimate_near_door']['uninitialized'],
        'R4_min_box_z_m': g.get('R4_box_held', {}).get('min_box_z_m'),
        'R4_end_box_z_m': g.get('R4_box_held', {}).get('end_box_z_m'),
        'gates_pass': {k: v['pass'] for k, v in g.items()},
        'student_sim_s': r['student_sim_s'], 'teacher_sim_s': r.get('teacher_sim_s'),
        'looks': r['looks'], 'look_reasons': r['look_reasons'],
        'student_commands': r['student_commands'], 'student_frames': r['student_frames'],
        'tag_visibility': r['student_tag_visibility'], 'contacts_student': r['contacts_student'],
        'load_average': m['load_average'], 'wall_s': m['wall_s'], 'threads': m['env']['threads'],
        'calibration_sha256': m['calibration_sha256'], 'static_map_sha256': m['static_map_sha256'],
        'scene_xml_sha256': m['scene_xml_sha256'], 'weld': m['weld'],
    }


def main() -> None:
    rows, index = [], {}
    for attempt in ATTEMPTS:
        base = RAW/attempt
        for run in sorted(p for p in base.glob('*') if (p/'result.json').exists()):
            rows.append(row(attempt, run))
            files = sorted(p for p in run.rglob('*') if p.is_file())
            index[f'{attempt}/{run.name}'] = {
                'files': len(files), 'bytes': sum(p.stat().st_size for p in files),
                'result_sha256': sha(run/'result.json'), 'manifest_sha256': sha(run/'manifest.json'),
                'tree_sha256': hashlib.sha256(''.join(f'{p.relative_to(run)} {sha(p)}\n'
                                                      for p in files).encode()).hexdigest(),
            }
    summary = {}
    for r in rows:
        key = f"{r['attempt']}/{r['condition']}"
        s = summary.setdefault(key, {'n': 0, 'pass': 0})
        s['n'] += 1
        s['pass'] += int(r['pass'])
    # Every pre-registered episode must have a result; missing ones are kept as
    # missing/infrastructure_failure and block the cohort gate (no silent n).
    prereg = json.loads((HERE/'prereg.json').read_text())
    expected = {'dev-a1': [e['episode_id'] for e in prereg['episodes'] if e['split'] == 'dev'],
                'test': [e['episode_id'] for e in prereg['episodes'] if e['split'] == 'test']}
    missing = [f'{a}/{e}' for a, ids in expected.items() for e in ids
               if not any(r['attempt'] == a and r['episode'] == e for r in rows)]
    test = [r for r in rows if r['attempt'] == 'test']
    gate = {'blocked': 'missing results', 'missing': missing} if missing else None
    if not missing:
        c1 = sum(r['pass'] for r in test) >= 5
        c2 = all(r['R2_wall_contacts'] == 0 and (r['R4_min_box_z_m'] or 0) > .045 for r in test)
        gate = {'C1_ge_5_of_6': c1, 'C2_no_wall_no_drop': c2, 'pass': c1 and c2,
                'passed': sum(r['pass'] for r in test)}
    out = {'schema': 'ugrp.owncam_loop_results.v2', 'attempts': ATTEMPTS, 'summary_k_of_n': summary,
           'expected_episodes': expected, 'missing': missing, 'cohort_gate': gate, 'runs': rows}
    (HERE/'results.json').write_text(json.dumps(out, indent=2, ensure_ascii=False) + '\n')
    (HERE/'raw_index.json').write_text(json.dumps({'root': str(RAW), 'storage': 'local only',
                                                   'runs': index}, indent=2) + '\n')
    for k, v in summary.items():
        print(k, f"{v['pass']}/{v['n']}")
    print('cohort gate', gate)


if __name__ == '__main__':
    main()
