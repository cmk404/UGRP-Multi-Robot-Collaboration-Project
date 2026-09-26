"""Collect every closed-loop attempt (dev a1-a4, test) into results.json and raw_index.json.

Raw runs stay local under outputs/owncam-loop-20260925/ (not a remote backup);
raw_index.json pins each run directory by per-file SHA-256.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
RAW = Path('/Users/changmin/projects/ugrp/outputs/owncam-loop-20260925')
ATTEMPTS = {
    'dev-a1': 'dev attempt 1 (155cb71): v1 calibration',
    'dev-a2': 'dev attempt 2 (a5559e7): + loaded/unloaded elevation bias',
    'dev-a3': 'dev attempt 3 (d3466db): + wide look sweep',
    'dev-a4': 'dev attempt 4 (9361a8d, frozen): + motion refit, loaded travel-based looks; reported dev',
    'test': 'test split, once, frozen source 9361a8d (records-only head 521ae7b)',
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
    out = {'schema': 'ugrp.owncam_loop_results.v1', 'attempts': ATTEMPTS, 'summary_k_of_n': summary,
           'runs': rows}
    (HERE/'results.json').write_text(json.dumps(out, indent=2, ensure_ascii=False) + '\n')
    (HERE/'raw_index.json').write_text(json.dumps({'root': str(RAW), 'storage': 'local only',
                                                   'runs': index}, indent=2) + '\n')
    for k, v in summary.items():
        print(k, f"{v['pass']}/{v['n']}")


if __name__ == '__main__':
    main()
