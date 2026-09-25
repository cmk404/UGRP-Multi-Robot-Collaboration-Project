"""Collect every M1 attempt (dev-a1.., test) into results.json and raw_index.json.

Raw runs stay local under outputs/m1-owncam-20260926/ (not a remote backup);
raw_index.json pins each run directory by per-file SHA-256. Every registered
test episode must have a result (missing ones are listed and block the claim);
dev attempts are reported as they are, including failures and runner bugs.
"""
from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
RAW = Path('/Users/changmin/projects/ugrp/outputs/m1-owncam-20260926')
ATTEMPTS = {
    'dev-a1': '969e03e, skill v4, loop v2 calibration (before amendment A1)',
    'dev-a2': '32c7069, amendment A1: skill v5 m1 mode, fine motion + kidnap reset, post-manipulation look, pan-probe re-anchor',
    'dev-a3': 'ce33ffd, amendment A2: bay half 0.15 m, rotated-box dev diagnostics (runner robot_id bug: non-r1 robots rejected)',
    'dev-a4': 'ea45e3d, amendment A3: skill gets the episode robot id',
    'dev-a5': '6352fde, amendment A4: loaded carry leg through the door to the pre-place goal',
}
if (RAW/'test').exists():
    ATTEMPTS['test'] = 'test split, once, frozen source (see frozen_source.json)'


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def row(attempt: str, run: Path) -> dict:
    r = json.loads((run/'result.json').read_text())
    m = json.loads((run/'manifest.json').read_text())
    ev = [json.loads(l) for l in (run/'controller_events.jsonl').read_text().splitlines()]
    sk = [json.loads(l) for l in (run/'skill_events.jsonl').read_text().splitlines()]
    e = r['evaluation_only']
    return {
        'attempt': attempt, 'episode': r['episode'], 'split': r['split'], 'robot_id': r['robot_id'],
        'code_sha': m['code']['sha'], 'code_dirty': m['code']['dirty'], 'skill': m['student'].get('skill'),
        'amendments': m['student'].get('amendments_applied', []), 'outcome': r['outcome'],
        'm1_success': r['m1_success'], 'counts_as_m1': r['counts_as_m1'], 'diagnostic_success': r['diagnostic_success'],
        'false_success': r['false_success'], 'm1_failed_checks': r['m1_failed_checks'],
        'setup_diagnostic': e.get('setup_diagnostic') or {},
        'stages': {'initialized': any(x['event'] == 'initialized' for x in ev),
                   'search_found': any(x['event'] == 'skill_start' for x in ev),
                   'grasp_attached': any(x['event'] == 'grasp_attached' for x in sk),
                   'carry_leg_arrived': any(x['event'] == 'carry_leg_end' and x.get('outcome') == 'arrived' for x in ev),
                   'released': any(x['event'] == 'release_confirmed' for x in sk),
                   'look_back': any(x['event'] == 'look_back' for x in sk)},
        'search_target_error_m': e.get('search_target_error_m'), 'gt_box_final_xyz': e['gt_box_final_xyz'],
        'contacts': e['contacts'], 'max_eq_active': e['max_eq_active'], 'sim_s': r['sim_s'], 'looks': r['looks'],
        'commands': r['commands'], 'localizer_stats': r['controller'].get('localizer_stats'),
        'load_average': m['load_average'], 'wall_s': m['wall_s'], 'threads': m['env']['threads'],
        'calibration_sha256': m['calibration_sha256'], 'pose_source': m['pose_source'],
        'static_map_sha256': m['static_map_sha256'], 'scene_xml_sha256': m['scene_xml_sha256'], 'weld': m['weld'],
    }


def main() -> None:
    rows, index = [], {}
    for attempt in ATTEMPTS:
        for run in sorted(p for p in (RAW/attempt).glob('*') if (p/'result.json').exists()):
            rows.append(row(attempt, run))
            files = sorted(p for p in run.rglob('*') if p.is_file())
            index[f'{attempt}/{run.name}'] = {
                'files': len(files), 'bytes': sum(p.stat().st_size for p in files),
                'result_sha256': sha(run/'result.json'), 'manifest_sha256': sha(run/'manifest.json'),
                'tree_sha256': hashlib.sha256(''.join(f'{p.relative_to(run)} {sha(p)}\n'
                                                      for p in files).encode()).hexdigest()}
    prereg = json.loads((HERE/'prereg.json').read_text())
    test_ids = [e['episode_id'] for e in prereg['episodes'] if e['split'] == 'test']
    test = [r for r in rows if r['attempt'] == 'test']
    missing = [e for e in test_ids if not any(r['episode'] == e for r in test)] if 'test' in ATTEMPTS else test_ids
    claim = None
    if 'test' in ATTEMPTS and not missing:
        k = sum(r['m1_success'] for r in test)
        claim = {'m1_success_k_of_6': k, 'false_success': sum(r['false_success'] for r in test),
                 'demonstrated': k >= 5 and not any(r['false_success'] for r in test)}
    out = {'schema': 'ugrp.m1_owncam_results.v1', 'attempts': ATTEMPTS,
           'test_status': 'not run' if 'test' not in ATTEMPTS else ('missing results' if missing else 'complete'),
           'test_expected': test_ids, 'test_missing': missing, 'claim': claim, 'runs': rows}
    (HERE/'results.json').write_text(json.dumps(out, indent=2, ensure_ascii=False) + '\n')
    (HERE/'raw_index.json').write_text(json.dumps({'root': str(RAW), 'storage': 'local only (not a remote backup)',
                                                   'runs': index}, indent=2) + '\n')
    for r in rows:
        print(r['attempt'], r['episode'], r['robot_id'], r['outcome'], 'm1', r['m1_success'], r['stages'])
    print('test', out['test_status'], claim)


if __name__ == '__main__':
    sys.exit(main())
