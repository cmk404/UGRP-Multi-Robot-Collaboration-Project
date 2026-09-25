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
    'dev-a6': 'amendment A5: skill v6, cargo_noslip_v1, per-frame look-back gate, retention/grasp judge',
    'dev-a7': 'amendment A6: skill v9 (A5 conditions); pre-freeze dev check s93 + s95',
}
if (RAW/'test').exists():
    ATTEMPTS['test'] = 'test split, once, frozen source (see frozen_source.json)'
if (RAW/'test-rerun').exists():
    ATTEMPTS['test-rerun'] = 'infrastructure reruns only (a test attempt without result.json), at most once'


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def infra_row(attempt: str, run: Path) -> dict:
    started = json.loads((run/'attempt_started.json').read_text()) if (run/'attempt_started.json').exists() else {}
    return {'attempt': attempt, 'episode': run.name, 'split': started.get('split'), 'outcome': 'INFRASTRUCTURE_FAILURE',
            'infrastructure_failure': True, 'm1_success': False, 'counts_as_m1': False, 'diagnostic_success': False,
            'false_success': False, 'code_sha': (started.get('code') or {}).get('sha'),
            'freeze': (started.get('student') or {}).get('freeze')}


def row(attempt: str, run: Path) -> dict:
    r = json.loads((run/'result.json').read_text())
    m = json.loads((run/'manifest.json').read_text())
    ev = [json.loads(l) for l in (run/'controller_events.jsonl').read_text().splitlines()]
    sk = [json.loads(l) for l in (run/'skill_events.jsonl').read_text().splitlines()]
    e = r['evaluation_only']
    cp = m.get('contact_profile')
    return {
        'attempt': attempt, 'episode': r['episode'], 'split': r['split'], 'robot_id': r['robot_id'],
        'infrastructure_failure': False,
        'code_sha': m['code']['sha'], 'code_dirty': m['code']['dirty'], 'skill': m['student'].get('skill'),
        'amendments': m['student'].get('amendments_applied', []), 'freeze': m['student'].get('freeze'),
        'contact_profile': cp['profile'] if isinstance(cp, dict) else cp,
        'noslip_iterations': cp.get('noslip_iterations') if isinstance(cp, dict) else None,
        'outcome': r['outcome'], 'exception': r.get('exception'),
        'm1_success': r['m1_success'], 'counts_as_m1': r['counts_as_m1'], 'diagnostic_success': r['diagnostic_success'],
        'false_success': r['false_success'], 'm1_failed_checks': r['m1_failed_checks'],
        'setup_diagnostic': e.get('setup_diagnostic') or {},
        'stages': {'initialized': any(x['event'] == 'initialized' for x in ev),
                   'search_found': any(x['event'] == 'skill_start' for x in ev),
                   'grasp_attached': any(x['event'] == 'grasp_attached' for x in sk),
                   'carry_leg_arrived': any(x['event'] == 'carry_leg_end' and x.get('outcome') == 'arrived' for x in ev),
                   'released': any(x['event'] == 'release_confirmed' for x in sk),
                   'look_back': any(x['event'] == 'look_back' for x in sk)},
        'retention': {k: v for k, v in (e.get('retention') or {}).items() if k != 'rule'} or None,
        'search_target_error_m': e.get('search_target_error_m'), 'gt_box_final_xyz': e['gt_box_final_xyz'],
        'contacts': e['contacts'], 'contact_steps': e.get('contact_steps_per_physics_step'),
        'max_eq_active': e['max_eq_active'], 'sim_s': r['sim_s'], 'looks': r['looks'],
        'commands': r['commands'], 'localizer_stats': r['controller'].get('localizer_stats'),
        'load_average': m['load_average'], 'wall_s': m['wall_s'], 'threads': m['env']['threads'],
        'calibration_sha256': m['calibration_sha256'], 'pose_source': m['pose_source'],
        'static_map_sha256': m['static_map_sha256'], 'scene_xml_sha256': m['scene_xml_sha256'], 'weld': m['weld'],
    }


def adopt_test(rows: list, test_ids: list) -> tuple[list, list, list]:
    """Exactly one adopted attempt per registered test episode (Codex pre-review #4).

    test/<ep> with result.json is adopted. A test/<ep> without result.json is an
    infrastructure failure; only then may test-rerun/<ep> (once) be adopted.
    Missing, duplicate or disallowed attempts are listed and block the claim.
    """
    adopted, missing, problems = [], [], []
    by = {(r['attempt'], r['episode']): r for r in rows}
    for ep in test_ids:
        first, rerun = by.get(('test', ep)), by.get(('test-rerun', ep))
        if first and not first['infrastructure_failure']:
            adopted.append(first)
            if rerun:
                problems.append(f'{ep}: rerun exists although the first attempt produced a result')
        elif first and rerun and not rerun['infrastructure_failure']:
            adopted.append({**rerun, 'label': 'infrastructure rerun (once)'})
        else:
            missing.append(ep)
    shas = {(r.get('freeze') or {}).get('frozen_source_sha') for r in adopted}
    if len(shas) > 1 or None in shas:
        problems.append(f'adopted test attempts do not share one frozen source: {sorted(map(str, shas))}')
    return adopted, missing, problems


def main() -> None:
    rows, index = [], {}
    for attempt in ATTEMPTS:
        for run in sorted(p for p in (RAW/attempt).glob('*') if p.is_dir()):
            if not (run/'result.json').exists():
                rows.append(infra_row(attempt, run))
                continue
            rows.append(row(attempt, run))
            files = sorted(p for p in run.rglob('*') if p.is_file())
            index[f'{attempt}/{run.name}'] = {
                'files': len(files), 'bytes': sum(p.stat().st_size for p in files),
                'result_sha256': sha(run/'result.json'), 'manifest_sha256': sha(run/'manifest.json'),
                'tree_sha256': hashlib.sha256(''.join(f'{p.relative_to(run)} {sha(p)}\n'
                                                      for p in files).encode()).hexdigest()}
    prereg = json.loads((HERE/'prereg.json').read_text())
    test_ids = [e['episode_id'] for e in prereg['episodes'] if e['split'] == 'test']
    adopted, missing, problems = adopt_test(rows, test_ids) if 'test' in ATTEMPTS else ([], test_ids, [])
    claim = None
    if 'test' in ATTEMPTS and not missing and not problems:
        k = sum(r['m1_success'] for r in adopted)
        claim = {'m1_success_k_of_6': k, 'false_success': sum(r['false_success'] for r in adopted),
                 'demonstrated': k >= 5 and not any(r['false_success'] for r in adopted),
                 'rule': 'test m1_success >= 5/6 AND false_success == 0 (prereg claim_rule)'}
    status = ('not run' if 'test' not in ATTEMPTS else
              'blocked: ' + '; '.join(problems + [f'missing {m}' for m in missing]) if (missing or problems) else 'complete')
    out = {'schema': 'ugrp.m1_owncam_results.v2', 'attempts': ATTEMPTS, 'test_status': status,
           'test_expected': test_ids, 'test_missing': missing, 'test_problems': problems,
           'test_adopted': [r['episode'] + '@' + r['attempt'] for r in adopted], 'claim': claim, 'runs': rows}
    (HERE/'results.json').write_text(json.dumps(out, indent=2, ensure_ascii=False) + '\n')
    (HERE/'raw_index.json').write_text(json.dumps({'root': str(RAW), 'storage': 'local only (not a remote backup)',
                                                   'runs': index}, indent=2) + '\n')
    for r in rows:
        print(r['attempt'], r['episode'], r.get('robot_id'), r['outcome'], 'm1', r['m1_success'], r.get('stages'))
    print('test', status, claim)


if __name__ == '__main__':
    sys.exit(main())
