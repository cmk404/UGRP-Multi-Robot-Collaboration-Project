"""Aggregate the wrist-view probes and the skill-isolation cohort into results.json.

Reads only local raw outputs (primary checkout outputs/, gitignored) and records
their paths and SHA-256. Usage:
  python experiments/2026-09-25-zone-owncam-skill/build_results.py [--allow-incomplete]

Every cohort is read against its FULL pre-registered seed list (Codex review
issue 8): a seed without result.json is recorded as
``missing/infrastructure_failure`` and blocks the report (results.json is not
written) unless --allow-incomplete, which writes it with
``cohort_report_blocked: true``.
"""
from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
RAW = Path('/Users/changmin/projects/ugrp/outputs/zone-owncam-skill-20260925')
PROBES = {'probe1_rest_drive_door': 'view-probe-4219506', 'probe2_door_first': 'view-probe-a868328-doorfirst'}
COHORT = 'cohort-d016c04'
TEST_SEEDS = (501, 502, 503, 504, 505)
DEV_RUNS = ('dev/401-a', 'dev/401-b', 'dev/402-a868328', 'dev/402-d016c04', 'dev/401-d016c04')
V2_COHORT = 'cohort-v2-edd075d'
V2_TEST_SEEDS = tuple(range(511, 521))
V2_DEV_RUNS = ('dev-v2/403-a', 'dev-v2/404-a', 'dev-v2/404-b', 'dev-v2/405-b', 'dev-v2/405-c', 'dev-v2/406-c')
GRIP_HOLD = ('grip-hold/local_contact_fine', 'grip-hold/cargo_noslip_v1')
V3_TEST_SEEDS = tuple(range(521, 531))
V3_ARMS = {'P': 'cargo_noslip_v1', 'S': 'local_contact_fine'}   # cohort-v3-<sha>/<arm>/<seed>
V4_TEST_SEEDS = tuple(range(531, 541))
V4_ZONE_C_SEEDS = (531, 532, 533, 534, 538, 540)
V4_ARMS = {'D': 'cargo_noslip_v1 + drop injection at carry+15 s', 'P': 'cargo_noslip_v1'}
V4_ARM_SEEDS = {'D': (531, 533), 'P': V4_TEST_SEEDS}
V3_ARM_SEEDS = {'P': V3_TEST_SEEDS, 'S': V3_TEST_SEEDS}
V5_TEST_SEEDS = tuple(range(541, 549))
V5_ARMS = {'P': 'cargo_noslip_v1 (pending user decision), diagnostic mode (gt_stub pose), coarse bay'}
V6_TEST_SEEDS = tuple(range(551, 561))
V6_ARMS = {'P': 'cargo_noslip_v1 (pending user decision), diagnostic mode (gt_stub pose), coarse bay, '
                'top-edge face yaw, static peer keep-outs'}
MISSING = 'missing/infrastructure_failure'


class CohortIncomplete(RuntimeError):
    """A pre-registered seed has no result; the cohort report is blocked."""


def collect_preregistered(load, seeds):
    """Load every pre-registered seed; a missing one is recorded, never silently dropped."""
    runs, missing = [], []
    for seed in seeds:
        run = load(seed)
        if run is None:
            missing.append({'seed': seed, 'status': MISSING})
        else:
            runs.append(run)
    return runs, missing


def require_complete(name, missing):
    if missing:
        raise CohortIncomplete(f'{name}: pre-registered seeds {[m["seed"] for m in missing]} are {MISSING}; '
                               'the cohort report is blocked')


def _loader(folder, summarise):
    def load(seed):
        path = folder / str(seed)
        return summarise(path) if (path / 'result.json').exists() else None
    return load


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def probe_summary(folder):
    p = json.loads((folder / 'probe.json').read_text())
    tags = {t['id']: t for t in p['test_tags']}
    rest = [{'posture': s['posture'], 'pan_pwm': s['pan_pwm'], 'held': s['held'],
             'box_offset_in_gripper_m': s['box_offset_in_gripper_m'],
             'held_box_pixel_fraction': s['held_box_pixel_fraction'],
             'camera_height_m': s['view']['camera_height_m'], 'optical_axis_pitch_deg': s['view']['optical_axis_pitch_deg'],
             'unoccluded_fraction_of_valid': s['view']['unoccluded_fraction_of_valid'],
             'elevation_deg': s['view']['elevation_deg'], 'azimuth_deg': s['view']['azimuth_deg'],
             'floor_centre_line_x_m': s['view']['floor'].get('centre_line_x_m'),
             'wall_z_range_centre_m': {k: w['z_range_centre_m'] for k, w in s['view']['wall_plane'].items()}}
            for s in p['stages'] if s['stage'] == 'rest']
    drive = [{k: s[k] for k in ('posture', 'held', 'box_offset_in_gripper_m', 'max_box_offset_during_drive_m')}
             for s in p['stages'] if s['stage'] == 'drive']
    door = []
    for s in p['stages']:
        if s['stage'] != 'door':
            continue
        door.append({'lens_distance_m': s['lens_distance_m'], 'posture': s['posture'], 'pan_pwm': s['pan_pwm'],
                     'held': s['held'], 'box_offset_in_gripper_m': s['box_offset_in_gripper_m'],
                     'held_box_pixel_fraction': s.get('held_box_pixel_fraction'),
                     'detected': [{'id': d['id'], 'lateral_m': tags[d['id']]['lateral_from_door_m'],
                                   'height_m': tags[d['id']]['centre_height_m'], 'size_m': tags[d['id']]['size_m'],
                                   'side_px': d['side_px']} for d in s['detected']]})
    zone = [{k: s[k] for k in ('posture', 'held', 'zone_c_purple_pixel_fraction')}
            for s in p['stages'] if s['stage'] == 'zone_c']
    files = sorted(folder.rglob('*'))
    return {'raw_dir': str(folder), 'probe_json_sha256': sha(folder / 'probe.json'),
            'source_sha': p['source_sha'], 'dirty_source': p['dirty_source'], 'order': p.get('order', 'rest-drive-door'),
            'scene_xml_sha256': p['scene_xml_sha256'], 'grasp': p['grasp'], 'final': p['final'],
            'sim_seconds': p['sim_seconds'], 'wall_seconds': p['wall_seconds'],
            'load_average_start': p['load_average_start'], 'load_average_end': p['load_average_end'],
            'frame_count': sum(1 for f in files if f.suffix == '.jpg'),
            'rest': rest, 'drive': drive, 'door': door, 'zone_c': zone}


def run_summary(folder):
    r = json.loads((folder / 'result.json').read_text())
    return {'raw_dir': str(folder), 'seed': r['seed'], 'development_seed': r.get('development_seed'),
            'source_sha': r['source_sha'], 'dirty_source': r['dirty_source'], 'reason': r['reason'],
            'pose_source': r['pose_source'], 'counts_as_m1': r['counts_as_m1'],
            'sim_seconds': r['sim_seconds'], 'steps': r['steps'], 'phase_start_sim_s': r['phase_start_sim_s'],
            'face_normal_source': next((e.get('face_normal_source') for e in r['events']
                                        if e['event'] == 'grasp_attached'), None),
            'face_nudges': next((e.get('face_nudges') for e in r['events'] if e['event'] == 'grasp_attached'), None),
            'placement_own_rgb': r['placement_own_rgb'], 'evaluation_only': r['evaluation_only'],
            'load_average_start': r['load_average_start'], 'load_average_end': r['load_average_end'],
            'wall_seconds': r['wall_seconds'],
            'artifact_sha256': json.loads((folder / 'hashes.json').read_text()),
            'input_frames': len(list((folder / 'inputs').glob('*.jpg'))),
            'profile': r.get('profile'), 'sim_limit_s': r.get('sim_limit_s'), 'skill_summary': r.get('skill_summary'),
            'contact_profile_selected': r.get('contact_profile_selected'),
            'cargo_contact_profile': r.get('cargo_contact_profile'),
            'solver_noslip_iterations': r.get('solver_noslip_iterations'),
            'fault_injection': r.get('fault_injection'), 'counts_for': r.get('counts_for')}


def grip_hold_summary(folder):
    r = json.loads((folder / 'result.json').read_text())
    return {'raw_dir': str(folder), 'result_sha256': sha(folder / 'result.json'),
            'samples_sha256': sha(folder / 'samples.jsonl'), **r}


def cohort_stats(runs):
    ev = [r['evaluation_only'] for r in runs]
    return {'n': len(runs), 'pose_source': 'gt_stub_eval_only', 'counts_as_m1': False,
            'place_in_slot_gt': sum(e['place_in_slot_gt'] for e in ev),
            'grasp_success_gt': sum(e['grasp_success_gt'] for e in ev),
            'skill_claim_agrees_with_gt': sum(e['skill_claim_agrees_with_gt'] for e in ev),
            'false_success': sum(e['skill_claim_in_slot'] and not e['place_in_slot_gt'] for e in ev),
            'weld_eq_active_max': max((e['weld_eq_active_max'] for e in ev), default=None),
            'wall_contact_steps': sum(e['r1_wall_contact_steps'] for e in ev),
            'sim_seconds': [r['sim_seconds'] for r in runs], 'steps': [r['steps'] for r in runs],
            'reasons': [r['reason'] for r in runs],
            'reseats': [(r['skill_summary'] or {}).get('reseats') for r in runs],
            'retreats': [(r['skill_summary'] or {}).get('retreats') for r in runs],
            'grip_checks': [[c.get('result') for c in (r['skill_summary'] or {}).get('grip_checks', [])] for r in runs],
            'placement_detector': [(r['skill_summary'] or {}).get('placement_detector') for r in runs],
            'contact_profiles': sorted({r.get('contact_profile_selected') or 'local_contact_fine' for r in runs}),
            'g4_violations': [r['seed'] for r in runs if (r['evaluation_only']['place_in_slot_gt'] and r['reason'].endswith('NO_UNIQUE_CYAN_BOX'))
                              or r['reason'] == 'CARRY_TOP_GEOMETRY_AMBIGUOUS_FOR_DROP']}


def run_summary_v5(folder):
    r = json.loads((folder / 'result.json').read_text())
    keep = ('seed', 'development_seed', 'runner', 'profile', 'mode', 'counts_as_m1', 'm1_success', 'diagnostic_success',
            'success', 'pose_source', 'pose_sources_seen', 'cameras_seen', 'input_contract', 'order',
            'scenario_setup_only', 'source_sha_start', 'dirty_source_start', 'source_sha_end',
            'source_changed_during_run', 'dependency_sha256_start', 'reason', 'steps', 'sim_seconds',
            'phase_start_sim_s', 'placement_own_rgb', 'evaluation_only', 'skill_summary', 'contact_profile_selected',
            'contact_profile_status', 'cargo_contact_profile', 'solver_noslip_iterations', 'wall_seconds',
            'load_average_start', 'load_average_end')
    return {'raw_dir': str(folder), **{k: r.get(k) for k in keep},
            'artifact_sha256': json.loads((folder / 'hashes.json').read_text()),
            'input_frames': len(list((folder / 'inputs').glob('*.jpg')))}


def cohort_stats_v5(runs):
    ev = [r['evaluation_only'] for r in runs]
    return {'n': len(runs), 'mode': sorted({r['mode'] for r in runs}),
            'counts_as_m1': sum(bool(r['counts_as_m1']) for r in runs),
            'm1_success': sum(bool(r['m1_success']) for r in runs),
            'diagnostic_success': sum(bool(r['diagnostic_success']) for r in runs),
            'place_in_slot_gt': sum(e['place_in_slot_gt'] for e in ev),
            'grasp_success_gt': sum(e['grasp_success_gt'] for e in ev),
            'false_success': sum(e['skill_claim_in_slot'] and not e['place_in_slot_gt'] for e in ev),
            'skill_claim_agrees_with_gt': sum(e['skill_claim_agrees_with_gt'] for e in ev),
            'face_normal_sources': [e['face_normal_sources'] for e in ev],
            'face_relooks': [(r['skill_summary'] or {}).get('face_relooks') for r in runs],
            'rejected_observations': sum(len((r['skill_summary'] or {}).get('rejected_observations') or []) for r in runs),
            'weld_eq_active_max': max((e['weld_eq_active_max'] for e in ev), default=None),
            'cargo_wall_contact_steps': [e['contacts_per_physics_step']['cargo_wall']['steps'] for e in ev],
            'r1_wall_contact_steps': [e['contacts_per_physics_step']['r1_wall']['steps'] for e in ev],
            'max_wall_normal_force_n': max((max(e['contacts_per_physics_step']['cargo_wall']['max_normal_force_n'],
                                                e['contacts_per_physics_step']['r1_wall']['max_normal_force_n'])
                                            for e in ev), default=None),
            'source_changed_during_run': [r['seed'] for r in runs if r['source_changed_during_run']],
            'source_sha_start': sorted({r['source_sha_start'] for r in runs}),
            'sim_seconds': [r['sim_seconds'] for r in runs], 'steps': [r['steps'] for r in runs],
            'reasons': [r['reason'] for r in runs]}


def face_error_eval(folder):
    """Evaluation only: |own-RGB face yaw - true relative yaw| (mod 90) on every face-ready approach frame."""
    import math
    folder = Path(folder)
    truth = {json.loads(l)['step']: json.loads(l) for l in (folder / 'evaluation-only.jsonl').open()}
    errors = []
    for line in (folder / 'control.jsonl').open():
        c = json.loads(line)
        fa = c.get('face_alignment') or {}
        if c['box_skill_phase'] != 'approach' or not fa.get('ready'):
            continue
        t = truth[c['step']]
        qw, _qx, _qy, qz = t['box_quat']
        rel = math.degrees(2 * math.atan2(qz, qw) - t['base_yaw'])
        errors.append(abs((fa['evidence']['yaw_mod90_deg'] - rel + 45.) % 90. - 45.))
    return {'ready_frames': len(errors), 'max_err_deg': round(max(errors), 2) if errors else None,
            'within_5deg': sum(e <= 5. for e in errors)}


def cohort_stats_v6(runs):
    stats = cohort_stats_v5(runs)
    ev = [r['evaluation_only'] for r in runs]
    summaries = [r['skill_summary'] or {} for r in runs]
    stats.update({
        'setup_yaw_deg': [r['scenario_setup_only']['box_yaw_deg'] for r in runs],
        'axis_aligned_seeds': [r['seed'] for r in runs if abs(r['scenario_setup_only']['box_yaw_deg']) <= 2.],
        'axis_aligned_diagnostic_success': sum(bool(r['diagnostic_success']) for r in runs
                                               if abs(r['scenario_setup_only']['box_yaw_deg']) <= 2.),
        'approach_side': [((s.get('approach_choice') or {}).get('side'), (s.get('approach_choice') or {}).get('candidate'))
                          for s in summaries],
        'approach_blocked': sum(bool((s.get('approach_choice') or {}).get('blocked')) for s in summaries),
        'keepout_guard_stops': sum(int(s.get('keepout_guard_stops') or 0) for s in summaries),
        'peer_contact_steps': [e.get('peer_contact_steps') for e in ev],
        'face_estimator': sorted({str(s.get('face_estimator')) for s in summaries}),
        'face_normal_source_at_grasp': [[(e.get('evidence') or {}).get('normal_source') for e in r['skill_summary']['face_evidence']]
                                        for r in runs],
        'face_error_eval_only': {r['seed']: face_error_eval(r['raw_dir']) for r in runs}})
    fe = stats['face_error_eval_only'].values()
    ready = sum(f['ready_frames'] for f in fe)
    stats['face_ready_frames_within_5deg'] = f"{sum(f['within_5deg'] for f in fe)}/{ready}"
    return stats


def contact_sheet():
    import cv2
    import numpy as np
    picks = [(PROBES['probe2_door_first'], f) for f in (
        'rest-teacher_hover-pan1500', 'rest-carry_p30-pan1500', 'rest-look_p20-pan1500',
        'door-d1.0-carry_p30-pan1500', 'door-d0.6-look_p20-pan1230', 'zoneC-carry_p30')]
    tiles = []
    for folder, name in picks:
        path = RAW / folder / 'frames' / f'{name}.jpg'
        if not path.exists():
            continue
        im = cv2.resize(cv2.imread(str(path)), (320, 240))
        cv2.putText(im, name, (6, 232), cv2.FONT_HERSHEY_SIMPLEX, .42, (0, 0, 255), 1)
        tiles.append(im)
    if len(tiles) == 6:
        sheet = np.vstack([np.hstack(tiles[:3]), np.hstack(tiles[3:])])
        cv2.imwrite(str(HERE / 'wrist-views.jpg'), sheet, [cv2.IMWRITE_JPEG_QUALITY, 80])


def main():
    out = {'experiment_id': '2026-09-25-zone-owncam-skill', 'raw_root_local_only': str(RAW),
           'raw_note': 'raw outputs are local (gitignored); not a remote backup',
           'probes': {k: probe_summary(RAW / v) for k, v in PROBES.items() if (RAW / v / 'probe.json').exists()},
           'cohort': collect_preregistered(_loader(RAW / COHORT, run_summary), TEST_SEEDS)[0],
           'development_runs': [run_summary(RAW / d) for d in DEV_RUNS if (RAW / d / 'result.json').exists()]}
    cohort = out['cohort']
    out['cohort_summary'] = {
        'n': len(cohort), 'pose_source': 'gt_stub_eval_only', 'counts_as_m1': False,
        'grasp_success_gt': sum(r['evaluation_only']['grasp_success_gt'] for r in cohort),
        'place_in_slot_gt': sum(r['evaluation_only']['place_in_slot_gt'] for r in cohort),
        'skill_claim_in_slot': sum(r['evaluation_only']['skill_claim_in_slot'] for r in cohort),
        'skill_claim_agrees_with_gt': sum(r['evaluation_only']['skill_claim_agrees_with_gt'] for r in cohort),
        'weld_eq_active_max': max((r['evaluation_only']['weld_eq_active_max'] for r in cohort), default=None),
        'sim_seconds': [r['sim_seconds'] for r in cohort],
        'face_normal_sources': [r['face_normal_source'] for r in cohort]}
    missing_all = {}
    missing_all['v1:' + COHORT] = collect_preregistered(_loader(RAW / COHORT, run_summary), TEST_SEEDS)[1]
    v2, missing_all['v2:' + V2_COHORT] = collect_preregistered(_loader(RAW / V2_COHORT, run_summary), V2_TEST_SEEDS)
    out['v2_cohort'] = v2
    out['v2_development_runs'] = [run_summary(RAW / d) for d in V2_DEV_RUNS if (RAW / d / 'result.json').exists()]
    out['v2_cohort_summary'] = {
        'n': len(v2), 'profile': 'wrist_zone_skill_v2', 'pose_source': 'gt_stub_eval_only', 'counts_as_m1': False,
        'place_in_slot_gt': sum(r['evaluation_only']['place_in_slot_gt'] for r in v2),
        'grasp_success_gt': sum(r['evaluation_only']['grasp_success_gt'] for r in v2),
        'skill_claim_agrees_with_gt': sum(r['evaluation_only']['skill_claim_agrees_with_gt'] for r in v2),
        'false_success': sum(r['evaluation_only']['skill_claim_in_slot'] and not r['evaluation_only']['place_in_slot_gt'] for r in v2),
        'weld_eq_active_max': max((r['evaluation_only']['weld_eq_active_max'] for r in v2), default=None),
        'wall_contact_steps': sum(r['evaluation_only']['r1_wall_contact_steps'] for r in v2),
        'sim_seconds': [r['sim_seconds'] for r in v2], 'reasons': [r['reason'] for r in v2],
        'reseats': [(r['skill_summary'] or {}).get('reseats') for r in v2],
        'retreats': [(r['skill_summary'] or {}).get('retreats') for r in v2]}
    out['grip_hold_probe'] = {Path(d).name: grip_hold_summary(RAW / d) for d in GRIP_HOLD
                              if (RAW / d / 'result.json').exists()}
    out['v3_development_runs'] = [run_summary(f.parent) for f in sorted(RAW.glob('dev-v3/*/result.json'))]
    out['v3_cohorts'] = {}
    for folder in sorted(RAW.glob('cohort-v3-*')):
        for arm, profile in V3_ARMS.items():
            runs, missing = collect_preregistered(_loader(folder / arm, run_summary), V3_ARM_SEEDS[arm])
            missing_all[f'v3:{folder.name}/{arm}'] = missing
            out['v3_cohorts'][f'{folder.name}/{arm}'] = {
                'arm': arm, 'contact_profile': profile, 'runs': runs, 'missing': missing, 'summary': cohort_stats(runs)}
        log = folder / 'cohort.log'
        if log.exists():
            out['v3_cohorts'].setdefault('logs', {})[folder.name] = {'path': str(log), 'sha256': sha(log)}
    out['v4_development_runs'] = [run_summary(f.parent) for f in sorted(RAW.glob('dev-v4/*/result.json'))]
    out['v4_cohorts'] = {}
    for folder in sorted(RAW.glob('cohort-v4-*')):
        for arm, condition in V4_ARMS.items():
            runs, missing = collect_preregistered(_loader(folder / arm, run_summary), V4_ARM_SEEDS[arm])
            missing_all[f'v4:{folder.name}/{arm}'] = missing
            summary = cohort_stats(runs)
            summary['self_occluded_rejections'] = [(r['skill_summary'] or {}).get('self_occluded_rejections') for r in runs]
            summary['zone_c_placed'] = sum(r['evaluation_only']['place_in_slot_gt'] for r in runs if r['seed'] in V4_ZONE_C_SEEDS)
            summary['zone_c_n'] = sum(r['seed'] in V4_ZONE_C_SEEDS for r in runs)
            summary['held_stops'] = [r['seed'] for r in runs if r['evaluation_only']['box_final_xyz'][2] > .08
                                     and (r['reason'] == 'CARRY_TOP_GEOMETRY_AMBIGUOUS_FOR_DROP' or r['reason'].startswith('GRIP_CHECK_'))]
            if arm == 'D':
                summary['drop_safety'] = [{'seed': r['seed'], 'reason': r['reason'], 'claim_in_slot': r['evaluation_only']['skill_claim_in_slot'],
                                           'box_final_z_m': r['evaluation_only']['box_final_xyz'][2],
                                           'fault_injection': r['fault_injection']} for r in runs]
            out['v4_cohorts'][f'{folder.name}/{arm}'] = {'arm': arm, 'condition': condition, 'runs': runs,
                                                         'missing': missing, 'summary': summary}
        log = folder / 'cohort.log'
        if log.exists():
            out['v4_cohorts'].setdefault('logs', {})[folder.name] = {'path': str(log), 'sha256': sha(log)}
    v2_log = RAW / V2_COHORT / 'cohort.log'
    if v2_log.exists():
        out['v2_cohort_log'] = {'path': str(v2_log), 'sha256': sha(v2_log)}
    cohort_log = RAW / COHORT / 'cohort.log'
    if cohort_log.exists():
        out['cohort_log'] = {'path': str(cohort_log), 'sha256': sha(cohort_log)}
    out['v5_development_runs'] = [run_summary_v5(f.parent) for f in sorted(RAW.glob('dev-v5/*/result.json'))]
    out['v5_cohorts'] = {}
    for folder in sorted(RAW.glob('cohort-v5-*')):
        for arm, condition in V5_ARMS.items():
            runs, missing = collect_preregistered(_loader(folder / arm, run_summary_v5), V5_TEST_SEEDS)
            missing_all[f'v5:{folder.name}/{arm}'] = missing
            out['v5_cohorts'][f'{folder.name}/{arm}'] = {'arm': arm, 'condition': condition, 'runs': runs,
                                                         'missing': missing, 'summary': cohort_stats_v5(runs)}
        log = folder / 'cohort.log'
        if log.exists():
            out['v5_cohorts'].setdefault('logs', {})[folder.name] = {'path': str(log), 'sha256': sha(log)}
    out['v6_development_runs'] = [run_summary_v5(f.parent) for f in sorted(RAW.glob('dev-v6/*/result.json'))]
    out['v6_cohorts'] = {}
    for folder in sorted(RAW.glob('cohort-v6-*')):
        for arm, condition in V6_ARMS.items():
            runs, missing = collect_preregistered(_loader(folder / arm, run_summary_v5), V6_TEST_SEEDS)
            missing_all[f'v6:{folder.name}/{arm}'] = missing
            out['v6_cohorts'][f'{folder.name}/{arm}'] = {'arm': arm, 'condition': condition, 'runs': runs,
                                                         'missing': missing, 'summary': cohort_stats_v6(runs)}
        log = folder / 'cohort.log'
        if log.exists():
            out['v6_cohorts'].setdefault('logs', {})[folder.name] = {'path': str(log), 'sha256': sha(log)}
    out['missing_seeds'] = {k: v for k, v in missing_all.items() if v}
    blocked = bool(out['missing_seeds'])
    out['cohort_report_blocked'] = blocked
    if blocked and '--allow-incomplete' not in sys.argv:
        for name, missing in out['missing_seeds'].items():
            require_complete(name, missing)
    (HERE / 'results.json').write_text(json.dumps(out, indent=1, ensure_ascii=False) + '\n')
    contact_sheet()
    print(json.dumps(out['cohort_summary'], ensure_ascii=False))
    print(json.dumps(out['v2_cohort_summary'], ensure_ascii=False))
    for key, value in {**out['v3_cohorts'], **out['v4_cohorts']}.items():
        if key != 'logs':
            print(key, json.dumps(value['summary'], ensure_ascii=False))
    for key, value in {**out['v5_cohorts'], **out['v6_cohorts']}.items():
        if key != 'logs':
            print(key, json.dumps(value['summary'], ensure_ascii=False))


if __name__ == '__main__':
    main()
