"""Collect the environment-v3 loop runs (dev, dev-plain, test, and amendment cohorts) into results.json.

Reads only finished run directories (result.json, manifest.json, eval_only/*) and never modifies them.
Every pre-registered episode of a split must have exactly one run, otherwise the cohort gate is
reported as blocked ('missing'). Door-region breakdowns by x band are POST HOC diagnostics
(evaluation-only ground truth), not gates.

  python build_results.py --raw <outputs/zone-env-v3-20260926/loop> --output loop/results.json
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import statistics
from pathlib import Path

X = Path(__file__).resolve().parent
ROOT = X.parents[1]
BANDS = ((1.6, 2.0), (2.0, 2.2), (2.2, 2.45), (2.45, 2.8))
# (record key, prereg file, raw sub-folder, split): one row per cohort attempt.
COHORTS = (
    ('v3-dev', 'loop/prereg.json', 'dev', 'dev'),
    ('v3-dev-plain', 'loop/prereg.json', 'dev-plain', 'dev'),
    ('v3-test', 'loop/prereg.json', 'test', 'test'),
    ('v3a1-test', 'loop/prereg_a1.json', 'a1-test', 'test'),
)


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def jsonl(path: Path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def pct(values, q):
    return None if not values else round(float(sorted(values)[min(len(values) - 1, int(q*len(values)))]), 4)


def door_bands(run: Path, static: dict) -> dict:
    """POST HOC: door-region estimate error by GT x band and the tag sites seen there."""
    site = {int(t['id']): t.get('site', t.get('wall')) for t in static['landmarks']['tags']}
    rows = [e for e in jsonl(run/'eval_only'/'frames_eval.jsonl')
            if e['phase'] == 'student' and e['door_region'] and 'pos_err_m' in e]
    out = {}
    for lo, hi in BANDS:
        band = [e for e in rows if lo <= e['gt'][0] < hi]
        if not band:
            continue
        seen = {}
        for e in band:
            for t in e['tags']:
                seen[site.get(int(t), '?')] = seen.get(site.get(int(t), '?'), 0) + 1
        out[f'x{lo:.2f}-{hi:.2f}'] = {
            'frames': len(band), 'p50_m': pct([e['pos_err_m'] for e in band], .5),
            'p90_m': pct([e['pos_err_m'] for e in band], .9),
            'mean_dx_m': round(statistics.mean(e['est'][0] - e['gt'][0] for e in band), 4),
            'mean_dy_m': round(statistics.mean(e['est'][1] - e['gt'][1] for e in band), 4),
            'mean_dyaw_deg': round(statistics.mean(math.degrees((e['est'][2] - e['gt'][2] + math.pi) % (2*math.pi) - math.pi)
                                                   for e in band), 3),
            'frames_without_tag': sum(not e['tags'] for e in band), 'tag_sightings_by_site': dict(sorted(seen.items()))}
    return out


def run_row(run: Path) -> dict:
    import sys
    if str(ROOT) not in sys.path:
        sys.path.insert(0, str(ROOT))
    from sim.research_dispatch_arena import digest
    result = json.loads((run/'result.json').read_text())
    manifest = json.loads((run/'manifest.json').read_text())
    map_path = ROOT/'maps'/'zones'/(manifest['map_id'] + '.json')
    static = json.loads(map_path.read_text())
    if manifest['static_map_sha256'] != digest(static):
        raise SystemExit(f'{run}: manifest static_map_sha256 differs from the committed map file')
    scene_xml = (run/'scene.xml').read_text()
    top = run/'eval_only'/'top_recording.json'
    g = result['gates']
    return {
        'episode': result['episode'], 'split': result['split'], 'condition': result['condition'],
        'robot_id': result['robot_id'], 'seed': manifest['spec']['seed'], 'spawn_y': manifest['spec']['spawn_y'],
        'map_id': manifest['map_id'], 'static_map_sha256': manifest['static_map_sha256'],
        'landmarks_sha256': manifest['landmarks_sha256'], 'scene_xml_sha256': manifest['scene_xml_sha256'],
        'contact_profile': manifest['contact_profile'], 'noslip_iterations_in_scene_xml': 'noslip_iterations="10"' in scene_xml,
        'weld': manifest['weld'], 'calibration_sha256': manifest['calibration_sha256'], 'student': manifest['student'],
        'code_sha': manifest['code']['sha'], 'code_dirty': manifest['code']['dirty'],
        'outcome': result['outcome'], 'pass': result['episode_pass'],
        'R1_pass': g['R1_exit_waypoint']['pass'], 'R1_gt_distance_m': g['R1_exit_waypoint']['gt_distance_m'],
        'R2_wall_contacts': g['R2_no_wall_contact']['wall_contacts'],
        'R3_pass': g['R3_estimate_near_door']['pass'], 'R3_p90_pos_m': g['R3_estimate_near_door']['p90_pos_m'],
        'R3_p90_yaw_deg': g['R3_estimate_near_door']['p90_yaw_deg'], 'R3_frames': g['R3_estimate_near_door']['frames'],
        'R3_uninitialized': g['R3_estimate_near_door']['uninitialized'],
        'R4_pass': g.get('R4_box_held', {}).get('pass'), 'R4_min_box_z_m': g.get('R4_box_held', {}).get('min_box_z_m'),
        'student_sim_s': result['student_sim_s'], 'teacher_sim_s': result['teacher_sim_s'], 'looks': result['looks'],
        'look_reasons': result['look_reasons'], 'student_commands': result['student_commands'],
        'student_frames': result['student_frames'], 'tag_visibility': result['student_tag_visibility'],
        'contacts_student': result['contacts_student'], 'wall_s': manifest['wall_s'],
        'load_average': manifest['load_average'], 'threads': manifest['env']['threads'],
        'top_recording_frames': json.loads(top.read_text())['frames'] if top.exists() else None,
        'posthoc_door_bands': door_bands(run, static),
        'raw': {'dir': str(run), 'result_sha256': sha(run/'result.json'), 'manifest_sha256': sha(run/'manifest.json'),
                'map_file_sha256': sha(map_path),
                'files': sum(1 for p in run.rglob('*') if p.is_file()),
                'bytes': sum(p.stat().st_size for p in run.rglob('*') if p.is_file()),
                'file_list_sha256': hashlib.sha256('\n'.join(sorted(
                    f'{p.relative_to(run)} {p.stat().st_size}' for p in run.rglob('*') if p.is_file())).encode()).hexdigest()}}


def cohort_gate(rows, episodes):
    test = {e['episode_id']: e for e in episodes if e['split'] == 'test'}
    got = {r['episode']: r for r in rows}
    missing = sorted(set(test) - set(got))
    box = [got[e] for e in test if test[e]['condition'] == 'box' and e in got]
    nobox = [got[e] for e in test if test[e]['condition'] == 'nobox' and e in got]
    lg1 = sum(r['pass'] for r in box)
    lg2 = sum(r['pass'] for r in nobox)
    walls = sum(r['R2_wall_contacts'] for r in got.values())
    drops = sum(r['R4_pass'] is False for r in box)
    return {'missing': missing,
            'LG1_box': f'{lg1}/{len(box)}', 'LG1_pass': not missing and lg1 >= 5,
            'LG2_nobox': f'{lg2}/{len(nobox)}', 'LG2_pass': not missing and lg2 == 3,
            'LG3_wall_contacts': walls, 'LG3_box_drops': drops, 'LG3_pass': not missing and walls == 0 and drops == 0,
            'pass': (not missing and lg1 >= 5 and lg2 == 3 and walls == 0 and drops == 0),
            'blocked': bool(missing)}


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('--raw', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args(argv)
    out = {'schema': 'ugrp.zone_env_v3.loop_results.v1', 'raw_root': str(a.raw), 'cohorts': {}}
    for key, prereg_name, folder, split in COHORTS:
        prereg_path = X/prereg_name
        if not prereg_path.exists() or not (a.raw/folder).exists():
            continue
        prereg = json.loads(prereg_path.read_text())
        episodes = [e for e in prereg['episodes'] if e['split'] == split]
        rows = []
        for e in episodes:
            run = a.raw/folder/e['episode_id']
            if (run/'result.json').exists():
                rows.append(run_row(run))
            elif run.exists():
                rows.append({'episode': e['episode_id'], 'split': split, 'status': 'no result.json (infrastructure failure or running)'})
        done = [r for r in rows if 'pass' in r]
        entry = {'prereg': prereg_name, 'prereg_sha256': sha(prereg_path), 'raw_folder': folder, 'runs': rows}
        if split == 'test':
            entry['cohort_gate'] = cohort_gate(done, episodes)
        out['cohorts'][key] = entry
    a.output.write_text(json.dumps(out, indent=1, ensure_ascii=False) + '\n')
    for key, entry in out['cohorts'].items():
        print(key, entry.get('cohort_gate'), [(r['episode'], r.get('pass'), r.get('R3_p90_pos_m')) for r in entry['runs']])


if __name__ == '__main__':
    main()
