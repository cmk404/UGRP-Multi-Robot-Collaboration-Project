"""Environment v3 TensorBoard snapshot (derived one-folder views -> main exporter -> short names -> checks).

Raw run directories are read only. For every run this writes ``<views>/<name>/result.json``: a
derived view that carries only values present in the raw result/manifest (no interpolation),
plus ``execution.mp4`` (wrist | evaluation-only TOP video, make_videos.py) when available. The
views are exported with ``scripts/export_tensorboard.py`` into a temporary collection outside
the shared logdir, renamed to the short view names, checked with EventAccumulator against the
raw values, and only then moved into ``outputs/tensorboard/<snapshot>``.

Baselines: loop v2 on the v2 map (test box s61-s66, never converted before) and, if M1 dev runs
exist, the M1 test on the v2 map (s101-s106). The loop v1 nobox test on the v2 map is already in
the snapshot 0926-zone-owncam-loop (test-nobox-s41..43) and is referenced, not converted again.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import subprocess
import sys
from pathlib import Path

X = Path(__file__).resolve().parent
ROOT = X.parents[1]
OUT_ROOT = Path('/Users/changmin/projects/ugrp/outputs')
RAW = OUT_ROOT/'zone-env-v3-20260926'
PY = '/Users/changmin/projects/ugrp/.venv-sim-worker-mac/bin/python'
SCHEMA = 'ugrp.zone_env_v3.tb_view.v1'
MEDIA_PORT = 6009
PINNED = ('evaluation/reported_success', 'result/sim_s', 'result/commands', 'result/wall_s', 'result/model_calls',
          'claims/protocol_complete')


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load(path: Path):
    return json.loads(path.read_text())


def entries():
    """(view name, raw run dir, kind, cohort text) for every run that exists."""
    out = []
    groups = (
        ('v3dev', RAW/'loop'/'dev', 'loop', 'v3 dev (infrastructure smoke, not gated), zone_wide_door_tags_v3'),
        ('v3devplain', RAW/'loop'/'dev-plain', 'loop', 'v3 dev s711 rerun WITHOUT the TOP recorder (determinism check)'),
        ('v3test', RAW/'loop'/'test', 'loop', 'v3 pre-registered test (once), zone_wide_door_tags_v3'),
        ('v3a1test', RAW/'loop'/'a1-test', 'loop', 'v3 amendment A1 pre-registered test (once), amended map'),
        ('m1env', RAW/'m1'/'dev', 'm1', 'M1 dev check on the v3 environment (skill v9 + localizer)'),
        ('m1a1', RAW/'m1'/'a1-dev', 'm1', 'M1 dev check on the amended A1 map (skill v9 + localizer)'),
        ('base-loopv2', OUT_ROOT/'owncam-loop-v2-20260926'/'test', 'loop',
         'BASELINE loop v2 test on zone_wide_door_tags_v2 (0.10 m walls, 80 tags), local_contact_fine'),
    )
    for prefix, folder, kind, cohort in groups:
        if not folder.is_dir():
            continue
        for run in sorted(p for p in folder.iterdir() if (p/'result.json').is_file()):
            r = load(run/'result.json')
            short = run.name
            for stem in ('envdev-', 'envtest-', 'a1test-', 'v2test-', 'm1env-', 'm1a1-'):
                short = short.replace(stem, '')
            out.append((f'{prefix}-{short}', run, kind, cohort))
    if any(k == 'm1' for _, _, k, _ in out):
        folder = OUT_ROOT/'m1-owncam-20260926'/'test'
        for run in sorted(p for p in folder.iterdir() if (p/'result.json').is_file()):
            out.append((f"base-m1-{run.name.replace('m1test-', '')}", run, 'm1',
                        'BASELINE M1 test on zone_wide_door_tags_v2 (frozen ca2fdb8; s103/s104 host ENOSPC)'))
    return out


def loop_view(run: Path, cohort: str) -> dict:
    r, m = load(run/'result.json'), load(run/'manifest.json')
    g = r['gates']
    return {
        'schema': SCHEMA, 'success': bool(r['episode_pass']), 'protocol_complete': r['outcome'] == 'arrived',
        'stop_reason': r['outcome'], 'sim_s': r['student_sim_s'], 'wall_s': m['wall_s'],
        'commands': r['student_commands'], 'model_calls': 0,
        'policy': f"owncam loop driver {m['student']['driver']} (tag PF + A*)",
        'case': f"{cohort.split(',')[0]} | {r['condition']} | {m['map_id']}", 'seed': m['spec']['seed'],
        'source_sha': m['code']['sha'], 'clock': 'sync SIM (0.25 ms)',
        'scope': ('student part only: own wrist RGB + own commands + static tagged map; GT only for evaluation. '
                  'success = pre-registered R1-R3 (+R4 box) episode_pass; protocol_complete = arrival declared'),
        'config': {'variant': m['map_id'], 'seed': m['spec']['seed'], 'contact_profile': m['contact_profile']},
        'evaluation': {'cohort': cohort, 'condition': r['condition'], 'gates': g, 'looks': r['looks'],
                       'look_reasons': r['look_reasons'], 'tag_visibility': r['student_tag_visibility'],
                       'teacher_sim_s_not_counted': r['teacher_sim_s'], 'contacts_student': r['contacts_student'],
                       'map_id': m['map_id'], 'static_map_sha256': m['static_map_sha256'],
                       'scene_xml_sha256': m['scene_xml_sha256'], 'weld': m['weld'], 'student': m['student'],
                       'load_average': m['load_average'], 'raw_dir': str(run),
                       'raw_result_sha256': sha(run/'result.json'), 'raw_manifest_sha256': sha(run/'manifest.json')},
        'model_provenance': 'no model: deterministic particle filter + A* (model_calls 0; no model latency)'}


def m1_view(run: Path, cohort: str) -> dict:
    r, m = load(run/'result.json'), load(run/'manifest.json')
    ev = r.get('evaluation_only', {})
    return {
        'schema': SCHEMA, 'success': bool(r['m1_success']), 'protocol_complete': r['skill_reason'] == 'SKILL_OWN_RGB_PLACEMENT_IN_SLOT',
        'stop_reason': r['outcome'], 'sim_s': r['sim_s'], 'wall_s': m.get('wall_s'),
        'commands': r.get('commands'), 'model_calls': 0, 'policy': f"M1 controller + skill {m['student']['skill']} + {r['pose_source']}",
        'case': f"{cohort.split('(')[0].strip()} | {m['map_id']} | slot {m['spec']['slot_id']}", 'seed': m['spec']['seed'],
        'source_sha': m['code']['sha'], 'clock': 'sync SIM (0.25 ms)',
        'scope': ('full M1 chain on own camera: search, grasp, carry through the door, place, re-look; success = m1_success '
                  '(12 judge items); protocol_complete = skill own-RGB IN_SLOT claim'),
        'config': {'variant': m['map_id'], 'seed': m['spec']['seed'], 'contact_profile': ev.get('contact_profile', {}).get('profile')},
        'evaluation': {'cohort': cohort, 'm1_checks': r.get('m1_checks'), 'm1_failed_checks': r.get('m1_failed_checks'),
                       'false_success': r.get('false_success'), 'looks': r.get('looks'), 'exception': r.get('exception'),
                       'gt_box_final_xyz': ev.get('gt_box_final_xyz'), 'slot_xy': ev.get('slot_xy'),
                       'contacts': ev.get('contacts'), 'map_id': m['map_id'], 'static_map_sha256': m.get('static_map_sha256'),
                       'raw_dir': str(run), 'raw_result_sha256': sha(run/'result.json'),
                       'raw_manifest_sha256': sha(run/'manifest.json')},
        'model_provenance': 'no model: deterministic controller (model_calls 0; no model latency)'}


def build_views(views: Path, videos: Path):
    views.mkdir(parents=True, exist_ok=False)
    rows = []
    for name, run, kind, cohort in entries():
        view = (loop_view if kind == 'loop' else m1_view)(run, cohort)
        folder = views/name
        folder.mkdir()
        (folder/'result.json').write_text(json.dumps(view, indent=1, ensure_ascii=False) + '\n')
        video = videos/f'{name}.mp4'
        if video.is_file():
            shutil.copy2(video, folder/'execution.mp4')
        rows.append({'name': name, 'raw': str(run), 'kind': kind, 'cohort': cohort, 'video': (folder/'execution.mp4').is_file(),
                     'view_sha256': sha(folder/'result.json'), 'raw_result_sha256': view['evaluation']['raw_result_sha256']})
    (views/'index.json').write_text(json.dumps({'schema': SCHEMA + '.index', 'views': rows}, indent=1, ensure_ascii=False) + '\n')
    return rows


def export(views: Path, staging: Path, rows):
    # media port of the running shared viewer (scripts/run_tensorboard.py --media-port 6009)
    cmd = [PY, str(ROOT/'scripts'/'export_tensorboard.py'), '--output', str(staging), '--max-images', '0',
           '--media-port', str(MEDIA_PORT)]
    for row in rows:
        cmd += ['--source', str(views/row['name'])]
    subprocess.run(cmd, cwd=ROOT, check=True)
    collection = load(staging/'collection.json')
    if collection['failed']:
        raise SystemExit(f"export failures: {collection['failed']}")
    by_source = {e['source']: e for e in collection['exported']}
    for row in rows:
        entry = by_source[str((views/row['name']).resolve())]
        (staging/entry['name']).rename(staging/row['name'])
        entry.update(original_name=entry['name'], name=row['name'], condition=row['cohort'])
    (staging/'collection.json').write_text(json.dumps(collection, indent=2, ensure_ascii=False) + '\n')


def verify(staging: Path, views: Path, rows) -> dict:
    from tensorboard.backend.event_processing.event_accumulator import EventAccumulator
    checks, bad = 0, []
    for row in rows:
        acc = EventAccumulator(str(staging/row['name']))
        acc.Reload()
        view = load(views/row['name']/'result.json')
        raw = load(Path(row['raw'])/'result.json')
        if sha(Path(row['raw'])/'result.json') != row['raw_result_sha256']:
            bad.append((row['name'], 'raw changed'))
        want = {'evaluation/reported_success': int(raw['episode_pass'] if row['kind'] == 'loop' else raw['m1_success']),
                'result/sim_s': raw['student_sim_s'] if row['kind'] == 'loop' else raw['sim_s'],
                'result/commands': view['commands'], 'result/model_calls': 0, 'result/wall_s': view['wall_s'],
                'claims/protocol_complete': int(view['protocol_complete'])}
        tags = set(acc.Tags()['scalars'])
        for tag, value in want.items():
            if value is None:
                continue
            checks += 1
            if tag not in tags:
                bad.append((row['name'], tag, 'missing'))
                continue
            got = acc.Scalars(tag)[-1].value
            if abs(got - float(value)) > 1e-3*max(1., abs(float(value))):
                bad.append((row['name'], tag, got, value))
    return {'runs': len(rows), 'scalar_checks': checks, 'mismatches': bad}


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('--snapshot', default='0926-zone-env-v3')
    p.add_argument('--videos', type=Path, default=RAW/'videos')
    a = p.parse_args(argv)
    work = RAW/'tensorboard'
    views, staging = work/'views', work/'staging'/a.snapshot
    final = OUT_ROOT/'tensorboard'/a.snapshot
    if final.exists() or views.exists() or staging.exists():
        raise SystemExit('snapshot, views or staging already exist; make a new snapshot id instead of overwriting')
    rows = build_views(views, a.videos)
    export(views, staging, rows)
    report = verify(staging, views, rows)
    if report['mismatches']:
        raise SystemExit(f'verification failed: {report}')
    shutil.move(str(staging), str(final))
    manifest_sha = {r['name']: sha(final/r['name']/'manifest.json') for r in rows}
    out = {'schema': SCHEMA + '.snapshot', 'snapshot': str(final), 'views': str(views), 'verification': report,
           'collection_sha256': sha(final/'collection.json'), 'manifest_sha256': manifest_sha,
           'videos': {r['name']: r['video'] for r in rows}, 'pinned': PINNED,
           'baseline_reference': '0926-zone-owncam-loop/test-nobox-s41..43 (loop v1 nobox on the v2 map, already converted)'}
    (X/'tensorboard_snapshot.json').write_text(json.dumps(out, indent=1, ensure_ascii=False) + '\n')
    print(json.dumps({'runs': len(rows), 'checks': report['scalar_checks'], 'snapshot': str(final)}))


if __name__ == '__main__':
    main()
