#!/usr/bin/env python3
"""Build results.json and media/ for this record from one final output root.

  python3 experiments/2026-09-25-zone-cargo-catalogue/collect.py outputs/zone-cargo/final-<sha>
"""
from __future__ import annotations

import hashlib
import json
import shutil
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
PROBES = ('solo_box', 'solo_can', 'solo_tile', 'pair_beam', 'pair_crate', 'trio_frame',
          'solo_beam', 'solo_crate', 'duo_frame')
KEEP = ('probe', 'kind', 'mass_kg', 'robots', 'required_carriers_claim', 'expect', 'attempt_carry_when_not_clear',
        'weld', 'eq_active_max', 'contact_profile', 'timestep_s', 'success', 'teacher_outcome', 'lifted_clear',
        'hold_min_z_m', 'max_lift_min_z_m', 'placement', 'max_cargo_tilt_deg', 'max_robot_tilt_deg', 'max_slip_mm',
        'hold_creep_mm_per_s', 'drop_events', 'floor_contact_samples_in_carry', 'cargo_robot_body_contact_samples',
        'cargo_robot_body_contact_geoms', 'finger_forces', 'grasp_forces_n', 'servo_saturated_samples',
        'max_track_err_m', 'phase_sim_s', 'sim_time_s', 'wall_s', 'load_avg_1m', 'failure_detail',
        'scene_xml_sha256', 'catalogue_sha256', 'git_sha', 'git_dirty', 'legs', 'target_pose')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main(root):
    root = Path(root).resolve()
    media = HERE/'media'
    media.mkdir(exist_ok=True)
    probes, raw = {}, {}
    for name in PROBES:
        d = root/name
        r = json.loads((d/'result.json').read_text())
        probes[name] = {k: r.get(k) for k in KEEP}
        for f in sorted(d.iterdir()):
            raw[f'{name}/{f.name}'] = {'sha256': sha(f), 'bytes': f.stat().st_size}
        mp4 = d/f'{name}.mp4'
        if mp4.exists():
            shutil.copy2(mp4, media/mp4.name)
            probes[name]['video'] = f'media/{mp4.name}'
    cap = json.loads((root/'sweep'/'capacity.json').read_text())
    for f in sorted((root/'sweep').rglob('*')):
        if f.is_file() and f.name in ('capacity.json', 'result.json', 'events.json'):
            raw[f'sweep/{f.relative_to(root/"sweep")}'] = {'sha256': sha(f), 'bytes': f.stat().st_size}
    cat = root/'catalogue'
    colour = json.loads((cat/'colour_check.json').read_text())
    catalogue = json.loads((cat/'catalogue.json').read_text())
    for f in sorted(cat.iterdir()):
        raw[f'catalogue/{f.name}'] = {'sha256': sha(f), 'bytes': f.stat().st_size}
    shutil.copy2(cat/'catalogue.png', media/'catalogue.png')
    shutil.copy2(cat/'top-TOP_NE.png', media/'catalogue-top-ne.png')
    shas = {p['git_sha'] for p in probes.values()}
    result = {
        'schema': 'ugrp.experiment.zone_cargo_catalogue.v1', 'date': '2026-09-25',
        'source_sha': sorted(shas), 'git_dirty_any': any(p['git_dirty'] for p in probes.values()),
        'condition': 'ground-truth TEACHER (poses, IK, contact forces); synchronous SIM; weld OFF; not an RGB/student result',
        'scene': {'variant': 'zone_wide', 'seed': 11, 'contact_profile': 'local_contact_fine',
                  'start_pose': [2.5, .2, 0.]},
        'catalogue': catalogue['catalogue'], 'existing_solo': catalogue['existing_solo'],
        'capacity_sweep': cap, 'probes': probes, 'colour_check': colour,
        'driver_log': (root/'driver.log').read_text().splitlines(),
        'raw_root_local_only': str(root), 'raw_files': raw,
        'media': sorted(p.name for p in media.iterdir()),
        'scope': ['1 run per probe (deterministic synchronous SIM); no seeds/positions varied',
                  'open floor patch of zone_wide; no walls, doors, other cargo or zone teacher integration',
                  'teacher uses ground truth; nothing here is an RGB/student capability',
                  'masses are relative to the SIM MasterPi model, not the real robot'],
    }
    (HERE/'results.json').write_text(json.dumps(result, indent=2, ensure_ascii=False) + '\n')
    print(json.dumps({k: {'success': v['success'], 'outcome': v['teacher_outcome']} for k, v in probes.items()}, indent=1))


if __name__ == '__main__':
    main(sys.argv[1])
