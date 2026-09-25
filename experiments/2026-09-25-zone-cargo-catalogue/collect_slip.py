#!/usr/bin/env python3
"""Build results-slip.json and slip media from the diagnosis and acceptance roots.

  python3 experiments/2026-09-25-zone-cargo-catalogue/collect_slip.py \
      outputs/zone-cargo/diag-1 outputs/zone-cargo/diag-2 outputs/zone-cargo/slip-<sha>
"""
from __future__ import annotations

import hashlib
import json
import shutil
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ITEMS = ('solo_box', 'solo_can', 'solo_tile', 'pair_beam', 'pair_crate', 'trio_frame')
LONG = ('pair_beam_long', 'pair_crate_long', 'trio_frame_long')
FEWER = ('solo_beam', 'solo_crate', 'duo_frame')
PROFILES = ('local_contact_fine', 'cargo_noslip_v1')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load(d):
    return json.loads((Path(d)/'result.json').read_text())


def pick(r, *keys):
    return {k: r.get(k) for k in keys}


def main(diag1, diag2, final):
    final = Path(final).resolve()
    media = HERE/'media'
    raw = {}
    diagnosis = {}
    for root in (Path(diag1), Path(diag2)):
        driver = (root/'driver.log').read_text().splitlines()
        for log in sorted(root.glob('*.log')):
            if log.name == 'driver.log':
                continue
            name = log.stem
            res = root/name/'result.json'
            if res.exists():
                r = load(root/name)
                diagnosis[name] = {**pick(r, 'lifted_clear', 'teacher_outcome', 'hold_slip_mm', 'hold_creep_mm_per_s',
                                          'hold_measured_s', 'max_robot_tilt_deg', 'grasp_forces_n', 'solver_options',
                                          'git_sha', 'git_dirty', 'load_avg_1m'), 'root': root.name}
                raw[f'{root.name}/{name}/result.json'] = sha(res)
            else:
                text = log.read_text()
                diagnosis[name] = {'teacher_outcome': 'simulation_unstable', 'root': root.name,
                                   'detail': next((l for l in text.splitlines() if 'WARNING' in l), text.splitlines()[-1])}
                raw[f'{root.name}/{log.name}'] = sha(log)
        raw[f'{root.name}/driver.log'] = sha(root/'driver.log')
        diagnosis.setdefault('_drivers', {})[root.name] = driver
    keys = ('success', 'teacher_outcome', 'lifted_clear', 'max_lift_min_z_m', 'placement', 'max_slip_mm',
            'hold_slip_mm', 'hold_measured_s', 'max_cargo_tilt_deg', 'max_robot_tilt_deg', 'drop_events',
            'floor_contact_samples_in_carry', 'cargo_robot_body_contact_geoms', 'finger_forces', 'grasp_forces_n',
            'servo_saturated_samples', 'max_track_err_m', 'phase_sim_s', 'sim_time_s', 'load_avg_1m', 'legs',
            'target_pose', 'contact_profile', 'contact_profile_record', 'solver_options', 'weld', 'eq_active_max',
            'git_sha', 'git_dirty', 'scene_xml_sha256', 'failure_detail')
    groups = {'static_hold_60s': {}, 'long_route': {}, 'fewer_robots': {}, 'solo_carry': {}}
    for d in sorted(p for p in final.iterdir() if p.is_dir()):
        for f in sorted(d.rglob('*')):
            if f.is_file() and f.suffix in ('.json', '.jsonl', '.mp4', '.xml'):
                raw[f'{final.name}/{f.relative_to(final)}'] = sha(f)
        if not (d/'result.json').exists():
            continue
        r = load(d)
        name = d.name
        if name.startswith('hold60-'):
            groups['static_hold_60s'][name[7:]] = pick(r, *keys)
        elif name.startswith('long-'):
            groups['long_route'][name[5:]] = pick(r, *keys)
        elif name.startswith('fewer-'):
            groups['fewer_robots'][name[6:]] = pick(r, *keys)
        elif name.startswith('solo-'):
            groups['solo_carry'][name[5:]] = pick(r, *keys)
        elif name.startswith('drive-'):
            groups.setdefault('drive_check', {})[name[6:]] = r
        mp4 = next(d.glob('*.mp4'), None)
        if mp4 is not None and (name.startswith('long-') or name.startswith('fewer-')):
            target = media/f'slip-{name}.mp4'
            shutil.copy2(mp4, target)
            groups[{'long': 'long_route', 'fewer': 'fewer_robots'}[name.split('-')[0]]][name.split('-', 1)[1]]['video'] = f'media/{target.name}'
    groups['capacity_sweep_cargo_noslip_v1'] = json.loads((final/'sweep-cargo_noslip_v1'/'capacity.json').read_text())
    a, b = groups['drive_check']['local_contact_fine'], groups['drive_check']['cargo_noslip_v1']
    groups['drive_check_max_abs_difference'] = max(
        abs(x-y) for s, t in zip(a['segments'], b['segments'])
        for x, y in zip(s['pose']+s['vel_xy_mps']+[s['yaw_rate']], t['pose']+t['vel_xy_mps']+[t['yaw_rate']]))
    from sim.zone_cargo_contact import profile_record  # noqa: E402
    result = {'schema': 'ugrp.experiment.zone_cargo_slip.v1', 'date': '2026-09-25',
              'request': 'user via coordinator: remove the slip (weld/fixation OFF, local_contact_fine unchanged)',
              'profile': profile_record('cargo_noslip_v1'),
              'diagnosis_60s_static_hold_0p5kg_one_robot': diagnosis,
              'acceptance': groups, 'driver_log': (final/'driver.log').read_text().splitlines(),
              'raw_root_local_only': str(final), 'raw_files_sha256': raw,
              'scope': ['1 run per condition, deterministic synchronous SIM; host load was very high (see driver logs), '
                        'which does not change SIM results; no wall-time claims',
                        'diagnosis ran from 8ee716f with the variant code uncommitted (git_dirty); the same variant '
                        'definitions were then committed in the acceptance source',
                        'ground-truth teacher only; not an RGB/student result']}
    (HERE/'results-slip.json').write_text(json.dumps(result, indent=2, ensure_ascii=False) + '\n')
    print(json.dumps({'profile_sha256': result['profile']['sha256'],
                      'media': sorted(p.name for p in media.glob('slip-*'))}, indent=1))


if __name__ == '__main__':
    sys.path.insert(0, str(HERE.parents[1]))
    main(*sys.argv[1:4])
