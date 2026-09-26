"""Build results.json / raw_index.json for the memory ON vs OFF experiment (offline, evaluation only).

Reads the raw episode folders written by scripts/run_m1_owncam_memory.py:
  <raw>/<attempt>/<condition>/<episode>/{result.json, manifest.json, memory_runner.json,
                                         inputs/commands.jsonl, inputs/frames.jsonl, controller_events.jsonl,
                                         eval_only/frames_eval.jsonl, ...}
Robot-side quantities (looks, pans, commands) are counted from the robot's own issued
commands and controller events; localization error, false confirmations and box
positions come from eval_only/ and the setup-only episode generator (scoring only).

  python experiments/2026-09-26-zone-owncam-memory/build_results.py --raw <raw root> [--attempt dev-a1 ...]
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT))

SCHEMA = 'ugrp.m1_owncam_memory_results.v1'
LOOK_P20 = {3: 1072, 4: 2400, 5: 1482}
DWELL_MIN_S = .5
SEARCH_PANS_OFF = 6                       # harness.m1_owncam_delivery.SEARCH_PANS
FALSE_TRACK_M = .10
FREE_NEAR_BOX_M = .05
KNOWN_FREE = -.6
CONDITIONS = ('off', 'memory_v1', 'memory_v2')
TEST_ON = 'memory_v2'                     # memory_v1 (ad78ef2) ran in dev-a1 only (amendment A1-A3)
INTERIM = {'memory_v2': 'interim, tag provider'}


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rows(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()] if path.exists() else []


def pct(values, q):
    v = sorted(values)
    if not v:
        return None
    k = (len(v) - 1)*q/100.
    lo, hi = math.floor(k), math.ceil(k)
    return round(v[lo] + (v[hi] - v[lo])*(k - lo), 5)


def look_stats(commands: list[dict]) -> dict:
    """LOOK_P20 sweeps, pan dwells and time in the look posture, from the robot's own issued commands."""
    servo, in_look, t_in, last_pan_t = {}, False, 0., 0.
    looks, dwells, look_time, per_look = 0, 0, 0., []
    for c in commands:
        t = float(c['t'])
        if c['kind'] == 'initial_servo_command':
            servo = {int(k): int(v) for k, v in c['pulses'].items()}
        elif c['kind'] == 'arm':
            servo[int(c['servo_id'])] = int(c['pulse'])
        elif c['kind'] == 'look':
            if in_look and t - last_pan_t >= DWELL_MIN_S:
                dwells += 1
                per_look[-1] += 1
            last_pan_t = t
            servo[6] = int(c['pan_pulse'])
        now = all(servo.get(k) == v for k, v in LOOK_P20.items())
        if now and not in_look:
            looks += 1
            t_in, last_pan_t = t, t
            per_look.append(0)
        elif in_look and not now:
            if t - last_pan_t >= DWELL_MIN_S:
                dwells += 1
                per_look[-1] += 1
            look_time += t - t_in
        in_look = now
    return {'look_sweeps': looks, 'look_dwells': dwells, 'look_time_s': round(look_time, 2),
            'dwells_per_look': per_look}


def search_stats(events: list[dict], condition: str) -> dict:
    sweeps = [e for e in events if e.get('event') == 'sweep_start' and e.get('purpose') == 'search']
    if condition == 'off':
        return {'search_sweeps': len(sweeps), 'search_dwells': len(sweeps)*SEARCH_PANS_OFF}
    plans = [e for e in events if e.get('event') == 'search_pans']
    reverify = [e for e in sweeps if e.get('reason') == 'reverify']
    return {'search_sweeps': len(sweeps), 'search_dwells': sum(len(e['pans']) for e in plans) + len(reverify),
            'skipped_search_pans': sum(len(e.get('skipped') or []) for e in plans),
            'skipped_viewpoints': sum(1 for e in events if e.get('event') == 'viewpoint_skipped'),
            'reverify_sweeps': len(reverify)}


def localization_error(frames_eval: list[dict], frames: list[dict], gate_frame_id) -> dict:
    ok = [r for r in frames_eval if 'pos_err_m' in r]
    carry = [r for r in ok if r.get('skill_phase') == 'nav_preplace']
    out = {'frames': len(ok),
           'pos_err_m_p50': pct([r['pos_err_m'] for r in ok], 50), 'pos_err_m_p90': pct([r['pos_err_m'] for r in ok], 90),
           'yaw_err_deg_p50': pct([r['yaw_err_deg'] for r in ok], 50),
           'yaw_err_deg_p90': pct([r['yaw_err_deg'] for r in ok], 90),
           'carry_pos_err_m_p90': pct([r['pos_err_m'] for r in carry], 90),
           'carry_yaw_err_deg_p90': pct([r['yaw_err_deg'] for r in carry], 90)}
    if gate_frame_id is not None:
        index = next((f['frame'] for f in frames if f['frame_id'] == gate_frame_id), None)
        row = next((r for r in ok if r['frame'] == index), None)
        if row:
            out.update(gate_pos_err_m=row['pos_err_m'], gate_yaw_err_deg=row['yaw_err_deg'])
    return out


def gt_boxes(spec: dict) -> list[dict]:
    """Setup-only initial box positions (scoring only)."""
    from sim.zone_arena import episode
    from sim.zone_landmarks import TAGGED_MAPS
    cfg = episode(TAGGED_MAPS[spec['map']]['base'], spec['seed'], goal=spec['goal'], extra_boxes=spec.get('extra_boxes'))
    return [{'id': k, 'kind': o['kind'], 'xy': o['position_m'][:2]} for k, o in sorted(cfg['setup_only']['objects'].items())]


def memory_checks(result: dict, events: list[dict], boxes: list[dict]) -> dict:
    ctl = result.get('controller') or {}
    target_box = min((b for b in boxes if b['kind'] == 'cyan'),
                     key=lambda b: math.dist(b['xy'], ctl.get('target_xy') or b['xy']), default=None)
    confirmed = [e for e in events if e.get('memory_event') == 'track_confirmed']
    false_tracks = []
    for e in confirmed:
        tr = e['track']
        same = [math.dist(tr['xy'], b['xy']) for b in boxes if b['kind'] == tr['kind']]
        d = min(same) if same else None
        if d is None or d > FALSE_TRACK_M:
            false_tracks.append({'track_id': tr['track_id'], 'kind': tr['kind'], 'xy': tr['xy'],
                                 'nearest_same_kind_m': None if d is None else round(d, 4), 't': e['t']})
    grid = ctl.get('memory_grid') or {}
    false_free = []
    if grid:
        x0, _, y0, _ = grid['bounds_m']
        nx = grid['shape'][1]
        cell = grid['cell_m']
        for idx, lo in zip(grid['cells'], grid['log_odds']):
            if lo > KNOWN_FREE:
                continue
            cx, cy = x0 + cell/2 + (idx % nx)*cell, y0 + cell/2 + (idx//nx)*cell
            for b in boxes:
                if b is target_box:
                    continue
                if math.dist((cx, cy), b['xy']) <= FREE_NEAR_BOX_M:
                    false_free.append({'cell': idx, 'xy': [round(cx, 3), round(cy, 3)], 'box': b['id'],
                                       'kind': b['kind'], 'log_odds': lo})
    names = ('driver_look', 'driver_look_skipped', 'driver_short_look_early_stop', 'driver_short_look_escalated',
             'driver_stale_fix_recheck', 'view_missing', 'look_plan', 'reverify', 'slot_check', 'track_new',
             'track_confirmed', 'track_absent')
    counts = {n: sum(1 for e in events if e.get('memory_event') == n) for n in names}
    modes = {}
    for e in events:
        if e.get('memory_event') == 'driver_look':
            modes[e['mode']] = modes.get(e['mode'], 0) + 1
    gate = ctl.get('gate_look_modes') or []
    return {'confirmed_tracks': len(confirmed), 'false_confirmed_tracks': false_tracks,
            'false_free_cells_near_boxes': false_free, 'event_counts': counts, 'driver_look_modes': modes,
            'gate_look_modes': {m: sum(1 for g in gate if g['mode'] == m) for m in ('short', 'full', 'skipped')},
            'early_stops_gate': sum(1 for e in events if e.get('event') == 'short_look_early_stop'),
            'memory_counts': (ctl.get('memory') or {}).get('counts')}


def episode_row(folder: Path, spec: dict, condition: str, attempt: str) -> dict:
    row = {'attempt': attempt, 'condition': condition, 'episode': spec['episode_id'], 'seed': spec['seed'],
           'split': spec['split'], 'folder': str(folder)}
    if not (folder/'result.json').exists():
        row['infrastructure_failure'] = True
        row['status'] = 'no result.json' + (' (attempt_started.json present)' if (folder/'attempt_started.json').exists()
                                            else '')
        return row
    result = json.loads((folder/'result.json').read_text())
    manifest = json.loads((folder/'manifest.json').read_text())
    runner = json.loads((folder/'memory_runner.json').read_text()) if (folder/'memory_runner.json').exists() else {}
    commands = rows(folder/'inputs'/'commands.jsonl')
    frames = rows(folder/'inputs'/'frames.jsonl')
    events = rows(folder/'controller_events.jsonl')
    frames_eval = rows(folder/'eval_only'/'frames_eval.jsonl')
    gate = result.get('lookback_gate') or {}
    row.update(infrastructure_failure=False, outcome=result['outcome'], m1_success=result['m1_success'],
               m1_failed_checks=result['m1_failed_checks'], false_success=result['false_success'],
               diagnostic_success=result['diagnostic_success'], sim_s=result['sim_s'], commands=result['commands'],
               frames=result['frames'], controller_schema=(result.get('controller') or {}).get('schema'),
               controller_class=runner.get('controller_class'), code_sha=manifest['code']['sha'],
               code_dirty=manifest['code']['dirty'], map=manifest['map_id'], static_map_sha256=manifest['static_map_sha256'],
               contact_profile=manifest['contact_profile']['profile'],
               noslip_iterations=manifest['contact_profile']['noslip_iterations'], weld=manifest['weld'],
               search_target_error_m=result['evaluation_only'].get('search_target_error_m'),
               slot_xy=result['evaluation_only'].get('slot_xy'),
               gt_box_final_xyz=result['evaluation_only'].get('gt_box_final_xyz'),
               contacts=result['evaluation_only'].get('contacts'),
               load_average=runner.get('load_average') or manifest.get('load_average'),
               wall_s=manifest.get('wall_s'), free_gib_before=runner.get('free_gib_before'),
               phase_times=result.get('phase_times'),
               **look_stats(commands), **search_stats(events, condition),
               localization=localization_error(frames_eval, frames, gate.get('frame_id')))
    if condition.startswith('memory'):
        row['memory'] = memory_checks(result, events, gt_boxes(spec))
        row['result_label'] = INTERIM.get(condition)
        row['landmark_provider'] = (result.get('controller') or {}).get('landmark_provider')
    return row


def compare(test_rows: list[dict], seeds: list[int]) -> dict:
    by = {(r['condition'], r['seed']): r for r in test_rows if not r.get('infrastructure_failure')}
    per_seed, both = [], []
    for s in seeds:
        on, off = by.get((TEST_ON, s)), by.get(('off', s))
        row = {'seed': s, 'off': None if off is None else {k: off[k] for k in ('outcome', 'm1_success', 'sim_s',
                                                                               'look_sweeps', 'look_dwells', 'look_time_s',
                                                                               'commands')},
               TEST_ON: None if on is None else {k: on[k] for k in ('outcome', 'm1_success', 'sim_s', 'look_sweeps',
                                                                        'look_dwells', 'look_time_s', 'commands')}}
        if on and off and on['m1_success'] and off['m1_success']:
            row['delta_sim_s'] = round(on['sim_s'] - off['sim_s'], 2)
            both.append(row)
        per_seed.append(row)
    succ = {c: sum(1 for s in seeds if by.get((c, s), {}).get('m1_success')) for c in ('off', TEST_ON)}
    false_on = sum(1 for s in seeds if by.get((TEST_ON, s), {}).get('false_success'))
    missing = [(c, s) for c in ('off', TEST_ON) for s in seeds if (c, s) not in by]
    claim = (not missing and succ[TEST_ON] >= succ['off'] and false_on == 0 and len(both) >= 3
             and all(r['delta_sim_s'] < 0 for r in both))
    return {'per_seed': per_seed, 'm1_success': succ, 'false_success_' + TEST_ON: false_on,
            'result_label': INTERIM[TEST_ON],
            'both_success_seeds': [r['seed'] for r in both],
            'delta_sim_s_both': [r['delta_sim_s'] for r in both], 'missing': missing,
            'claim_memory_faster': bool(claim),
            'claim_rule': 'memory_v2 m1_success >= off, memory_v2 false_success == 0, >= 3 seeds succeed in both, '
                          'memory_v2 faster on every one of them (prereg.json with amendment A1: memory_v1 -> memory_v2)'}


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument('--raw', required=True)
    p.add_argument('--attempt', action='append', default=[])
    p.add_argument('--output', default=str(HERE/'results.json'))
    args = p.parse_args(argv)
    raw = Path(args.raw)
    prereg = json.loads((HERE/'prereg.json').read_text())
    amend = HERE/'prereg_amendments.json'
    specs = {e['episode_id']: e for e in prereg['episodes']}
    attempts = args.attempt or sorted(p.name for p in raw.iterdir() if p.is_dir() and p.name not in ('logs',))
    out_rows, index = [], {}
    for attempt in attempts:
        for condition in CONDITIONS:
            base = raw/attempt/condition
            if not base.is_dir():
                continue
            for folder in sorted(p for p in base.iterdir() if p.is_dir()):
                if folder.name not in specs:
                    raise SystemExit(f'unregistered episode folder {folder}')
                out_rows.append(episode_row(folder, specs[folder.name], condition, attempt))
                for f in sorted(folder.rglob('*')):
                    if f.is_file() and f.suffix in ('.json', '.jsonl', '.xml'):
                        index[str(f.relative_to(raw))] = sha(f)
    test = [r for r in out_rows if r['split'] == 'test' and r['attempt'].startswith('test')]
    seeds = [e['seed'] for e in prereg['episodes'] if e['split'] == 'test']
    results = {'schema': SCHEMA, 'raw_root': str(raw), 'prereg_sha256': sha(HERE/'prereg.json'),
               'amendments_sha256': sha(amend) if amend.exists() else None,
               'attempts': attempts, 'episodes': out_rows,
               'test_comparison': compare(test, seeds) if test else None}
    Path(args.output).write_text(json.dumps(results, indent=1, ensure_ascii=False) + '\n')
    (Path(args.output).parent/'raw_index.json').write_text(json.dumps(
        {'schema': 'ugrp.raw_index.v1', 'raw_root': str(raw), 'files_sha256': index}, indent=1) + '\n')
    for r in out_rows:
        if r.get('infrastructure_failure'):
            print(r['attempt'], r['condition'], r['episode'], 'INFRA', r['status'])
            continue
        mem = r.get('memory') or {}
        print(f"{r['attempt']:8s} {r['condition']:9s} {r['episode']:15s} {r['outcome'][:34]:34s} m1={r['m1_success']!s:5s} "
              f"sim={r['sim_s']:7.1f} looks={r['look_sweeps']:2d} dwells={r['look_dwells']:3d} "
              f"look_t={r['look_time_s']:6.1f} search={r['search_sweeps']}/{r['search_dwells']} cmds={r['commands']} "
              f"err90={r['localization']['pos_err_m_p90']} false_tracks={len(mem.get('false_confirmed_tracks', []))}")
    if results['test_comparison']:
        print(json.dumps({k: v for k, v in results['test_comparison'].items() if k != 'per_seed'}))


if __name__ == '__main__':
    main()
