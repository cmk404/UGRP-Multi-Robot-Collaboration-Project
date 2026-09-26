"""POST HOC failure analysis of the environment-v3 loop test cohort (evaluation-only ground truth).

Nothing here is a gate or a robot input. It reads finished run folders only and writes
loop/failure_analysis.json:

* per wall contact event: geoms, SIM time, student state, GT pose, estimate error and the
  tags seen in that frame (grouped into events 1 s apart);
* the loaded door-approach strip (GT x 1.80-2.05, y -1.30..-0.30, the path along the
  divider west face): frames, frames without any tag, mean estimate error, and the tag
  sightings by wall, for the v3 test runs and the loop-v2 test runs on the v2 map (baseline);
* the door region (GT |x-2.2| <= 0.6, |y-0.05| <= 0.5) per x band: frames without tag;
* the look count and the SIM time per student state.

  python analyze_failures.py --v3 <outputs/zone-env-v3-20260926/loop/test> \
      --v2 <outputs/owncam-loop-v2-20260926/test> --output loop/failure_analysis.json
"""
from __future__ import annotations

import argparse
import collections
import json
import math
import statistics
from pathlib import Path

X = Path(__file__).resolve().parent
ROOT = X.parents[1]
if str(ROOT) not in __import__('sys').path:
    __import__('sys').path.insert(0, str(ROOT))
STRIP = {'x': (1.80, 2.05), 'y': (-1.30, -.30)}
BANDS = ((1.6, 2.0), (2.0, 2.2), (2.2, 2.45), (2.45, 2.8))


def jsonl(path):
    return [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]


def tags_of(map_id):
    static = json.loads((ROOT/'maps'/'zones'/f'{map_id}.json').read_text())
    return {int(t['id']): t for t in static['landmarks']['tags']}


def wrap_deg(a):
    return math.degrees((a + math.pi) % (2*math.pi) - math.pi)


def contact_events(run, ev, log):
    ts = [e['t'] for e in ev]
    states = [(e['t'], e['state'], e.get('reason')) for e in log if e.get('event') == 'state']
    events, last = [], None
    for c in jsonl(run/'eval_only'/'contacts.jsonl'):
        if c.get('kind') != 'wall' or c.get('phase') != 'student':
            continue
        if last is not None and c['t'] - last['t_end'] <= 1.0 and last['geoms'] == c['geoms']:
            last['t_end'], last['samples'] = round(c['t'], 2), last['samples'] + 1
            continue
        import bisect
        i = min(bisect.bisect_left(ts, c['t']), len(ev) - 1)
        e = ev[i]
        st = [s for s in states if s[0] <= c['t']]
        last = {'t_start': round(c['t'], 2), 't_end': round(c['t'], 2), 'samples': 1, 'geoms': c['geoms'],
                'student_state': st[-1][1] if st else None,
                'gt_xy_yaw': [round(e['gt'][0], 3), round(e['gt'][1], 3), round(math.degrees(e['gt'][2]), 1)],
                'est_err_m': e.get('pos_err_m'), 'tags_in_frame': e['tags']}
        events.append(last)
    return events


def strip_stats(ev, tags):
    rows = [e for e in ev if e['phase'] == 'student' and 'pos_err_m' in e
            and STRIP['x'][0] <= e['gt'][0] < STRIP['x'][1] and STRIP['y'][0] <= e['gt'][1] < STRIP['y'][1]]
    by_wall = collections.Counter()
    for e in rows:
        for t in e['tags']:
            by_wall[tags[int(t)]['wall']] += 1
    return {'frames': len(rows), 'frames_without_tag': sum(not e['tags'] for e in rows),
            'mean_err_m': round(statistics.mean(e['pos_err_m'] for e in rows), 4) if rows else None,
            'max_err_m': round(max(e['pos_err_m'] for e in rows), 4) if rows else None,
            'tag_sightings_by_wall': dict(by_wall.most_common())}


def door_bands(ev):
    rows = [e for e in ev if e['phase'] == 'student' and e['door_region'] and 'pos_err_m' in e]
    out = {}
    for lo, hi in BANDS:
        band = [e for e in rows if lo <= e['gt'][0] < hi]
        if band:
            out[f'x{lo:.2f}-{hi:.2f}'] = {
                'frames': len(band), 'frames_without_tag': sum(not e['tags'] for e in band),
                'mean_dy_m': round(statistics.mean(e['est'][1] - e['gt'][1] for e in band), 4),
                'mean_dyaw_deg': round(statistics.mean(wrap_deg(e['est'][2] - e['gt'][2]) for e in band), 3)}
    return out


def state_time(log):
    total, prev = collections.Counter(), None
    for e in log:
        if e.get('event') == 'state':
            if prev:
                total[prev[1]] += e['t'] - prev[0]
            prev = (e['t'], e['state'])
    return {k: round(v, 1) for k, v in total.most_common()}


def v2_sighting_fate(ev, v2_tags, v3_static):
    """Classify every v2-map tag sighting (loop-v2 baseline) by what v3 would leave of it.

    hidden_by_0.40m_walls: the 2-D sight line from the camera to the tag crosses another
      wall footprint; with 0.40 m walls (camera <= 0.321 m, tags <= 0.25 m) it is blocked.
    removed_by_rule: sight line clear, but v3 has no tag column at that wall position.
    kept_in_v3: sight line clear and a v3 site stands within 0.10 m on the same face.
    Camera xy = GT base + 0.12 m along the heading (approximate LOOK/CARRY camera offset).
    Frames count as 'still tagged' if at least one sighting is kept.
    """
    from sim.zone_tag_rule_v3 import line_of_sight
    sites = [(s['wall'], tuple(s['normal_xy']), s['center_m']) for s in v3_static['landmarks']['sites']]
    out = {}
    regions = {'loaded_strip': lambda e: STRIP['x'][0] <= e['gt'][0] < STRIP['x'][1] and STRIP['y'][0] <= e['gt'][1] < STRIP['y'][1],
               **{f'door_x{lo:.2f}-{hi:.2f}': (lambda lo, hi: lambda e: e['door_region'] and lo <= e['gt'][0] < hi)(lo, hi)
                  for lo, hi in BANDS}}
    for name, inside in regions.items():
        rows = [e for e in ev if e['phase'] == 'student' and inside(e)]
        fate, still = collections.Counter(), 0
        for e in rows:
            x, y, yaw = e['gt']
            cam = (x + .12*math.cos(yaw), y + .12*math.sin(yaw))
            kept = False
            for t in e['tags']:
                tag = v2_tags[int(t)]
                nx, ny = tag['normal_xy']
                front = (tag['center_m'][0] + .01*nx, tag['center_m'][1] + .01*ny)
                if not line_of_sight(v3_static, cam, front):
                    fate['hidden_by_0.40m_walls'] += 1
                elif any(w == tag['wall'] and n == (nx, ny) and math.dist(c, tag['center_m'][:2]) <= .10
                         for w, n, c in sites):
                    fate['kept_in_v3'] += 1
                    kept = True
                else:
                    fate['removed_by_rule'] += 1
            still += kept
        out[name] = {'frames': len(rows), 'frames_with_tag_v2': sum(bool(e['tags']) for e in rows),
                     'frames_still_tagged_in_v3': still, 'sightings': dict(fate)}
    return out


def run_entry(run):
    result = json.loads((run/'result.json').read_text())
    manifest = json.loads((run/'manifest.json').read_text())
    tags = tags_of(manifest['map_id'])
    ev = jsonl(run/'eval_only'/'frames_eval.jsonl')
    log = jsonl(run/'student_log.jsonl')
    entry = {'episode': result['episode'], 'map_id': manifest['map_id'], 'condition': result['condition'],
             'pass': result['episode_pass'], 'outcome': result['outcome'], 'looks': result['looks'],
             'look_reasons': dict(collections.Counter(result['look_reasons'])),
             'student_sim_s': result['student_sim_s'], 'sim_s_by_state': state_time(log),
             'wall_contact_events': contact_events(run, ev, log), 'loaded_strip': strip_stats(ev, tags),
             'door_bands': door_bands(ev)}
    student = [e for e in ev if e['phase'] == 'student' and 'pos_err_m' in e]
    if student:
        entry['handover'] = {'t': student[0]['t'], 'gt_xy': [round(v, 3) for v in student[0]['gt'][:2]],
                             'est_err_m': student[0]['pos_err_m'], 'std_xy_m': student[0].get('std_xy_m')}
    if manifest['map_id'] == 'zone_wide_door_tags_v2':
        v3 = json.loads((ROOT/'maps'/'zones'/'zone_wide_door_tags_v3.json').read_text())
        entry['v2_sightings_under_v3'] = v2_sighting_fate(ev, tags, v3)
    return entry


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('--v3', type=Path, required=True)
    p.add_argument('--v2', type=Path, required=True)
    p.add_argument('--a1', type=Path, default=None, help='amendment A1 test folder (loop/a1-test)')
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args(argv)
    out = {'schema': 'ugrp.zone_env_v3.failure_analysis.v1', 'posthoc': True,
           'note': 'evaluation-only ground truth; diagnostics, not gates, never robot input',
           'strip': STRIP, 'v3_test': [], 'v2_baseline_test': []}
    groups = [('v3_test', a.v3, 'envtest-*'), ('v2_baseline_test', a.v2, 'v2test-*')]
    if a.a1 is not None:
        out['v3a1_test'] = []
        groups.append(('v3a1_test', a.a1, 'a1test-*'))
    for key, root, pattern in groups:
        for run in sorted(root.glob(pattern)):
            if (run/'result.json').exists():
                out[key].append(run_entry(run))
    total = {}
    for r in out['v2_baseline_test']:
        for region, v in r['v2_sightings_under_v3'].items():
            t = total.setdefault(region, {'frames': 0, 'frames_with_tag_v2': 0, 'frames_still_tagged_in_v3': 0,
                                          'sightings': collections.Counter()})
            for k in ('frames', 'frames_with_tag_v2', 'frames_still_tagged_in_v3'):
                t[k] += v[k]
            t['sightings'].update(v['sightings'])
    out['v2_sightings_under_v3_total'] = {k: {**v, 'sightings': dict(v['sightings'])} for k, v in total.items()}
    a.output.write_text(json.dumps(out, indent=1, ensure_ascii=False) + '\n')
    for region, v in out['v2_sightings_under_v3_total'].items():
        print(region, v)
    for key in ('v3_test', 'v3a1_test', 'v2_baseline_test'):
        for r in out.get(key, []):
            s = r['loaded_strip']
            print(f"{r['episode']:20s} pass={r['pass']!s:5s} contacts={len(r['wall_contact_events'])} "
                  f"strip frames={s['frames']} no_tag={s['frames_without_tag']} mean_err={s['mean_err_m']} "
                  f"handover_err={r.get('handover', {}).get('est_err_m')}")


if __name__ == '__main__':
    main()
