"""Assemble results.json from the local analysis outputs (observer-only)."""
import hashlib
import json
import statistics as st
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
O = ROOT/'outputs/zone-hard-routes-20260925'
E = Path(__file__).resolve().parent


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def src(run_dir):
    return json.load(open(run_dir/'result.json'))['source_sha']


def agg(group):
    out = {}
    for r in group:
        out.setdefault(f"{r['variant']}|{r['goal']}|{r['coordination']}", []).append(r)
    res = {}
    for k, g in sorted(out.items()):
        res[k] = {'runs': len(g), 'seeds': sorted(r['seed'] for r in g),
                  'goal_met_referee': sum(r['goal_met_referee'] for r in g),
                  'goal_met_rgb': sum(r['goal_met_rgb'] for r in g),
                  'makespan_sim_s_mean': round(st.mean(r['makespan_sim_s'] for r in g), 1),
                  'makespan_sim_s': [r['makespan_sim_s'] for r in g],
                  'teacher_path_blocked': sum(r['teacher_path_blocked'] for r in g),
                  'generic_yields': sum(r['generic_yields'] for r in g),
                  'passage_standoffs': sum(r['passage_standoffs'] for r in g),
                  'passage_wait_s': round(sum(r['door_wait_s'] for r in g), 1),
                  'drive_wait_near_passage_s': round(sum(r['drive_wait_near_passage_s'] for r in g), 1),
                  'robot_robot_contact_s': round(sum(float(r['contacts']['robot_robot']['seconds']) for r in g), 2),
                  'robot_wall_contact_episodes': sum(r['contacts']['robot_wall']['episodes'] for r in g),
                  'box_taken_by_peer': sum(r['box_taken_by_peer'] for r in g)}
    return res


def clean(r):
    r = dict(r)
    r['contacts'] = {k: {'episodes': v['episodes'], 'seconds': float(v['seconds'])} for k, v in r['contacts'].items()}
    return r


def main():
    rows = [r for r in json.load(open(O/'matrix/analysis.json')) if 'goal_met_referee' in r]
    base = json.load(open(O/'wide-baseline/analysis.json'))
    for r in rows:
        r['source_sha'] = src(O/'matrix'/r['run'])
    for r in base:
        r['source_sha'] = src(O/'wide-baseline'/r['run'])
    ident = {f: {'origin_main_3cbf4e4': sha(O/'base-v1-wide-G8-dyn-s12'/f), 'branch_c6cd334': sha(O/'ident-v1-wide-G8-dyn-s12'/f)}
             for f in ('teacher-events.json', 'scene.xml', 'box-labels.json', 'replay/states.npz')}
    media = json.load(open(O/'matrix/media.json'))
    videos = json.load(open(O/'media/videos.json'))
    value = {
        'experiment': '2026-09-25-zone-hard-routes', 'branch': 'claude/zone-hard-routes',
        'claim_scope': ('teacher-executor + scripted fixture (LLM 0 calls): route feasibility and traffic on new maps; '
                        'not evidence of LLM coordination, not an RGB-skill result'),
        'sources': {'retire_zone_open': '8935ed9', 'routes_and_rules': 'c6cd334', 'matrix_driver': 'dc27695',
                    'route_drive_limit_240s': '3183650', 'door_and_two_doors_runs_source': 'dc27695',
                    'corridor_runs_source': '3183650', 'wide_baseline_source': 'dc27695',
                    'note': ('dc27695 -> 3183650 changes only the drive-phase limit on maps with interior walls '
                             '(120 -> 240 s); no door/two_doors run reached it (teacher_path_blocked = 0 in all), '
                             'so their behaviour is identical under 3183650')},
        'conditions': {'mode': 'fixture (scripted protocol replies, LLM 0 calls)', 'sim': 'synchronous',
                       'contact_profile': 'local_contact_fine', 'weld': 'off', 'record_replay': True,
                       'goals': {'G5': {'goal': {'A': {'red': 2}, 'B': {'cyan': 1}, 'C': {'green': 1, 'red': 1}}, 'extra': {}},
                                 'G8': {'goal': {'A': {'red': 2}, 'B': {'cyan': 2}, 'C': {'green': 1, 'yellow': 1}},
                                        'extra': {'red': 1, 'cyan': 1}}},
                       'coordination': ['plan_first', 'dynamic', 'independent'],
                       'seeds': {'zone_wide_door': [11, 12, 13], 'zone_wide_two_doors': [11, 12],
                                 'zone_wide_corridor': [11, 12], 'zone_wide (baseline)': [11, 12, 13]}},
        'raw_root': '/Users/changmin/projects/ugrp-worktrees/zone-hard-routes/outputs/zone-hard-routes-20260925 (local only, gitignored)',
        'v1_identity_sim': {'run': 'zone_wide G8 dynamic seed 12 fixture', 'sha256': ident,
                            'identical': all(len(set(v.values())) == 1 for v in ident.values()),
                            'teacher_events_matches_ZC2_gate2_dynamic_nominal':
                                ident['teacher-events.json']['branch_c6cd334'].startswith('ae804c28c0d2ec67')},
        'v1_identity_tests': {'planner_digest': 'ee5961d94ad6c143d50cf3ec2fee7969440215d9527570a2b2f987307d88f455',
                              'map_sha256': {'zone_open': '2057e40536f0f71077ff362ee127b36c6a4a486fca409d92cfb348a3a8c1c1e0',
                                             'zone_wide': '6b7b9eb671e5114c78b52e1ee0a4cc02288038aa04edfe37ab4f908bbac6f852'}},
        'map_sha256': {v: sha(ROOT/f'maps/zones/{v}.json') for v in ('zone_wide_door', 'zone_wide_two_doors', 'zone_wide_corridor')},
        'aggregates_routes': agg(rows), 'aggregates_zone_wide_baseline': agg(base),
        'runs': [clean(r) for r in rows], 'baseline_runs': [clean(r) for r in base],
        'not_run': {'runs': sorted(p.name for p in (O/'matrix').iterdir() if (p/'NOT-RUN.txt').is_file()),
                    'reason': ('host overload by concurrent agents (1-min load 240-490); seed 13 dropped for two_doors '
                               'and corridor, placeholder folders block the runner')},
        'superseded': {'runs': ['corr-G5-plan_first-s11', 'corr-G5-dynamic-s11'], 'source': 'dc27695',
                       'where': 'raw_root/matrix-superseded',
                       'reason': ('corr-G5-plan_first-s11 ended teacher_path_blocked 1.3 m short of the B slot by the 120 s '
                                  'drive limit after 42 s queueing at the corridor; rerun under 3183650 (240 s)'),
                       'results': {'corr-G5-plan_first-s11': 'goal not met, 4/5 placed, 1 teacher_path_blocked',
                                   'corr-G5-dynamic-s11': 'goal met'}},
        'aborted': {'runs': sorted(p.name for p in (O/'matrix-aborted').iterdir() if p.is_dir()),
                    'reason': ('stopped at 1.5-10 min wall to cap concurrency at 2 (coordinator) or when the drive limit '
                               'was changed; no result, rerun from scratch')},
        'loads': {'rule': ('os.getloadavg() 1-min at each run start and end, in runs[]; sync SIM results do not depend on '
                           'load'), 'agent_lock': 'no realtime/training lock at start (zone-comm-audit offline-test lock only)'},
        'contacts_method': ('observer-only: replay qpos -> mj_forward at 10 Hz, penetrating contacts (dist < 0) between '
                            'geoms of different robots, robot and zone_wall_* geoms, cargo box and wall'),
        'ci': {'command': 'scripts/run_ci_tests.py at cefc41d (after merging origin/main 23fcaaa)',
               'result': '2350 passed, 9 skipped, 1 xfailed, 205 subtests passed'},
        'media': {'maps': media['maps'], 'videos': {k: {**v, 'path': f'media/{k}.mp4'} for k, v in videos.items()},
                  'png_sha256': {p.name: sha(p) for p in sorted(E.glob('*.png'))}},
        'known_issues': [
            ('corr-G5-independent-s12: r1 (to_box) and r2 (carry) met just outside the corridor east exit '
             '(x 4.15-4.40, y 0.48-0.78) and stayed in contact 199-400 s; only r2 was inside the passage zone, so no '
             "standoff, and the planner's no-closer peer rule kept paths for both, so the generic yield never fired. "
             'Both jobs ended teacher_path_blocked; goal not met. Same mechanism without walls in the zone_wide baseline '
             'wide-G8-independent-s12 (37 s contact near C). Not fixed here.'),
            ('robot-robot sub-mm touches at 9.6-14 s in every seed-11 G5 run (all maps incl. zone_wide), pickup area '
             'near spawns: pre-existing.'),
            ('independent mode misses the goal in most runs on every map (box_taken_by_peer / over-fill), as in ZC2; '
             'not a route effect. _taken_by_peer intention-based arbitration (audit PR #161 L1) is left for the '
             'follow-up task.')],
    }
    (E/'results.json').write_text(json.dumps(value, indent=2, ensure_ascii=False) + '\n')
    return value


if __name__ == '__main__':
    v = main()
    for name in ('aggregates_routes', 'aggregates_zone_wide_baseline'):
        for k, a in v[name].items():
            print(k, f"{a['goal_met_referee']}/{a['runs']}", a['makespan_sim_s_mean'], a['passage_standoffs'],
                  a['generic_yields'], a['passage_wait_s'], a['teacher_path_blocked'], a['robot_robot_contact_s'],
                  a['robot_wall_contact_episodes'])
    print(v['v1_identity_sim']['identical'], v['v1_identity_sim']['teacher_events_matches_ZC2_gate2_dynamic_nominal'],
          len(v['runs']), len(v['baseline_runs']))
