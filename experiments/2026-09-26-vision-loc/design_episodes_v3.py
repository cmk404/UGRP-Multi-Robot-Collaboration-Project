#!/usr/bin/env python3
"""Design the round-3 evaluation episodes (``episodes_v3.json``) BEFORE any render.

Why: round-2 seeds were distinct but the episodes were not. Test ``vl-test-s917``
re-drove train ``vl-train-s903`` (same spawn row, slot and cyan pickup cell; the
GT teacher and the physics are deterministic), and every episode sharing a spawn
row shared its first leg exactly. Door passages to one slot converged within
1 mm (``results/overlap_round2_vs_train.json``). Round 3 therefore changes the
trajectories themselves, not only the seeds:

1. scenario: new seeds; each assignment (spawn row, destination slot) takes the
   first seed from its range whose cyan pickup cell (``sim.zone_arena.episode``,
   the setup-only scenario config) gives a (spawn row, cyan cell) pair and a
   (cyan cell, slot) pair used by no round-2 episode (train, dev or test) and by
   no round-3 episode designed before it;
2. start pose: a setup-only spawn offset of the own robot (dx, dy, dyaw), drawn
   per seed; the student's prior stays the nominal dock (within its std);
3. path: a constant teacher pose bias (DART-style noise injection, the teacher
   believes truth + bias), 1.2-3.0 cm in a random direction and up to 1 deg.

Only the scenario config is read (no simulator state, no render). The rendered
episodes are then audited with ``overlap_check.py`` (JPEG bytes, replay poses,
aligned trajectories); the exclusion rule is registered here, before rendering.
"""
from __future__ import annotations

import argparse
import json
import math
import random
from pathlib import Path

HERE = Path(__file__).resolve().parent
SCHEMA = 'ugrp.vision_loc_episodes.v2'
EXTRA_BOXES = {'red': 2, 'green': 1}
DEV2 = ((.55, 'C2'), (-.85, 'A2'), (-2.25, 'B3'), (.55, 'A1'), (-.85, 'C3'), (-2.25, 'B2'))
TEST2 = ((.55, 'B1'), (-.85, 'A1'), (-2.25, 'A2'), (.55, 'C3'), (-.85, 'B3'), (-2.25, 'C2'), (.55, 'A3'), (-.85, 'C1'))
SEED_RANGES = {'dev': range(941, 951), 'test': range(951, 971)}
SPAWN_OFFSET = {'dx_m': (-.04, .06), 'dy_m': (-.08, .08), 'dyaw_deg': (-6., 6.)}
BIAS = {'r_m': (.012, .030), 'dyaw_deg': (-1., 1.)}


def cyan_cell(seed: int, goal: dict) -> tuple[float, float]:
    from sim.zone_arena import episode
    cfg = episode('zone_wide_door', seed, goal=goal, extra_boxes=EXTRA_BOXES)
    return next(tuple(round(v, 3) for v in o['position_m'][:2]) for o in cfg['setup_only']['objects'].values()
                if o['kind'] == 'cyan')


def spawn_offset(seed: int) -> list[float]:
    r = random.Random(f'vl3-spawn-{seed}')
    return [round(r.uniform(*SPAWN_OFFSET['dx_m']), 4), round(r.uniform(*SPAWN_OFFSET['dy_m']), 4),
            round(r.uniform(*SPAWN_OFFSET['dyaw_deg']), 3)]


def pose_bias(seed: int) -> list[float]:
    r = random.Random(f'vl3-bias-{seed}')
    rad, th = r.uniform(*BIAS['r_m']), r.uniform(0., 2*math.pi)
    return [round(rad*math.cos(th), 4), round(rad*math.sin(th), 4), round(r.uniform(*BIAS['dyaw_deg']), 3)]


def design(round2: dict) -> dict:
    used_spawn_cell, used_cell_slot = set(), set()
    for e in round2['episodes']:
        cell = cyan_cell(e['seed'], e['goal'])
        used_spawn_cell.add((e['spawn_y'], cell))
        used_cell_slot.add((cell, e['slot_id']))
    episodes, rejected = [], []
    for split, plan in (('dev', DEV2), ('test', TEST2)):
        seeds = iter(SEED_RANGES[split])
        for spawn_y, slot in plan:
            goal = {slot[0]: {'cyan': 1}}
            for seed in seeds:
                cell = cyan_cell(seed, goal)
                why = [k for k, bad in (('spawn_row_and_cyan_cell_used', (spawn_y, cell) in used_spawn_cell),
                                        ('cyan_cell_and_slot_used', (cell, slot) in used_cell_slot)) if bad]
                if why:
                    rejected.append({'seed': seed, 'split': split, 'spawn_y': spawn_y, 'slot_id': slot,
                                     'cyan_cell': list(cell), 'reasons': why})
                    continue
                used_spawn_cell.add((spawn_y, cell))
                used_cell_slot.add((cell, slot))
                episodes.append({'episode_id': f'vl3-{split}-s{seed}', 'split': split,
                                 'role': 'pf_dev_round3' if split == 'dev' else 'test_once_round3', 'seed': seed,
                                 'spawn_y': spawn_y, 'slot_id': slot, 'goal': goal, 'extra_boxes': dict(EXTRA_BOXES),
                                 'cyan_cell': list(cell), 'spawn_offset': spawn_offset(seed),
                                 'teacher_pose_bias': pose_bias(seed)})
                break
            else:
                raise SystemExit(f'no seed in {SEED_RANGES[split]} for {split} {spawn_y} {slot}')
    return {'episodes': episodes, 'rejected_seeds': rejected}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--output', default=str(HERE/'episodes_v3.json'))
    args = ap.parse_args(argv)
    out = Path(args.output)
    if out.exists():
        raise SystemExit(f'refusing to overwrite {out} (registered before rendering)')
    round2 = json.loads((HERE/'episodes.json').read_text())
    d = design(round2)
    table = {
        'schema': SCHEMA,
        'registered': '2026-09-26, Kiro, round 3: before any round-3 render, filter change evaluation or scoring',
        'base_map': round2['base_map'], 'map': round2['map'], 'controller': round2['controller'],
        'design_rule': __doc__.split('\n\n', 1)[1].strip(),
        'design_parameters': {'dev_plan': [list(p) for p in DEV2], 'test_plan': [list(p) for p in TEST2],
                              'seed_ranges': {k: [v.start, v.stop - 1] for k, v in SEED_RANGES.items()},
                              'spawn_offset': SPAWN_OFFSET, 'teacher_pose_bias': BIAS,
                              'rng': "random.Random('vl3-spawn-<seed>') / random.Random('vl3-bias-<seed>')"},
        'units': {'spawn_offset': '[dx m, dy m, dyaw deg] added to the own robot spawn (setup-only)',
                  'teacher_pose_bias': '[dx m, dy m, dyaw deg] added to the truth the teacher controller receives'},
        'independence': {
            'tool': 'overlap_check.py (JPEG bytes, replay poses within 1 mm / 0.1 deg with the same commanded pulses, '
                    'aligned trajectories)',
            'references': 'dev: every round-2 episode (train, dev, test); test: every round-2 episode and every '
                          'round-3 dev episode',
            'rule': 'a round-3 episode that is not independent is excluded from its split before any '
                    'segmentation, localization or scoring of it; the exclusion is recorded'},
        'splits': {'train': 'round-2 train only (vl-train-s901..s908), model and calibration unchanged',
                   'dev': 'round-2 dev (vl-dev-s909..s911) + round-3 dev (vl3-dev-*); round-2 test s912-s917 is a '
                          'post-hoc diagnosis record (contaminated: s917 = train s903) and is not used for tuning',
                   'test': 'round-3 test (vl3-test-*) only, scored once after prereg_v3.json is committed'},
        'episodes': d['episodes'], 'rejected_seeds': d['rejected_seeds']}
    out.write_text(json.dumps(table, indent=1) + '\n')
    for e in d['episodes']:
        print(e['episode_id'], e['spawn_y'], e['slot_id'], e['cyan_cell'], e['spawn_offset'], e['teacher_pose_bias'])
    print(f"rejected seeds: {[r['seed'] for r in d['rejected_seeds']]}")


if __name__ == '__main__':
    main()
