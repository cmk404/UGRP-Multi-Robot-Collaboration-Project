"""Offline check: the landmark-agnostic planner (memory v2 + INTERIM tag provider) vs memory v1.

A1 moved every tag-specific piece of memory_v1 (commit ad78ef2) behind the landmark
provider interface; "where to look" is now computed from the static-map catalogue
(door posts, wall corners, door gap, wall faces) weighted by the provider's support.
This script loads memory_v1 from git (``git show ad78ef2:harness/owncam_memory.py``),
plans a look for the same random own estimates with both, and records whether the
look mode (short / full) and the first pan agree. No simulator, no GT, no episode data.

  python experiments/2026-09-26-zone-owncam-memory/planner_equivalence_check.py
"""
from __future__ import annotations

import hashlib
import importlib.util
import json
import math
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT))

from harness.owncam_landmark_tags import TagLandmarkProvider  # noqa: E402
from harness.owncam_memory import OwnCamMemory  # noqa: E402

V1_SHA = 'ad78ef2bafdcd3f1669aa0d8c2eb80f5cb01f151'
N = 400


def main():
    src = subprocess.run(['git', 'show', f'{V1_SHA}:harness/owncam_memory.py'], cwd=ROOT, capture_output=True,
                         text=True, check=True).stdout
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp)/'owncam_memory_v1.py'
        path.write_text(src)
        spec = importlib.util.spec_from_file_location('owncam_memory_v1', path)
        v1 = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(v1)
    static = json.loads((ROOT/'maps/zones/zone_wide_door_tags_v1.json').read_text())
    params = json.loads((ROOT/'experiments/2026-09-26-zone-m1-owncam/calibration_m1_dev.json').read_text())['params']
    m1 = v1.OwnCamMemory(static, params, robot_id='r1')
    m2 = OwnCamMemory(static, params, robot_id='r1', provider=TagLandmarkProvider(static, params))
    rng = np.random.default_rng(20260926)
    rows = []
    for k in range(N):
        loaded = bool(k % 2)
        x = rng.uniform(-.8, 5.1) if loaded else rng.uniform(-.8, 2.)
        est = {'initialized': True, 'x': float(x), 'y': float(rng.uniform(-2.9, 1.2)),
               'yaw': float(rng.uniform(-math.pi, math.pi)),
               'cov': np.diag([rng.uniform(.0003, .003)]*2 + [rng.uniform(.0005, .005)]).tolist()}
        a = m1.plan_look(est, loaded=loaded, now=1., reason='check')
        b = m2.plan_look(est, loaded=loaded, now=1., reason='check')
        rows.append({'loaded': loaded, 'v1': (a['mode'], a['pans']), 'v2': (b['mode'], b['pans'])})
    same_mode = sum(r['v1'][0] == r['v2'][0] for r in rows)
    both_short = [r for r in rows if r['v1'][0] == r['v2'][0] == 'short']
    same_first = sum(r['v1'][1][:1] == r['v2'][1][:1] for r in both_short)
    same_set = sum(set(r['v1'][1]) == set(r['v2'][1]) for r in both_short)
    out = {'schema': 'ugrp.owncam_memory_planner_equivalence.v1', 'v1_sha': V1_SHA,
           'v2_files_sha256': {f: hashlib.sha256((ROOT/f).read_bytes()).hexdigest()
                               for f in ('harness/owncam_memory.py', 'harness/owncam_landmarks.py',
                                         'harness/owncam_landmark_tags.py')},
           'map': 'zone_wide_door_tags_v1', 'estimates': N, 'same_mode': same_mode, 'both_short': len(both_short),
           'same_first_pan': same_first, 'same_pan_set': same_set,
           'mode_pairs': {f'{m1_}/{m2_}': sum(1 for r in rows if (r['v1'][0], r['v2'][0]) == (m1_, m2_))
                          for m1_ in ('short', 'full') for m2_ in ('short', 'full')},
           'result_label': 'interim, tag provider'}
    (HERE/'planner_equivalence_check.json').write_text(json.dumps(out, indent=1) + '\n')
    print(json.dumps(out))


if __name__ == '__main__':
    main()
