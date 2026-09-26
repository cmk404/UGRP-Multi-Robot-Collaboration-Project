"""Write frozen_source.json for the memory ON/OFF test split (clean tree, AFTER the dev check, BEFORE the test).

``scripts/run_m1_owncam_memory.py --split test --frozen <this file>`` refuses to start
unless HEAD differs from source_sha only in records_only_paths, the tree is clean and
every hashed file (the frozen M1 runtime + the memory files + calibration + prereg)
matches.
"""
import hashlib
import json
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT))
from scripts.run_m1_owncam import RUNTIME_FILES  # noqa: E402
from scripts.run_m1_owncam_memory import FROZEN_PATHS, MEMORY_FILES  # noqa: E402

EXP = 'experiments/2026-09-26-zone-owncam-memory/'
RECORDS_ONLY = [EXP + f for f in ('frozen_source.json', 'results.json', 'raw_index.json', 'README.md',
                                  'build_results.py', 'build_tensorboard.py', 'tensorboard_verify.json',
                                  'box_bias_check.py', 'box_bias_check.json', 'planner_equivalence_check.py',
                                  'planner_equivalence_check.json', 'launch_test.sh')]


def git(*a):
    return subprocess.run(['git', *a], cwd=ROOT, capture_output=True, text=True, check=True).stdout.strip()


def main():
    if git('status', '--porcelain', '--', *FROZEN_PATHS):
        raise SystemExit('refuse: tree not clean')
    prereg = json.loads((HERE/'prereg.json').read_text())
    student = prereg['student']
    files = (list(RUNTIME_FILES) + list(MEMORY_FILES) +
             [student['calibration'], EXP + 'prereg.json',
              'experiments/2026-09-26-zone-owncam-loop-v2/calibration_loop_v2.json',
              'maps/zones/zone_wide_door_tags_v1.json'])
    amendments = HERE/'prereg_amendments.json'
    if amendments.exists():
        files.append(EXP + 'prereg_amendments.json')
    out = {'schema': 'ugrp.m1_owncam_memory_frozen.v1', 'source_sha': git('rev-parse', 'HEAD'),
           'frozen_note': 'source frozen after the dev check, before any test episode of either condition ran',
           'student': {k: student.get(k) for k in ('skill', 'calibration', 'contact_profile')},
           'conditions': sorted(prereg['conditions']),
           'test_episodes': [e['episode_id'] for e in prereg['episodes'] if e['split'] == 'test'],
           'records_only_paths': RECORDS_ONLY,
           'sha256': {f: hashlib.sha256((ROOT/f).read_bytes()).hexdigest() for f in dict.fromkeys(files)}}
    (HERE/'frozen_source.json').write_text(json.dumps(out, indent=2) + '\n')
    print('frozen at', out['source_sha'][:9], len(out['sha256']), 'files')


if __name__ == '__main__':
    main()
