"""Write frozen_source.json for the M1 test split (run on a clean tree AFTER the dev check, BEFORE the test).

The runner's --split test refuses to start unless HEAD differs from source_sha only
in records_only_paths, the tree is clean and every hashed file matches.
"""
import hashlib
import json
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT))
from scripts.run_m1_owncam import FROZEN_PATHS, RUNTIME_FILES, effective_prereg  # noqa: E402

RECORDS_ONLY = ['experiments/2026-09-26-zone-m1-owncam/' + f for f in
                ('frozen_source.json', 'results.json', 'raw_index.json', 'README.md')]


def git(*a):
    return subprocess.run(['git', *a], cwd=ROOT, capture_output=True, text=True, check=True).stdout.strip()


def main():
    if git('status', '--porcelain', '--', *FROZEN_PATHS):
        raise SystemExit('refuse: tree not clean')
    prereg = HERE/'prereg.json'
    student, episodes = effective_prereg(prereg, json.loads(prereg.read_text()))
    files = list(RUNTIME_FILES) + [student['calibration'], 'experiments/2026-09-26-zone-m1-owncam/prereg.json',
                                   'experiments/2026-09-26-zone-m1-owncam/prereg_amendments.json',
                                   'experiments/2026-09-26-zone-owncam-loop-v2/calibration_loop_v2.json']
    out = {'schema': 'ugrp.m1_owncam_frozen.v1', 'source_sha': git('rev-parse', 'HEAD'),
           'frozen_note': 'source frozen after the A5 dev check, before any test episode ran',
           'student': {k: student.get(k) for k in ('skill', 'calibration', 'contact_profile', 'controller_schema',
                                                   'runner_schema', 'amendments_applied', 'amendments_sha256')},
           'test_episodes': [e['episode_id'] for e in episodes if e['split'] == 'test'],
           'records_only_paths': RECORDS_ONLY,
           'sha256': {f: hashlib.sha256((ROOT/f).read_bytes()).hexdigest() for f in dict.fromkeys(files)}}
    (HERE/'frozen_source.json').write_text(json.dumps(out, indent=2) + '\n')
    print('frozen at', out['source_sha'][:9], len(out['sha256']), 'files')


if __name__ == '__main__':
    main()
