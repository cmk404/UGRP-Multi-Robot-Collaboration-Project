import json
from pathlib import Path
import subprocess
import sys

import pytest

from scripts.approach_speed_teacher import teacher_command


def test_speed_scaling_preserves_stop_and_saturation_and_overshoot():
    for remaining in (.4, .25, .05, .005, .004, 0., -.004, -.01):
        slow = teacher_command(remaining, 1.)
        fast = teacher_command(remaining, 2.)
        assert fast['forward'] == min(.15, 2 * slow['forward'])
        assert fast['ready'] == slow['ready']
    assert teacher_command(-.01, 2.)['forward'] == 0
    assert not teacher_command(-.01, 2.)['ready']
    for remaining, multiplier in [(float('nan'), 2), (.2, 3)]:
        with pytest.raises(ValueError):
            teacher_command(remaining, multiplier)


def test_privileged_teacher_cannot_be_mixed_into_act(tmp_path):
    run = subprocess.run([sys.executable, 'scripts/run_camera_approach_student.py',
        '--distance','.2','.2','--condition','visual','--out-dir',str(tmp_path/'out'),
        '--grasp-model-dir','unused','--approach-model-dir','unused',
        '--act-python',sys.executable,'--act-model-dir','unused',
        '--teacher-reference-dir','unused'],capture_output=True,text=True)
    assert run.returncode == 2
    assert 'privileged teacher cannot be combined' in run.stderr
    assert not (tmp_path/'out').exists()


def test_preregistered_protocol_is_disjoint_and_all_starts_valid():
    from scripts.camera_approach_scene import validate_start_poses
    folder = Path('experiments/2026-09-16-act-double-speed')
    p = json.loads((folder/'protocol.json').read_text())
    data = json.loads((folder/'teacher-cases.json').read_text())['cases']
    assert len(data) == 55 and len(p['development_cases']) == 5
    assert set(p['development_cases']) < {c['id'] for c in data}
    assert len(p['test_cases']) == 32
    training = {(c['distance_m']['r1'],c['distance_m']['r3'],0.,0.,0.,0.) for c in data}
    keys = set()
    for case in p['test_cases']:
        s = validate_start_poses(case['start_poses'])
        key = tuple(s[r][k] for k in ('distance_m','lateral_m','yaw_deg') for r in ('r1','r3'))
        assert key not in training and key not in keys
        keys.add(key)
