import json
from pathlib import Path
import hashlib

from scripts.cloud_collection import write_json
from scripts.run_cpu_model_diagnostic import audit, cpu_environment, diagnose


def evidence(root, *, model=True):
    root.mkdir()
    for name in ('own.jpg', 'top.jpg', 'motion.mp4'):
        (root/name).write_bytes(b'fixture')
    write_json(root/'result.json', {'success': False, 'stop_reason': 'sim_budget',
               'error': None, 'commands': 9, 'camera_geometry_unchanged': True, 'weld_steps': 0})
    row = {'images': {key: {'path': name, 'sha256': hashlib.sha256(b'fixture').hexdigest()}
                     for key, name in [('own_rgb', 'own.jpg'), ('shared_top_rgb', 'top.jpg')]},
           'command': {'kind': 'drive'}}
    if model:
        row.update(request={'model': 'fixture'}, answers={'action': 'observe'}, response={'status': 'ok'})
    write_json(root/'turns.json', [row])


def test_short_budget_success_is_not_required_but_real_response_and_images_are(tmp_path):
    trial = tmp_path/'jev'
    evidence(trial)
    process = {'exit_code': 0, 'timed_out': False}
    row = audit(trial, 'jev', process)
    assert row['ready'] and row['task_success'] is False
    (trial/'own.jpg').unlink()
    assert not audit(trial, 'jev', process)['ready']
    (trial/'own.jpg').write_bytes(b'fixture')
    turns = json.loads((trial/'turns.json').read_text())
    turns[0].pop('answers')
    write_json(trial/'turns.json', turns)
    assert not audit(trial, 'jev', process)['ready']
    assert audit(trial, 'rule', process)['ready']


def test_timeout_does_not_skip_or_repeat_later_policies(tmp_path):
    calls = []
    def launch(command, log, **kwargs):
        policy = command[-1]
        calls.append(policy)
        directory = Path(command[command.index('--output')+1])
        if policy == 'jev':
            return {'exit_code': -15, 'timed_out': True, 'wall_s': 180}
        evidence(directory, model=policy != 'rule')
        return {'exit_code': 0, 'timed_out': False, 'wall_s': 1}
    report = diagnose(tmp_path/'result', tmp_path/'mailbox', 'frozen', launch=launch)
    assert calls == ['rule', 'jev', 'gemini']
    assert report['complete'] and not report['ready']
    assert [row['ready'] for row in report['trials']] == [True, False, True]
    assert (tmp_path/'result/cpu-diagnostic.json').is_file()


def test_cpu_backend_removes_gpu_vendor_override_without_mutating_parent():
    parent = {'MUJOCO_GL': 'egl', 'PYOPENGL_PLATFORM': 'egl',
              '__EGL_VENDOR_LIBRARY_FILENAMES': '/nvidia.json', 'KEEP': 'yes'}
    child = cpu_environment(parent)
    assert child == {'MUJOCO_GL': 'osmesa', 'PYOPENGL_PLATFORM': 'osmesa', 'KEEP': 'yes'}
    assert parent['MUJOCO_GL'] == 'egl'
