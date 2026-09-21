"""Three finite CPU smoke trials; deliberately separate from failure-rate cohorts.

Model secrets stay on the host relay. A failed/timeout trial is retained and
does not cancel the remaining policies. This is not a task-success estimate.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts.cloud_collection import digest, write_json
from scripts.cloud_progress import emit, run_logged

POLICIES = ('rule', 'jev', 'gemini')
LIMITS = {'max_sim_s': 4., 'max_wall_s': 150., 'max_calls': 2,
          'max_input_tokens': 16000}
CHILD_TIMEOUT = 180
SCOPE = 'CPU/OSMesa execution diagnostic only; excluded from all failure-rate and GPU timing cohorts'


def cpu_environment(parent):
    env = dict(parent)
    env.pop('__EGL_VENDOR_LIBRARY_FILENAMES', None)
    env.update(MUJOCO_GL='osmesa', PYOPENGL_PLATFORM='osmesa')
    return env


def audit(directory, policy, process):
    """Readiness requires actual RGB, commands and parsed model responses."""
    result_path = directory/'result.json'
    result = json.loads(result_path.read_text()) if result_path.exists() else {}
    turns_path = directory/'turns.json'
    turns = json.loads(turns_path.read_text()) if turns_path.exists() else []
    pairs = [row['images'] for row in turns if row.get('images')]
    images = [pair[key] for pair in pairs
                   for key in ('own_rgb', 'shared_top_rgb') if pair.get(key)]
    images_ok = bool(pairs) and len(images) == 2*len(pairs) and all(
        (path := (directory/image['path']).resolve()).is_relative_to(directory.resolve())
        and path.is_file() and path.stat().st_size > 0 and digest(path) == image['sha256']
        for image in images)
    responses = [row['response'] for row in turns if row.get('response')]
    model_ok = policy == 'rule' or bool(responses) and all(
        row.get('status') == 'ok' for row in responses) and any(
        row.get('request') and row.get('answers') and row.get('command') for row in turns)
    checks = {'process': process['exit_code'] == 0 and not process['timed_out'],
              'no_trial_error': bool(result) and result.get('error') is None,
              'rgb_pairs_saved': images_ok,
              'commands_issued': result.get('commands', 0) > 6,
              'model_response_parsed': model_ok,
              'camera_unchanged': result.get('camera_geometry_unchanged') is True,
              'weld_off': result.get('weld_steps') == 0,
              'video_saved': (directory/'motion.mp4').is_file() and (directory/'motion.mp4').stat().st_size > 0}
    return {'policy': policy, 'ready': all(checks.values()), 'checks': checks,
            'process': process, 'task_success': result.get('success'),
            'stop_reason': result.get('stop_reason'), 'error': result.get('error'),
            'wall_s': result.get('wall_s'), 'sim_s': result.get('sim_s'),
            'commands': result.get('commands'), 'model_calls': result.get('model_calls'),
            'rgb_pair_count': len(pairs),
            'model_latency_s': [r.get('latency_s') for r in responses],
            'provider_latency_s': [r.get('provider_latency_s') for r in responses]}


def worker(output, policy, mailbox, source):
    import mujoco
    from OpenGL import GL
    from scripts.run_jev_skill_motion import DEVELOPMENT, run_trial
    context = mujoco.GLContext(64, 64)
    try:
        context.make_current()
        renderer = {k: GL.glGetString(v).decode() for k, v in
                    [('vendor', GL.GL_VENDOR), ('renderer', GL.GL_RENDERER)]}
    finally:
        context.free()
    # Scene.open owns the new trial directory.
    args = SimpleNamespace(output=output, case='dev_open', policy=policy,
                           arm='full', clock='paused', model_mailbox=mailbox,
                           gemini_url='http://127.0.0.1:8391/v1/chat/completions', **LIMITS)
    result = run_trial(args, DEVELOPMENT['dev_open'], policy, None, source)
    write_json(output/'cpu-renderer.json', {**renderer, 'backend': os.environ['MUJOCO_GL']})
    return int(result.get('error') is not None)


def diagnose(output, mailbox, source, *, launch=run_logged):
    output.mkdir(parents=True, exist_ok=False)
    protocol = {'source_sha': source, 'scope': SCOPE, 'policies': POLICIES,
                'case': 'dev_open', 'arm': 'full', 'clock': 'paused',
                'limits': LIMITS, 'child_timeout_s': CHILD_TIMEOUT,
                'http_max_attempts': 1, 'render_backend': 'osmesa',
                'reuse_previous_trials': False, 'repeat_failed_trials': False}
    write_json(output/'protocol.json', protocol)
    summary = {'scope': SCOPE, 'source_sha': source, 'started_unix': time.time(),
               'complete': False, 'ready': False, 'trials': []}
    env = cpu_environment(os.environ)
    for policy in POLICIES:
        directory = output/policy
        process = launch([sys.executable, '-m', 'scripts.run_cpu_model_diagnostic',
                          '--output', directory, '--model-mailbox', mailbox,
                          '--worker', policy], output/(policy+'.log'),
                         name='cpu-'+policy, cwd=ROOT, env=env, timeout=CHILD_TIMEOUT)
        try:
            row = audit(directory, policy, process)
        except (OSError, ValueError, TypeError, KeyError) as exc:
            row = {'policy': policy, 'ready': False, 'process': process,
                   'audit_error_type': type(exc).__name__}
        summary['trials'].append(row)
        write_json(output/'cpu-diagnostic.json', summary)
        emit('cpu_trial_complete', name=policy, ready=row['ready'],
             planned=3, attempted=len(summary['trials']), pending=3-len(summary['trials']))
    summary.update(complete=True, ready=all(row['ready'] for row in summary['trials']),
                   finished_unix=time.time())
    write_json(output/'cpu-diagnostic.json', summary)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--model-mailbox', type=Path, required=True)
    parser.add_argument('--worker', choices=POLICIES)
    args = parser.parse_args()
    source = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
    if subprocess.check_output(['git', 'status', '--porcelain'], cwd=ROOT).strip():
        parser.error('commit source before diagnostic')
    if os.environ.get('MUJOCO_GL') != 'osmesa' or os.environ.get('PYOPENGL_PLATFORM') != 'osmesa':
        parser.error('run in an explicit CPU/OSMesa environment')
    if args.worker:
        return worker(args.output.resolve(), args.worker, args.model_mailbox, source)
    report = diagnose(args.output.resolve(), args.model_mailbox, source)
    return 0 if report['ready'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
