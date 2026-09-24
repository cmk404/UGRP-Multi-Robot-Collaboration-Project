#!/usr/bin/env python3
"""Offline replay: saved fine-stage RGB -> saved pose models -> gain schedule.

No simulator, referee state or model call. Recomputes each recorded moving
fine decision from its original own/TOP JPEG, checks it matches the recorded
decision, then reports the command the opt-in schedule would have issued.
This does not predict physical motion or mission time.
"""
import argparse, collections, hashlib, json, subprocess, sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from scripts.run_camera_varied_start_student import load_stage_models
from harness.camera_varied_start_student import predict_stage
from harness.fine_gain_schedule import schedule_decision, settings


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--stage-model-dir', type=Path, required=True)
    p.add_argument('--run', type=Path, action='append', required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    git = lambda *x: subprocess.check_output(['git', *x], cwd=ROOT, text=True).strip()
    _, models = load_stage_models(a.stage_model_dir)
    out = {'schema': 'ugrp.fine_gain_schedule_replay.v1',
           'source': {'git_sha': git('rev-parse', 'HEAD'), 'dirty': bool(git('status', '--porcelain'))},
           'scope': 'offline RGB decision replay only; no physics, referee, model call, speed or success claim',
           'settings': settings(),
           'stage_models': {'dir': str(a.stage_model_dir),
                            'files_sha256': {f.name: sha(f) for f in sorted(a.stage_model_dir.glob('*.json'))}},
           'runs': {}}
    for run in a.run:
        rows = json.loads((run / 'pair-decisions.json').read_text())
        report = [r for r in rows if r['kind'] == 'learned_approach'][0]['report']
        n = mismatches = image_hash_failures = 0
        stages = collections.defaultdict(lambda: {'moving_decisions': 0, 'scheduled': 0,
                                                  'max_model_command': 0., 'max_scheduled_command': 0.})
        for call in report['approach_calls']:
            if call['stationary'] or not call['decision'].get('ok'):
                continue
            own_path, top_path = run / call['images']['own']['path'], run / call['images']['top']['path']
            if sha(own_path) != call['images']['own']['sha256'] or sha(top_path) != call['images']['top']['sha256']:
                image_hash_failures += 1
                continue
            model = models[call['robot_id']][call['stage']]
            decision = predict_stage(model, own_path.read_bytes(), top_path.read_bytes())
            n += 1
            recorded = call['decision']
            if decision['command'] != recorded['command'] or decision['ready'] != recorded['ready']:
                mismatches += 1
            scheduled = schedule_decision(model, decision)
            row = stages[call['stage']]
            row['moving_decisions'] += 1
            row['scheduled'] += bool(scheduled['gain_schedule']['applied'])
            row['max_model_command'] = max(row['max_model_command'], abs(decision['command']))
            row['max_scheduled_command'] = max(row['max_scheduled_command'], abs(scheduled['command']))
        out['runs'][run.name] = {'pair_decisions_sha256': sha(run / 'pair-decisions.json'),
                                 'replayed_moving_decisions': n, 'recorded_decision_mismatches': mismatches,
                                 'image_hash_failures': image_hash_failures, 'stages': dict(stages)}
    a.output.write_text(json.dumps(out, indent=2) + '\n')
    print(json.dumps({k: v for k, v in out['runs'].items()}, indent=1))


if __name__ == '__main__':
    main()
