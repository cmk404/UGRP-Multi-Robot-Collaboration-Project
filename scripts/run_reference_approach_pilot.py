#!/usr/bin/env python3
"""Serial matched physical pilot using the existing approach/grasp executor."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('mjpython', 'act-python', 'comparison-dir', 'grasp-model-dir', 'protocol', 'out-dir'):
        p.add_argument('--' + name, required=True, type=Path)
    args = p.parse_args()
    out = args.out_dir.resolve()
    out.mkdir(parents=True, exist_ok=False)
    protocol = json.loads(args.protocol.read_text())
    source = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
    if subprocess.check_output(['git', 'status', '--porcelain', '--untracked-files=no'], cwd=ROOT):
        raise ValueError('commit execution code before running pilot')
    training = json.loads((args.comparison_dir / 'report.json').read_text())
    if training.get('complete') is not True or training['source_sha'] != source:
        raise ValueError('training must be complete and use this fixed source')
    records = []
    for index, distance in enumerate(protocol['physical_pilot_distances']):
        # Alternate order to reduce a fixed ordering effect; never parallelize physics here.
        conditions = protocol['physical_pilot_conditions'][::1 if index % 2 == 0 else -1]
        for condition in conditions:
            case = out / f'{index + 1:02d}-{condition}'
            command = [str(args.mjpython.resolve()), str(ROOT / 'scripts/run_camera_approach_student.py'),
                       '--grasp-model-dir', str(args.grasp_model_dir.resolve()),
                       '--approach-model-dir', str(args.comparison_dir.resolve()),
                       '--distance', *map(str, distance), '--condition', 'visual', '--out-dir', str(case)]
            if condition == 'act':
                command += ['--act-python', str(args.act_python.resolve()),
                            '--act-model-dir', str(args.comparison_dir.resolve())]
            elif condition != 'kernel':
                raise ValueError('unknown comparison condition')
            print(json.dumps({'case': case.name, 'status': 'running'}), flush=True)
            with (out / f'{case.name}.log').open('w') as log:
                process = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                                           start_new_session=True)
                timed_out = False
                try:
                    code = process.wait(timeout=300)
                except subprocess.TimeoutExpired:
                    timed_out = True
                    os.killpg(process.pid, signal.SIGTERM)
                    try:
                        code = process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        code = process.wait(timeout=5)
            result = json.loads((case / 'result.json').read_text()) if (case / 'result.json').exists() else {}
            record = {'case': case.name, 'condition': condition, 'distance': distance,
                      'returncode': code, 'timed_out': timed_out,
                      'success': bool(code == 0 and result.get('success')),
                      'approach_ok': result.get('approach_ok', False),
                      'grasp_success': result.get('grasp_success', False),
                      'wall_s': result.get('wall_elapsed_s'), 'sim_s': result.get('approach_elapsed_sim_s'),
                      'approach_calls': len(result.get('approach_calls', [])),
                      'contact_steps': result.get('approach_payload_contact_steps'),
                      'stop_reason': result.get('stop_reason'), 'error': result.get('error'),
                      'source_sha': result.get('source_sha')}
            records.append(record)
            summary = {'source_sha': source, 'protocol': protocol, 'cases': records,
                       'complete': len(records) == len(protocol['physical_pilot_distances']) * 2,
                       'external_model_calls': 0, 'external_model_tokens': 0}
            (out / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
            print(json.dumps(record), flush=True)
    manifest = {str(path.relative_to(out)): hashlib.sha256(path.read_bytes()).hexdigest()
                for path in sorted(out.rglob('*')) if path.is_file()}
    (out / 'raw-manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    return 0 if all(r['returncode'] == 0 for r in records) else 1


if __name__ == '__main__':
    raise SystemExit(main())
