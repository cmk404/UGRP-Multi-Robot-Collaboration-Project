"""Finite ACT continuation: verified data, export diagnostic, training, physics.

Run on the allocated Linux VM, never on the Mac. Each stage keeps its own
archive and manifest. Previously interrupted Mac cohorts remain untouched.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import zipfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts.colab_carry_bundle import confined, digest, unpack, write


def extract_assets(archive, destination, expected):
    if digest(archive) != expected:
        raise ValueError('asset archive hash mismatch')
    with zipfile.ZipFile(archive) as bundle:
        names = bundle.namelist()
        if len(names) != len(set(names)):
            raise ValueError('duplicate asset members')
        manifest = json.loads(bundle.read('assets-manifest.json'))
        if set(names) != set(manifest) | {'assets-manifest.json'}:
            raise ValueError('unlisted asset members')
        for info in bundle.infolist():
            confined(destination, info.filename)
            if (info.external_attr >> 16) & 0o170000 == 0o120000:
                raise ValueError('asset symlink')
        destination.mkdir(parents=True, exist_ok=False)
        bundle.extractall(destination)
    for name, expected_hash in manifest.items():
        if digest(confined(destination, name)) != expected_hash:
            raise ValueError('asset member hash mismatch')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--data-bundle', type=Path, required=True)
    p.add_argument('--data-sha', required=True)
    p.add_argument('--assets', type=Path, required=True)
    p.add_argument('--assets-sha', required=True)
    p.add_argument('--act-python', required=True)
    p.add_argument('--sim-python', required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    if sys.platform != 'linux':
        p.error('cloud Linux execution required')
    if subprocess.check_output(['git', 'status', '--porcelain'], cwd=ROOT).strip():
        p.error('commit source before execution')
    a.output = a.output.resolve()
    if not a.output.is_relative_to(ROOT/'outputs'):
        p.error('output must be inside source outputs')
    a.output.mkdir(parents=True, exist_ok=False)
    state = {'source_sha': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
             'complete': False, 'stages': [], 'scope': 'new cloud screening cohort; interrupted Mac results retained separately'}
    status = a.output/'continuation.json'
    write(status, state)
    env = os.environ.copy()
    env.update(MUJOCO_GL='osmesa', PYOPENGL_PLATFORM='osmesa', PYTHONPATH=str(ROOT), OMP_NUM_THREADS='2')

    def stage(name, command, hours):
        output = a.output/name
        cmd = [a.sim_python, str(ROOT/'scripts/run_colab_simulation.py'), '--output', str(output), '--',
               'timeout', '--signal=INT', '--kill-after=30s', f'{hours}h', *map(str, command)]
        code = subprocess.call(cmd, cwd=ROOT, env=env)
        state['stages'].append({'name': name, 'exit_code': code, 'archive': str(output.with_suffix('.zip'))})
        write(status, state)
        if code:
            raise RuntimeError(f'{name} failed with exit code {code}; subsequent stages not started')
        return output/'result'

    try:
        data = Path(unpack(a.data_bundle, a.output/'data', a.data_sha))
        assets = a.output/'assets'
        extract_assets(a.assets, assets, a.assets_sha)
        protocol = ROOT/'experiments/2026-09-21-carry-resolution-sweep/protocol.json'
        diagnostic = stage('diagnostic', [a.act_python, ROOT/'scripts/train_carry_input_act.py',
            '--dataset', data, '--out', '{output}', '--size', '512', '--history', '4',
            '--steps', '2', '--seed', '20260921', '--device', 'cuda', '--checkpoint-encoder',
            '--cpu-evaluation-batch-size', '1'], 2)
        report = json.loads((diagnostic/'report.json').read_text())
        if not report.get('complete') or report.get('completed_steps') != 2:
            raise ValueError('full diagnostic export incomplete')
        if digest(diagnostic/'act/model.safetensors') != report['model_sha256']:
            raise ValueError('diagnostic model mismatch')
        trained = stage('training512', [a.act_python, ROOT/'scripts/run_colab_carry_training.py',
            '--dataset', data, '--out', '{output}', '--protocol', protocol], 16)
        cohort = json.loads((trained/'cohort.json').read_text())
        if not cohort.get('complete') or len(cohort.get('models', [])) != 4:
            raise ValueError('four completed models required before physics')
        for name, models, plan in [
            ('physics128256', assets/'training', ROOT/'experiments/2026-09-21-carry-input-ablation/protocol.json'),
            ('physics512', trained, protocol),
        ]:
            stage(name, [a.sim_python, ROOT/'scripts/run_carry_input_ablation.py', '--out', '{output}',
                '--dataset', data, '--reuse-training', models, '--protocol', plan,
                '--act-python', a.act_python, '--mjpython', a.sim_python,
                '--grasp', assets/'grasp', '--stages', assets/'varied'], 4)
        state['complete'] = True
    except Exception as error:
        state['error'] = str(error)
        raise
    finally:
        write(status, state)


if __name__ == '__main__':
    main()
