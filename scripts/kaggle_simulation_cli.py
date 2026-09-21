"""Prepare, submit and recover private Kaggle CPU simulation jobs through its CLI."""
from __future__ import annotations
import argparse
import base64
import inspect
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys
import uuid
import zipfile

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from scripts.colab_simulation_cli import pack, digest
from scripts.kaggle_offline_dependencies import prepare_dependencies


def write(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')


def cli(*args):
    result = subprocess.run(['kaggle', *map(str, args)], text=True, capture_output=True)
    if result.returncode:
        raise RuntimeError(result.stderr.strip() or result.stdout.strip() or 'Kaggle CLI failed')
    return result.stdout


def driver(record, job_id, module, arguments, dependencies_sha256, source_filename=None, source_delta=None):
    remote = '/kaggle/temp/ugrp-' + job_id
    from scripts.kaggle_source_delta import apply_source_delta
    delta_code = inspect.getsource(apply_source_delta) if source_delta else ''
    return f'''from pathlib import Path
import base64, hashlib, json, os, shutil, subprocess, sys, tarfile, traceback
{delta_code}
working = Path('/kaggle/working')
working.mkdir(exist_ok=True)
base = Path({remote!r})
job = {{'job_id': {job_id!r}, 'source_sha': {record['source_sha']!r}, 'provider': 'kaggle', 'status': 'setup', 'exit_code': None}}
def save():
    (working/'remote-job.json').write_text(json.dumps(job, indent=2)+'\\n')
    print('UGRP_JOB_STATUS', json.dumps(job), flush=True)
save()
try:
    candidates = list(Path('/kaggle/input').rglob({(source_filename or 'ugrp-source-' + job_id + '.bin')!r}))
    if len(candidates) != 1:
        raise RuntimeError('expected exactly one source bundle input')
    base.parent.mkdir(parents=True, exist_ok=True)
    archive = candidates[0]
    inputs = archive.parent
    manifest_file = inputs/'dependencies-manifest.json'
    if hashlib.sha256(manifest_file.read_bytes()).hexdigest() != {dependencies_sha256!r}:
        raise ValueError('dependency manifest hash mismatch')
    if hashlib.sha256(archive.read_bytes()).hexdigest() != {record['sha256']!r}:
        raise ValueError('source archive hash mismatch')
    base.mkdir(exist_ok=False)
    with tarfile.open(archive) as bundle:
        for member in bundle.getmembers():
            if member.issym() or member.islnk():
                raise ValueError('source archive links are forbidden')
        bundle.extractall(base, filter='data')
    source_delta = {source_delta!r}
    if source_delta:
        apply_source_delta(base/'source', source_delta)
    with (working/'setup.log').open('w') as log:
        subprocess.run([sys.executable, str(base/'source/scripts/setup_kaggle_offline.py'), str(base), str(inputs), str(manifest_file)], stdout=log, stderr=subprocess.STDOUT, check=True)
    source = base/'source'
    py = str(base/'sim-env/bin/python')
    env = os.environ.copy()
    env.update(MUJOCO_GL='osmesa', PYOPENGL_PLATFORM='osmesa', PYTHONPATH=str(source), LD_LIBRARY_PATH=str(base/'system/usr/lib/x86_64-linux-gnu'))
    output = source/'outputs'/('kaggle-'+{job_id!r})
    job['status'] = 'running'
    save()
    arguments = [arg.replace('{{python}}', py) for arg in {arguments!r}]
    run = subprocess.run([py, str(source/'scripts/run_colab_simulation.py'), '--output', str(output), '--', py, '-m', {module!r}, *arguments], cwd=source, env=env)
    job['exit_code'] = run.returncode
    job['status'] = 'complete' if run.returncode == 0 else 'failed'
    for suffix in ('.zip', '.zip.sha256'):
        artifact = output.with_suffix(suffix)
        if artifact.exists():
            shutil.copyfile(artifact, working/('result'+suffix))
except Exception:
    job['status'] = 'setup_or_transport_failed'
    (working/'failure.txt').write_text(traceback.format_exc())
    raise
finally:
    save()
if job['exit_code']:
    raise SystemExit(job['exit_code'])
'''


def prepare(output, owner=None, *, root=ROOT, module='scripts.sim_quickstart', arguments=None, include=(), wheelhouse=None):
    if owner is not None and not re.fullmatch(r'[A-Za-z0-9_-]+', owner):
        raise ValueError('invalid Kaggle username')
    record = pack(root, output, include)
    kernel = output/'kernel'
    kernel.mkdir()
    if owner is None:
        cli('kernels', 'init', '-p', kernel)
        owner = json.loads((kernel/'kernel-metadata.json').read_text())['id'].split('/')[0]
        if not re.fullmatch(r'[A-Za-z0-9_-]+', owner):
            raise ValueError('Kaggle CLI did not return a valid authenticated username')
    dependencies = prepare_dependencies(root, output, wheelhouse)
    job_id = uuid.uuid4().hex[:12]
    dataset_slug = 'ugrp-source-' + job_id
    kernel_slug = 'ugrp-simulation-' + job_id
    data = output/'dataset'
    data.mkdir()
    shutil.copyfile(output/'source.tar.gz', data/('ugrp-source-'+job_id+'.bin'))
    write(data/'source-manifest.json', record)
    write(data/'dependencies-manifest.json', dependencies)
    for name in dependencies['files']:
        shutil.copyfile(output/'dependencies'/name, data/name)
    dependencies_sha256 = digest(data/'dependencies-manifest.json')
    write(data/'dataset-metadata.json', {'id': owner+'/'+dataset_slug, 'title': dataset_slug,
          'licenses': [{'name': 'other'}],
          'description': 'Private execution copy only. Original copyright notices and licenses remain applicable; no additional redistribution license is granted.'})
    metadata = {'id': owner+'/'+kernel_slug, 'title': kernel_slug, 'code_file': 'run.py', 'language': 'python',
                'kernel_type': 'script', 'is_private': True, 'enable_gpu': False, 'enable_tpu': False,
                'enable_internet': False, 'dataset_sources': [owner+'/'+dataset_slug],
                'competition_sources': [], 'kernel_sources': [], 'model_sources': []}
    write(kernel/'kernel-metadata.json', metadata)
    (kernel/'run.py').write_text(driver(record, job_id, module, arguments or ['--output', '{output}'], dependencies_sha256))
    state = {'job_id': job_id, 'source_sha': record['source_sha'], 'source_sha256': record['sha256'],
             'dataset': owner+'/'+dataset_slug, 'kernel': owner+'/'+kernel_slug, 'stage': 'prepared',
             'driver_sha256': digest(kernel/'run.py'), 'cpu_only': True,
             'dependencies_sha256': dependencies_sha256}
    write(output/'job.json', state)
    return state


def reuse_inputs(previous, output, *, module, arguments, refresh_source=False):
    """New kernel using a verified private input dataset; no upload or mutation."""
    original = json.loads((previous/'job.json').read_text())
    validate(previous, original)
    if original.get('dataset_private_verified') is not True:
        raise ValueError('previous dataset privacy was not verified')
    output.mkdir(parents=True, exist_ok=False)
    shutil.copytree(previous/'dataset', output/'dataset')
    shutil.copyfile(previous/'source-manifest.json', output/'source-manifest.json')
    record = json.loads((output/'source-manifest.json').read_text())
    job_id = uuid.uuid4().hex[:12]
    owner = original['kernel'].split('/')[0]
    kernel = owner+'/ugrp-simulation-'+job_id
    folder = output/'kernel';folder.mkdir()
    metadata = json.loads((previous/'kernel/kernel-metadata.json').read_text())
    metadata.update(id=kernel, title='ugrp-simulation-'+job_id)
    write(folder/'kernel-metadata.json', metadata)
    filename = original.get('source_filename', 'ugrp-source-'+original['job_id']+'.bin')
    delta = None
    if refresh_source:
        snapshot = pack(ROOT, output/'updated-source')
        bundle = output/'source-update.bundle'
        subprocess.run(['git','bundle','create',str(bundle.resolve()),'HEAD','^'+original['source_sha']],cwd=ROOT,check=True)
        delta = {'base_sha':original['source_sha'],'target_sha':snapshot['source_sha'],
                 'included':snapshot['included'],'sha256':digest(bundle),
                 'content':base64.b64encode(bundle.read_bytes()).decode()}
        record = {**record, 'source_sha':snapshot['source_sha']}
    (folder/'run.py').write_text(driver(record, job_id, module, arguments,
                                       original['dependencies_sha256'], filename, delta))
    state = {**original, 'job_id':job_id, 'kernel':kernel, 'stage':'dataset_submitted',
             'source_filename':filename, 'reused_dataset_from_kernel':original['kernel'],
             'driver_sha256':digest(folder/'run.py')}
    if delta:
        state.update(source_sha=delta['target_sha'],source_base_sha=delta['base_sha'],source_delta_sha256=delta['sha256'])
    for key in ('kernel_version','downloaded','exit_code','kernel_private_verified'):
        state.pop(key, None)
    write(output/'job.json', state)
    validate(output, state)
    return state


def validate(output, state):
    manifest_file = output/'dataset/dependencies-manifest.json'
    if digest(manifest_file) != state['dependencies_sha256']:
        raise ValueError('dependency manifest changed since preparation')
    dependencies = json.loads(manifest_file.read_text())
    for name, expected in dependencies['files'].items():
        if Path(name).name != name or digest(output/'dataset'/name) != expected:
            raise ValueError('dependency changed since preparation')
    expected_files = set(dependencies['files']) | {'dependencies-manifest.json', 'dataset-metadata.json', 'source-manifest.json', state.get('source_filename', 'ugrp-source-'+state['job_id']+'.bin')}
    files = list((output/'dataset').iterdir())
    if {p.name for p in files} != expected_files or any(not p.is_file() or p.is_symlink() for p in files):
        raise ValueError('unexpected file in dataset upload directory')
    metadata = json.loads((output/'kernel/kernel-metadata.json').read_text())
    if metadata['is_private'] is not True or metadata['enable_gpu'] is not False or metadata.get('enable_tpu') is not False or metadata.get('enable_internet') is not False:
        raise ValueError('this runner requires a private CPU kernel')
    if metadata['id'] != state['kernel'] or metadata['dataset_sources'] != [state['dataset']]:
        raise ValueError('kernel identity/input changed since preparation')
    data = json.loads((output/'dataset/dataset-metadata.json').read_text())
    if data['id'] != state['dataset'] or data['licenses'] != [{'name': 'other'}]:
        raise ValueError('dataset identity or source rights metadata changed')
    if digest(output/'kernel/run.py') != state['driver_sha256']:
        raise ValueError('remote driver changed since preparation')
    if digest(output/'dataset'/(state.get('source_filename', 'ugrp-source-'+state['job_id']+'.bin'))) != state['source_sha256']:
        raise ValueError('source bundle changed since preparation')


def submit(output):
    state = json.loads((output/'job.json').read_text())
    validate(output, state)
    if state['stage'] == 'prepared':
        state['stage'] = 'dataset_create_requested'
        write(output/'job.json', state)
        # Omit --public: official CLI creates a private dataset by default.
        response = cli('datasets', 'create', '-p', output/'dataset', '--keep-tabular')
        (output/'dataset-create.log').write_text(response)
        if 'Dataset creation error:' in response:
            state['stage'] = 'dataset_create_failed'
            write(output/'job.json', state)
            raise RuntimeError(response.strip())
        state['stage'] = 'dataset_submitted'
        write(output/'job.json', state)
    if state['stage'] not in {'dataset_submitted', 'dataset_create_requested'}:
        raise ValueError('submission already attempted; use status/collect instead of starting another run')
    try:
        status = json.loads(cli('datasets', 'status', state['dataset'], '--format', 'json'))
    except RuntimeError as error:
        # Newly created private datasets can return 403 until indexing finishes.
        # Preserve uncertainty; never treat this response as ready or recreate.
        if '403' not in str(error):
            raise
        (output/'dataset-status-error.log').write_text(str(error)+'\n')
        print('Dataset status is not accessible yet (403). Check login/creation log and retry submit later; no new upload was started.')
        return 2
    write(output/'dataset-status.json', status)
    if str(status['status']).lower().split('.')[-1] not in {'ready', 'complete'}:
        print('Dataset indexing is not ready. Run submit again later; it will not create another dataset.')
        return 2
    remote_meta = output/'dataset-remote-metadata'
    remote_meta.mkdir(exist_ok=True)
    cli('datasets', 'metadata', state['dataset'], '-p', remote_meta)
    observed = json.loads((remote_meta/'dataset-metadata.json').read_text())
    observed = observed.get('info', observed)
    private = observed.get('isPrivate', observed.get('is_private'))
    if private is not True:
        raise ValueError('remote dataset privacy was not confirmed; kernel will not be submitted')
    state['dataset_private_verified'] = True
    state['stage'] = 'kernel_submit_requested'
    write(output/'job.json', state)
    response = cli('kernels', 'push', '-p', output/'kernel', '--timeout', '1800')
    (output/'kernel-push.log').write_text(response)
    match = re.search(r'Kernel version (\d+) successfully pushed', response)
    if not match or 'not valid' in response or 'error:' in response.lower():
        raise RuntimeError('Kaggle did not confirm kernel submission; inspect kernel-push.log before retrying')
    state.update(stage='submitted', kernel_version=int(match.group(1)))
    write(output/'job.json', state)
    print(response.strip())
    return 0


def status(output):
    state = json.loads((output/'job.json').read_text())
    response = cli('kernels', 'status', state['kernel'])
    (output/'kernel-status.log').write_text(response)
    print(response.strip())
    match = re.search(r'has status "([^"]+)"', response)
    return re.sub(r'[^a-z]', '', match.group(1).lower().split('.')[-1]) if match else 'unknown'


def verify(output, downloaded):
    state = json.loads((output/'job.json').read_text())
    remote = json.loads((downloaded/'remote-job.json').read_text())
    if remote['job_id'] != state['job_id'] or remote['source_sha'] != state['source_sha']:
        raise ValueError('downloaded result identity mismatch')
    archive = downloaded/'result.zip'
    if digest(archive) != (downloaded/'result.zip.sha256').read_text().split()[0]:
        raise ValueError('downloaded ZIP hash mismatch')
    with zipfile.ZipFile(archive) as bundle:
        report = json.loads(bundle.read('run.json'))
        if report['source_sha'] != state['source_sha'] or report['exit_code'] != remote['exit_code']:
            raise ValueError('result source/exit status mismatch')
        for name, expected in report['artifacts'].items():
            if hashlib.sha256(bundle.read(name)).hexdigest() != expected:
                raise ValueError('result member hash mismatch: '+name)
    write(output/'verified-result.json', report)
    return report['exit_code']


def collect(output):
    current = status(output)
    if current not in {'complete', 'error', 'failed', 'cancelled', 'cancelacknowledged'}:
        raise ValueError('kernel is not terminal; leave it running and collect later')
    state = json.loads((output/'job.json').read_text())
    downloaded = output/('download-'+uuid.uuid4().hex[:8])
    downloaded.mkdir()
    (downloaded/'download.log').write_text(cli('kernels', 'output', state['kernel'], '-p', downloaded))
    metadata = downloaded/'kernel-metadata'
    metadata.mkdir()
    cli('kernels', 'pull', state['kernel'], '-p', metadata, '--metadata')
    privacy = json.loads((metadata/'kernel-metadata.json').read_text())['is_private']
    if privacy is not True and privacy != 'true':
        raise ValueError('remote kernel is not private')
    # Failure logs remain available even when setup never produced a result ZIP.
    if not (downloaded/'result.zip').exists():
        state.update(stage='remote_failed', downloaded=str(downloaded), kernel_private_verified=True)
        write(output/'job.json', state)
        raise RuntimeError('Remote setup produced no result ZIP; failure evidence saved in '+str(downloaded))
    result = verify(output, downloaded)
    state.update(stage='collected', downloaded=str(downloaded), exit_code=result, kernel_private_verified=True)
    write(output/'job.json', state)
    print(json.dumps({'verified_download': str(downloaded), 'exit_code': result}))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='action', required=True)
    p = sub.add_parser('prepare')
    p.add_argument('--output', required=True, type=Path)
    p.add_argument('--owner', help='omit to read the authenticated username via kaggle kernels init')
    p.add_argument('--include', action='append', default=[])
    p.add_argument('--module', default='scripts.sim_quickstart')
    p.add_argument('--wheelhouse', type=Path, help='reuse Linux CPython 3.12 wheels, including pip 26.2.1')
    p.add_argument('args', nargs=argparse.REMAINDER)
    p = sub.add_parser('reuse')
    p.add_argument('--from-output', required=True, type=Path)
    p.add_argument('--refresh-source', action='store_true', help='Embed a verified Git delta; reuse large private input bytes')
    p.add_argument('--output', required=True, type=Path)
    p.add_argument('--module', required=True)
    p.add_argument('args', nargs=argparse.REMAINDER)
    for name in ('submit', 'status', 'collect'):
        q = sub.add_parser(name)
        q.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    output = args.output.resolve()
    if args.action == 'reuse':
        arguments = args.args[1:] if args.args[:1] == ['--'] else args.args
        print(json.dumps(reuse_inputs(args.from_output.resolve(), output, module=args.module, arguments=arguments, refresh_source=args.refresh_source)))
        return 0
    if args.action == 'prepare':
        arguments = args.args[1:] if args.args[:1] == ['--'] else args.args
        print(json.dumps(prepare(output, args.owner, module=args.module, arguments=arguments, include=args.include, wheelhouse=args.wheelhouse)))
        return 0
    if args.action == 'status':
        status(output)
        return 0
    return submit(output) if args.action == 'submit' else collect(output)


if __name__ == '__main__':
    raise SystemExit(main())
