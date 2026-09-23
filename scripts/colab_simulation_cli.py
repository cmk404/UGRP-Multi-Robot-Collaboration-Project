"""Send a committed, sparse Git checkout to an existing Colab CLI session.

No GitHub credentials, notebook UI, Drive, VM creation or VM shutdown is implicit.
"""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tarfile
import uuid

ROOT = Path(__file__).resolve().parents[1]
SOURCE_DIRS = {'harness', 'sim', 'scripts', 'maps', 'calibration', 'config', 'configs', 'boot', 'tests'}


def call(args, **kwargs):
    return subprocess.run([str(x) for x in args], check=True, **kwargs)


def digest(path):
    value = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            value.update(block)
    return value.hexdigest()


def pack(root: Path, output: Path, include: list[str] = ()):
    if subprocess.check_output(['git', 'status', '--porcelain', '--untracked-files=no'], cwd=root).strip():
        raise ValueError('commit tracked source changes before packing')
    sha = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=root, text=True).strip()
    output.mkdir(parents=True, exist_ok=False)
    snapshot = output / 'source'
    rows = subprocess.check_output(['git', 'ls-tree', '-rl', '-z', 'HEAD'], cwd=root).decode().split('\0')
    included, omitted = [], []
    for row in filter(None, rows):
        header, name = row.split('\t', 1)
        path = Path(name)
        size = int(header.split()[3])
        take = (path.parts[0] in SOURCE_DIRS or name in {'.gitignore', 'AGENTS.md', 'README.md', 'CONTRIBUTING.md'}
                or (len(path.parts) == 1 and name.startswith('requirements') and path.suffix == '.txt')
                or (path.parts[0] == 'experiments' and path.suffix == '.json' and size <= 128 * 1024)
                or name in include)
        (included if take else omitted).append(name)
    missing = set(include) - set(included)
    if missing:
        raise ValueError(f'--include paths are not tracked files: {sorted(missing)}')
    # A real shallow partial clone preserves the exact original commit SHA and
    # Git queries used by experiment entrypoints, without old raw media blobs.
    call(['git', 'clone', '--quiet', '--depth', '1', '--no-checkout', '--filter=blob:none',
          '--upload-pack=git -c uploadpack.allowFilter=true upload-pack', root.as_uri(), snapshot])
    call(['git', 'config', 'remote.origin.uploadpack', 'git -c uploadpack.allowFilter=true upload-pack'], cwd=snapshot)
    patterns = ''.join('/' + name.replace('\\', '\\\\').replace('[', '\\[').replace('*', '\\*').replace('?', '\\?') + '\n' for name in included)
    call(['git', 'sparse-checkout', 'set', '--no-cone', '--stdin'], cwd=snapshot, input=patterns, text=True)
    call(['git', 'checkout', '--detach', sha], cwd=snapshot)
    # Keep no host paths or credential helper in the uploaded Git configuration.
    call(['git', 'config', '--remove-section', 'remote.origin'], cwd=snapshot)
    if subprocess.check_output(['git', 'status', '--porcelain'], cwd=snapshot).strip():
        raise ValueError('sparse source checkout is not clean')
    actual = sorted(str(p.relative_to(snapshot)) for p in snapshot.rglob('*') if p.is_file() and '.git' not in p.relative_to(snapshot).parts)
    if actual != sorted(included):
        raise ValueError('sparse checkout file list differs from requested source')
    archive = output / 'source.tar.gz'
    with tarfile.open(archive, 'w:gz') as bundle:
        bundle.add(snapshot, arcname='source')
    record = {'source_sha': sha, 'sha256': digest(archive), 'included': included, 'omitted': omitted,
              'scope': 'Committed source and small protocols; omitted media/models and local raw inputs must be supplied explicitly.'}
    (output / 'source-manifest.json').write_text(json.dumps(record, indent=2) + '\n')
    return record


def setup_code(remote, archive_sha):
    return f'''from pathlib import Path
import hashlib, os, subprocess, sys, tarfile
base = Path({remote!r})
archive = Path({(remote + '.tar.gz')!r})
hash_value = hashlib.sha256()
with archive.open('rb') as stream:
    for block in iter(lambda: stream.read(1024*1024), b''):
        hash_value.update(block)
assert hash_value.hexdigest() == {archive_sha!r}, 'source upload hash mismatch'
base.mkdir(exist_ok=False)
with tarfile.open(archive) as bundle:
    for member in bundle.getmembers():
        p = Path(member.name)
        if p.is_absolute() or '..' in p.parts or member.issym() or member.islnk() or not (member.isfile() or member.isdir()):
            raise ValueError('unsafe source member: ' + member.name)
    bundle.extractall(base, filter='data')
source = base/'source'
def checked(args):
    subprocess.run(args, check=True)
checked(['git', '-C', str(source), 'status', '--porcelain'])
checked(['apt-get', 'update', '-qq'])
checked(['apt-get', 'install', '-y', '-qq', 'libosmesa6', 'libegl1', 'libgl1', 'libglfw3', 'ffmpeg', 'fonts-noto-cjk'])
checked([sys.executable, '-m', 'pip', 'install', 'uv'])
checked([sys.executable, '-m', 'uv', 'venv', '--seed', '--python', '3.12', str(base/'sim-env')])
py = str(base/'sim-env/bin/python')
checked([sys.executable, '-m', 'uv', 'pip', 'install', '--python', py, '-r', str(source/'requirements-sim.txt'), '-r', str(source/'requirements-test.txt')])
checked([py, '-m', 'pip', 'check'])
print('UGRP_COLAB_SETUP_OK')
'''


def job_code(remote, module, arguments):
    return f'''from pathlib import Path
import os, subprocess
base = Path({remote!r})
source = base/'source'
py = str(base/'sim-env/bin/python')
env = os.environ.copy()
env.pop('__EGL_VENDOR_LIBRARY_FILENAMES', None)
env.update(MUJOCO_GL='osmesa', PYOPENGL_PLATFORM='osmesa', PYTHONPATH=str(source))
result = subprocess.run([py, str(source/'scripts/run_colab_simulation.py'), '--output', str(source/'outputs/cli-job'), '--', py, '-m', {module!r}, *{arguments!r}], cwd=source, env=env)
print('UGRP_COLAB_EXIT_CODE', result.returncode)
'''


def execute(session, output, record, module, arguments, timeout):
    remote = '/content/ugrp-cli-' + uuid.uuid4().hex[:12]
    (output/'remote.json').write_text(json.dumps({'session': session, 'remote': remote, 'source_sha': record['source_sha']}, indent=2)+'\n')
    setup = output/'setup.py'
    setup.write_text(setup_code(remote, record['sha256']))
    job = output/'job.py'
    job.write_text(job_code(remote, module, arguments))
    call(['colab', 'upload', '-s', session, output/'source.tar.gz', remote+'.tar.gz'])
    call(['colab', 'exec', '-s', session, '-f', setup, '--timeout', '900'])
    # A failed simulation is still downloaded. The manifest, not the CLI's text,
    # determines whether the job completed successfully.
    executed = subprocess.run(['colab', 'exec', '-s', session, '-f', str(job), '--timeout', str(timeout)])
    if executed.returncode:
        raise RuntimeError(f'Colab execution/transport failed; retain session {session} and inspect {remote}. No automatic retry.')
    archive = output/'result.zip'
    call(['colab', 'download', '-s', session, remote+'/source/outputs/cli-job.zip', archive])
    call(['colab', 'download', '-s', session, remote+'/source/outputs/cli-job.zip.sha256', output/'result.zip.sha256'])
    if digest(archive) != (output/'result.zip.sha256').read_text().split()[0]:
        raise ValueError('downloaded result hash mismatch')
    import zipfile
    with zipfile.ZipFile(archive) as bundle:
        report = json.loads(bundle.read('run.json'))
        if report['source_sha'] != record['source_sha']:
            raise ValueError('result source mismatch')
        for name, expected in report['artifacts'].items():
            if hashlib.sha256(bundle.read(name)).hexdigest() != expected:
                raise ValueError('artifact hash mismatch: ' + name)
    (output/'verified-result.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({'verified_archive': str(archive), 'exit_code': report['exit_code']}))
    return report['exit_code']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', required=True, help='existing CLI session; never provisions or stops it')
    parser.add_argument('--output', required=True, type=Path, help='new local transport/evidence directory')
    parser.add_argument('--module', default='scripts.sim_quickstart')
    parser.add_argument('--timeout', type=float, default=600)
    parser.add_argument('--include', action='append', default=[], help='additional tracked file required by this experiment')
    parser.add_argument('args', nargs=argparse.REMAINDER)
    a = parser.parse_args()
    args = a.args[1:] if a.args[:1] == ['--'] else a.args
    if not args:
        args = ['--output', '{output}']
    output = a.output.resolve()
    record = pack(ROOT, output, a.include)
    return execute(a.session, output, record, a.module, args, a.timeout)


if __name__ == '__main__':
    raise SystemExit(main())
