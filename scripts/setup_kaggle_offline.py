"""Install only verified input packages into a disposable Kaggle environment."""
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess
import sys


def setup(base, inputs, manifest):
    if sys.version_info[:2] != (3, 12) or platform.machine() != 'x86_64':
        raise RuntimeError('offline payload requires CPython 3.12 on x86_64')
    for name, expected in manifest['files'].items():
        if Path(name).name != name or not name.endswith(('.whl', '.deb')):
            raise ValueError('invalid dependency filename')
        if hashlib.sha256((inputs/name).read_bytes()).hexdigest() != expected:
            raise ValueError('dependency hash mismatch: '+name)
    system = base/'system'
    for name in manifest['files']:
        if name.endswith('.deb'):
            subprocess.run(['dpkg-deb', '-x', str(inputs/name), str(system)], check=True)
    subprocess.run([sys.executable, '-m', 'venv', '--without-pip', str(base/'sim-env')], check=True)
    py = str(base/'sim-env/bin/python')
    pip_wheels = [name for name in manifest['files'] if name.startswith('pip-') and name.endswith('.whl')]
    if len(pip_wheels) != 1:
        raise ValueError('exactly one pip wheel is required')
    env = os.environ.copy()
    env.update(PYTHONPATH=str(inputs/pip_wheels[0]), PIP_DISABLE_PIP_VERSION_CHECK='1', PIP_CONFIG_FILE=os.devnull)
    # pip can target a venv which does not yet contain pip; no ensurepip or network needed.
    subprocess.run([sys.executable, '-m', 'pip', '--python', py, 'install', '--no-index',
                    '--find-links', str(inputs), '-r', str(base/'source/requirements-kaggle.txt'), 'pip==26.2.1'], env=env, check=True)
    subprocess.run([py, '-m', 'pip', 'check'], check=True)
    env.pop('PYTHONPATH', None)
    env['LD_LIBRARY_PATH'] = str(system/'usr/lib/x86_64-linux-gnu')
    subprocess.run([py, '-c', 'import ctypes; ctypes.CDLL("libOSMesa.so.6")'], env=env, check=True)
    print('UGRP_KAGGLE_OFFLINE_SETUP_OK')


if __name__ == '__main__':
    setup(Path(sys.argv[1]), Path(sys.argv[2]), json.loads(Path(sys.argv[3]).read_text()))
