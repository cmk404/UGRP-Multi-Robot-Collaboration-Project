"""Build a hash-locked Linux/Python 3.12 payload locally for offline Kaggle jobs."""
from __future__ import annotations
import json
from pathlib import Path
import shutil
import subprocess
import sys
import urllib.request
from scripts.colab_simulation_cli import digest


def prepare_dependencies(root, output, wheelhouse=None):
    cache = output/'dependencies'
    cache.mkdir()
    if wheelhouse is None:
        subprocess.run([sys.executable, '-m', 'pip', 'download', '--only-binary=:all:',
                        '--platform', 'manylinux_2_28_x86_64', '--platform', 'manylinux_2_27_x86_64',
                        '--platform', 'manylinux2014_x86_64', '--python-version', '312',
                        '--implementation', 'cp', '--abi', 'cp312', '-r', str(root/'requirements-sim.txt'),
                        'pip==26.2.1', '-d', str(cache)], check=True)
    else:
        for path in sorted(wheelhouse.glob('*.whl')):
            if path.is_symlink():
                raise ValueError('wheelhouse symlinks are not supported')
            shutil.copyfile(path, cache/path.name)
        if not list(cache.glob('pip-*.whl')):
            raise ValueError('wheelhouse must include pip')
    packages = json.loads((root/'configs/kaggle-jammy-packages.json').read_text())
    for package in packages.values():
        # Kaggle strips '~' from uploaded names; use a stable transport name.
        path = cache/(package['Package']+'.deb')
        urllib.request.urlretrieve('https://archive.ubuntu.com/ubuntu/'+package['Filename'], path)
        if digest(path) != package['SHA256']:
            raise ValueError('Ubuntu package hash mismatch: '+path.name)
    return {'target': 'Ubuntu 22.04 x86_64 / CPython 3.12', 'ubuntu_packages': packages,
            'files': {p.name: digest(p) for p in sorted(cache.iterdir())}}
