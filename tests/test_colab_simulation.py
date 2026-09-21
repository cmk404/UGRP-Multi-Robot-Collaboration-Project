from pathlib import Path
import hashlib
import json
import shutil
import subprocess
import sys
import zipfile

import pytest
from scripts.run_colab_simulation import run


@pytest.fixture
def repo(tmp_path, monkeypatch):
    root = tmp_path / 'repo'
    (root / 'scripts').mkdir(parents=True)
    shutil.copy(Path(__file__).resolve().parents[1] / 'scripts/ugrp_session.py', root / 'scripts')
    (root / '.gitignore').write_text('outputs/\n')
    subprocess.run(['git', 'init', '-q', str(root)], check=True)
    subprocess.run(['git', '-C', str(root), 'add', '.'], check=True)
    subprocess.run(['git', '-C', str(root), '-c', 'user.name=Test', '-c', 'user.email=test@example.invalid', 'commit', '-qm', 'fixture'], check=True)
    monkeypatch.setenv('UGRP_SESSION_DIR', str(tmp_path / 'sessions'))
    return root


@pytest.mark.parametrize('exit_code', [0, 7])
def test_preserves_success_and_failure_with_source_and_hashes(repo, exit_code):
    job = repo / 'outputs' / 'trial'
    command = [sys.executable, '-c', "import pathlib, sys; p=pathlib.Path(sys.argv[1]); p.mkdir(); (p/'result.txt').write_text('evidence'); print('saved'); sys.exit(int(sys.argv[2]))", '{output}', str(exit_code)]
    assert run(job, command, root=repo) == exit_code
    record = json.loads((job / 'run.json').read_text())
    assert record['exit_code'] == exit_code
    assert record['status'] == ('complete' if exit_code == 0 else 'failed')
    assert record['source_sha'] == subprocess.check_output(['git', '-C', str(repo), 'rev-parse', 'HEAD'], text=True).strip()
    assert record['artifacts']['result/result.txt'] == hashlib.sha256(b'evidence').hexdigest()
    archive = job.with_suffix('.zip')
    with zipfile.ZipFile(archive) as bundle:
        assert bundle.read('result/result.txt') == b'evidence'
        assert b'saved' in bundle.read('run.log')
        assert not any('.git/' in name for name in bundle.namelist())
    assert archive.with_suffix('.zip.sha256').read_text().split()[0] == hashlib.sha256(archive.read_bytes()).hexdigest()
    assert not list((repo.parent / 'sessions').glob('*.json'))
    with pytest.raises(FileExistsError):
        run(job, command, root=repo)


def test_rejects_dirty_source_and_outside_output(repo):
    with pytest.raises(ValueError, match='inside'):
        run(repo.parent / 'outside', [sys.executable, '-V'], root=repo)
    (repo / '.gitignore').write_text('outputs/\nchanged\n')
    with pytest.raises(ValueError, match='commit'):
        run(repo / 'outputs' / 'dirty', [sys.executable, '-V'], root=repo)
    assert not (repo / 'outputs' / 'dirty').exists()



def test_cli_snapshot_preserves_real_source_and_excludes_untracked(repo):
    from scripts.colab_simulation_cli import pack, setup_code, job_code
    (repo/'private.env').write_text('untracked credentials stay local')
    record = pack(repo, repo.parent/'bundle')
    snapshot = repo.parent/'bundle/source'
    assert record['source_sha'] == subprocess.check_output(['git', '-C', str(snapshot), 'rev-parse', 'HEAD'], text=True).strip()
    assert not (snapshot/'private.env').exists()
    assert (snapshot/'scripts/ugrp_session.py').exists()
    assert subprocess.check_output(['git', '-C', str(snapshot), 'status', '--porcelain']) == b''
    assert subprocess.check_output(['git', '-C', str(snapshot), 'remote']) == b''
    compile(setup_code('/content/ugrp-test', record['sha256']), 'setup.py', 'exec')
    compile(job_code('/content/ugrp-test', 'scripts.sim_quickstart', ['--output', '{output}']), 'job.py', 'exec')
