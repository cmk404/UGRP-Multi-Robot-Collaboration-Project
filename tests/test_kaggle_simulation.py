import hashlib
import json
from pathlib import Path
import subprocess
import zipfile

import pytest
from scripts import kaggle_simulation_cli as k


def snapshot_repo(tmp_path):
    root = tmp_path/'repo'
    root.mkdir()
    (root/'README.md').write_text('source')
    subprocess.run(['git', 'init', '-q', str(root)], check=True)
    subprocess.run(['git', '-C', str(root), 'add', '.'], check=True)
    subprocess.run(['git', '-C', str(root), '-c', 'user.name=Test', '-c', 'user.email=test@example.invalid', 'commit', '-qm', 'source'], check=True)
    return root


@pytest.fixture
def prepared(tmp_path):
    output = tmp_path/'job'
    state = k.prepare(output, 'testowner', root=snapshot_repo(tmp_path))
    return output, state


def test_preparation_is_private_cpu_and_preserves_rights(prepared):
    output, state = prepared
    meta = json.loads((output/'kernel/kernel-metadata.json').read_text())
    assert meta['is_private'] is True
    assert meta['enable_gpu'] is False
    assert meta['dataset_sources'] == [state['dataset']]
    assert json.loads((output/'dataset/dataset-metadata.json').read_text())['licenses'] == [{'name':'other'}]
    assert len(list((output/'dataset').iterdir())) == 3
    compile((output/'kernel/run.py').read_text(), 'kaggle-run.py', 'exec')
    k.validate(output, state)


def test_rejects_public_or_changed_payload(prepared):
    output, state = prepared
    path = output/'kernel/kernel-metadata.json'
    meta = json.loads(path.read_text())
    meta['is_private'] = False
    k.write(path, meta)
    with pytest.raises(ValueError, match='private CPU'):
        k.validate(output, state)
    meta['is_private'] = True
    k.write(path, meta)
    (output/'kernel/run.py').write_text('changed')
    with pytest.raises(ValueError, match='driver changed'):
        k.validate(output, state)


def test_submit_never_public_and_does_not_duplicate_dataset(prepared, monkeypatch):
    output, state = prepared
    calls=[]
    indexing=[True]
    def fake(*args):
        calls.append(args)
        if args[:2] == ('datasets','metadata'):
            k.write(Path(args[-1])/'dataset-metadata.json', {'isPrivate': True})
        if args[:2] == ('datasets','status'):
            return json.dumps({'status': 'processing' if indexing[0] else 'ready'})
        if args[:2] == ('kernels','push'):
            return 'Kernel version 1 successfully pushed. Please check progress at test'
        return 'Created'
    monkeypatch.setattr(k,'cli',fake)
    assert k.submit(output) == 2
    indexing[0]=False
    assert k.submit(output) == 0
    assert len([c for c in calls if c[:2] == ('datasets','create')]) == 1
    assert all('--public' not in c for c in calls)
    with pytest.raises(ValueError,match='already attempted'):
        k.submit(output)


def test_rejects_cli_success_exit_with_push_error(prepared, monkeypatch):
    output,_=prepared
    def fake(*args):
        if args[:2] == ('datasets','metadata'):
            k.write(Path(args[-1])/'dataset-metadata.json', {'is_private': True})
        if args[:2] == ('datasets','status'):return '{"status":"ready"}'
        if args[:2] == ('kernels','push'):return 'Kernel push error: invalid source'
        return 'Created'
    monkeypatch.setattr(k,'cli',fake)
    with pytest.raises(RuntimeError,match='did not confirm'):
        k.submit(output)
    assert json.loads((output/'job.json').read_text())['stage']=='kernel_submit_requested'


@pytest.mark.parametrize('exit_code',[0,7])
def test_recovery_verifies_identity_hashes_and_failure(prepared, exit_code):
    output,state=prepared
    downloaded=output/'download';downloaded.mkdir()
    k.write(downloaded/'remote-job.json',{'job_id':state['job_id'],'source_sha':state['source_sha'],'exit_code':exit_code})
    report={'source_sha':state['source_sha'],'exit_code':exit_code,'artifacts':{'result.txt':hashlib.sha256(b'evidence').hexdigest()}}
    with zipfile.ZipFile(downloaded/'result.zip','w') as z:
        z.writestr('run.json',json.dumps(report));z.writestr('result.txt',b'evidence')
    (downloaded/'result.zip.sha256').write_text(k.digest(downloaded/'result.zip')+' result.zip\n')
    assert k.verify(output,downloaded)==exit_code
    remote=json.loads((downloaded/'remote-job.json').read_text());remote['job_id']='wrong'
    k.write(downloaded/'remote-job.json',remote)
    with pytest.raises(ValueError,match='identity'):
        k.verify(output,downloaded)


def test_refuses_extra_dataset_files(prepared):
    output,state=prepared
    (output/'dataset/unexpected.txt').write_text('do not upload')
    with pytest.raises(ValueError,match='unexpected file'):
        k.validate(output,state)


def test_dataset_error_at_exit_zero_stops_before_status(prepared, monkeypatch):
    output,_=prepared
    calls=[]
    def fake(*args):
        calls.append(args)
        return 'Dataset creation error: Please select a valid license'
    monkeypatch.setattr(k,'cli',fake)
    with pytest.raises(RuntimeError,match='valid license'):
        k.submit(output)
    assert len(calls)==1
    assert json.loads((output/'job.json').read_text())['stage']=='dataset_create_failed'
