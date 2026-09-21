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
def prepared(tmp_path, monkeypatch):
    def dependencies(root, output, wheelhouse):
        cache = output/'dependencies'; cache.mkdir()
        (cache/'fake.whl').write_bytes(b'wheel')
        return {'files': {'fake.whl': k.digest(cache/'fake.whl')}}
    monkeypatch.setattr(k, 'prepare_dependencies', dependencies)
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
    assert len(list((output/'dataset').iterdir())) == 5
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
            k.write(Path(args[-1])/'dataset-metadata.json', {'info': {'isPrivate': True}})
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


def test_private_dataset_indexing_403_is_not_resubmitted(prepared, monkeypatch):
    output,_=prepared
    calls=[]
    def fake(*args):
        calls.append(args)
        if args[:2]==('datasets','status'):raise RuntimeError('403 Forbidden')
        return 'Your private Dataset is being created'
    monkeypatch.setattr(k,'cli',fake)
    assert k.submit(output)==2
    assert k.submit(output)==2
    assert len([c for c in calls if c[:2]==('datasets','create')])==1
    assert not [c for c in calls if c[:2]==('kernels','push')]


def test_dependency_tampering_is_rejected(prepared):
    output,state=prepared
    (output/'dataset/fake.whl').write_bytes(b'changed')
    with pytest.raises(ValueError,match='dependency changed'):
        k.validate(output,state)


def test_remote_setup_rejects_dependency_before_install(tmp_path, monkeypatch):
    from scripts import setup_kaggle_offline as offline
    monkeypatch.setattr(offline.sys, 'version_info', (3, 12))
    monkeypatch.setattr(offline.platform, 'machine', lambda: 'x86_64')
    calls=[]
    monkeypatch.setattr(offline.subprocess, 'run', lambda *a, **kw: calls.append(a))
    (tmp_path/'package.whl').write_bytes(b'tampered')
    with pytest.raises(ValueError, match='dependency hash'):
        offline.setup(tmp_path/'base', tmp_path, {'files': {'package.whl': '0'*64}})
    assert calls == []


def test_remote_setup_installs_offline_in_isolated_directory(tmp_path, monkeypatch):
    from scripts import setup_kaggle_offline as offline
    monkeypatch.setattr(offline.sys, 'version_info', (3, 12))
    monkeypatch.setattr(offline.platform, 'machine', lambda: 'x86_64')
    calls=[]
    monkeypatch.setattr(offline.subprocess, 'run', lambda command, **kw: calls.append(command))
    manifest={'files': {}}
    for name in ('pip-26.2.1-py3-none-any.whl', 'osmesa.deb'):
        path=tmp_path/name;path.write_bytes(b'package');manifest['files'][name]=k.digest(path)
    offline.setup(tmp_path/'base', tmp_path, manifest)
    install=next(c for c in calls if 'install' in c)
    assert '--no-index' in install and '--find-links' in install
    assert install[install.index('--python')+1] == str(tmp_path/'base/sim-env/bin/python')
    assert calls[0] == ['dpkg-deb', '-x', str(tmp_path/'osmesa.deb'), str(tmp_path/'base/system')]


def test_deb_transport_names_survive_kaggle_normalization(tmp_path, monkeypatch):
    from scripts import kaggle_offline_dependencies as deps
    root=tmp_path/'repo';(root/'configs').mkdir(parents=True)
    payload=b'deb package'
    k.write(root/'configs/kaggle-jammy-packages.json', {'libosmesa6': {
        'Package':'libosmesa6', 'Filename':'pool/libosmesa6_23~22.04.deb',
        'SHA256':hashlib.sha256(payload).hexdigest()}})
    wheelhouse=tmp_path/'wheels';wheelhouse.mkdir()
    (wheelhouse/'pip-26.2.1-py3-none-any.whl').write_bytes(b'pip')
    output=tmp_path/'out';output.mkdir()
    monkeypatch.setattr(deps.urllib.request,'urlretrieve',lambda url,path:path.write_bytes(payload))
    manifest=deps.prepare_dependencies(root,output,wheelhouse)
    assert 'libosmesa6.deb' in manifest['files']
    assert all('~' not in name for name in manifest['files'])


@pytest.mark.parametrize('raw', ['KernelWorkerStatus.CANCEL_ACKNOWLEDGED', 'cancelacknowledged', 'cancel_acknowledged'])
def test_cancelled_enum_reaches_failure_evidence_collection(prepared, monkeypatch, raw):
    output,state=prepared
    calls=[]
    def fake(*args):
        calls.append(args)
        if args[:2]==('kernels','status'):return f'kernel has status "{raw}"'
        if args[:2]==('kernels','pull'):
            target=Path(args[args.index('-p')+1])
            k.write(target/'kernel-metadata.json', {'is_private':True})
        return ''
    monkeypatch.setattr(k,'cli',fake)
    with pytest.raises(RuntimeError,match='no result ZIP'):k.collect(output)
    assert any(c[:2]==('kernels','output') for c in calls)
    assert json.loads((output/'job.json').read_text())['stage']=='remote_failed'


def test_reuse_private_inputs_gets_new_run_identity_without_dataset_upload(prepared, monkeypatch):
    old,original=prepared
    original['dataset_private_verified']=True
    k.write(old/'job.json',original)
    new=old.parent/'reuse'
    state=k.reuse_inputs(old,new,module='scripts.sim_quickstart',arguments=['--output','{output}'])
    assert state['job_id']!=original['job_id'] and state['kernel']!=original['kernel']
    assert state['dataset']==original['dataset']
    assert state['source_filename']=='ugrp-source-'+original['job_id']+'.bin'
    assert state['source_sha']==original['source_sha']
    calls=[]
    def fake(*args):
        calls.append(args)
        if args[:2]==('datasets','status'):return '{"status":"ready"}'
        if args[:2]==('datasets','metadata'):
            k.write(Path(args[-1])/'dataset-metadata.json',{'isPrivate':True})
        if args[:2]==('kernels','push'):return 'Kernel version 1 successfully pushed'
        return ''
    monkeypatch.setattr(k,'cli',fake)
    assert k.submit(new)==0
    assert not any(c[:2]==('datasets','create') for c in calls)


def test_git_delta_reproduces_exact_committed_source_and_rejects_tampering(tmp_path):
    import base64
    from scripts.colab_simulation_cli import pack
    from scripts.kaggle_source_delta import apply_source_delta, object_delta
    root=snapshot_repo(tmp_path)
    base=subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip()
    old=tmp_path/'old';pack(root,old)
    (root/'README.md').write_text('updated source')
    (root/'harness').mkdir();(root/'harness/new.py').write_text('fixed = True\n')
    subprocess.run(['git','add','.'],cwd=root,check=True)
    subprocess.run(['git','-c','user.name=Test','-c','user.email=test@example.invalid','commit','-qm','update'],cwd=root,check=True)
    new=tmp_path/'new';record=pack(root,new)
    bundle=tmp_path/'delta.bundle'
    bundle.write_bytes(object_delta(old/'source',new/'source'))
    delta={'base_sha':base,'target_sha':record['source_sha'],'included':record['included'],
           'sha256':k.digest(bundle),'content':base64.b64encode(bundle.read_bytes()).decode()}
    with pytest.raises(ValueError,match='hash'):apply_source_delta(old/'source',{**delta,'sha256':'0'*64})
    apply_source_delta(old/'source',delta)
    assert (old/'source/harness/new.py').read_text()=='fixed = True\n'
    assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=old/'source',text=True).strip()==record['source_sha']
