import copy
import json
import zipfile
import pytest
from scripts.colab_carry_bundle import digest, unpack, verify_dataset, write, source_identity


def fixture_bundle(tmp_path):
    root=tmp_path/'fixture';root.mkdir();image=root/'frame.jpg';image.write_bytes(b'image')
    canonical={'train':[{'root':'/original/train','files':{},'sequences':{'r1':[{'action':[1,0,0,0], 'images':{'own':{'path':'frame.jpg','sha256':digest(image)}}}]}}], 'development':[]}
    original=root/'canonical.json';write(original,canonical)
    source=root/'source.py';source.write_text('x=1\n')
    manifest={'source_sha':'a'*40,'dataset_sha256':digest(original),'roots':{'/original/train':'data/train'},'files':{'data/canonical.json':digest(original),'data/train/frame.jpg':digest(image),'source/example.py':digest(source)}}
    archive=tmp_path/'fixture.zip'
    with zipfile.ZipFile(archive,'w') as z:
        z.write(original,'data/canonical.json');z.write(image,'data/train/frame.jpg');z.write(source,'source/example.py')
        z.writestr('bundle.json',json.dumps(manifest));z.writestr('source/colab-source.json',json.dumps({'source_sha':'a'*40,'files':{'example.py':digest(source)}}))
    return archive


def test_relocation_preserves_identity_and_checks_labels(tmp_path):
    archive=fixture_bundle(tmp_path);out=tmp_path/'out'
    path=unpack(archive,out,digest(archive));verified=verify_dataset(path)
    assert verified['canonical_verified'] and verified['dataset_sha256']!=verified['relocated_dataset_sha256']
    data=json.loads(open(path).read());data['train'][0]['sequences']['r1'][0]['action'][0]=99
    write(out/'data/dataset.json',data)
    p=out/'data/dataset.provenance.json';prov=json.loads(p.read_text());prov['relocated_sha256']=digest(path);write(p,prov)
    with pytest.raises(ValueError,match='labels'):verify_dataset(path)


def test_corrupted_evidence_and_source_are_rejected(tmp_path):
    archive=fixture_bundle(tmp_path);out=tmp_path/'out';path=unpack(archive,out,digest(archive))
    (out/'data/train/frame.jpg').write_bytes(b'changed')
    with pytest.raises(ValueError,match='evidence'):verify_dataset(path)
    (out/'source/example.py').write_text('x=2')
    with pytest.raises(ValueError,match='source changed'):source_identity(out/'source')


@pytest.mark.parametrize('name',['../escape','/absolute'])
def test_archive_path_escape_rejected(tmp_path,name):
    archive=tmp_path/'bad.zip'
    with zipfile.ZipFile(archive,'w') as z:z.writestr(name,'bad')
    with pytest.raises(ValueError,match='unsafe'):unpack(archive,tmp_path/'out',digest(archive))
    assert not (tmp_path/'escape').exists()


def test_archive_hash_rejected_before_extract(tmp_path):
    archive=fixture_bundle(tmp_path)
    with pytest.raises(ValueError,match='digest'):unpack(archive,tmp_path/'out','0'*64)
    assert not (tmp_path/'out').exists()
