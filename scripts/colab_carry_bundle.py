"""Portable, hash-verified ACT training data/source; never mounts Google Drive."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import subprocess
import zipfile

ROOT = Path(__file__).resolve().parents[1]
PROTOCOL = 'experiments/2026-09-21-carry-input-ablation/protocol.json'


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(1024*1024), b''): h.update(block)
    return h.hexdigest()


def write(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n')


def confined(root, relative):
    p = Path(relative)
    if p.is_absolute() or '..' in p.parts: raise ValueError('unsafe relative path')
    result = (root/p).resolve()
    if not result.is_relative_to(root.resolve()) or (root/p).is_symlink(): raise ValueError('outside source')
    return result


def source_identity(root=ROOT):
    marker = root/'colab-source.json'
    if marker.exists():
        m = json.loads(marker.read_text())
        for rel, expected in m['files'].items():
            if digest(confined(root, rel)) != expected: raise ValueError('source changed: '+rel)
        return m['source_sha']
    if subprocess.check_output(['git','status','--porcelain'], cwd=root): raise ValueError('commit source before training')
    return subprocess.check_output(['git','rev-parse','HEAD'], cwd=root, text=True).strip()


def pack(output, protocol_path=ROOT/PROTOCOL):
    source_sha = source_identity()
    protocol_path = Path(protocol_path).resolve()
    protocol_relative = str(protocol_path.relative_to(ROOT.resolve()))
    protocol = json.loads(protocol_path.read_text())
    dataset = Path(protocol['dataset']); canonical = json.loads(dataset.read_text())
    if digest(dataset) != protocol['dataset_sha256']: raise ValueError('canonical dataset changed')
    members = {'data/canonical.json': dataset, 'data/protocol.json': protocol_path}
    source_files = subprocess.check_output(['git','ls-files','harness','scripts','requirements-reference-act.txt'], cwd=ROOT, text=True).splitlines()
    source_manifest = {'source_sha': source_sha, 'files': {}}
    for name in source_files:
        if name.endswith(('.py','.txt')):
            path = confined(ROOT,name);members['source/'+name] = path;source_manifest['files'][name]=digest(path)
    source_manifest['files'][protocol_relative]=digest(protocol_path);members['source/'+protocol_relative]=protocol_path
    roots = {}
    for split in ('train','development'):
        for i,e in enumerate(canonical[split]):
            root = Path(e['root']).resolve(); portable=f'data/episodes/{split}-{i}'
            if str(root) in roots: raise ValueError('episode shared across splits')
            roots[str(root)] = portable
            expected = dict(e['files'])
            for seq in e['sequences'].values():
                for row in seq:
                    for ref in row['images'].values():
                        old=expected.setdefault(ref['path'],ref['sha256'])
                        if old!=ref['sha256']: raise ValueError('conflicting image hash')
            for name, sha in expected.items():
                path=confined(root,name)
                if digest(path)!=sha: raise ValueError('source hash mismatch: '+name)
                members[portable+'/'+name]=path
    manifest={'schema':'ugrp.colab-carry.v1','source_sha':source_sha,'dataset_sha256':digest(dataset),
              'protocol_sha256':digest(protocol_path),'protocol_path':protocol_relative,'roots':roots,'files':{n:digest(p) for n,p in members.items()}}
    output=Path(output)
    output.parent.mkdir(parents=True,exist_ok=True)
    with zipfile.ZipFile(output,'x',compression=zipfile.ZIP_DEFLATED,compresslevel=1) as z:
        for name,path in members.items():z.write(path,name)
        z.writestr('source/colab-source.json',json.dumps(source_manifest))
        z.writestr('bundle.json',json.dumps(manifest))
    record={'path':str(output.resolve()),'sha256':digest(output),'bytes':output.stat().st_size,
            'source_sha':source_sha,'dataset_sha256':manifest['dataset_sha256'],'files':len(members)}
    write(output.with_suffix('.manifest.json'),record)
    return record


def unpack(archive, output, expected_sha):
    archive,output=Path(archive),Path(output)
    if digest(archive)!=expected_sha:raise ValueError('archive digest mismatch')
    if output.exists():raise ValueError('use a new extraction directory')
    with zipfile.ZipFile(archive) as z:
        names=z.namelist()
        if len(names)!=len(set(names)):raise ValueError('duplicate members')
        for info in z.infolist():
            confined(output,info.filename)
            if (info.external_attr>>16)&0o170000==0o120000:raise ValueError('archive symlink')
        if sum(i.file_size for i in z.infolist())>4*1024**3:raise ValueError('oversized bundle')
        z.extractall(output)
    manifest=json.loads((output/'bundle.json').read_text())
    for name,sha in manifest['files'].items():
        if digest(confined(output,name))!=sha:raise ValueError('bundle member mismatch: '+name)
    canonical_path=output/'data/canonical.json'
    if digest(canonical_path)!=manifest['dataset_sha256']:raise ValueError('canonical mismatch')
    canonical=json.loads(canonical_path.read_text());relocated=copy.deepcopy(canonical)
    for split in ('train','development'):
        for entry in relocated[split]:entry['root']=str(confined(output,manifest['roots'][entry['root']]))
    local_path=output/'data/dataset.json';write(local_path,relocated)
    provenance={'schema':'ugrp.dataset-relocation.v1','canonical_path':str(canonical_path.resolve()),
                'canonical_sha256':digest(canonical_path),'relocated_sha256':digest(local_path),
                'root_map':{k:str(confined(output,v)) for k,v in manifest['roots'].items()}}
    write(output/'data/dataset.provenance.json',provenance)
    verify_dataset(local_path)
    source_identity(output/'source')
    return str(local_path)


def verify_dataset(path):
    path=Path(path); marker=path.with_suffix('.provenance.json')
    if not marker.exists():return {'dataset_sha256':digest(path),'canonical_verified':False}
    p=json.loads(marker.read_text());canonical_path=Path(p['canonical_path'])
    if digest(path)!=p['relocated_sha256'] or digest(canonical_path)!=p['canonical_sha256']:raise ValueError('dataset manifest changed')
    canonical=json.loads(canonical_path.read_text());expected=copy.deepcopy(canonical)
    for split in ('train','development'):
        for entry in expected[split]:entry['root']=p['root_map'][entry['root']]
    actual=json.loads(path.read_text())
    if actual!=expected:raise ValueError('relocation changed labels, sequence or split')
    # Check every evidence file and image before using the canonical identity.
    for split in ('train','development'):
        for e in actual[split]:
            files=dict(e['files'])
            for seq in e['sequences'].values():
                for row in seq:
                    for ref in row['images'].values():files[ref['path']]=ref['sha256']
            for rel,sha in files.items():
                if digest(confined(Path(e['root']),rel))!=sha:raise ValueError('relocated evidence changed')
    return {'dataset_sha256':p['canonical_sha256'],'relocated_dataset_sha256':p['relocated_sha256'],
            'canonical_verified':True,'relocation_provenance':p}


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='command',required=True)
    a=sub.add_parser('pack');a.add_argument('--output',type=Path,required=True);a.add_argument('--protocol',type=Path,default=ROOT/PROTOCOL)
    a=sub.add_parser('unpack');a.add_argument('--archive',type=Path,required=True);a.add_argument('--output',type=Path,required=True);a.add_argument('--sha256',required=True)
    args=p.parse_args()
    print(json.dumps(pack(args.output,args.protocol) if args.command=='pack' else unpack(args.archive,args.output,args.sha256),indent=2))
