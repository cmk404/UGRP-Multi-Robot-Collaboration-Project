"""Transfer committed source changes through the private kernel, reusing input bytes."""
import base64
import hashlib
from pathlib import Path
import subprocess


def apply_source_delta(source, delta):
    bundle = source.parent/'source-update.pack'
    content = base64.b64decode(delta['content'], validate=True)
    if hashlib.sha256(content).hexdigest() != delta['sha256']:
        raise ValueError('source delta hash mismatch')
    base = subprocess.check_output(['git','rev-parse','HEAD'],cwd=source,text=True).strip()
    if base != delta['base_sha']:
        raise ValueError('source delta base mismatch')
    if subprocess.check_output(['git','status','--porcelain'],cwd=source).strip():
        raise ValueError('source delta requires a clean disposable snapshot')
    bundle.write_bytes(content)
    subprocess.run(['git','index-pack','--stdin'],input=content,cwd=source,check=True,stdout=subprocess.DEVNULL)
    paths = delta['included']
    if any(Path(p).is_absolute() or '..' in Path(p).parts or '\n' in p for p in paths):
        raise ValueError('unsafe sparse path')
    patterns=''.join('/'+p.replace(' ','\\ ').replace('[','\\[').replace('*','\\*').replace('?','\\?')+'\n' for p in paths)
    # Updating patterns through sparse-checkout first re-applies them to the
    # old shallow tree, whose newly requested blobs need not exist. The base
    # is already sparse; write its patterns before checking out the new tree.
    sparse = source/'.git/info/sparse-checkout'
    sparse.parent.mkdir(parents=True, exist_ok=True)
    sparse.write_text(patterns)
    (source/'.git/shallow').write_text(delta['target_sha']+'\n')
    # The disposable upload checkout can contain index entries for omitted old
    # blobs. Build its index directly from the verified target tree so checkout
    # never attempts to refresh unavailable base-tree files.
    subprocess.run(['git','read-tree','--reset','-u',delta['target_sha']],cwd=source,check=True)
    subprocess.run(['git','update-ref','--no-deref','HEAD',delta['target_sha']],cwd=source,check=True)
    if subprocess.check_output(['git','status','--porcelain'],cwd=source).strip():
        raise ValueError('patched source is dirty')


def object_delta(base_source, target_source):
    """Pack only new objects from a complete sparse target snapshot, no ancestry prerequisites."""
    def objects(root):
        return set(subprocess.check_output(['git','cat-file','--batch-all-objects','--batch-check=%(objectname)'],cwd=root,text=True).splitlines())
    added=sorted(objects(target_source)-objects(base_source))
    return subprocess.check_output(['git','pack-objects','--stdout'],input=('\n'.join(added)+'\n').encode(),cwd=target_source)
