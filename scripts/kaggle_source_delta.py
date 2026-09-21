"""Transfer committed source changes through the private kernel, reusing input bytes."""
import base64
import hashlib
from pathlib import Path
import subprocess


def apply_source_delta(source, delta):
    bundle = source.parent/'source-update.bundle'
    content = base64.b64decode(delta['content'], validate=True)
    if hashlib.sha256(content).hexdigest() != delta['sha256']:
        raise ValueError('source delta hash mismatch')
    base = subprocess.check_output(['git','rev-parse','HEAD'],cwd=source,text=True).strip()
    if base != delta['base_sha']:
        raise ValueError('source delta base mismatch')
    bundle.write_bytes(content)
    subprocess.run(['git','bundle','verify',str(bundle)],cwd=source,check=True)
    subprocess.run(['git','fetch',str(bundle),delta['target_sha']],cwd=source,check=True)
    paths = delta['included']
    if any(Path(p).is_absolute() or '..' in Path(p).parts or '\n' in p for p in paths):
        raise ValueError('unsafe sparse path')
    patterns=''.join('/'+p.replace(' ','\\ ').replace('[','\\[').replace('*','\\*').replace('?','\\?')+'\n' for p in paths)
    subprocess.run(['git','sparse-checkout','set','--no-cone','--stdin'],input=patterns,text=True,cwd=source,check=True)
    subprocess.run(['git','checkout','--detach',delta['target_sha']],cwd=source,check=True)
    if subprocess.check_output(['git','status','--porcelain'],cwd=source).strip():
        raise ValueError('patched source is dirty')
