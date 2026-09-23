"""Immutable completed-trial checkpoints and explicit remote observation health."""
from __future__ import annotations
import hashlib
import json
from pathlib import Path
import time
import zipfile


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def write_json(path, value):
    path = Path(path)
    temp = path.with_suffix(path.suffix + '.tmp')
    temp.write_text(json.dumps(value, indent=2) + '\n')
    temp.replace(path)


def checkpoint_trial(trial, destination, source_sha):
    """Call only after the trial has closed all outputs and finalized result.json."""
    trial, destination = Path(trial), Path(destination)
    result = json.loads((trial / 'result.json').read_text())
    if result['source_sha'] != source_sha:
        raise ValueError('trial source mismatch')
    destination.mkdir(parents=True, exist_ok=True)
    manifest = destination / (trial.name + '.json')
    if manifest.exists():
        raise FileExistsError('completed checkpoint is immutable')
    files = sorted(p for p in trial.rglob('*') if p.is_file())
    if any(p.is_symlink() for p in trial.rglob('*')):
        raise ValueError('checkpoint symlink is not permitted')
    artifacts = {str(p.relative_to(trial)): digest(p) for p in files}
    archive = destination / (trial.name + '.zip')
    temp = archive.with_suffix('.zip.tmp')
    with zipfile.ZipFile(temp, 'w', zipfile.ZIP_DEFLATED) as bundle:
        for p in files:
            bundle.write(p, str(p.relative_to(trial)))
        bundle.writestr('checkpoint.json', json.dumps({'source_sha': source_sha, 'trial_id': trial.name, 'artifacts': artifacts}))
    # A late writer is an error, never a successfully published checkpoint.
    if any(digest(trial / name) != value for name, value in artifacts.items()):
        temp.unlink()
        raise ValueError('trial changed during checkpoint')
    temp.replace(archive)
    record = {'source_sha': source_sha, 'trial_id': trial.name, 'archive': archive.name,
              'sha256': digest(archive), 'bytes': archive.stat().st_size,
              'artifact_count': len(artifacts), 'complete': True}
    write_json(manifest, record)
    return record


def verify_checkpoint(archive, manifest, source_sha):
    archive = Path(archive)
    if manifest.get('source_sha') != source_sha or manifest.get('complete') is not True:
        raise ValueError('checkpoint source or completion mismatch')
    if archive.stat().st_size != manifest['bytes'] or digest(archive) != manifest['sha256']:
        raise ValueError('checkpoint archive mismatch')
    with zipfile.ZipFile(archive) as bundle:
        info = json.loads(bundle.read('checkpoint.json'))
        if info['source_sha'] != source_sha or info['trial_id'] != manifest['trial_id']:
            raise ValueError('checkpoint identity mismatch')
        if len(info['artifacts']) != manifest['artifact_count']:
            raise ValueError('checkpoint artifact count mismatch')
        for name, expected in info['artifacts'].items():
            if Path(name).is_absolute() or '..' in Path(name).parts:
                raise ValueError('unsafe checkpoint member')
            if hashlib.sha256(bundle.read(name)).hexdigest() != expected:
                raise ValueError('checkpoint member mismatch')
        result = json.loads(bundle.read('result.json'))
        if result['source_sha'] != source_sha:
            raise ValueError('result source mismatch')
        return result


class RemoteHealth:
    def __init__(self, limit=3):
        self.limit = limit
        self.state = {'remote_state': 'unverified', 'consecutive_errors': 0,
                      'last_success_unix': None}

    def success(self, remote_state, now=None):
        self.state.update(remote_state=remote_state, consecutive_errors=0,
                          last_success_unix=time.time() if now is None else now)
        self.state.pop('last_error_type', None)

    def failure(self, exc, now=None):
        self.state.update(remote_state='unreachable', last_error_type=type(exc).__name__,
                          last_attempt_unix=time.time() if now is None else now)
        self.state['consecutive_errors'] += 1
        return self.state['consecutive_errors'] >= self.limit
