import json
import pytest
from scripts.cloud_collection import checkpoint_trial, verify_checkpoint, RemoteHealth


def test_complete_failed_trial_preserves_raw_and_detects_corruption(tmp_path):
    trial = tmp_path/'failed-trial'; trial.mkdir()
    (trial/'result.json').write_text(json.dumps({'source_sha':'abc', 'success':False}))
    (trial/'input.jpg').write_bytes(b'actual input fixture')
    manifest = checkpoint_trial(trial, tmp_path/'checkpoints', 'abc')
    archive = tmp_path/'checkpoints'/manifest['archive']
    assert verify_checkpoint(archive, manifest, 'abc')['success'] is False
    with pytest.raises(FileExistsError): checkpoint_trial(trial, tmp_path/'checkpoints', 'abc')
    with pytest.raises(ValueError): verify_checkpoint(archive, manifest, 'different')
    archive.write_bytes(archive.read_bytes()[:-1]+b'x')
    with pytest.raises(ValueError): verify_checkpoint(archive, manifest, 'abc')


def test_incomplete_and_symlink_not_published(tmp_path):
    trial = tmp_path/'trial'; trial.mkdir()
    with pytest.raises(FileNotFoundError): checkpoint_trial(trial, tmp_path/'out', 'abc')
    (trial/'result.json').write_text(json.dumps({'source_sha':'abc'}))
    (trial/'link').symlink_to(trial/'result.json')
    with pytest.raises(ValueError): checkpoint_trial(trial, tmp_path/'out', 'abc')
    assert not list((tmp_path/'out').glob('*.json'))


def test_remote_failure_never_keeps_running_and_preserves_last_success():
    health = RemoteHealth(limit=2)
    health.success('running', now=100)
    assert health.failure(FileNotFoundError(), now=110) is False
    assert health.state['remote_state']=='unreachable'
    assert health.state['last_success_unix']==100
    assert health.failure(TimeoutError(), now=120) is True
    health.success('complete', now=130)
    assert health.state['consecutive_errors']==0
    assert health.state['remote_state']=='complete'
