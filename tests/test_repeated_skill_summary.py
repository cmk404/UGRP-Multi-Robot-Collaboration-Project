import hashlib
import json
import pytest
from scripts.summarize_repeated_skill_cohort import summarize


def snapshot(tmp_path):
    jobs = [{'case': 'open', 'policy': 'jev', 'arm': 'full', 'repeat': i, 'trial_id': f'open-{i}'} for i in range(3)]
    (tmp_path/'phase-templates.json').write_text(json.dumps({'actor_source_sha': 'actor', 'phases': {
        'holdout': {'protocol': {'jobs': jobs}}}}))
    result = {**jobs[0], 'source_sha': 'actor', 'partition': 'holdout', 'success': False,
              'error': {'message': 'model_http_error'}}
    raw = json.dumps(result).encode();(tmp_path/'result.json').write_bytes(raw)
    row = {'phase': 'holdout', 'trial_id': 'open-0', 'result_file': 'result.json',
           'result_sha256': hashlib.sha256(raw).hexdigest()}
    index = {'source_sha': 'actor', 'snapshot_unix': 1, 'rows': [row]}
    (tmp_path/'index.json').write_text(json.dumps(index))
    return index


def test_all_attempts_and_pending_have_separate_denominators(tmp_path):
    snapshot(tmp_path)
    report = summarize(tmp_path);group = report['groups'][0]
    assert (report['planned'], report['attempted'], report['pending']) == (3, 1, 2)
    assert not report['complete'] and group['failure_fraction_attempted'] == 1
    assert group['failure_reasons'] == {'model_http_error': 1}
    assert group['by_case']['open'] == {'planned': 3, 'attempted': 1, 'successes': 0, 'failures': 1}


def test_duplicate_and_changed_result_cannot_inflate_failure_count(tmp_path):
    index = snapshot(tmp_path);index['rows'] *= 2
    (tmp_path/'index.json').write_text(json.dumps(index))
    with pytest.raises(ValueError, match='duplicate'):summarize(tmp_path)
    snapshot(tmp_path);(tmp_path/'result.json').write_text('{}')
    with pytest.raises(ValueError, match='hash'):summarize(tmp_path)
