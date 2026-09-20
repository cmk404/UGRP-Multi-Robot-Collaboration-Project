import pytest
from scripts.summarize_carry_resolution import rank

ARMS = [{'id': 'r128-h1', 'size': 128, 'history': 1}, {'id': 'r512-h4', 'size': 512, 'history': 4}]
CASES = [{'id': 'a'}, {'id': 'b'}]


def rows():
    return [{'condition': f"{a['id']}-s{s}", 'case': c['id'], 'completed_report': True,
             'whole_success': False, 'beam_success': False, 'wall_s': 100,
             'inference_wall_s': {'p95': .1 if a['size'] == 128 else .5}}
            for a in ARMS for s in (1, 2) for c in CASES]


def test_zero_success_never_promoted():
    result = rank(rows(), ARMS, [1, 2], CASES)
    assert result['decision'] == 'no_usable_optimum' and result['selected_arm'] is None


def test_consistency_precedes_aggregate_and_latency():
    values = rows()
    for r in values:
        r['whole_success'] = (r['condition'] == 'r128-h1-s1' or
                              r['condition'].startswith('r512-h4') and r['case'] == 'a')
        r['beam_success'] = r['whole_success']
    result = rank(values, ARMS, [1, 2], CASES)
    assert result['selected_arm'] == ARMS[1]
    assert result['decision'] == 'fresh_confirmation_required'


@pytest.mark.parametrize('change', ['missing', 'duplicate', 'unreported'])
def test_incomplete_evidence_rejected(change):
    values = rows()
    if change == 'missing': values.pop()
    elif change == 'duplicate': values.append(values[0])
    else: values[0]['completed_report'] = False
    with pytest.raises(ValueError): rank(values, ARMS, [1, 2], CASES)
