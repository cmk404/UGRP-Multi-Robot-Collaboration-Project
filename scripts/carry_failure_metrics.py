"""Descriptive, post-run failure counts; never used by the robot controller."""
from collections import Counter
import json
from pathlib import Path


def schedule(protocol, conditions):
    repeats = protocol.get('evaluation', {}).get('repeats', 1)
    if type(repeats) is not int or repeats < 1:
        raise ValueError('positive integer repeat count required')
    jobs = []
    for repeat in range(repeats):
        for index, case in enumerate(protocol['test']):
            shift = (repeat + index) % len(conditions)
            for condition in conditions[shift:] + conditions[:shift]:
                jobs.append({'case': case, 'condition': condition, 'repeat': repeat,
                             'trial_id': f"{condition}--{case['id']}--r{repeat}"})
    return jobs


def outcome(output, exit_code, timed_out=False):
    """Missing result is an operational failure, not a made-up robot verdict."""
    output = Path(output)
    try:
        result = json.loads((output/'result.json').read_text())
        if not isinstance(result, dict):
            raise ValueError('result is not an object')
    except (OSError, ValueError):
        return {'whole_success': False, 'robot_result_available': False,
                'failure_kind': 'timeout' if timed_out else 'missing_or_invalid_result',
                'carry_entered': None, 'act_carry_entered': None, 'beam_success': None}
    error = result.get('error')
    stop = str(result.get('stop_reason') or '')
    phase = str(result.get('phase') or 'unknown')
    success = (exit_code == 0 and not timed_out and result.get('physical_success') is True
               and result.get('protocol_complete') is True and not error
               and result.get('obstacle_contact_steps', 0) == 0)
    decisions = None
    try:
        decisions = json.loads((output/'pair-decisions.json').read_text())
        if not isinstance(decisions, list):
            decisions = None
    except (OSError, ValueError):
        pass
    beam = result.get('evaluation', {}).get('cargo', {}).get('beam', {})
    window = beam.get('carry_clearance', {}).get('window_s')
    act_entered = None if decisions is None else any(d.get('kind') == 'act_carry' for d in decisions)
    entered = True if window else (None if decisions is None else any(
        d.get('kind') in ('act_carry', 'carry', 'rotating_carry') for d in decisions))
    if success:
        kind = None
    elif timed_out or 'budget' in stop.lower() or any(term in str(error).lower() for term in ('timeout', 'budget exhausted')):
        kind = 'timeout'
    elif any(x in str(error) for x in ('model_http_error', 'model_transport_error')):
        kind = 'model_transport'
    elif error:
        kind = 'execution_error'
    else:
        kind = 'task_incomplete'
    return {'whole_success': success, 'robot_result_available': True,
            'failure_kind': kind, 'failure_phase': phase, 'error': error,
            'stop_reason': stop, 'carry_entered': entered, 'act_carry_entered': act_entered,
            'beam_success': beam.get('physical_success') is True,
            'wall_s': result.get('wall_s', result.get('wall_seconds'))}


def aggregate(jobs, rows):
    expected = {j['trial_id']: j for j in jobs}
    actual = {r['trial_id']: r for r in rows}
    if len(actual) != len(rows) or not actual.keys() <= expected.keys():
        raise ValueError('duplicate or unplanned trial result')
    groups = {}
    for condition in sorted({j['condition'] for j in jobs}):
        planned = [j for j in jobs if j['condition'] == condition]
        finished = [r['outcome'] for r in rows if r['condition'] == condition]
        entered = [r for r in finished if r['carry_entered'] is True]
        failures = sum(not r['whole_success'] for r in finished)
        groups[condition] = {
            'planned': len(planned), 'attempted': len(finished),
            'pending': len(planned)-len(finished), 'failures': failures,
            'failure_fraction_attempted': failures/len(finished) if finished else None,
            'failure_kinds': dict(Counter(r['failure_kind'] for r in finished if not r['whole_success'])),
            'failure_phases': dict(Counter(r.get('failure_phase', 'unknown') for r in finished if not r['whole_success'])),
            'robot_results_missing': sum(not r['robot_result_available'] for r in finished),
            'carry_entries': len(entered),
            'carry_entry_unknown': sum(r['carry_entered'] is None for r in finished),
            'act_carry_entries': sum(r['act_carry_entered'] is True for r in finished),
            'carry_failures': sum(not r['beam_success'] for r in entered),
            'carry_failure_fraction': sum(not r['beam_success'] for r in entered)/len(entered) if entered else None,
            'by_case': {case: {'attempted': len(rs), 'failures': sum(not r['outcome']['whole_success'] for r in rs)}
                        for case in sorted({j['case']['id'] for j in planned})
                        for rs in [[r for r in rows if r['condition'] == condition and r['case']['id'] == case]]},
        }
    return {'complete': len(actual) == len(expected), 'planned': len(expected), 'attempted': len(actual),
            'conditions': groups,
            'scope': 'Descriptive failure fractions on the fixed case suite. Repeats of a deterministic scene are not independent new environments; no population confidence claim. Pending trials are not failures or successes. Zero carry entries means conditional carry performance is unavailable.'}
