"""Post-run evaluation only. Never expose referee samples to an actor."""
from __future__ import annotations


def carry_clearance(samples, command_history, plan):
    """Check sampled ground clearance throughout each issued TRANSIT window.

    A final placement cannot establish continuous transport. Missing commands,
    missing coverage, or a floor contact make this evidence insufficient. The
    result concerns recorded samples, not unobserved instants between samples.
    """
    results = {}
    for task in (plan or {}).get('tasks', []):
        obj = task['object']
        commands = [c for rid in task['participants']
                    for c in command_history.get(rid, [])
                    if c.get('stage') == 'TRANSIT' and 'issued_at_s' in c]
        report = {'sampled_continuous_clearance': False, 'samples': 0,
                  'floor_contact_samples': 0, 'window_s': None,
                  'scope': 'Post-run referee samples only; never actor input. '
                           'No claim about contact between samples.'}
        if commands:
            start = min(c['issued_at_s'] for c in commands)
            end = max(c['issued_at_s'] + c.get('action', {}).get(
                'duration_s', c.get('duration_s', 0.)) for c in commands)
            window = [s for s in samples if start <= s['sim_time_s'] <= end]
            times = [start] + [s['sim_time_s'] for s in window] + [end]
            max_gap = max(b-a for a, b in zip(times, times[1:]))
            floor = [s for s in window if s['cargo'][obj]['floor_contact']]
            report.update(window_s=[start, end], samples=len(window),
                          floor_contact_samples=len(floor), max_sample_gap_s=max_gap,
                          first_floor_contact_s=floor[0]['sim_time_s'] if floor else None,
                          sampled_continuous_clearance=bool(
                              len(window) >= 2 and max_gap <= .11 and not floor))
        results[obj] = report
    return results
