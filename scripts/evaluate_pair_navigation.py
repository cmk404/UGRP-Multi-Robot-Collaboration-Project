"""Pure output-only evaluation; never imported by the navigation actor."""
from __future__ import annotations
import math
from harness.pair_navigation import ROBOTS, wrap


def evaluate_grasp_stability(samples):
    """Whole lift/hold assessment; values never enter an actor or transition."""
    close = [r for r in samples if r['phase']=='grasp_close']
    rows = [r for r in samples if r['phase'] in ('grasp_lift','grasp_hold')]
    hold = [r for r in rows if r['phase']=='grasp_hold']
    if not close or not rows or not hold or any('bases' not in r for r in [close[-1],*rows]):
        return {'success':False,'reason':'complete close/lift/hold samples required','gates':{'full_grasp_samples':False}}
    distance = lambda r:math.dist(r['bases']['r1'][:2],r['bases']['r3'][:2])
    initial = distance(close[-1])
    maximum = max(abs(distance(r)-initial) for r in rows)
    nonbilateral = sum(not all(r['contacts'][rid]['bilateral'] for rid in ROBOTS) for r in rows)
    gap = max(b['sim_time_s']-a['sim_time_s'] for a,b in zip([close[-1],*rows],rows))
    gates = {'full_grasp_samples':len(hold)>=100 and gap<=.15,
             'grasp_spacing_preserved':maximum<=.02,
             'grasp_bilateral_through_lift_hold':nonbilateral==0,
             'grasp_hold_lifted_every_sample':all(r['height_above_start_m']>=.03 for r in hold)}
    return {'success':all(gates.values()),'gates':gates,'anchor_spacing_m':initial,
            'final_spacing_m':distance(rows[-1]),'max_spacing_change_m':maximum,
            'nonbilateral_samples':nonbilateral,'samples':len(rows),'max_sample_gap_s':gap}


def evaluate_samples(samples, data, *, arrived, invariants_match, weld_ticks,
                     wall_contact_ticks, unexpected_contact_ticks=0, require_full_grasp=False):
    rows = [r for r in samples if r['phase'] in ('carry', 'carry_stop')]
    if not rows:
        return {'success': False, 'reason': 'no navigation samples'}
    first, last = rows[0], rows[-1]
    bilateral = lambda r: all(r['contacts'][rid]['bilateral'] for rid in ROBOTS)
    grip = [bilateral(r) for r in rows]
    lifted = [r['height_above_start_m'] >= .03 for r in rows]
    pre = [r for r in samples if r['phase'] == 'grasp_hold'
           and first['sim_time_s']-2.01 <= r['sim_time_s'] < first['sim_time_s']]
    release = [r for r in samples if r['phase'] == 'release_hold']
    angle = wrap(last['yaw_rad']-first['yaw_rad'])
    goal = data['goal']
    position_error = math.dist(last['position_m'][:2], goal['center_m'])
    yaw_error = abs(wrap(angle-math.radians(goal['relative_yaw_deg'])))
    gap = max((b['sim_time_s']-a['sim_time_s'] for a,b in zip(rows,rows[1:])), default=0.)
    released = bool(release and all(r['payload_floor_contact'] and
                    all(not r['contacts'][rid][side] for rid in ROBOTS for side in ('left','right'))
                    for r in release[-10:]))
    gates = {'actor_arrived': arrived,
             'grasp_stable_before_departure': len(pre) >= 19 and all(bilateral(r) and r['height_above_start_m'] >= .03 for r in pre),
             'bilateral_every_sample': all(grip), 'lifted_every_sample': all(lifted),
             'continuous_samples': len(rows) >= 2 and gap <= .15,
             'level_payload': all(r['tilt_deg'] <= 10 for r in rows),
             'wall_contact_free': wall_contact_ticks == 0,
             'other_collision_free': unexpected_contact_ticks == 0,
             'within_authored_bounds': all(r.get('within_authored_bounds', False) for r in rows),
             'weld_off': weld_ticks == 0 and all(not any(r['constraints_active'].values()) for r in samples),
             'cameras_geometry_unchanged': invariants_match,
             'goal_position': position_error <= .08,
             'goal_orientation': yaw_error <= math.radians(10), 'released_on_floor': released}
    if require_full_grasp:
        stability=evaluate_grasp_stability(samples)
        gates.update(stability['gates'])
        anchor=stability.get('anchor_spacing_m')
        gates['carry_spacing_preserved'] = bool(anchor is not None and all('bases' in r and
            abs(math.dist(r['bases']['r1'][:2],r['bases']['r3'][:2])-anchor)<=.02 for r in rows))
    return {'success': all(gates.values()), 'gates': gates, 'samples': len(rows),
            'bilateral_fraction': sum(grip)/len(rows), 'lifted_fraction': sum(lifted)/len(rows),
            'min_lift_m': min(r['height_above_start_m'] for r in rows),
            'payload_turn_deg': math.degrees(angle), 'goal_position_error_m': position_error,
            'goal_yaw_error_deg': math.degrees(yaw_error), 'wall_contact_ticks': wall_contact_ticks,
            'unexpected_contact_ticks': unexpected_contact_ticks,
            'navigation_sim_seconds': last['sim_time_s']-first['sim_time_s'],
            'sample_gap_max_s': gap}
