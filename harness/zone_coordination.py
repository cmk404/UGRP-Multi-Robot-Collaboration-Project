"""LLM coordination for the zone benchmark: plan-first vs. talk-when-needed.

Robots see their own RGB, every TOP RGB image of the map, RGB-derived box labels and zone
counts, their own issued jobs and peer messages. They never see simulator poses
or the referee. Motions are executed by a ground-truth TEACHER executor; the
coordination layer only receives its command receipts (finished / stopped).

plan_first: the team negotiates one complete assignment (every box and zone
for every robot, in order) before any motion, with the existing rotating
proposer and unanimous ACK.
dynamic: whenever robots are idle, each idle robot claims ONE next job for
itself; the host only checks the claims against the goal, the RGB view and the
active peer claims. Robots talk only when their claims collide.
"""
from __future__ import annotations

import base64
import copy
import json

from harness.three_robot_plan import ROBOTS, digest, parse, text_fields

_COMMON = '''You are robot {rid}, an equal peer in a three-robot team (r1, r2, r3).
Mission: {instruction}
Goal (zone -> colour -> count): {goal}
The pickup area is west, zones A, B, C are painted floor areas east. You carry
one box at a time. A motion executor moves you when you are given a job; you
coordinate WHICH box goes to WHICH zone and WHO does it. Ground choices in the
images: {images}. box_labels are
fixed names from the first TOP images (per colour, west to east then south to
north). rgb_view is the current TOP-RGB estimate, not ground truth. Never claim
simulator coordinates or physical success.'''

_PLAN = _COMMON + '''
PLAN FIRST: agree on the COMPLETE assignment before anyone moves. Only
agreement.proposer may propose while agreement.proposal is null; the others
then reply accept=false, plan=null with a useful peer message. Once
agreement.proposal exists, every robot (the proposer too) either ACCEPTS by
replying accept=true with plan set to an exact copy of agreement.proposal.plan,
or REJECTS with accept=false, plan=null and a reason. accept=true with
plan=null is invalid. Copy proposal_id and plan_hash from agreement.proposal.
A plan lists, for every robot, its ordered jobs {{"box": label, "zone": "A|B|C"}}
so that every zone receives exactly its goal count per colour, each box is used
at most once and each box colour matches the zone's need. Balance the work;
robots run their lists in parallel.
Reply JSON only: {{"request_id": copied, "proposal_id": copied or null,
"plan_hash": copied or null, "accept": true|false, "plan": {{"assignments":
{{"r1": [...], "r2": [...], "r3": [...]}}}} (your proposal, or the exact accepted
plan) or null when rejecting/waiting, "reason": "brief",
"message": "brief message to peers"}}. reason/message under 240 characters.'''

_CLAIM = _COMMON + '''
TALK WHEN NEEDED: there is no global plan. You are idle now: claim ONE next job
for yourself, or null if nothing useful remains for you. Choose a box that is
still in pickup (rgb_view.pickup_boxes_still_visible), not claimed in
team_board.active, and a zone that still needs that colour (goal minus
rgb_view.zone_counts_seen minus active peer claims to that zone and colour).
Prefer jobs that keep the team busy and avoid peers' current paths. If
conflict is present, you and a peer claimed the same box or the same last need:
agree who takes it (read peer_messages) and choose again. Everyone answers at
once, so if you all yield nobody takes it: unless a peer's message already
gives it to a specific robot, the conflicting robot with the lowest robot_id
keeps it and the others choose a different job or null.
Reply JSON only: {{"request_id": copied, "claim": {{"box": label or null,
"zone": "A|B|C" or null}}, "reason": "brief", "message": "brief message to
peers"}}. reason/message under 240 characters.'''


# zone_open TOP views; other maps pass sim.zone_arena.top_views(static_map).
DEFAULT_VIEWS = (('cctv_top', 'TOP_WEST', 'top_west', 'top-west', 'pickup'),
                 ('cctv_top_east', 'TOP_EAST', 'top_east', 'top-east', 'zones'))


def _images(frame, views):
    return [{'label': label, 'image': 'data:image/jpeg;base64,' + base64.b64encode(frame[key]).decode()}
            for label, key in (('CURRENT OWN RGB', 'own'), *((v[1], v[2]) for v in views))]


def _image_text(views):
    names = [f'{label} ({role})' for _, label, _, _, role in views]
    return 'CURRENT OWN RGB, ' + ', '.join(names[:-1]) + ' and ' + names[-1]


def context(rid, *, task, labels, view, board, own_jobs, inbox, extra=None):
    value = {'robot_id': rid, 'box_labels': {k: {'kind': v['kind'], 'rgb_floor_xy_m': v['floor_xy_m']}
                                             for k, v in labels.items()},
             'rgb_view': copy.deepcopy(view), 'team_board': copy.deepcopy(board),
             'own_jobs': copy.deepcopy(list(own_jobs)[-8:]), 'peer_messages': copy.deepcopy(list(inbox)[-8:])}
    value.update(extra or {})
    return value


def _system(template, rid, task, views):
    return template.format(rid=rid, instruction=task['instruction'],
                           goal=json.dumps(task['goal'], sort_keys=True), images=_image_text(views))


def build_plan_request(rid, *, request_id, task, frame, agreement, ctx, views=DEFAULT_VIEWS):
    body = {'request_id': request_id, 'agreement': copy.deepcopy(agreement), **ctx}
    return {'request_id': request_id,
            'messages': [{'role': 'system', 'content': _system(_PLAN, rid, task, views)},
                         {'role': 'user', 'content': json.dumps(body, sort_keys=True)}],
            'images': _images(frame, views)}


def build_claim_request(rid, *, request_id, task, frame, ctx, views=DEFAULT_VIEWS):
    body = {'request_id': request_id, **ctx}
    return {'request_id': request_id,
            'messages': [{'role': 'system', 'content': _system(_CLAIM, rid, task, views)},
                         {'role': 'user', 'content': json.dumps(body, sort_keys=True)}],
            'images': _images(frame, views)}


def plan_validator(goal, labels):
    """Complete assignment that meets the goal exactly with distinct, matching boxes."""
    def validate(plan):
        if not isinstance(plan, dict) or set(plan) != {'assignments'}:
            raise ValueError('plan requires assignments only')
        assignments = plan['assignments']
        if not isinstance(assignments, dict) or set(assignments) != set(ROBOTS):
            raise ValueError('assignments need r1, r2 and r3 (empty lists allowed)')
        used, delivered = set(), {}
        for rid, jobs in assignments.items():
            if not isinstance(jobs, list):
                raise ValueError('each robot needs a job list')
            for job in jobs:
                if not isinstance(job, dict) or set(job) != {'box', 'zone'}:
                    raise ValueError('each job needs box and zone only')
                if job['box'] not in labels or job['zone'] not in goal:
                    raise ValueError(f'unknown box label or zone: {job}')
                if job['box'] in used:
                    raise ValueError(f"box {job['box']} is used twice")
                used.add(job['box'])
                kind = labels[job['box']]['kind']
                delivered.setdefault(job['zone'], {}).setdefault(kind, 0)
                delivered[job['zone']][kind] += 1
        if delivered != goal:
            raise ValueError(f'plan delivers {json.dumps(delivered, sort_keys=True)}, goal is '
                             f'{json.dumps(goal, sort_keys=True)}')
        return copy.deepcopy(plan)
    return validate


def validate_claim_reply(raw, request_id):
    value = parse(raw) if isinstance(raw, str) else copy.deepcopy(raw)
    if not isinstance(value, dict) or set(value) != {'request_id', 'claim', 'reason', 'message'}:
        raise ValueError('claim reply requires request_id, claim, reason and message only')
    if value['request_id'] != request_id:
        raise ValueError('stale claim reply')
    text_fields(value)
    claim = value['claim']
    if not isinstance(claim, dict) or set(claim) != {'box', 'zone'}:
        raise ValueError('claim requires box and zone')
    if (claim['box'] is None) != (claim['zone'] is None):
        raise ValueError('claim box and zone are both set or both null')
    return value


def remaining_need(goal, view, active, finished=None):
    """Goal minus zone counts seen in RGB minus active claims (never below zero).

    finished: {'zone', 'kind'} of jobs with a finished executor receipt (None
    keeps the plain subtraction, for callers without receipts). A box
    that is already visible in its zone while its job is still active (the
    robot is releasing or backing off) must not be counted twice: boxes seen
    beyond the finished deliveries are attributed to active jobs first
    (ZW1-G5-dyn rejected a valid claim this way).
    """
    need = {}
    for zone, kinds in goal.items():
        for kind, count in kinds.items():
            seen = view['zone_counts_seen'].get(zone, {}).get(kind, 0)
            claimed = sum(1 for job in active.values() if job['zone'] == zone and job['kind'] == kind)
            if finished is not None:
                delivered = sum(1 for job in finished if job['zone'] == zone and job['kind'] == kind)
                claimed -= min(claimed, max(0, seen - delivered))
            if count - seen - claimed > 0:
                need.setdefault(zone, {})[kind] = count - seen - claimed
    return need


def check_claims(claims, *, goal, labels, view, active, finished=None):
    """Accept independent claims that fit; report collisions and invalid claims.

    Host checks only; it never picks a box or zone for anyone. Returns
    {'accepted': {rid: job}, 'collisions': [...], 'invalid': {rid: reason}, 'idle': [rid]}.
    """
    accepted, invalid, idle, collisions = {}, {}, [], []
    pending = dict(active)
    by_box = {}
    for rid, claim in claims.items():
        if claim is None:
            invalid[rid] = 'no valid reply'
        elif claim['box'] is None:
            idle.append(rid)
        else:
            by_box.setdefault(claim['box'], []).append(rid)
    for box, rids in sorted(by_box.items()):
        if len(rids) > 1:
            collisions.append({'kind': 'same_box', 'box': box, 'robots': sorted(rids)})
            continue
        rid = rids[0]
        claim = claims[rid]
        if box not in labels or box not in view['pickup_boxes_still_visible']:
            invalid[rid] = f'{box} is not a box still visible in pickup'
            continue
        if any(job['box'] == box for job in pending.values()):
            invalid[rid] = f'{box} is already claimed by a peer'
            continue
        kind = labels[box]['kind']
        need = remaining_need(goal, view, pending, finished).get(claim['zone'], {}).get(kind, 0)
        if need <= 0:
            clash = sorted(r for r, j in pending.items() if j['zone'] == claim['zone'] and j['kind'] == kind
                           and r in claims)
            if clash:
                collisions.append({'kind': 'same_need', 'zone': claim['zone'], 'colour': kind,
                                   'robots': sorted([rid, *clash])})
                for other in clash:
                    accepted.pop(other, None)
                    pending.pop(other, None)
            else:
                invalid[rid] = f"zone {claim['zone']} needs no more {kind}"
            continue
        job = {'box': box, 'zone': claim['zone'], 'kind': kind}
        accepted[rid] = job
        pending[rid] = job
    return {'accepted': accepted, 'collisions': collisions, 'invalid': invalid, 'idle': idle}


def goal_met(goal, view):
    return all(view['zone_counts_seen'].get(zone, {}) == kinds for zone, kinds in goal.items())


def referee(goal, static_map, box_positions):
    """Output only: exact final counts per zone from simulator poses on the floor."""
    counts = {}
    for item in box_positions.values():
        x, y, z = item['xyz']
        if z > .03:
            continue
        for zone in goal:
            region = static_map['regions']['zone_'+zone]
            (cx, cy), (hx, hy) = region['center_m'], region['half_extents_m']
            if abs(x-cx) <= hx and abs(y-cy) <= hy:
                counts.setdefault(zone, {}).setdefault(item['kind'], 0)
                counts[zone][item['kind']] += 1
    per_zone = {zone: counts.get(zone, {}) == kinds for zone, kinds in goal.items()}
    return {'zone_counts': counts, 'per_zone_exact': per_zone, 'goal_met': all(per_zone.values()),
            'scope': 'referee-only simulator poses after control ends; never robot input'}


__all__ = ['build_plan_request', 'build_claim_request', 'plan_validator', 'validate_claim_reply',
           'check_claims', 'remaining_need', 'goal_met', 'referee', 'context', 'digest']
