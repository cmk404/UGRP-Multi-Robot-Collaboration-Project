"""No-communication condition for the zone benchmark ("independent").

Each idle robot picks its own next job from its own images and its own job
receipts only. It cannot send or receive messages and never sees peer claims,
peer receipts or a shared plan. The host checks a claim only against what that
robot could know (the box is visible in pickup, the zone still needs that
colour in the current TOP-RGB view); it does not arbitrate between robots, so
two robots may go for the same box or over-fill a zone. The teacher executor
then stops the robot that arrives second (``box_taken_by_peer``).
"""
from __future__ import annotations

import copy
import json

from harness import zone_coordination as zc

_SOLO = zc._COMMON + '''
NO COMMUNICATION: you cannot send or receive messages, and you never see the
peers' plans, claims or reports. The peers choose on their own, at the same
time as you, and may pick the same box. You are idle now: choose ONE next job
for yourself, or null if nothing useful remains for you. Choose a box that is
still in pickup (rgb_view.pickup_boxes_still_visible) and a zone that still
needs that colour (goal minus rgb_view.zone_counts_seen). The images show where
the peers are and whether they carry a box; use them to avoid doing the same
job as a peer. own_jobs holds your own jobs and their executor receipts.
Reply JSON only: {{"request_id": copied, "claim": {{"box": label or null,
"zone": "A|B|C" or null}}, "reason": "brief"}}. reason under 240 characters.'''

ZONES = ('A', 'B', 'C')


def solo_context(rid, *, labels, view, own_jobs, extra=None):
    value = {'robot_id': rid, 'box_labels': {k: {'kind': v['kind'], 'rgb_floor_xy_m': v['floor_xy_m']}
                                             for k, v in labels.items()},
             'rgb_view': copy.deepcopy(view), 'own_jobs': copy.deepcopy(list(own_jobs)[-8:])}
    value.update(extra or {})
    return value


def build_solo_request(rid, *, request_id, task, frame, ctx, views=zc.DEFAULT_VIEWS):
    body = {'request_id': request_id, **ctx}
    return {'request_id': request_id,
            'messages': [{'role': 'system', 'content': zc._system(_SOLO, rid, task, views)},
                         {'role': 'user', 'content': json.dumps(body, sort_keys=True)}],
            'images': zc._images(frame, views)}


def _json_object(raw):
    text = raw.strip()
    start, end = text.find('{'), text.rfind('}')
    if start < 0 or end < start:
        raise ValueError('reply is not a JSON object')
    return json.loads(text[start:end+1])


def validate_solo_reply(raw, request_id):
    """{"request_id", "claim": {"box", "zone"}, "reason"}; no message field."""
    value = _json_object(raw)
    if not isinstance(value, dict) or set(value) != {'request_id', 'claim', 'reason'}:
        raise ValueError('solo reply needs exactly request_id, claim and reason')
    if value['request_id'] != request_id:
        raise ValueError('request_id mismatch')
    claim = value['claim']
    if not isinstance(claim, dict) or set(claim) != {'box', 'zone'}:
        raise ValueError('claim needs box and zone')
    if (claim['box'] is None) != (claim['zone'] is None):
        raise ValueError('box and zone are both set or both null')
    if claim['box'] is not None and (not isinstance(claim['box'], str) or claim['zone'] not in ZONES):
        raise ValueError('claim needs a box label and a zone among A, B, C')
    if not isinstance(value['reason'], str) or len(value['reason']) > 400:
        raise ValueError('reason must be a short string')
    return value


def check_solo_claims(claims, *, goal, labels, view):
    """Each claim alone against the robot's own view; no arbitration between robots."""
    need = zc.remaining_need(goal, view, {})
    accepted, invalid, idle = {}, {}, []
    for rid, claim in sorted(claims.items()):
        if claim is None:
            invalid[rid] = 'no valid reply'
        elif claim['box'] is None:
            idle.append(rid)
        elif claim['box'] not in labels or claim['box'] not in view['pickup_boxes_still_visible']:
            invalid[rid] = f"{claim['box']} is not a box still visible in pickup"
        elif need.get(claim['zone'], {}).get(labels[claim['box']]['kind'], 0) <= 0:
            invalid[rid] = f"zone {claim['zone']} needs no more {labels[claim['box']]['kind']}"
        else:
            accepted[rid] = {'box': claim['box'], 'zone': claim['zone'], 'kind': labels[claim['box']]['kind']}
    boxes = [job['box'] for job in accepted.values()]
    same_box = sorted({b for b in boxes if boxes.count(b) > 1})
    return {'accepted': accepted, 'invalid': invalid, 'idle': idle, 'same_box_accepted': same_box}


def fixture_solo_claim(rid, request_id, goal, labels, view, robots=('r1', 'r2', 'r3')):
    """Scripted protocol fixture (not visual reasoning): robot i takes the i-th
    open need unit of its own view; peers are unknown, so later rounds can pick
    a box a peer is already fetching."""
    need = zc.remaining_need(goal, view, {})
    units = [(zone, kind) for zone, kinds in sorted(need.items()) for kind in sorted(kinds)
             for _ in range(kinds[kind])]
    free = {kind: [b for b in view['pickup_boxes_still_visible'] if labels[b]['kind'] == kind]
            for kind in {k for _, k in units}}
    order = []
    for zone, kind in units:
        if free[kind]:
            order.append({'box': free[kind].pop(0), 'zone': zone})
    rank = list(robots).index(rid)
    claim = order[rank] if rank < len(order) else {'box': None, 'zone': None}
    return {'request_id': request_id, 'claim': claim,
            'reason': 'scripted protocol fixture, not visual reasoning'}


__all__ = ['build_solo_request', 'validate_solo_reply', 'check_solo_claims', 'solo_context',
           'fixture_solo_claim']
