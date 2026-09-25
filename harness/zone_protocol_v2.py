"""Messages and claims for mixed zone goals (goal v2, zone team A2, 2026-09-25).

Every robot-facing text of the v2 zone protocol lives here: the static task
text (identical in every coordination mode), the three condition blocks, the
request builders, the reply validators, the robot-visible context and the
scripted fixture replies. Colour-only goals keep ``harness.zone_coordination``
and ``harness.zone_solo`` byte for byte. A later dialogue redesign (Korean
natural language) is meant to change this module only.

Claim form in all three modes: ``{"item": label, "zone": "A|B|C", "role": role}``
(``harness.zone_team_jobs.RoleClaim``). The host never picks teammates: a team
forms only by the physical rendezvous rule, the same in every mode.

Per-condition features are explicit switches (``CONDITIONS``) so that a later
design can turn the host board, host arbitration, event wake-ups or peer
messages on or off independently (R1 audit C1/C3).
"""
from __future__ import annotations

import copy
import json

from harness import zone_coordination as zc
from harness.three_robot_plan import ROBOTS, parse, text_fields
from harness.zone_goal_v2 import formation, is_legacy_goal, landing_layout, required_carriers
from harness.zone_perception_v2 import public_labels
from harness.zone_team_jobs import (RECEIPT_FINISHED, RECEIPT_STOPPED, RendezvousRule, normalize_claim,
                                    remaining_need_items)
from sim.zone_arena import LAYOUTS, ZONE_IDS, static_map_text

TASK_SCHEMA = 'ugrp.zone_task.v2'
ZONES = ZONE_IDS

# ---------------------------------------------------------------------------
# Condition switches (what the host adds beyond the robot's own channels)

CONDITIONS = {
    # peer_board: the host shows active peer claims and every robot's receipts.
    # host_arbitration: the host rejects a claim against peers' claims (collisions, joint need).
    # wake_on_peer_job_end: idle robots are asked again when any peer's claim ends.
    # peer_messages: reply messages reach the peers.
    'independent': {'peer_board': False, 'host_arbitration': False, 'wake_on_peer_job_end': False,
                    'peer_messages': False},
    'dynamic': {'peer_board': True, 'host_arbitration': True, 'wake_on_peer_job_end': True, 'peer_messages': True},
    'plan_first': {'peer_board': False, 'host_arbitration': False, 'wake_on_peer_job_end': False,
                   'peer_messages': True},
}

# ---------------------------------------------------------------------------
# Static task text (same for every robot and every mode)

RULE = RendezvousRule()
TEAM_RULE_TEXT = (
    'Team rule (the same for every robot and every mode; nobody assigns teammates): an item that needs N robots '
    'starts only when N robots stand at N different grasp stations of that SAME item, each having claimed that '
    'item, the SAME zone and the role of its own station. Claim a role by its handle position (role names of a '
    f'symmetric item may swap between views). A robot waits at its station at most {RULE.wait_s:.0f} s of SIM time '
    'after it arrives; if its team is not complete by then, its claim ends with the plain "executor stopped before '
    'finishing" receipt. A claim also ends with that receipt when another robot already stands nearer its station, '
    'or when the item is already being grasped. The whole team lifts, carries and sets the item down together.')


def _fmt(xy):
    return '(' + ', '.join(f'{v:.2f}' for v in xy) + ')'


def task_static_text(goal, static_map):
    """Catalogue kinds, carriers, roles, landing areas (no ids) and the team rule."""
    kinds = sorted({k for z in goal.values() for k in z})
    lines = ['Item kinds in this goal:']
    for k in kinds:
        n = required_carriers(k)
        roles = ', '.join(formation(k))
        lines.append(f'- {k}: {"1 robot" if n == 1 else f"{n} robots at once"}, role{"s" if n > 1 else ""} {roles}.')
    lines.append('Landing areas (one per requested item; the item must end fully inside its zone):')
    for zone, areas in landing_layout(goal, static_map=static_map).items():
        parts = [f'{a["kind"]} at {_fmt(a["landing_center_m"])} +-{_fmt(a["landing_half_extents_m"])} m'
                 for a in areas]
        lines.append(f'- zone {zone}: ' + '; '.join(parts) + '.')
    lines.append(TEAM_RULE_TEXT)
    return '\n'.join(lines)


def actor_task_v2(static_map, goal, *, allow_colour_only=False):
    """What every robot is told for a v2 goal: goal, kinds, landing areas, team rule, static map.

    allow_colour_only: a colour-only goal on protocol v2 (explicit --protocol v2).
    """
    if is_legacy_goal(goal) and not allow_colour_only:
        raise ValueError('colour-only goals use sim.zone_arena.actor_task')
    task = {'schema': TASK_SCHEMA, 'goal': copy.deepcopy(goal),
            'instruction': ('Deliver items so that each zone ends with exactly the requested number of items of '
                            'each kind. Any item of the right kind counts. Items (colour boxes and cargo) start '
                            'in the pickup area (west). Zones A, B and C are painted floor areas (east). A robot '
                            'holds one item (or one handle of a team item) at a time.'),
            'zones': {z: {'center_m': static_map['regions']['zone_'+z]['center_m'],
                          'half_extents_m': static_map['regions']['zone_'+z]['half_extents_m']} for z in ZONES},
            'bounds_m': static_map['bounds_m'], 'cameras': LAYOUTS[static_map['map_id']]['cameras_text'],
            'static_task_text': task_static_text(goal, static_map)}
    text = static_map_text(static_map)
    if text:
        task['static_map_text'] = text
    return task


# ---------------------------------------------------------------------------
# System prompts

_COMMON = '''You are robot {rid}, an equal peer in a three-robot team (r1, r2, r3).
Mission: {instruction}
Goal (zone -> item kind -> count): {goal}
The pickup area is west, zones A, B, C are painted floor areas east. A motion
executor moves you when you have a claim; you coordinate WHICH item goes to
WHICH zone and WHO holds WHICH handle. Ground choices in the images: {images}.
item_labels are fixed names from the first TOP images (per kind, west to east
then south to north); a team item lists its handles (role -> RGB grip point).
rgb_view is the current TOP-RGB estimate, not ground truth. Never claim
simulator coordinates or physical success.'''

_PLAN = _COMMON + '''
PLAN FIRST: agree on the COMPLETE assignment before anyone moves. Only
agreement.proposer may propose while agreement.proposal is null; the others
then reply accept=false, plan=null with a useful peer message. Once
agreement.proposal exists, every robot (the proposer too) either ACCEPTS by
replying accept=true with plan set to an exact copy of agreement.proposal.plan,
or REJECTS with accept=false, plan=null and a reason. accept=true with
plan=null is invalid. Copy proposal_id and plan_hash from agreement.proposal.
A plan lists, for every robot, its ordered jobs {{"item": label, "zone":
"A|B|C", "role": role}} so that every zone receives exactly its goal count per
kind, each item goes to one zone, and every role of a team item is taken by
exactly one robot. Team members arrive at a team item at about the same time
only if it has the same position in their lists; orders that make robots wait
for each other are rejected. Robots run their lists in parallel.
Reply JSON only: {{"request_id": copied, "proposal_id": copied or null,
"plan_hash": copied or null, "accept": true|false, "plan": {{"assignments":
{{"r1": [...], "r2": [...], "r3": [...]}}}} (your proposal, or the exact accepted
plan) or null when rejecting/waiting, "reason": "brief",
"message": "brief message to peers"}}. reason/message under 240 characters.'''

_CLAIM = _COMMON + '''
TALK WHEN NEEDED: there is no global plan. You are idle now: claim ONE next job
for yourself, or null if nothing useful remains for you. Choose an item that is
still in pickup (rgb_view.pickup_items_still_visible), a zone that still needs
that kind (goal minus rgb_view.zone_counts_seen minus items already claimed in
team_board.active), and a role (handle) of that item that no peer holds. A free
role of an item a peer already claimed for the same zone joins that peer's
team. If conflict is present, you and a peer claimed the same handle or
different zones for one item: agree (read peer_messages) and choose again.
Everyone answers at once, so if you all yield nobody takes it: unless a peer's
message already gives it to a specific robot, the conflicting robot with the
lowest robot_id keeps it and the others choose a different job or null.
Reply JSON only: {{"request_id": copied, "claim": {{"item": label or null,
"zone": "A|B|C" or null, "role": role or null}}, "reason": "brief", "message":
"brief message to peers"}}. reason/message under 240 characters.'''

_SOLO = _COMMON + '''
NO COMMUNICATION: you cannot send or receive messages, and you never see the
peers' plans, claims or reports. The peers choose on their own, at the same
time as you, and may pick the same item or handle. You are idle now: choose ONE
next job for yourself, or null if nothing useful remains for you. Choose an
item still in pickup (rgb_view.pickup_items_still_visible), a zone that still
needs that kind (goal minus rgb_view.zone_counts_seen) and a role (handle) of
that item. A team item starts only if peers choose the same item and zone and
the other roles on their own. The images show where the peers are and what
they carry. own_jobs holds your own jobs and their executor receipts.
Reply JSON only: {{"request_id": copied, "claim": {{"item": label or null,
"zone": "A|B|C" or null, "role": role or null}}, "reason": "brief"}}. reason
under 240 characters.'''

TEMPLATES = {'plan_first': _PLAN, 'dynamic': _CLAIM, 'independent': _SOLO}


def system_text(mode, rid, task, views):
    """System prompt: condition block + the static task text + the static map (same in every mode)."""
    text = TEMPLATES[mode].format(rid=rid, instruction=task['instruction'],
                                  goal=json.dumps(task['goal'], sort_keys=True), images=zc._image_text(views))
    text += '\nStatic task: ' + task['static_task_text']
    if task.get('static_map_text'):
        text += '\nStatic map: ' + task['static_map_text']
    return text


# ---------------------------------------------------------------------------
# Robot-visible context

OWN_JOB_KEYS = ('item', 'zone', 'role', 'issued_at_sim_s', 'status')


def visible_own_jobs(own_jobs, last=8):
    """Own claims as the model sees them: no landing-area id (as L2), no executor detail."""
    return [{k: copy.deepcopy(job[k]) for k in OWN_JOB_KEYS if k in job} for job in list(own_jobs)[-last:]]


def context(rid, *, labels, view, own_jobs, board=None, inbox=None, extra=None):
    """board/inbox are passed only where the condition switches allow them."""
    value = {'robot_id': rid, 'item_labels': public_labels(labels), 'rgb_view': copy.deepcopy(view),
             'own_jobs': visible_own_jobs(own_jobs)}
    if board is not None:
        value['team_board'] = copy.deepcopy(board)
    if inbox is not None:
        value['peer_messages'] = copy.deepcopy(list(inbox)[-8:])
    value.update(extra or {})
    return value


def build_request(mode, rid, *, request_id, task, frame, ctx, views, agreement=None):
    body = {'request_id': request_id, **({'agreement': copy.deepcopy(agreement)} if mode == 'plan_first' else {}),
            **ctx}
    return {'request_id': request_id,
            'messages': [{'role': 'system', 'content': system_text(mode, rid, task, views)},
                         {'role': 'user', 'content': json.dumps(body, sort_keys=True)}],
            'images': zc._images(frame, views)}


# ---------------------------------------------------------------------------
# Reply validation (form only; the host checks meaning with zone_team_jobs)

def _claim_form(claim):
    if not isinstance(claim, dict) or set(claim) != {'item', 'zone', 'role'}:
        raise ValueError('claim requires item, zone and role')
    if claim['item'] is None:
        if claim['zone'] is not None or claim['role'] is not None:
            raise ValueError('an idle claim has item, zone and role all null')
        return
    if not isinstance(claim['item'], str) or claim['zone'] not in ZONES:
        raise ValueError('claim needs an item label and a zone among A, B, C')
    if claim['role'] is not None and not isinstance(claim['role'], str):
        raise ValueError('role must be a role name or null')


def validate_claim_reply(raw, request_id):
    value = parse(raw) if isinstance(raw, str) else copy.deepcopy(raw)
    if not isinstance(value, dict) or set(value) != {'request_id', 'claim', 'reason', 'message'}:
        raise ValueError('claim reply requires request_id, claim, reason and message only')
    if value['request_id'] != request_id:
        raise ValueError('stale claim reply')
    text_fields(value)
    _claim_form(value['claim'])
    return value


def validate_solo_reply(raw, request_id):
    value = parse(raw) if isinstance(raw, str) else copy.deepcopy(raw)
    if not isinstance(value, dict) or set(value) != {'request_id', 'claim', 'reason'}:
        raise ValueError('solo reply needs exactly request_id, claim and reason')
    if value['request_id'] != request_id:
        raise ValueError('request_id mismatch')
    if not isinstance(value['reason'], str) or len(value['reason']) > 400:
        raise ValueError('reason must be a short string')
    _claim_form(value['claim'])
    return value


# ---------------------------------------------------------------------------
# Scripted fixture replies (protocol checks, not visual reasoning)

FIXTURE_REASON = 'scripted protocol fixture, not visual reasoning'


def _visible(view):
    return view.get('pickup_items_still_visible', view.get('pickup_boxes_still_visible', []))


def open_slots(goal, labels, view, active=None, finished=None):
    """[{'item', 'zone', 'role'}]: free roles of already-claimed items first (joins), then
    new items for the remaining need, team items (more carriers) first, all roles in a row."""
    active = active or {}
    slots = []
    by_item = {}
    for c in active.values():
        by_item.setdefault(c.item, {'zone': c.zone, 'kind': c.kind, 'roles': set()})['roles'].add(c.role)
    for item in sorted(by_item):
        spec = by_item[item]
        if item not in _visible(view):
            continue
        for role in formation(spec['kind']):
            if role not in spec['roles']:
                slots.append({'item': item, 'zone': spec['zone'], 'role': role})
    need = remaining_need_items(goal, view, active, finished)
    units = sorted(((zone, kind) for zone, kinds in need.items() for kind in kinds for _ in range(kinds[kind])),
                   key=lambda u: (-required_carriers(u[1]), u[0], u[1]))
    free = {kind: [b for b in _visible(view) if labels[b]['kind'] == kind and b not in by_item]
            for kind in {k for _, k in units}}
    for zone, kind in units:
        if free[kind]:
            item = free[kind].pop(0)
            slots += [{'item': item, 'zone': zone, 'role': role} for role in formation(kind)]
    return slots


def _idle_claim():
    return {'item': None, 'zone': None, 'role': None}


def fixture_claim(rid, request_id, goal, labels, view, active, askers, finished=None):
    """dynamic: the i-th asking robot takes the i-th open slot (joins first)."""
    slots = open_slots(goal, labels, view, active, finished)
    rank = list(askers).index(rid)
    return {'request_id': request_id, 'claim': slots[rank] if rank < len(slots) else _idle_claim(),
            'reason': FIXTURE_REASON, 'message': ''}


def fixture_solo_claim(rid, request_id, goal, labels, view, robots=ROBOTS):
    """independent: robot i takes the i-th open slot of its own view (id-order convention, no peers)."""
    slots = open_slots(goal, labels, view)
    rank = list(robots).index(rid)
    return {'request_id': request_id, 'claim': slots[rank] if rank < len(slots) else _idle_claim(),
            'reason': FIXTURE_REASON}


def fixture_plan(goal, labels, robots=ROBOTS):
    """plan_first: team items first, taken by the first robots in the same list position;
    solo items round-robin starting with the robots left out of the teams."""
    view = {'pickup_items_still_visible': sorted(labels), 'zone_counts_seen': {}}
    slots = open_slots(goal, labels, view)
    items, order = {}, []
    for s in slots:
        if s['item'] not in items:
            order.append(s['item'])
        items.setdefault(s['item'], []).append(s)
    assignments = {r: [] for r in robots}
    team_load = {r: 0 for r in robots}
    solo = []
    for item in order:
        roles = items[item]
        if len(roles) == 1:
            solo.append(roles[0])
            continue
        for rid, s in zip(robots, roles):
            assignments[rid].append(s)
            team_load[rid] += 1
    rota = sorted(robots, key=lambda r: (team_load[r], r))
    for i, s in enumerate(solo):
        assignments[rota[i % len(rota)]].append(s)
    return {'assignments': assignments}


def fixture_plan_reply(request_id, context, goal, labels):
    proposal = context['proposal']
    if proposal:
        return {'request_id': request_id, 'proposal_id': proposal['proposal_id'], 'plan_hash': proposal['plan_hash'],
                'accept': True, 'plan': proposal['plan'], 'reason': FIXTURE_REASON, 'message': ''}
    return {'request_id': request_id, 'proposal_id': None, 'plan_hash': None, 'accept': True,
            'plan': fixture_plan(goal, labels), 'reason': FIXTURE_REASON, 'message': ''}


def receipt(finished):
    return RECEIPT_FINISHED if finished else RECEIPT_STOPPED


def parse_claim(rid, raw, labels):
    """Reply claim -> RoleClaim or None; ValueError with a reason when invalid."""
    return normalize_claim(rid, raw, labels)


__all__ = ['CONDITIONS', 'TEAM_RULE_TEXT', 'TASK_SCHEMA', 'actor_task_v2', 'task_static_text', 'system_text',
           'build_request', 'context', 'visible_own_jobs', 'OWN_JOB_KEYS', 'validate_claim_reply',
           'validate_solo_reply', 'open_slots', 'fixture_claim', 'fixture_solo_claim', 'fixture_plan',
           'fixture_plan_reply', 'receipt', 'parse_claim']
