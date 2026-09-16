"""Executable task allocation: two carriers and one independent carrier.

This module receives RGB/issued commands, never a simulator reference. Task
identity and capability descriptions are authored instructions, not detections.
"""
from __future__ import annotations

import copy
import json

from harness.three_robot_plan import ROBOTS, images, validate_plan_reply

PAIR_TASK = {'skill': 'rgb_pair_goal_v1',
             'participants': {'r1': 'bottom_end', 'r3': 'top_end'},
             'object': 'orange_beam', 'goal': 'green_zone'}
GOALS = ('near_magenta', 'far_magenta')
COMPLETION = 'lift_transport_release_stable_inside'


def validate_mission(plan):
    if not isinstance(plan, dict) or set(plan) != {'transport', 'solo'}:
        raise ValueError('both physical transport tasks are required')
    if plan['transport'] != PAIR_TASK:
        raise ValueError('unsupported pair assignment')
    solo = plan['solo']
    if not isinstance(solo, dict) or set(solo) != {'skill', 'robot_id', 'object', 'goal', 'completion'}:
        raise ValueError('solo object, destination, owner and completion are required')
    if (solo['skill'] != 'rgb_solo_box_v1' or solo['robot_id'] != 'r2'
            or solo['object'] != 'cyan_box' or solo['goal'] not in GOALS
            or solo['completion'] != COMPLETION):
        raise ValueError('unsupported independent transport task')
    return copy.deepcopy(plan)


def mission_fixture(goal='near_magenta'):
    return validate_mission({'transport': copy.deepcopy(PAIR_TASK), 'solo': {
        'skill': 'rgb_solo_box_v1', 'robot_id': 'r2', 'object': 'cyan_box',
        'goal': goal, 'completion': COMPLETION}})


def validate_mission_reply(raw, request_id, agreement):
    return validate_plan_reply(raw, request_id, agreement, plan_validator=validate_mission)


def build_mission_request(rid, *, request_id, own_rgb, top_rgb, agreement,
                          inbox=(), own_history=(), preference='auto'):
    if rid not in ROBOTS or preference not in ('auto', *GOALS):
        raise ValueError('unknown robot or task preference')
    system = '''You are one of three independent robots planning physical transport.
REPLY BINDING: Copy request_id, proposal_id and plan_hash EXACTLY from the
reply_binding object in the user message. Never invent IDs or compute hashes.
If reply_binding has null values, return JSON null for those fields, INCLUDING
when you are the designated proposer creating a new plan. The host assigns the
ID/hash only AFTER receiving your proposal, then supplies them in the next turn.
Mission: deliver the orange beam to the green floor zone AND deliver the small
cyan box in the lower image lane to a magenta floor zone. Every robot must have
a physical transport job. Merely watching or driving without cargo is not completion.
Use CURRENT OWN RGB and CURRENT SHARED TOP RGB to identify cargo, destinations,
reachable space and hazards. Reject unsupported/invisible/obstructed tasks.
Capabilities: rgb_pair_goal_v1 supports only r1 bottom_end and r3 top_end on the
orange beam. rgb_solo_box_v1 supports r2 picking the cyan box and moving it
rightward in its separate lower lane. These are skill limits, NOT measured poses.
The two magenta floor rectangles in the lower lane are candidate box destinations:
near_magenta is the left one; far_magenta is the right one. If preference=auto,
choose the nearest visibly usable destination. Otherwise the operator requires
the named destination; reject if it is not visually usable. No arbitrary detours
or carrier swapping are supported. The pair and solo task execute concurrently.
Completion requires actual lift, transport, release and stability in the chosen
zone. Issued commands and peer visual claims do not establish physical success.
No live coordinates, joints, contacts or evaluator labels are available.
Only the designated proposer can create a proposal. Before a proposal exists,
other peers return accept=false, plan=null. A proposal is frozen: all three must
explicitly accept its exact proposal_id, plan_hash and plan before task actuation.
Reject rather than changing an ACK. No permanent leader.
Reply JSON only: request_id, proposal_id, plan_hash, accept (boolean), plan,
reason, message. Each text is at most 600 characters. Before a proposal exists,
proposal_id/plan_hash are null. Explain the visual target/destination choice.
plan has exactly transport and solo. transport equals the provided pair_task.
solo has exactly skill=rgb_solo_box_v1, robot_id=r2, object=cyan_box,
goal=near_magenta OR far_magenta, completion=lift_transport_release_stable_inside.'''
    proposal = agreement.get('proposal')
    context = {'robot_id': rid, 'request_id': request_id,
               'reply_binding': {'request_id':request_id,
                   'proposal_id':proposal['proposal_id'] if proposal else None,
                   'plan_hash':proposal['plan_hash'] if proposal else None},
               'agreement': copy.deepcopy(agreement), 'pair_task': copy.deepcopy(PAIR_TASK),
               'destination_preference': preference,
               'received_peer_claims': copy.deepcopy(list(inbox)[-6:]),
               'own_issued_commands': copy.deepcopy(list(own_history)[-16:])}
    return {'request_id': request_id,
            'messages': [{'role': 'system', 'content': system},
                         {'role': 'user', 'content': json.dumps(context, sort_keys=True)}],
            'images': images(own_rgb, top_rgb)}
