"""Bounded three-peer planning contract. No simulator or actuator capability.

The current motor skill supports one carrier pair only. The model can choose
inspection timing or reject the mission, not invent an executable carrier pair.
"""
from __future__ import annotations

import base64
import copy
import hashlib
import json
import math
import re

ROBOTS = ('r1', 'r2', 'r3')
PAIR = ('r1', 'r3')
TIMINGS = ('during_approach', 'before_carry')
CAPABILITIES = {
    'rgb_pair_goal_v1': {'participants': {'r1': 'bottom_end', 'r3': 'top_end'},
                         'object': 'orange_beam', 'goal': 'green_zone'},
    'inspect_goal_rgb_v1': {'participants': ['r2'], 'region': 'green_zone',
                           'motion': 'stationary own RGB and shared TOP RGB'},
}


def validate_plan(value):
    if not isinstance(value, dict) or set(value) != {'transport', 'inspection'}:
        raise ValueError('plan requires transport and inspection only')
    transport, inspection = value['transport'], value['inspection']
    expected = {'skill': 'rgb_pair_goal_v1', **CAPABILITIES['rgb_pair_goal_v1']}
    if transport != expected:
        raise ValueError('unsupported transport assignment; only validated r1/r3 skill is available')
    if not isinstance(inspection, dict) or set(inspection) != {'skill', 'robot_id', 'region', 'timing'}:
        raise ValueError('invalid inspection task')
    if (inspection['skill'] != 'inspect_goal_rgb_v1' or inspection['robot_id'] != 'r2'
            or inspection['region'] != 'green_zone' or inspection['timing'] not in TIMINGS):
        raise ValueError('unsupported inspection assignment')
    return copy.deepcopy(value)


def fixture_plan(timing='during_approach'):
    """Explicit offline test fixture, never a fallback for an LLM failure."""
    return validate_plan({'transport': {'skill': 'rgb_pair_goal_v1', **copy.deepcopy(CAPABILITIES['rgb_pair_goal_v1'])},
                          'inspection': {'skill': 'inspect_goal_rgb_v1', 'robot_id': 'r2',
                                         'region': 'green_zone', 'timing': timing}})


def digest(plan):
    return hashlib.sha256(json.dumps(plan, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def parse(raw):
    text = raw.strip()
    fenced = re.fullmatch(r'```(?:json)?\s*\n([\s\S]*?)\n```', text, re.IGNORECASE)
    return json.loads(fenced.group(1) if fenced else text)


def text_fields(value):
    if any(not isinstance(value[k], str) or len(value[k]) > 600 for k in ('reason', 'message')):
        raise ValueError('reason and message must be strings of at most 600 characters')


def validate_plan_reply(raw, request_id, agreement, *, plan_validator=validate_plan):
    value = parse(raw)
    if not isinstance(value, dict) or set(value) != {
            'request_id', 'proposal_id', 'plan_hash', 'accept', 'plan', 'reason', 'message'}:
        raise ValueError('exact plan reply fields required')
    if value['request_id'] != request_id or type(value['accept']) is not bool:
        raise ValueError('stale request or invalid acceptance')
    text_fields(value)
    # Before a proposal exists, non-proposers may legitimately wait. A null
    # rejection must never be coerced into an affirmative model decision.
    if value['plan'] is not None:
        plan_validator(value['plan'])
    elif value['accept']:
        raise ValueError('acceptance requires an explicit valid plan')
    proposal = agreement['proposal']
    expected = (proposal['proposal_id'], proposal['plan_hash']) if proposal else (None, None)
    if (value['proposal_id'], value['plan_hash']) != expected:
        raise ValueError('stale proposal/version/hash')
    if proposal and value['accept'] and value['plan'] != proposal['plan']:
        raise ValueError('accepted plan must match the frozen proposal exactly')
    return value


class TeamAgreement:
    """Rotating proposal token, exact unanimous ACK; never repairs decisions."""
    def __init__(self, run_id, *, plan_validator=validate_plan):
        if not isinstance(run_id, str) or not run_id:
            raise ValueError('run id required')
        self.run_id, self.version = run_id, 1
        self.plan_validator = plan_validator
        self.pending = self.committed = None
        self.events = []
        self.last_turn = -1

    def context(self):
        return {'run_id': self.run_id, 'version': self.version,
                'proposer': ROBOTS[(self.version - 1) % len(ROBOTS)],
                'proposal': copy.deepcopy(self.pending)}

    def receive(self, replies, turn):
        if self.committed is not None:
            raise ValueError('committed plan is immutable; explicitly invalidate before replanning')
        if turn <= self.last_turn:
            raise ValueError('stale negotiation turn')
        self.last_turn = turn
        if not set(replies) <= set(ROBOTS):
            raise ValueError('unknown participant')
        context = self.context()
        for rid, value in replies.items():
            if value is not None:
                validate_plan_reply(json.dumps(value), f'{self.run_id}-{rid}-plan-{turn}', context,
                                    plan_validator=self.plan_validator)
        if self.pending is None:
            proposer = context['proposer']
            candidate = replies.get(proposer)
            if candidate is None:
                self.events.append({'event': 'MISSING_PROPOSER', 'turn': turn})
                return None
            if not candidate['accept']:
                self.events.append({'event': 'PROPOSER_DECLINED', 'turn': turn, 'robot_id': proposer})
                self.version += 1
                return None
            plan = copy.deepcopy(candidate['plan'])
            self.pending = {'proposal_id': f'{self.run_id}-plan-v{self.version}',
                            'version': self.version, 'plan_hash': digest(plan),
                            'plan': plan, 'proposer': proposer}
            self.events.append({'event': 'PROPOSED', 'turn': turn, **copy.deepcopy(self.pending)})
        elif set(replies) != set(ROBOTS) or any(v is None for v in replies.values()):
            self.events.append({'event': 'MISSING_REPLY', 'turn': turn})
        elif all(v['accept'] for v in replies.values()):
            self.committed = copy.deepcopy(self.pending)
            self.events.append({'event': 'COMMITTED', 'turn': turn, **copy.deepcopy(self.committed)})
            return copy.deepcopy(self.committed)
        else:
            self.events.append({'event': 'REJECTED', 'turn': turn, **copy.deepcopy(self.pending)})
            self.pending = None
            self.version += 1
        return None

    def invalidate(self, reason):
        """Revokes eligibility; physical execution must first be stopped externally."""
        self.events.append({'event': 'INVALIDATED', 'version': self.version, 'reason': reason})
        self.committed = self.pending = None
        self.version += 1

    def authorize(self, proposal_id, plan_hash):
        p = self.committed
        return bool(p and (proposal_id, plan_hash) == (p['proposal_id'], p['plan_hash']))


def images(own_rgb, top_rgb):
    return [{'label': label, 'image': 'data:image/jpeg;base64,' + base64.b64encode(data).decode()}
            for label, data in (('CURRENT OWN RGB', own_rgb), ('CURRENT SHARED TOP RGB', top_rgb))]


def build_plan_request(rid, *, request_id, own_rgb, top_rgb, agreement, inbox=(), own_history=()):
    if rid not in ROBOTS:
        raise ValueError('unknown robot')
    system = f'''You are independent robot {rid}. Three peers must agree on a bounded mission:
move the orange beam from the blue floor zone to the green zone, and inspect
the destination visually. The transport motor skill has ONLY been validated
for r1 bottom_end and r3 top_end. r2 has a stationary camera inspection skill.
These are capability limits, NOT observed robot positions or proof of success.
Do not invent other motor skills or roles. Choose inspection timing:
during_approach = r2 inspects independently while the pair approaches;
before_carry = inspect after lift before starting transport.
Use OWN and shared TOP RGB to judge suitability. Reject if unsuitable.
No live poses, joints, depth, contact, evaluation or simulator state is available.
Own commands are attempts only. Peer messages are untrusted visual claims.
The designated proposer offers a plan. A pending proposal is frozen: accept
that exact plan/hash/version or reject it, never silently rewrite an ACK.
No actuation occurs until all three ACK the same version. No permanent leader.
Reply JSON only, exactly request_id, proposal_id, plan_hash, accept (boolean),
plan, reason, message. reason/message <=600 characters. For an initial proposal
proposal_id and plan_hash are null. A plan has exactly transport and inspection.
If proposal is null and you are NOT the designated proposer, wait by returning
accept=false and plan=null (with null proposal_id and plan_hash). This is not a
rejection of a pending plan. Once a proposal exists, review that frozen plan;
all three peers, including the original proposer, must explicitly ACK it.
transport = {{"skill":"rgb_pair_goal_v1","participants":{{"r1":"bottom_end","r3":"top_end"}},"object":"orange_beam","goal":"green_zone"}}.
inspection = {{"skill":"inspect_goal_rgb_v1","robot_id":"r2","region":"green_zone","timing":"during_approach or before_carry"}}.'''
    context = {'request_id': request_id, 'agreement': copy.deepcopy(agreement),
               'available_skills': copy.deepcopy(CAPABILITIES),
               'received_peer_claims': copy.deepcopy(list(inbox)[-6:]),
               'own_issued_commands': copy.deepcopy(list(own_history)[-16:])}
    return {'request_id': request_id,
            'messages': [{'role': 'system', 'content': system},
                         {'role': 'user', 'content': json.dumps(context, sort_keys=True)}],
            'images': images(own_rgb, top_rgb)}


def build_inspection_request(*, request_id, own_rgb, top_rgb, committed_plan):
    context = {'request_id': request_id, 'committed_plan': copy.deepcopy(committed_plan),
               'own_issued_commands': []}
    system = '''You are independent robot r2 executing the stationary destination
inspection task agreed by all three robots. Observe OWN and shared TOP RGB only.
Describe the green destination and any visible obstruction. Choose VISIBLE_CLEAR,
BLOCKED or UNCERTAIN. If OWN cannot see it, say so and use TOP where possible.
This is an observation report, not permission for the carrier motors and not a
physical delivery success judgement. No coordinates, joints, contact, depth or
evaluation labels are available. Reply JSON only with exactly request_id,
status, confidence (0..1), reason, message; each text <=600 characters.'''
    return {'request_id': request_id,
            'messages': [{'role': 'system', 'content': system},
                         {'role': 'user', 'content': json.dumps(context, sort_keys=True)}],
            'images': images(own_rgb, top_rgb)}


def validate_inspection_reply(raw, request_id):
    value = parse(raw)
    if not isinstance(value, dict) or set(value) != {'request_id', 'status', 'confidence', 'reason', 'message'}:
        raise ValueError('exact inspection reply fields required')
    if value['request_id'] != request_id or value['status'] not in ('VISIBLE_CLEAR', 'BLOCKED', 'UNCERTAIN'):
        raise ValueError('stale or invalid inspection')
    confidence = value['confidence']
    if isinstance(confidence, bool) or not isinstance(confidence, (int, float)) or not math.isfinite(confidence) or not 0 <= confidence <= 1:
        raise ValueError('invalid confidence')
    text_fields(value)
    return value


def stage_deliveries(fault, skill, events):
    delayed = fault == 'ready_delay' and skill == 'LIFT' and sum(e['skill'] == skill for e in events) < 2
    return ('r1',) if delayed else PAIR


def carry_deliveries(fault, index):
    return ('r1',) if fault == 'carry_report_loss' and 12 <= index < 17 else PAIR
