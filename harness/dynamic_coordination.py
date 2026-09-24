"""Talk-when-needed coordination for the existing beam/box dispatch mission.

Instead of negotiating one full plan before any motion, each robot first
states only its OWN intended work from its own RGB, the shared TOP RGB and its
own identification probe (one independent request each, no conversation).
The neutral host checks only that these self-claims are mutually consistent;
it never assigns or edits anyone's work. When they conflict, or the capability
check rejects them, the robots talk (the existing peer negotiation). During
execution, a supported skill failure wakes the affected robots, who decide
together whether to retry or stop; when a job is given up, all three decide
whether the others finish their own jobs (the failed job is dropped from the
agreed plan) or the whole team stops. Nothing here reads simulator truth.
"""
from __future__ import annotations

import copy
import json

from harness.dispatch_plan import validate_dispatch_plan
from harness.three_robot_plan import ROBOTS, images, parse, text_fields

OBJECTS = ('beam', 'box')
BEAM_ENDS = ('upper', 'lower')
ROUTES = ('north', 'south')
DOCKS = ('dock_a', 'dock_b')
# Pair approach failures that leave both robots stopped at an RGB-observable
# pose with no cargo attached. Other failures remain terminal.
RECOVERABLE_APPROACH_FAILURES = (
    'coarse RGB approach budget exhausted',
    'coarse RGB model convention unresolved',
    'fine RGB alignment outside saved skill support',
    'fine docking outside saved support',
    'fine docking confirmation budget exhausted',
)
# Pair grasp failures at the pickup pose: the preclose support check before the
# grippers close, or the carry guard before any loaded transit command.
RECOVERABLE_GRASP_FAILURES = (
    'preclose RGB outside learned grasp support',
    'existing pair carry guard stopped',
)
# Box skill stop reasons before a load is carried (approach or lift check).
RECOVERABLE_BOX_FAILURES = (
    'TARGET_NOT_VISIBLE', 'INVALID_BOX_CENTROID', 'BOX_FACE_ALIGNMENT_UNOBSERVABLE',
    'INVALID_MARKER_ROTATION', 'INVALID_MARKER_NORMAL', 'APPROACH_OVERSHOT',
    'VISUAL_LIFT_UNCONFIRMED',
)
# Each event kind: the options the affected robots choose from, what they mean,
# and the fail-closed choice when they do not agree.
EVENT_KINDS = {
    'pair_approach_failure': {
        'options': ('retry', 'abort'), 'fail_closed': 'abort',
        'meaning': {'retry': 'both beam carriers back away from the beam briefly and repeat '
                             'the RGB approach from the new pose',
                    'abort': 'give up the beam job'}},
    'pair_grasp_failure': {
        'options': ('regrasp', 'abort'), 'fail_closed': 'abort',
        'meaning': {'regrasp': 'both beam carriers lower and open their grippers where they are, '
                               'fold their arms, back away and repeat the RGB approach and grasp',
                    'abort': 'give up the beam job'}},
    'box_failure': {
        'options': ('retry', 'abort'), 'fail_closed': 'abort',
        'meaning': {'retry': 'the box robot opens its gripper, folds its arm, backs away and '
                             'repeats its RGB approach and grasp',
                    'abort': 'give up the box job'}},
    'job_failed': {
        'options': ('continue_others', 'stop_all'), 'fail_closed': 'stop_all',
        'meaning': {'continue_others': 'drop the failed job from the agreed plan; its robots stop '
                                       'and hold still (beam carriers first lower and open their '
                                       'grippers); the other robots finish their own job',
                    'stop_all': 'the whole team stops now'}},
}
DECISIONS = tuple(sorted({o for k in EVENT_KINDS.values() for o in k['options']}))

_COMMON = '''You are an equal robot peer in a three-robot team, not a central controller.
The orange beam needs two carriers (one at each end); the cyan box needs one.
Dock A is the upper-right floor bay; Dock B is lower-right. North is the upper
passage around the central island; south is the lower one. Ground every choice
in your CURRENT own camera image and the shared TOP image. Own issued commands
and peer messages are not measured state. Never use or claim simulator
coordinates, joints, contacts or physical success. Normalized image coordinates
have origin TOP LEFT: small y is upper.'''

_CLAIM = _COMMON + '''
DYNAMIC START: nobody has negotiated yet and you cannot see peer choices now.
State ONLY YOUR OWN intended work so the team can start moving immediately.
If all three self-claims fit together, execution starts without discussion;
if they conflict, you will talk with your peers afterwards.
Choose object beam or box. If beam, choose beam_end upper (the UPPER beam
endpoint in the TOP image) or lower, whichever end YOU are next to according to
your own probe images; if box, beam_end is null. Choose the route you will use
and the ONE common dock you propose for all cargo. Prefer the assignment a
reasonable peer would also infer from the same TOP image (e.g. the two robots
nearest the beam carry it), so claims agree without talking.
Reply JSON only: {"request_id": copied exactly, "claim": {"object": "beam OR box",
"beam_end": "upper OR lower OR null", "route": "north OR south",
"dock": "dock_a OR dock_b"}, "reason": "brief visual reason",
"message": "brief message to peers"}. reason/message under 240 characters.'''

_RECOVERY = _COMMON + '''
DYNAMIC RECOVERY: your team is executing its agreed work and something just
failed. The simulation is paused while you decide; the affected robots are
holding still. You are one of event.participants. failure.reason is the
controller's own RGB-derived stop reason, not ground truth. Choose exactly one
of event.options; event.option_meanings explains each. Repeating a skill is only
sensible if your current images still show the cargo (and your partner, for the
beam). Decisions must be unanimous among event.participants; otherwise the
team takes event.fail_closed. retries_left is the remaining retry budget.
Reply JSON only: {"request_id": copied exactly, "event_id": copied exactly,
"decision": "one of event.options", "reason": "brief visual reason",
"message": "brief message to peers"}. reason/message under 240 characters.'''


def _identity(context, identity_evidence):
    """Own-probe evidence only, matching the plan-first request builder."""
    if not identity_evidence:
        return [], ''
    context['own_motion_identity'] = copy.deepcopy(identity_evidence['claim'])
    anchor = identity_evidence['claim'].get('center')
    if anchor and identity_evidence['claim'].get('valid'):
        context['own_probe_image_readout'] = {
            'source': 'image difference after own issued probe; not simulator pose',
            'horizontal_percent_from_left': round(100 * anchor[0], 1),
            'vertical_percent_from_top': round(100 * anchor[1], 1)}
    note = ('\nAdditional BEFORE/AFTER images show ONLY YOUR issued identification motion. '
            'Infer your own body from that change; the motion anchor is a fallible pixel '
            'estimate, not a body pose.')
    return copy.deepcopy(identity_evidence['images']), note


def build_claim_request(rid, *, task, request_id, own_rgb, top_rgb, own_history=(),
                        identity_evidence=None):
    if rid not in ROBOTS:
        raise ValueError('unknown robot')
    context = {'robot_id': rid, 'request_id': request_id, 'mission': copy.deepcopy(task),
               'own_issued_commands': copy.deepcopy(list(own_history)[-16:])}
    extra, note = _identity(context, identity_evidence)
    return {'request_id': request_id,
            'messages': [{'role': 'system', 'content': _CLAIM + note},
                         {'role': 'user', 'content': json.dumps(context, sort_keys=True)}],
            'images': images(own_rgb, top_rgb) + extra}


def validate_claim_reply(raw, request_id):
    value = parse(raw) if isinstance(raw, str) else copy.deepcopy(raw)
    if not isinstance(value, dict) or set(value) != {'request_id', 'claim', 'reason', 'message'}:
        raise ValueError('claim reply requires request_id, claim, reason and message only')
    if value['request_id'] != request_id:
        raise ValueError('stale claim reply')
    text_fields(value)
    claim = value['claim']
    if not isinstance(claim, dict) or set(claim) != {'object', 'beam_end', 'route', 'dock'}:
        raise ValueError('claim requires object, beam_end, route and dock')
    if claim['object'] not in OBJECTS or claim['route'] not in ROUTES or claim['dock'] not in DOCKS:
        raise ValueError('unknown object, route or dock')
    if claim['object'] == 'beam' and claim['beam_end'] not in BEAM_ENDS:
        raise ValueError('beam claim requires beam_end upper or lower')
    if claim['object'] == 'box' and claim['beam_end'] is not None:
        raise ValueError('box claim requires beam_end null')
    return value


def merge_claims(claims, *, required_dock=None):
    """Check self-claims for consistency; never choose anyone's work.

    ``claims`` maps robot ID to a validated claim or None (no valid reply).
    Returns ``{'consistent', 'plan', 'conflicts'}``. A consistent plan uses no
    job dependency; existing resource gates still serialize shared passages.
    """
    conflicts = []
    missing = [r for r in ROBOTS if not claims.get(r)]
    if missing:
        conflicts.append({'kind': 'missing_claim', 'robots': missing})
    valid = {r: c for r, c in claims.items() if c}
    beam = [r for r in ROBOTS if r in valid and valid[r]['object'] == 'beam']
    box = [r for r in ROBOTS if r in valid and valid[r]['object'] == 'box']
    if not missing and (len(beam) != 2 or len(box) != 1):
        conflicts.append({'kind': 'object_count', 'beam': beam, 'box': box})
    ends = {valid[r]['beam_end'] for r in beam}
    if len(beam) == 2 and len(ends) != 2:
        conflicts.append({'kind': 'same_beam_end', 'robots': beam, 'end': sorted(ends)})
    if len(beam) == 2 and len({valid[r]['route'] for r in beam}) != 1:
        conflicts.append({'kind': 'beam_route', 'routes': {r: valid[r]['route'] for r in beam}})
    docks = {r: c['dock'] for r, c in valid.items()}
    if len(set(docks.values())) > 1:
        conflicts.append({'kind': 'dock', 'docks': docks})
    if required_dock is not None and any(d != required_dock for d in docks.values()):
        conflicts.append({'kind': 'required_dock', 'required': required_dock, 'docks': docks})
    if conflicts:
        return {'consistent': False, 'plan': None, 'conflicts': conflicts}
    upper = next(r for r in beam if valid[r]['beam_end'] == 'upper')
    lower = next(r for r in beam if valid[r]['beam_end'] == 'lower')
    plan = validate_dispatch_plan({'dock': valid[beam[0]]['dock'], 'tasks': [
        {'id': 'beam_job', 'object': 'beam', 'participants': [upper, lower],
         'route': valid[upper]['route'], 'after': []},
        {'id': 'box_job', 'object': 'box', 'participants': box,
         'route': valid[box[0]]['route'], 'after': []}]}, required_dock=required_dock)
    return {'consistent': True, 'plan': plan, 'conflicts': []}


def recoverable(error, reasons=RECOVERABLE_APPROACH_FAILURES):
    return any(str(error).startswith(reason) for reason in reasons)


def make_event(kind, event_id, *, participants, failure, **details):
    """Robot-facing event: the options come from the kind, never from the host's choice."""
    spec = EVENT_KINDS[kind]
    if not participants or any(r not in ROBOTS for r in participants):
        raise ValueError('event participants must be robots')
    return {'event_id': event_id, 'kind': kind, 'participants': list(participants),
            'failure': {'reason': str(failure)}, 'options': list(spec['options']),
            'option_meanings': dict(spec['meaning']), 'fail_closed': spec['fail_closed'],
            **details}


def build_recovery_request(rid, *, task, request_id, event, own_rgb, top_rgb,
                           own_history=(), inbox=()):
    if rid not in ROBOTS:
        raise ValueError('unknown robot')
    context = {'robot_id': rid, 'request_id': request_id, 'mission': copy.deepcopy(task),
               'event': copy.deepcopy(event), 'peer_messages': copy.deepcopy(list(inbox)[-6:]),
               'own_issued_commands': copy.deepcopy(list(own_history)[-16:])}
    return {'request_id': request_id,
            'messages': [{'role': 'system', 'content': _RECOVERY},
                         {'role': 'user', 'content': json.dumps(context, sort_keys=True)}],
            'images': images(own_rgb, top_rgb)}


def validate_recovery_reply(raw, request_id, event_id, options=('retry', 'abort')):
    value = parse(raw) if isinstance(raw, str) else copy.deepcopy(raw)
    if (not isinstance(value, dict)
            or set(value) != {'request_id', 'event_id', 'decision', 'reason', 'message'}):
        raise ValueError('recovery reply requires request_id, event_id, decision, reason and message')
    if value['request_id'] != request_id or value['event_id'] != event_id:
        raise ValueError('stale recovery reply')
    if value['decision'] not in options:
        raise ValueError('decision must be one of ' + ', '.join(options))
    text_fields(value)
    return value


def unanimous(replies):
    """Return the shared decision, or None when any reply is missing or differs."""
    values = [r['decision'] if r else None for r in replies.values()]
    if values and None not in values and len(set(values)) == 1:
        return values[0]
    return None


def start(team, frames, history, task, static_map, sim_time, *, identity=None, reference_top=None,
          required_dock=None, claim_fixture=None, **negotiation):
    """Independent self-claims; talk (existing negotiation) only when needed.

    Returns a log with ``path`` in {claims, conflict_then_talk,
    claims_rejected_then_talk} and the accepted ``feasibility`` report.
    """
    from harness.dispatch_feasibility import inspect_routes, negotiate_executable
    from scripts.three_robot_runtime import write
    if team.mode == 'fixture' and claim_fixture is None:
        raise ValueError('fixture coordination requires explicit claim fixtures')

    def build(rid, request_id):
        return build_claim_request(rid, task=task, request_id=request_id,
                                   own_rgb=frames[rid]['own_bytes'], top_rgb=frames[rid]['top_bytes'],
                                   own_history=history[rid],
                                   identity_evidence=(identity or {}).get(rid))

    def fixture(rid, request_id):
        claim = (claim_fixture or {}).get(rid)
        return {'request_id': request_id, 'claim': claim,
                'reason': 'scripted protocol fixture, not visual reasoning', 'message': ''}

    replies = team.ask(ROBOTS, build, validate_claim_reply, fixture, phase='claim', turn=0,
                       sim_time=sim_time)
    claims = {r: copy.deepcopy(v['claim']) if v else None for r, v in replies.items()}
    merged = merge_claims(claims, required_dock=required_dock)
    log = {'claims': claims, 'conflicts': merged['conflicts']}
    if merged['consistent']:
        committed = team.agreement.commit_claims(merged['plan'], claims, 0)
        report = inspect_routes(committed, static_map, frames['r1']['top_bytes'], identity, reference_top)
        write(team.output.parent / 'plan-feasibility-claims.json', report)
        if report['feasible']:
            team.event('STARTED_WITHOUT_NEGOTIATION', sim_time, plan_hash=committed['plan_hash'])
            return {**log, 'path': 'claims', 'feasibility': report}
        team.agreement.invalidate('RGB/map capability rejected the self-claims')
        task['execution_feedback'] = {
            'rejected_self_claims': claims, 'capability_result': report,
            'instruction': 'Your consistent self-claims failed the RGB/map capability check. '
                           'Talk and agree on a feasible plan (participants, routes, dependencies).'}
        path = 'claims_rejected_then_talk'
    else:
        task['claim_conflict'] = {
            'claims': claims, 'conflicts': merged['conflicts'],
            'instruction': 'Your independent self-claims conflict. Talk with your peers and '
                           'agree on ONE plan; keep your own claim if it still fits.'}
        path = 'conflict_then_talk'
    team.event('TALK_REQUIRED', sim_time, reason=path, conflicts=merged['conflicts'])
    report = negotiate_executable(team, frames, history, task, static_map, sim_time,
                                  identity=identity, reference_top=reference_top, **negotiation)
    return {**log, 'path': path, 'feasibility': report}


def consult(team, participants, event, frames, history, task, sim_time, *, turn,
            fixture_decisions=None, rounds=2):
    """Affected robots choose one event option unanimously, else the fail-closed one.

    Peer messages of this consultation reach only the participants.
    """
    options = tuple(event.get('options', ('retry', 'abort')))
    fail_closed = event.get('fail_closed', 'abort')
    if fail_closed not in options:
        raise ValueError('fail-closed decision must be an option')
    history_rounds = []
    for index in range(rounds):
        def build(rid, request_id):
            return build_recovery_request(rid, task=task, request_id=request_id, event=event,
                                          own_rgb=frames[rid]['own_bytes'],
                                          top_rgb=frames[rid]['top_bytes'],
                                          own_history=history[rid], inbox=team.inbox[rid])

        def fixture(rid, request_id):
            return {'request_id': request_id, 'event_id': event['event_id'],
                    'decision': (fixture_decisions or {}).get(rid, fail_closed),
                    'reason': 'scripted protocol fixture, not visual reasoning', 'message': ''}

        replies = team.ask(participants, build,
                           lambda raw, req: validate_recovery_reply(raw, req, event['event_id'], options),
                           fixture, phase=f"recovery-{event['event_id']}-r{index}", turn=turn,
                           sim_time=sim_time, recipients=participants)
        decision = unanimous(replies)
        history_rounds.append({'round': index, 'replies': copy.deepcopy(replies), 'decision': decision})
        if decision is not None:
            return decision, history_rounds
    return fail_closed, history_rounds
