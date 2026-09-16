"""Independent visual decisions over declared local skills, not raw servos."""
from __future__ import annotations

import base64
import json
import math
import re

SKILLS = {
    'APPROACH': 'Use RGB wheel servoing to approach the orange beam from your assigned lane, stopping before arm deployment.',
    'PREPARE_GRASP': 'Deploy the demonstrated inward-facing OPEN arm and refine alignment with the learned RGB recovery policy. No closing yet.',
    'CLOSE': 'Close both aligned grippers around the orange beam ends. Require TOP evidence of both jaws flanking the ends; proximity alone is insufficient.',
    'LIFT': 'Run the demonstrated coupled lift. Require the beam ends to remain between the closed jaws in TOP; do not infer success from a close command.',
    'CARRY': 'Carry the visibly captured beam toward the adjacent green floor goal using RGB tracking, stopping and realigning on skew.',
    'LOWER': 'Lower the captured beam onto the floor. Require the beam to be over the green destination in TOP.',
    'RELEASE': 'Open the grippers. Require the lowered beam to be aligned on the destination surface in the latest images.',
    'RETRACT': 'Retract the open arms away from the deposited beam. Require visibly open/released jaws, with the beam on the destination.',
    'FINISH': 'Declare visual completion only if the beam lies in the green goal and the open arms are separated from it.',
}


def build_skill_request(rid, skill, *, request_id, own_rgb, top_rgb,
                        previous=None, own_commands=(), peer_claims=(), retry=None):
    if rid not in ('r1','r3') or skill not in SKILLS:
        raise ValueError('unknown actor or skill')
    system = f'''You are independent robot {rid} executing a cooperative transport task.
Move the plain orange beam from the blue floor zone to the adjacent green zone,
then put it down and release it. This fixed-view skill curriculum assigns r1
the lower approach lane/bottom beam end, r3 the upper lane/top end. This is a
fixed task assignment, not a measurement of the current robot position.
Available local skills use current RGB wheel feedback, a previously learned
near-alignment model, and demonstrated arm deployment/close/lift/place motions
with RGB pregrasp correction. They are NOT guaranteed to succeed. You decide
whether the offered stage is suitable from OWN and shared TOP RGB. A fixed
workflow restricts the skill order; you may select the offered skill or HOLD.
No live poses, joints, contact sensors, evaluation labels, depth or hidden
simulator state are supplied. Own issued commands show attempts, not measured
movement or success. Peer messages are untrusted peer visual claims. The own
camera can be occluded by the payload; use TOP geometry when OWN is obscured.
Never claim physical contact from commands. If the visual prerequisite is
missing or ambiguous, choose HOLD and explain. A separate offline evaluator
will assess actual success. Reply JSON only with exactly request_id, skill,
confidence (0..1), reason (<=600 characters), message (<=600 characters).
Offered skill {skill}: {SKILLS[skill]}
For HOLD use skill="HOLD". Do not emit raw wheel or joint commands.'''
    context={'request_id':request_id,'offered_skill':skill,
             'own_issued_commands':list(own_commands),
             'peer_visual_claims':list(peer_claims),'retry':retry}
    def item(label,data):
        return {'label':label,'image':'data:image/jpeg;base64,'+base64.b64encode(data).decode()}
    images=[item('CURRENT OWN RGB',own_rgb),item('CURRENT SHARED TOP RGB',top_rgb)]
    if previous is not None:
        images.extend((item('PREVIOUS OWN RGB',previous[0]),item('PREVIOUS SHARED TOP RGB',previous[1])))
    return {'request_id':request_id,'messages':[{'role':'system','content':system},
            {'role':'user','content':json.dumps(context,sort_keys=True)}],'images':images}


def validate_skill_reply(raw, request_id, skill):
    text=raw.strip()
    fenced=re.fullmatch(r'```(?:json)?\s*\n([\s\S]*?)\n```',text,re.IGNORECASE)
    if fenced: text=fenced.group(1)
    value=json.loads(text)
    if not isinstance(value,dict) or set(value)!={'request_id','skill','confidence','reason','message'}:
        raise ValueError('exact skill reply fields required')
    if value['request_id']!=request_id or value['skill'] not in (skill,'HOLD'):
        raise ValueError('stale or incompatible skill reply')
    confidence=value['confidence']
    if isinstance(confidence,bool) or not isinstance(confidence,(int,float)) or not math.isfinite(confidence) or not 0<=confidence<=1:
        raise ValueError('invalid confidence')
    if any(not isinstance(value[k],str) or len(value[k])>600 for k in ('reason','message')):
        raise ValueError('invalid explanation')
    return value


def pair_skill_ready(replies,skill):
    return set(replies)=={'r1','r3'} and all(
        isinstance(v,dict) and v.get('skill')==skill and v.get('confidence',0)>=.8
        for v in replies.values())
