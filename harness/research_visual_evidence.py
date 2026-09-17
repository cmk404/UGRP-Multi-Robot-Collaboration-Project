"""Independent RGB witness for preparation/grasp claims, never a contact sensor.

The witness cannot issue actions and never sees the actor's proposed answer.
Only a positive, resolved image interpretation can preserve that answer. This
reduces self-confirmation; two model interpretations still do not prove physics.
"""
from __future__ import annotations

import base64
import copy
import json
import re


def detail_image(jpeg_base64):
    """Deterministic center detail from original RGB; no detection or world state.

    Keep the full image as well. A fixed central half-width/half-height crop is
    merely a display enlargement and cannot add resolution or reveal occlusion.
    """
    import cv2
    import numpy as np
    frame = cv2.imdecode(np.frombuffer(base64.b64decode(jpeg_base64), dtype=np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        raise ValueError("valid camera JPEG required")
    h, w = frame.shape[:2]
    crop = frame[h//4:3*h//4, w//4:3*w//4]
    ok, encoded = cv2.imencode('.jpg', cv2.resize(crop, (w, h), interpolation=cv2.INTER_NEAREST),
                               [cv2.IMWRITE_JPEG_QUALITY, 95])
    if not ok:
        raise ValueError("detail encoding failed")
    return base64.b64encode(encoded.tobytes()).decode('ascii')


def build_review_request(robot_id, phase, camera, *, roles, previous, request_id, retry=None):
    if phase not in ('PREPARE', 'GRASP'):
        raise ValueError("unsupported visual witness phase")
    system = '''You are an independent visual witness, not a robot controller.
Assess only the named robot and the plain orange beam in the blue square using
its OWN camera and shared TOP camera. Role is an agreed task assignment, not a
true robot position. Resolve body identity from the two views; do not guess from ID.
You receive no actor answer, commands, contacts, measured joints or success labels.
Inspect the actual pixels. Yellow wheel rims, image borders, dark floor, an empty
closed gripper, and being near a beam are NOT evidence of enclosing the beam.
Require two individually resolved fingers with the orange target visibly BETWEEN
them at the assigned endpoint. A visible gap between the entire gripper and target
contradicts enclosure. If fingers or target are offscreen/occluded/too small, mark
the relevant fields false and jaws_state unknown. TOP_DETAIL is the central half
of TOP enlarged 2x, with exactly the same pixels and no extra physical information.
OWN may not see the jaws; TOP can supply evidence only if it actually resolves them.
Compare previous/current images to judge rest. Camera/command assumptions are not
visual evidence. Confidence describes the entire conjunction, not body proximity.
Return exactly JSON keys request_id, identity_resolved, target_visible,
two_fingers_resolved, target_between_fingers, jaws_state, stationary, confidence,
reason. The five visibility/identity/rest fields are booleans. jaws_state is
open, closed or unknown. confidence is 0..1; reason <=600 characters must say
which view shows the fingers, their relation to the beam, and any uncertainty.
Copy current_request_id exactly. No actions or messages to another robot.'''
    system += ('\nPREPARE asks whether OPEN jaws straddle the endpoint, ready to close.' if phase == 'PREPARE'
               else '\nGRASP asks whether CLOSED jaws visibly enclose the endpoint after closure. Closed jaws alone do not establish a grasp.')
    context = {'robot_id':robot_id, 'phase':phase, 'agreed_roles':copy.deepcopy(roles),
               'current_request_id':request_id, 'retry':copy.deepcopy(retry)}
    images = []
    for prefix, pair in (('CURRENT',camera),('PREVIOUS',previous)):
        if pair is None: continue
        for label in ('own_rgb','top_rgb'):
            images.append({'label':prefix+'_'+label.upper(),
                           'image':'data:image/jpeg;base64,'+pair['images'][label]['jpeg_base64']})
        images.append({'label':prefix+'_TOP_DETAIL',
                       'image':'data:image/jpeg;base64,'+detail_image(pair['images']['top_rgb']['jpeg_base64'])})
    return {'messages':[{'role':'system','content':system},
                        {'role':'user','content':json.dumps(context,ensure_ascii=False)}], 'images':images}


def validate_review(raw, request_id):
    text = raw.strip()
    fenced = re.fullmatch(r'```(?:json)?\s*\n([\s\S]*?)\n```', text, re.IGNORECASE)
    if fenced: text = fenced.group(1)
    value = json.loads(text)
    booleans = {'identity_resolved','target_visible','two_fingers_resolved','target_between_fingers','stationary'}
    if not isinstance(value,dict) or set(value) != booleans|{'request_id','jaws_state','confidence','reason'}:
        raise ValueError('invalid visual witness fields')
    if value['request_id'] != request_id: raise ValueError('visual witness request_id mismatch')
    if any(type(value[k]) is not bool for k in booleans): raise ValueError('visual witness booleans required')
    if value['jaws_state'] not in ('open','closed','unknown'): raise ValueError('invalid jaws_state')
    if type(value['confidence']) not in (int,float) or not 0<=value['confidence']<=1:
        raise ValueError('invalid witness confidence')
    if not isinstance(value['reason'],str) or not value['reason'].strip() or len(value['reason'])>600:
        raise ValueError('invalid witness reason')
    return value


def supports_claim(review, phase):
    return bool(review and review['confidence'] >= .8
                and all(review[k] for k in ('identity_resolved','target_visible','two_fingers_resolved',
                                           'target_between_fingers','stationary'))
                and review['jaws_state'] == ('open' if phase=='PREPARE' else 'closed'))


def reviewed_reply(actor_reply, review, phase):
    """Never promote a claim or create an action; preserve both original records."""
    result = copy.deepcopy(actor_reply)
    if not supports_claim(review, phase):
        result.update(status='UNCERTAIN',confidence=0.,checks=[],action={'kind':'wait'},
                      reason='Independent RGB witness did not establish '+phase+' evidence.')
    return result
