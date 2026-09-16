"""RGB visual servoing around the existing demonstrated local grasp skill.

Fixed-view, two-lane curriculum only. These are image measurements, not robot
poses or contact measurements. The learner/runner must report grasp playback
separately from image-feedback wheel control and from LLM planning.
"""
from __future__ import annotations

import math

import cv2
import numpy as np

from harness.camera_beam_features import extract_beams


def decode(jpeg):
    if not isinstance(jpeg, bytes) or not jpeg:
        raise ValueError('nonempty JPEG required')
    frame = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        raise ValueError('invalid JPEG')
    return frame


def lane_features(top_jpeg, rid):
    if rid not in ('r1', 'r3'):
        raise ValueError('unknown robot')
    top = decode(top_jpeg)
    hsv = cv2.cvtColor(top, cv2.COLOR_BGR2HSV)
    yellow = cv2.inRange(hsv, np.array((20, 70, 50), np.uint8),
                        np.array((40, 255, 255), np.uint8))
    h, w = yellow.shape
    # Lane identity is a fixed task assignment in this curriculum, not a
    # general identity recognizer. Reject absent/ambiguous foreground.
    if rid == 'r1':
        yellow[:h//2] = 0
    else:
        yellow[h//2:] = 0
    yellow[:int(.25*h)] = 0
    yellow[int(.8*h):] = 0
    ys, xs = np.nonzero(yellow)
    beams = [b for b in extract_beams(top_jpeg)
             if .35 <= b['center'][1] <= .65 and not b['touches_border']]
    if len(xs) < 40 or len(beams) != 1 or np.ptp(xs) > .15*w:
        return None
    return {'robot_x': float(xs.mean()/w), 'robot_y': float(ys.mean()/h),
            'beam_x': beams[0]['center'][0], 'beam_y': beams[0]['center'][1]}


def coarse_approach(top_jpeg, reference_top, rid):
    current, reference = lane_features(top_jpeg, rid), lane_features(reference_top, rid)
    if current is None or reference is None:
        return dict(ok=False, ready=False, forward=0., reason='lane_or_payload_unresolved')
    error = ((current['beam_x']-current['robot_x']) -
             (reference['beam_x']-reference['robot_x']))
    # Handoff while still well inside the independently trained 15..40 cm
    # alignment domain. Readiness here is not grasp readiness.
    ready = error <= .065
    return dict(ok=True, ready=ready, forward=0. if ready else min(.12, max(.03, error)),
                reason='visual_near_domain' if ready else 'visual_coarse_approach',
                image_gap=error, features=current)


def own_payload(jpeg):
    frame = decode(jpeg)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array((3,105,45), np.uint8), np.array((24,255,255), np.uint8))
    ys, xs = np.nonzero(mask)
    if len(xs) < 50:
        return None
    return [float(len(xs)/mask.size), float(xs.mean()/mask.shape[1]), float(ys.mean()/mask.shape[0])]


def goal_features(top_jpeg, *, remembered_goal_x=None):
    frame = decode(top_jpeg)
    h,w = frame.shape[:2]
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    green = cv2.inRange(hsv, np.array((40,60,30), np.uint8), np.array((90,255,255), np.uint8))
    green[:int(.3*h)] = 0
    green[int(.72*h):] = 0
    count, labels, stats, centers = cv2.connectedComponentsWithStats(green)
    goals = [i for i in range(1,count) if stats[i,4] > .004*w*h
             and .4 <= centers[i,1]/h <= .6]
    beams = [b for b in extract_beams(top_jpeg)
             if .35 <= b['center'][1] <= .65 and not b['touches_border']]
    if (remembered_goal_x is None and len(goals) != 1) or len(beams) != 1:
        return None
    goal_x = (float((stats[goals[0],0]+(stats[goals[0],2]-1)/2)/w)
              if remembered_goal_x is None else remembered_goal_x)
    return dict(goal_x=goal_x, beam_x=beams[0]['center'][0],
                beam_y=beams[0]['center'][1], beam_length=beams[0]['length_px']/h)


def goal_carry(own_jpeg, top_jpeg, anchor_own, anchor_top):
    # The static floor goal is remembered from the first carry frame. A robot
    # may occlude/split its green pixels later; that must not move the target.
    initial_top = goal_features(anchor_top)
    if initial_top is None:
        return dict(ok=False, held_estimate=False, ready=False, forward=0., reason='initial_goal_unresolved')
    current, anchor = own_payload(own_jpeg), own_payload(anchor_own)
    top = goal_features(top_jpeg, remembered_goal_x=initial_top['goal_x'])
    if current is None or anchor is None or top is None:
        return dict(ok=False, held_estimate=False, ready=False, forward=0., reason='visual_evidence_missing')
    # Consistency with the post-lift RGB, never a claim of measured contact.
    consistent = (.25 <= current[0]/anchor[0] <= 4.
                  and math.dist(current[1:],anchor[1:]) <= .15)
    error = top['goal_x']-top['beam_x']
    valid = consistent and -.015 <= error <= .25
    ready = valid and -.006 <= error <= .0025
    # The loaded platform barely advances with very small wheel commands.
    # Use the demonstrated .04..10 motion range, then stop on fresh RGB.
    # A small bounded crossing of the target is also a stop, never more drive.
    forward = 0. if ready or not valid else min(.10, max(.04, 1.2*error))
    return dict(ok=valid, held_estimate=consistent, ready=ready, forward=forward,
                reason='visual_goal' if ready else 'visual_goal_error', image_gap=error,
                features=top, own_features=current, anchor_features=anchor)
