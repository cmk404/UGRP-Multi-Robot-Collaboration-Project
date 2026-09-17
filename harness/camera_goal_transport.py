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


def lane_yellow(top_jpeg, rid):
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
    return yellow


def lane_heading(top_jpeg, rid):
    """Coarse image angle of the four wheels in the fixed, right-facing task.

    The authored camera/task convention is image-right = forward and positive
    turn = counterclockwise in the image. This is an unoriented rectangle:
    it cannot recognize reversed robots or arbitrary headings. Keep the near
    learned estimator for precise docking, and reject ambiguous wheel shapes.
    """
    return wheel_heading(lane_yellow(top_jpeg, rid))


def wheel_heading(yellow, *, pixel_tolerance=0.):
    """Existing four-corner heading gate on an explicitly selected RGB mask."""
    if not 0 <= pixel_tolerance <= 2.:
        raise ValueError("wheel mask tolerance must be at most two pixels")
    h, w = yellow.shape
    ys, xs = np.nonzero(yellow)
    if len(xs) < 80 or len(xs) > 700:
        return None
    (cx, cy), (a, b), angle = cv2.minAreaRect(np.column_stack((xs, ys)).astype(np.float32))
    long, short = max(a, b), min(a, b)
    if a < b:
        angle += 90
    angle = (angle + 90) % 180 - 90
    if not (.05*w-pixel_tolerance <= long <= .075*w+pixel_tolerance
            and .045*h-pixel_tolerance <= short <= .07*h+pixel_tolerance
            and 1.12 <= long/short <= 1.65 and abs(angle) <= 18):
        return None
    # All four wheel corners must contribute; a partial silhouette is not
    # sufficient evidence to turn. Coordinates here remain image pixels.
    theta = math.radians(angle)
    dx, dy = xs-cx, ys-cy
    u = dx*math.cos(theta)+dy*math.sin(theta)
    v = -dx*math.sin(theta)+dy*math.cos(theta)
    counts = [int(np.sum((u*s > .15*long) & (v*t > .15*short)))
              for s in (-1, 1) for t in (-1, 1)]
    if min(counts) < 5:
        return None
    return dict(angle_deg=float(angle), wheel_pixels=len(xs), corner_pixels=counts)


def lane_features(top_jpeg, rid):
    yellow = lane_yellow(top_jpeg, rid)
    h, w = yellow.shape
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
        return dict(ok=False, ready=False, forward=0., turn=0., reason='lane_or_payload_unresolved')
    heading = lane_heading(top_jpeg, rid)
    if heading is None:
        return dict(ok=False, ready=False, forward=0., turn=0., reason='wheel_heading_unresolved')
    error = ((current['beam_x']-current['robot_x']) -
             (reference['beam_x']-reference['robot_x']))
    # Handoff while still well inside the independently trained 15..40 cm
    # alignment domain. Readiness here is not grasp readiness.
    # Far forward-only driving amplified a +/-10 degree starting error into
    # 7..15 cm of lateral drift. Turn in place before advancing, and recheck
    # from fresh RGB at every slice. Issued turn is never assumed to succeed.
    heading_ready = abs(heading['angle_deg']) <= 1.5
    turn = (0. if heading_ready else math.copysign(
        min(.10, max(.01, .5*abs(math.radians(heading['angle_deg'])))), heading['angle_deg']))
    ready = error <= .065 and heading_ready
    forward = min(.12, max(.03, error)) if heading_ready and not ready else 0.
    reason = ('visual_coarse_heading' if not heading_ready else
              'visual_near_domain' if ready else 'visual_coarse_approach')
    return dict(ok=True, ready=ready, forward=forward, turn=turn, reason=reason,
                image_gap=error, features=current, heading=heading)


def own_payload(jpeg, *, hue_upper=24):
    frame = decode(jpeg)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    if hue_upper not in (24,35):raise ValueError("unsupported beam appearance calibration")
    mask = cv2.inRange(hsv, np.array((3,105,45), np.uint8), np.array((hue_upper,255,255), np.uint8))
    ys, xs = np.nonzero(mask)
    if len(xs) < 50:
        return None
    return [float(len(xs)/mask.size), float(xs.mean()/mask.shape[1]), float(ys.mean()/mask.shape[0])]


def dock_command(prediction):
    """Tighter folded-arm docking using an existing RGB error estimate only."""
    error = prediction.get('diagnostics', {}).get('image_derived_error')
    valid = (prediction.get('ok') is True and prediction.get('precision') == 'fine'
             and isinstance(error, (int, float)) and not isinstance(error, bool)
             and math.isfinite(error) and abs(error) <= .015)
    ready = valid and abs(error) <= .001
    return dict(ok=bool(valid), ready=bool(ready),
                forward=0. if ready or not valid else math.copysign(.01, error),
                image_derived_error=error)


def preclose_supported(predictions):
    return set(predictions) == {'r1', 'r3'} and all(
        d.get('observable') is True and d.get('confidence', 0) >= .8
        for d in predictions.values())


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
    valid = consistent and -.006 <= error <= .25
    ready = valid and -.006 <= error <= .0025
    # The loaded platform barely advances with very small wheel commands.
    # Use the demonstrated .04..10 motion range, then stop on fresh RGB.
    # A small bounded crossing of the target is also a stop, never more drive.
    forward = 0. if ready or not valid else min(.10, max(.04, 1.2*error))
    return dict(ok=valid, held_estimate=consistent, ready=ready, forward=forward,
                reason='visual_goal' if ready else 'visual_goal_error', image_gap=error,
                features=top, own_features=current, anchor_features=anchor)
