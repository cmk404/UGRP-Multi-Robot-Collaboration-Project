"""Own-RGB manipulation plus shared-TOP goal servo; no simulator access."""
from __future__ import annotations

import base64
import hashlib
import cv2
import numpy as np

from harness.camera_goal_transport import decode
from harness.visual_box_skill import VisualBoxSkill, SEARCH
from harness.three_robot_mission import GOALS


def normalize_own_rgb(observation):
    """Use the demonstrated 640x480 own-camera pixel contract without changing FOV.

    The caller retains the original JPEG separately. This produces the exact
    cv2.resize/JPEG-95 input used by the successful dispatch solo adapter.
    """
    raw = base64.b64decode(observation['image'], validate=True)
    decoded = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
    if decoded is None:
        raise ValueError('invalid own RGB JPEG')
    height, width = decoded.shape[:2]
    if (width, height) not in {(640, 480), (960, 720)}:
        raise ValueError('unsupported own RGB pixel contract; expected 640x480 or 960x720')
    raw_sha = hashlib.sha256(raw).hexdigest()
    if observation.get('sha256') is not None and observation['sha256'] != raw_sha:
        raise ValueError('own RGB SHA does not match original JPEG')
    resized = cv2.resize(decoded, (640, 480))
    encoded = cv2.imencode('.jpg', resized, [cv2.IMWRITE_JPEG_QUALITY, 95])
    if not encoded[0]:
        raise ValueError('own RGB normalization failed')
    normalized = encoded[1].tobytes()
    result = dict(observation, image=base64.b64encode(normalized).decode(),
                  sha256=hashlib.sha256(normalized).hexdigest())
    evidence = {'method': 'cv2.resize 640x480 then JPEG quality 95',
                'raw_size': [width, height], 'skill_size': [640, 480],
                'raw_sha256': raw_sha,
                'skill_sha256': result['sha256']}
    return result, evidence


def solo_top_features(jpeg):
    frame = decode(jpeg)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, w = hsv.shape[:2]
    def components(low, high, area):
        mask = cv2.inRange(hsv, np.array(low, np.uint8), np.array(high, np.uint8))
        mask[:int(.80*h)] = 0  # Authored lower-lane image convention.
        count, _, stats, centers = cv2.connectedComponentsWithStats(mask)
        return [{'center': (centers[i]/[w,h]).tolist(),
                 'bounds': (stats[i,:4]/[w,h,w,h]).tolist(), 'area': int(stats[i,4])}
                for i in range(1,count) if stats[i,4] >= area]
    zones = components((135, 70, 40), (175, 255, 255), 200)
    boxes = components((80, 70, 35), (105, 255, 255), 30)
    return {'zones': sorted(zones, key=lambda z:z['center'][0]), 'boxes': boxes}


class SoloBoxTransport:
    """Skill-bound curriculum, not a general route planner or raw-action LLM."""
    def __init__(self, goal=None, *, robot_id='r2', navigator=None, attachment_min_saturation=65, release_refine_ground_fit=False):
        if robot_id not in ('r1','r2','r3'):
            raise ValueError('unknown solo robot')
        if navigator is None and goal not in GOALS:
            raise ValueError('unsupported goal')
        self.goal = goal
        self.navigator = navigator
        self.box = VisualBoxSkill(task='external_navigation', robot_id=robot_id,
                                  attachment_home_reference='previous_endpoint',attachment_min_saturation=attachment_min_saturation,
                                  release_refine_ground_fit=release_refine_ground_fit)
        self.initialized = False
        self.target = None
        self.done = False
        self.reason = None
        self.steps = 0
        self.goal_confirmations = 0
        self.grip_reobservations = 0

    @property
    def phase(self):
        return self.box.phase

    def decide(self, own, top_jpeg):
        if self.done:
            raise ValueError('completed task cannot issue more actions')
        self.steps += 1
        if self.steps > 600:
            raise RuntimeError('solo RGB decision budget exhausted')
        features = solo_top_features(top_jpeg) if self.navigator is None else {}
        if self.target is None and self.navigator is None:
            if len(features['zones']) != 2 or len(features['boxes']) != 1:
                raise RuntimeError('solo cargo/destination unresolved in TOP RGB')
            self.target = features['zones'][GOALS.index(self.goal)]
        if not self.initialized:
            self.initialized = True
            return {'kind': 'pose', 'pulses': dict(SEARCH)}, features
        previous_phase = self.box.phase
        recorder=getattr(self.navigator,'record_attachment_top',None)
        if recorder is not None:recorder(previous_phase,top_jpeg)
        action = self.box.decide(own)
        if previous_phase == 'lift' and self.box.phase == 'verify_lift':
            # Establish the visual attachment anchor after lift settling, not
            # during the final arm transient. Keep all comotion thresholds.
            action = {'kind':'wait', 'duration':1.0}
        if (action['kind']=='finish' and previous_phase=='carry'
                and action['reason'] in {'VISUAL_GRASP_DRIFT', 'TOP_GEOMETRY_AMBIGUOUS_FOR_DROP'}
                and self.grip_reobservations < 4):
            # A stale held-image anchor is not a new contact measurement.
            # Stop and request the same complete physical pan intervention;
            # resume only after new three-view comotion evidence passes.
            self.grip_reobservations += 1
            self.box.phase='verify_lift'
            self.box.reason='RUNNING'
            self.box.held=False
            return {'kind':'wait','duration':.3}, {**features,
                'grip_reobservation':self.grip_reobservations,'trigger':action['reason']}
        if action['kind'] == 'finish':
            self.reason = action['reason']
            self.done = True
            return action, features
        if self.box.phase == 'carry':
            if self.navigator is not None:
                action, features = self.navigator.observe(top_jpeg)
                if features['done']:
                    self.box.phase = 'release'
                    return {'kind':'wait','duration':.1}, features
                return action, features
            if len(features['boxes']) != 1:
                raise RuntimeError('carried box unresolved in TOP RGB')
            current = features['boxes'][0]['center']
            goal = self.target['center']
            dx, dy = goal[0]-current[0], goal[1]-current[1]
            features.update(goal=self.target, image_error=[dx,dy])
            if abs(dy) > .04 or dx < -.02:
                raise RuntimeError('solo path left supported visual lane')
            # Destination is a floor region, not a zero-error point. Require
            # the entire visible cargo width inside it with an image margin;
            # the fixed lane check above governs lateral alignment.
            bx, _, bw, _ = features['boxes'][0]['bounds']
            gx, _, gw, _ = self.target['bounds']
            ready = gx+.01 <= bx and bx+bw <= gx+gw-.01
            self.goal_confirmations = self.goal_confirmations+1 if ready else 0
            if self.goal_confirmations >= 2:
                self.box.phase = 'release'
                return {'kind': 'wait', 'duration': .1}, features
            action = {'kind':'drive', 'fwd':0. if ready else min(.15,max(.04,2*dx)),
                      'turn':0., 'duration':.25}
        return action, features
