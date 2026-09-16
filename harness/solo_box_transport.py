"""Own-RGB manipulation plus shared-TOP goal servo; no simulator access."""
from __future__ import annotations

import cv2
import numpy as np

from harness.camera_goal_transport import decode
from harness.visual_box_skill import VisualBoxSkill, SEARCH
from harness.three_robot_mission import GOALS


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
    boxes = components((80, 70, 35), (105, 255, 255), 12)
    return {'zones': sorted(zones, key=lambda z:z['center'][0]), 'boxes': boxes}


class SoloBoxTransport:
    """Skill-bound curriculum, not a general route planner or raw-action LLM."""
    def __init__(self, goal):
        if goal not in GOALS:
            raise ValueError('unsupported goal')
        self.goal = goal
        self.box = VisualBoxSkill(task='external_navigation', robot_id='r2')
        self.initialized = False
        self.target = None
        self.done = False
        self.reason = None
        self.steps = 0
        self.goal_confirmations = 0

    @property
    def phase(self):
        return self.box.phase

    def decide(self, own, top_jpeg):
        if self.done:
            raise ValueError('completed task cannot issue more actions')
        self.steps += 1
        if self.steps > 600:
            raise RuntimeError('solo RGB decision budget exhausted')
        features = solo_top_features(top_jpeg)
        if self.target is None:
            if len(features['zones']) != 2 or len(features['boxes']) != 1:
                raise RuntimeError('solo cargo/destination unresolved in TOP RGB')
            self.target = features['zones'][GOALS.index(self.goal)]
        if not self.initialized:
            self.initialized = True
            return {'kind': 'pose', 'pulses': dict(SEARCH)}, features
        action = self.box.decide(own)
        if action['kind'] == 'finish':
            self.reason = action['reason']
            self.done = True
            return action, features
        if self.box.phase == 'carry':
            if len(features['boxes']) != 1:
                raise RuntimeError('carried box unresolved in TOP RGB')
            current = features['boxes'][0]['center']
            goal = self.target['center']
            dx, dy = goal[0]-current[0], goal[1]-current[1]
            features.update(goal=self.target, image_error=[dx,dy])
            if abs(dy) > .04 or dx < -.02:
                raise RuntimeError('solo path left supported visual lane')
            ready = -.01 <= dx <= .003
            self.goal_confirmations = self.goal_confirmations+1 if ready else 0
            if self.goal_confirmations >= 2:
                self.box.phase = 'release'
                return {'kind': 'wait', 'duration': .1}, features
            action = {'kind':'drive', 'fwd':0. if ready else min(.10,max(.04,dx)),
                      'turn':0., 'duration':.25}
        return action, features
