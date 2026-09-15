"""Keep the visually observed grasp formation during the fixed pre-drive hold.

Actors receive their own RGB and the common top RGB only. Anchors, velocities,
and readiness are derived from those images, never from physics/evaluation.
"""
from __future__ import annotations

import math
import numpy as np

from harness.pair_navigation import ROBOTS, digest, rotate, wrap
from harness.pair_transport_vision import GeometryPairVision

HOLD_DT = .1


class PairGraspSpacing:
    def __init__(self, data, robot_id, *, integral_gain=0., lateral_limit=.10):
        if robot_id not in ROBOTS:
            raise ValueError('invalid robot')
        self.rid = robot_id
        self.vision = GeometryPairVision(data)
        self.anchor = None
        self.previous = None
        self.velocity = {r: np.zeros(2) for r in ROBOTS}
        self.previous_angles = None
        self.angular_velocity = {r: 0. for r in ROBOTS}
        self.stable_frames = 0
        self.terminal = None
        self.plan_hash = None
        self.integral_gain, self.lateral_limit = integral_gain, lateral_limit
        self.integral = np.zeros(2)

    def decide(self, own_rgb, top_rgb):
        action = {'kind':'mecanum', 'forward':0., 'left':0., 'turn':0., 'duration_s':HOLD_DT}
        if self.terminal:
            return {'action':action,'status':'spacing_stop','ready':False,'done':False,
                    'error':self.terminal,'plan_hash':self.plan_hash}
        try:
            obs = self.vision.observe(own_rgb, top_rgb)
        except ValueError as error:
            self.terminal = str(error)
            return self.decide(own_rgb, top_rgb)
        positions = {r:np.array(obs[r]['xy_m']) for r in ROBOTS}
        angles = dict(self.vision.geometry_angles)
        first = self.anchor is None
        if first:
            self.anchor = {r:p.copy() for r,p in positions.items()}
            self.anchor_angles = dict(angles)
            self.spacing = float(np.linalg.norm(positions['r1']-positions['r3']))
            self.plan_hash = digest({'map':self.vision.map, 'spacing_anchor':
                {r:self.anchor[r].tolist() for r in ROBOTS}, 'angles':angles})
        elif self.previous is not None:
            for r in ROBOTS:
                self.velocity[r] = .5*self.velocity[r]+.5*(positions[r]-self.previous[r])/HOLD_DT
                omega = wrap(angles[r]-self.previous_angles[r])/HOLD_DT
                self.angular_velocity[r] = .5*self.angular_velocity[r]+.5*omega
        errors = {r:self.anchor[r]-positions[r] for r in ROBOTS}
        separation = float(np.linalg.norm(positions['r1']-positions['r3']))
        angle_errors = {r:wrap(self.anchor_angles[r]-angles[r]) for r in ROBOTS}
        if (max(np.linalg.norm(e) for e in errors.values()) > .025
                or abs(separation-self.spacing) > .03
                or max(abs(e) for e in angle_errors.values()) > .12):
            self.terminal = 'visual grasp formation exceeded recovery envelope'
            return self.decide(own_rgb, top_rgb)
        stable = (max(np.linalg.norm(e) for e in errors.values()) <= .006
                  and abs(separation-self.spacing) <= .008
                  and max(abs(e) for e in angle_errors.values()) <= .03)
        self.stable_frames = self.stable_frames+1 if stable else 0
        if not first:
            self.integral = np.clip(self.integral+self.integral_gain*HOLD_DT*errors[self.rid], -.25, .25)
            local = rotate(6.*errors[self.rid]-.6*self.velocity[self.rid]+self.integral, -angles[self.rid])
            angular = 1.5*angle_errors[self.rid]-.25*self.angular_velocity[self.rid]
            action.update(forward=float(np.clip(local[0],-.05,.08)),
                          left=float(np.clip(local[1],-self.lateral_limit,self.lateral_limit)),
                          turn=float(np.clip(angular,-.10,.10)))
        # The anchor precedes the fixed lift replay. Its next frame is not
        # spaced by HOLD_DT, so start velocity differencing after that frame.
        self.previous = None if first else positions
        self.previous_angles = None if first else angles
        return {'action':action,'status':'spacing_anchor' if first else 'spacing_hold',
                'ready':True,'done':False,'plan_hash':self.plan_hash,
                'observations':obs,'anchor_xy_m':{r:p.tolist() for r,p in self.anchor.items()},
                'spacing_error_m':separation-self.spacing,'stable_frames':self.stable_frames,
                'own_carry_observation':self.vision.carry_monitor.last}
