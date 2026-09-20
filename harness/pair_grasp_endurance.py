"""RGB-only bounded stationary/translation endurance controller.

The clock is the count of this actor's issued 0.1s commands. No measured state,
contact, joint values, or evaluation enters this module.
"""
import math
import numpy as np
from harness.pair_navigation import ROBOTS, digest, rotate, wrap, swept_clear
from harness.pair_transport_vision import GeometryPairVision

DT = .1

class EnduranceActor:
    def __init__(self, data, rid, mode, *, slip_guard=False):
        if rid not in ROBOTS or mode not in ('stationary','shuttle'):
            raise ValueError('invalid endurance actor configuration')
        self.data,self.rid,self.mode=data,rid,mode
        self.slip_guard=slip_guard
        self.vision=GeometryPairVision(data,slip_guard=slip_guard,interval_s=DT)
        self.anchor=self.previous=self.anchor_angles=None
        self.velocity={r:np.zeros(2) for r in ROBOTS}
        self.sequence=0;self.terminal=None;self.plan_hash=None

    def decide(self, own_rgb, top_rgb):
        action={'kind':'mecanum','forward':0.,'left':0.,'turn':0.,'duration_s':DT}
        if self.terminal:
            return {'action':action,'status':'endurance_stop','ready':False,'done':False,
                    'error':self.terminal,'plan_hash':self.plan_hash,
                    **({'own_carry_observation':self.vision.carry_monitor.last} if self.slip_guard else {})}
        try:obs=self.vision.observe(own_rgb,top_rgb)
        except ValueError as e:
            self.terminal=str(e);return self.decide(own_rgb,top_rgb)
        positions={r:np.array(obs[r]['xy_m']) for r in ROBOTS}
        angles=dict(self.vision.geometry_angles)
        if self.anchor is None:
            self.anchor={r:p.copy() for r,p in positions.items()};self.anchor_angles=dict(angles)
            self.spacing=float(np.linalg.norm(positions['r1']-positions['r3']))
            center=(positions['r1']+positions['r3'])/2
            endpoint=center+np.array([.16 if self.mode=='shuttle' else 0.,0.])
            if not swept_clear((*center,0.),(*endpoint,0.),self.data):
                self.terminal='authored map blocks the endurance corridor';return self.decide(own_rgb,top_rgb)
            self.plan_hash=digest({'map':self.data,'anchor':{r:p.tolist() for r,p in self.anchor.items()},'angles':angles,'mode':self.mode,'period_s':12.,'travel_m':.16})
        elapsed=self.sequence*DT
        theta=2*math.pi*elapsed/12.
        offset=np.array([.08*(1-math.cos(theta)),0.]) if self.mode=='shuttle' else np.zeros(2)
        desired_velocity=np.array([.08*2*math.pi/12*math.sin(theta),0.]) if self.mode=='shuttle' else np.zeros(2)
        if self.previous is not None:
            for r in ROBOTS:self.velocity[r]=.5*self.velocity[r]+.5*(positions[r]-self.previous[r])/DT
        errors={r:self.anchor[r]+offset-positions[r] for r in ROBOTS}
        yaw_errors={r:wrap(self.anchor_angles[r]-angles[r]) for r in ROBOTS}
        separation=float(np.linalg.norm(positions['r1']-positions['r3']))
        if (max(np.linalg.norm(e) for e in errors.values())>.03 or abs(separation-self.spacing)>.03
                or max(abs(e) for e in yaw_errors.values())>.12):
            self.terminal='visual endurance formation exceeded tracking envelope';return self.decide(own_rgb,top_rgb)
        local=rotate(desired_velocity+6*errors[self.rid]-.6*(self.velocity[self.rid]-desired_velocity),-angles[self.rid])
        action.update(forward=float(np.clip(local[0],-.05,.05)),left=float(np.clip(local[1],-.10,.10)),turn=float(np.clip(1.5*yaw_errors[self.rid],-.10,.10)))
        self.previous=positions;self.sequence+=1
        return {'action':action,'status':'endurance_'+self.mode,'ready':True,'done':False,'plan_hash':self.plan_hash,
                'observations':obs,'elapsed_command_s':elapsed,'target_offset_m':offset.tolist(),
                'own_carry_observation':self.vision.carry_monitor.last}

    def after_regrasp(self):
        """Keep the original path/command clock; require fresh RGB before GO."""
        from harness.pair_transport_vision import OwnCarryMonitor
        self.terminal=None
        self.previous=None
        self.velocity={r:np.zeros(2) for r in ROBOTS}
        self.vision.carry_monitor=OwnCarryMonitor(slip_guard=self.slip_guard,interval_s=DT)
