#!/usr/bin/env python3
"""Frozen, bounded numerical/formation diagnosis; privileged values are outputs only."""
from pathlib import Path
import argparse, json, sys
from unittest.mock import patch
import numpy as np
ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path: sys.path.insert(0, str(ROOT))
from scripts import run_pair_grasp_endurance as endurance
from scripts.run_camera_approach_student import write


class DiagnosticScene(endurance.EnduranceScene):
    def evaluation_snapshot(self, *, full_state=True):
        import mujoco
        row = super().evaluation_snapshot(full_state=full_state)
        m, d = self.world.model, self.world.data
        records = []
        for i, c in enumerate(d.contact[:d.ncon]):
            if self.beam_geom not in (c.geom1, c.geom2): continue
            other = int(c.geom2 if c.geom1 == self.beam_geom else c.geom1)
            name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, other)
            if name not in {r+'__'+side+'_finger' for r in ('r1','r3') for side in ('left','right')}: continue
            force = np.zeros(6); mujoco.mj_contactForce(m, d, i, force)
            velocities = []
            for gid in (c.geom1, c.geom2):
                jp, jr = np.zeros((3,m.nv)), np.zeros((3,m.nv))
                mujoco.mj_jac(m, d, jp, jr, c.pos, int(m.geom_bodyid[gid]))
                velocities.append(jp@d.qvel)
            relative = c.frame.reshape(3,3)@(velocities[1]-velocities[0])
            records.append({'finger':name,'force_contact_frame_n':force.tolist(),
                'relative_velocity_contact_frame_m_s':relative.tolist(),
                'friction_coefficients':c.friction.tolist(),
                'friction_cone_utilization':float(np.linalg.norm(force[1:3]/c.friction[:2])/max(force[0],1e-12)),
                'distance_m':float(c.dist),'position_m':c.pos.tolist()})
        row['contact_diagnostics_output_only'] = records
        return row


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--grasp-model-dir', type=Path, required=True)
    p.add_argument('--out-dir', type=Path, required=True)
    p.add_argument('--noslip', type=int, choices=(0,1,3), default=0)
    p.add_argument('--tracked-lift', action='store_true')
    p.add_argument('--finger-damping', type=int, choices=(0,3000), default=0)
    p.add_argument('--timestep', type=float, choices=(.002,.001,.00025), default=.002)
    p.add_argument('--diagexact', action='store_true')
    p.add_argument('--formation-integral', type=float, choices=(0.,2.), default=0.)
    p.add_argument('--lateral-limit', type=float, choices=(.10,.30), default=.10)
    p.add_argument('--duration', type=int, choices=(30,60,300), default=30)
    p.add_argument('--mode', choices=('stationary','shuttle'), default='stationary')
    a=p.parse_args()
    data=json.loads((ROOT/'maps/pair_navigation/narrow-door.json').read_text())
    def factory(*args, **kwargs):
        return DiagnosticScene(*args, **kwargs, noslip_iterations=a.noslip, tracked_lift=a.tracked_lift,
                               finger_damping=a.finger_damping, timestep=a.timestep, diagexact=a.diagexact,
                               formation_integral=a.formation_integral, lateral_limit=a.lateral_limit)
    original_actor = endurance.EnduranceActor
    def actor(*args):
        return original_actor(*args, integral_gain=a.formation_integral, lateral_limit=a.lateral_limit)
    with patch.object(endurance,'EnduranceScene',factory), patch.object(endurance,'EnduranceActor',actor):
        result=endurance.run(data,a.grasp_model_dir.resolve(),a.out_dir.resolve(),a.mode,a.duration)
    result.update(schema='ugrp.pair_grasp_retention_diagnostic.v1', noslip_iterations=a.noslip,
                  tracked_lift=a.tracked_lift, diagnostic_only=True, finger_damping=a.finger_damping,
                  timestep=a.timestep, diagexact=a.diagexact,
                  formation_integral=a.formation_integral, lateral_limit=a.lateral_limit)
    write(a.out_dir/'result.json', result)
    return 0 if result['error'] is None else 1

if __name__=='__main__': raise SystemExit(main())
