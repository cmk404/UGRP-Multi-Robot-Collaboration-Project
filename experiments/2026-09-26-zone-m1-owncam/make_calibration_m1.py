"""Compose calibration_m1_dev.json = loop v2 calibration + two dev-only additions (M1 dev s91).

1. reset: settled-frame kidnap test (min_best_loglik_per_tag, kidnap_settle_s,
   kidnap_frames). Replay of dev-a1 s91: every normal-operation frame at the
   per-tag outlier floor was taken 0 s after an own servo command, none of the
   521 frames >= 0.3 s after one; after the grasp all settled tag frames sat at
   the floor (kidnapped) and were never reset (v2 threshold -12 < single-tag floor -9).
2. motion_profiles.fine: dead reckoning while the controller reports a skill
   manipulation phase (fit_fine_motion.py on dev s91 grasp commands).
The loop v2 parameters are unchanged; absent keys keep v1/v2 behaviour.
"""
import copy
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
V2 = HERE.parent/'2026-09-26-zone-owncam-loop-v2'/'calibration_loop_v2.json'


def main():
    base = json.loads(V2.read_text())
    fit = json.loads((HERE/'fine_motion_fit_dev.json').read_text())
    params = copy.deepcopy(base['params'])
    params['reset'].update({'min_best_loglik_per_tag': -8.5, 'kidnap_settle_s': .3, 'kidnap_frames': 2})
    f, t = fit['forward'], fit['turn']
    params['motion_profiles'] = {'fine': {
        'gain': [[f['gain'], 0., 0.], [0., f['gain'], 0.], [0., 0., t['gain']]],
        'tau_s': f['tau_s'], 'tau_axis_s': [f['tau_s'], f['tau_s'], t['tau_s']], 'tau_stop_s': fit['tau_stop_s'],
        'noise_rel': [.25, .4, .2], 'noise_abs': [.003, .003, .005], 'scale_std': .2, 'scale_walk': .01, 'use_scale': False}}
    out = {'schema': 'ugrp.owncam_calibration.m1_dev.v1', 'base': str(V2.relative_to(HERE.parents[1])),
           'additions': {'reset_kidnap': 'settled-frame kidnap reset (dev s91 replay)',
                         'motion_profiles.fine': 'fine_motion_fit_dev.json (dev s91 grasp commands)'},
           'split_used': 'dev only (m1dev-s91 dev-a1); GT used offline for the fit only', 'params': params}
    (HERE/'calibration_m1_dev.json').write_text(json.dumps(out, indent=2) + '\n')
    print('wrote calibration_m1_dev.json')


if __name__ == '__main__':
    main()
