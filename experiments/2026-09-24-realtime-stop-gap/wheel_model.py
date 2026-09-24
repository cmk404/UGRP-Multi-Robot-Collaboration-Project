#!/usr/bin/env python3
"""1-D forward model of MasterPi v2 drive dynamics for lease patterns.

Uses sim/masterpi_dynamics_v2.py constants (motor lag, force, moving/stop
damping, 1.10 kg). Contacts, yaw and lateral coupling are ignored, so absolute
displacements are indicative only; the ratio between lease patterns is the claim.
"""
import json, math, sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from sim import masterpi_dynamics_v2 as dyn  # noqa: E402

MASS = dyn.OFFICIAL_TOTAL_MASS_KG
DT = .00025


def displacement(on_s, gap_s, count, command=.01):
    x = v = motor = 0.
    alpha = 1 - math.exp(-DT / dyn.MOTOR_TIME_CONSTANT_S)
    segments = [(on_s, command), (gap_s, 0.)] * count + [(2., 0.)]
    for duration, cmd in segments:
        for _ in range(int(round(duration / DT))):
            motor += alpha * (cmd - motor)
            damping = dyn.STOP_LINEAR_DAMPING_N_PER_MPS if cmd == 0 else dyn.LINEAR_DAMPING_N_PER_MPS
            v += DT * (dyn.MAX_FORWARD_FORCE_N * motor - damping * v) / MASS
            x += v * DT
    return x


def main():
    patterns = {'synchronous_0.20_back_to_back': (.2, 0.), 'realtime_0.25_no_gap': (.25, 0.),
                'realtime_0.25_gap_0.05': (.25, .05), 'realtime_0.25_gap_0.175': (.25, .175)}
    rows = {name: {str(n): round(displacement(on, gap, n) / (n * .01), 4) for n in (1, 5, 20)}
            for name, (on, gap) in patterns.items()}
    result = {'schema': 'ugrp.wheel_lease_model.v1', 'unit': 'metres per unit forward command per slice',
              'constants': {'motor_time_constant_s': dyn.MOTOR_TIME_CONSTANT_S,
                            'max_forward_force_n': dyn.MAX_FORWARD_FORCE_N,
                            'moving_damping': dyn.LINEAR_DAMPING_N_PER_MPS,
                            'stop_damping': dyn.STOP_LINEAR_DAMPING_N_PER_MPS, 'mass_kg': MASS},
              'per_unit_command_by_slice_count': rows}
    Path(__file__).with_name('wheel-model.json').write_text(json.dumps(result, indent=1) + '\n')
    print(json.dumps(result, indent=1))


if __name__ == '__main__':
    main()
