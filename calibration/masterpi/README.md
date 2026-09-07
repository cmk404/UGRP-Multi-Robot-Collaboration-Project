# MasterPi calibrated task digital twin

This directory contains the physical evidence used to turn the clean nominal MuJoCo V2 model into a validated task digital twin of hardware unit `ugrp1` for `search -> track -> approach -> pick/place`.

## Non-negotiable rule

SIM/MuJoCo/synthetic measurements are never admissible as calibration evidence. Synthetic data is used only in unit tests of the fitting code. `sim/masterpi_dynamics_calibration.json` stays `validated=false` until the final validator passes independent physical hold-outs.

## Evidence sets

- `static_measurements.json`: measured wheel radius, wheelbase, track, block mass and block/floor friction.
- `chassis_trials.jsonl`: 108 external-motion trials: 72 fit + 36 hold-out. Use the two supplied ArUco markers and a fixed overhead camera; each completed row must include endpoint x/y/yaw, peak speed and stop distance.
- `servo_trials.jsonl`: 60 externally observed servo steps: 40 fit + 20 hold-out across servos 3/4/5/6, including small PWM steps so deadband is identifiable.
- `hand_eye_trials.jsonl`: 24 external floor-ground-truth observations: 12 fit + 12 hold-out. These identify actual camera link/z/pitch and physical servo-6 center while the REAL controller keeps its own internal nominal geometry.
- `task_trials.jsonl`: 12 physical pick fit cases + 20 untouched hold-out cases. The fit cases identify gripper contact parameters; the hold-outs measure final REAL-vs-SIM outcome parity.

## Pipeline

1. Measure static properties and run `apply_masterpi_static_measurements.py`.
2. Execute one chassis trial at a time with `run_masterpi_calibration_trial.py`; analyze an external overhead recording with `analyze_masterpi_overhead_video.py`, then apply the result with `apply_masterpi_trial_measurement.py`.
3. Fill the externally measured servo endpoint/settle fields, then run `fit_masterpi_servo.py --write-manifest`.
4. Fill hand-eye image observations plus independent floor x/y truth, then run `fit_masterpi_hand_eye.py --write-manifest`.
5. Run `fit_masterpi_dynamics.py ... --write-manifest` on the completed chassis dataset. It fits running and braking dynamics but never validates the twin.
6. Record physical outcomes for the 12 task fit cases; run `fit_masterpi_gripper.py --write-manifest`.
7. Record the 20 reserved task hold-outs; run `evaluate_masterpi_task_parity.py --split holdout --write-manifest`.
8. Run `validate_masterpi_digital_twin.py`. Only when every physical evidence count and held-out error threshold passes may `--promote` set `validated=true`.

After successful promotion, `MasterPiProductionV2.state()` reports `physics_fidelity=V2_REAL_CALIBRATED_VALIDATED` and `training_ready=true`. Before that it is intentionally not a completed calibrated twin.

## Safety

Calibration runners do not chain unattended physical trials. The chassis must be returned to the externally defined start reference between trials. Servo/hand-eye/task measurements require a physically safe setup and external ground truth; they are not auto-actuated merely because a plan exists.
