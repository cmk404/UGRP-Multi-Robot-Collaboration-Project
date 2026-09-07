# MasterPi SIM physics fidelity and calibration

Status: **platform/demo work, NOT an approved research result**.

## Why training is currently blocked

The deployed `MasterPiPhysicsWorld` is useful for behavior/UI regression, but it is not a valid sim-to-real dynamics source. It directly actuates chassis `x/y/yaw`, has decorative non-contact wheels, disables most robot/workcell collisions, compiles to the wrong robot mass, and does not match the physical arm/camera calibration. The historical PPO world is even less transferable (mocap base, 150 mm block, different arm topology, hidden simulator-state observations).

Run the gate at any time:

```bash
.venv-sim/bin/python scripts/benchmarks/physics_fidelity.py
```

A nonzero exit is intentional while training is unsafe.

## Candidate v2

`sim/masterpi_dynamics_v2.py` is a separate candidate physical core. It is not deployed to the production Lightning worker yet.

Structural properties already enforced by tests:

- free 6-DoF chassis (z/roll/pitch are real state);
- four rotating wheel joints with wheel/floor contact;
- ABAB reduced mecanum traction interface driven by the same four logical motor commands as the real controller;
- total robot mass = 1.10 kg;
- 4-DOF+gripper arm geometry built from the current physical pickup FK (`9.3 / 6.5 / 6.2 cm`, 10 cm gripper tool length);
- PWM-to-joint mapping numerically matches the REAL FK at search/grasp/observe poses;
- robot camera VFOV = 48 degrees, matching the current physical geometry calibration;
- simplified robot + solid workcell collision geometry enabled;
- no direct world-frame `x/y/yaw` chassis actuators.

The mecanum model is deliberately described as **reduced dynamics**, not exact roller geometry. Each 45-degree passive roller is not modeled as a separate rigid body; its average traction/slip effect is represented by an ABAB motor-to-body-wrench model plus wheel/floor contacts. This is appropriate only after those parameters are fitted to the physical robot.

## What remains uncalibrated

The authoritative manifest is:

`sim/masterpi_dynamics_calibration.json`

It defaults to `validated: false`. Do not flip that flag manually.

Values requiring physical fit/validation include:

- motor response time constant;
- forward and lateral traction force scales;
- yaw torque scale;
- linear and yaw damping / coast behavior;
- mecanum forward/lateral slip ratio;
- wheel radius, wheelbase and track if CAD/physical measurement becomes available;
- servo deadband, endpoint error and movement/settle time;
- gripper contact force and grasp success behavior;
- block mass and floor friction;
- mass/inertia distribution (the 1.10 kg total is fixed, distribution is provisional);
- camera extrinsics beyond the current FK-derived optical-axis model.

## Physical chassis trial format

The chassis fitter consumes JSONL. A row looks like:

```json
{"split":"fit","direction":"forward","command":20,"drive_s":0.5,"coast_s":0.5,"dx_m":0.12,"dy_m":0.00,"dyaw_deg":0.4,"peak_speed_mps":0.31,"stop_distance_m":0.025}
```

Directions are `forward`, `backward`, `left`, `right`, `rotate-left`, `rotate-right`. `command` is the same `1..40` range used by `scripts/masterpi_control.py`.

Fit without changing the manifest:

```bash
.venv-sim/bin/python scripts/benchmarks/fit_masterpi_dynamics.py measurements.jsonl
```

Write fitted chassis parameters/results into the manifest (still leaves `validated=false`):

```bash
.venv-sim/bin/python scripts/benchmarks/fit_masterpi_dynamics.py measurements.jsonl --write-manifest
```

The fitter uses only measured files and never drives the real robot.

## Calibration experiment design

Use at least 5 repeats per command and split trials before fitting. The current manifest requests speed levels 10/20/30/40 for forward, backward, left, right, rotate-left and rotate-right. Keep at least 20 genuinely held-out trials across directions and speeds.

For every drive trial record:

1. command direction and command magnitude;
2. exact drive duration;
3. coast/stop observation duration;
4. world-frame endpoint displacement (`dx`, `dy`);
5. yaw change;
6. preferably peak translation speed;
7. preferably distance traveled after the stop command.

Endpoint/yaw must come from an external or otherwise independently calibrated reference. Commanded motor state is not ground truth. The MasterPi base controller does not provide a trustworthy chassis pose simply because a motor command was sent.

For servos, separately measure several PWM step sizes in both directions, endpoint angle and settle time. For grasping, run repeated controlled contact trials with fixed block geometry/surface and compare success/failure and release behavior.

## Acceptance gate

The manifest currently requires, on held-out physical trials:

- translation endpoint MAE ≤ 2.5 cm;
- yaw endpoint MAE ≤ 5 degrees;
- peak-speed relative error ≤ 15%;
- stop-distance MAE ≤ 2 cm;
- servo endpoint MAE ≤ 3 degrees;
- servo settle-time relative error ≤ 20%;
- SIM-vs-REAL grasp success-rate gap ≤ 15 percentage points.

These are engineering acceptance thresholds for this platform, not research claims. Tighten them if the downstream policy proves sensitive.

## Policy observation/action boundary

Even after dynamics calibration, the policy must not receive privileged simulator state that the physical robot cannot observe.

Allowed policy inputs should be limited to transferable signals such as camera images/features, commanded servo PWM/pose state, and actual hardware telemetry that exists on MasterPi. MuJoCo block xyz, block velocity and contact forces may be used internally for reward/evaluation, but not as policy observations unless the physical robot has an equivalent sensor/estimator.

Policy actions should use the real command domain: four chassis motor commands and physical servo/PWM targets/rates. Do not train a policy on direct world-frame base translation or simulator-only joint forces and expect transfer.

## Promotion rule

Do not replace the current Lightning production simulator with v2 until all of the following are true:

1. v2 structural tests pass;
2. physical calibration dataset is collected;
3. fit and held-out acceptance are repeatable;
4. calibration manifest is validated by the calibration workflow, not manual editing;
5. existing seeded task and camera/UI regression suites pass against v2;
6. v2 is first enabled behind a feature flag and compared against the existing production service.

Only after promotion should a new training environment be allowed to initialize without an explicit fidelity override.
