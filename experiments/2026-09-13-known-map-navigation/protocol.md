# Authored map navigation: protocol

User authorization: on 2026-09-13 the user selected providing a premade map directly to robots. Dynamic simulator truth remains forbidden as runtime actor input. This is a new, unloaded navigation course; previous grasping camera placement/FOV, robot appearance and weld-OFF constraints remain.

## Questions and acceptance

1. Does map-aware route planning enable a collision-free detour when a direct-to-goal controller is blocked? Compare `map` and `direct` with identical RGB localization and actuator calibration.
2. Does footprint inflation refuse a 0.32 m opening narrower than the declared 0.44 m clearance diameter before issuing motion?
3. Do start-position and yaw changes work through new image observations, without a prerecorded movement sequence?

The v1 actor is a classical reference navigator: supplied static map -> top RGB localization -> image-displacement actuator calibration -> inflated A* -> bounded mecanum motion -> new RGB. Own RGB is retained and decoded but does not currently determine control. This experiment does not establish learned/LLM navigation, multi-robot coordination, loaded transport, dynamic obstacle handling, or traversable ramps/steps.

The physical boxes and actor map come from the same validated JSON and canonical SHA-256. Map schema v1 supports static axis-aligned nontraversable obstacles only. The 0.18 m unloaded radius includes the documented compact robot planar extent (about 0.16 m circumscribed radius); 0.04 m extra clearance addresses projection/control error. The arm is folded before the run; actor commands cannot alter it and request zero turn. The image feature plane at 0.09 m is a static approximation, not a measured runtime height.

Runtime actor inputs: immutable map/calibration, robot ID, two raw JPEGs per frame, sequential frame number, own previously issued commands. Private setup poses are sent only to the scene before the run. Ground truth contact/pose/weld/tilt is written by an output-only referee and cannot stop, correct, or advance the actor.

Success requires actor `arrived`, referee distance <= 0.18 m goal radius, no robot-map contact on any physics step, no active weld, sampled center inside map, tilt <10 degrees, and unchanged robot/camera invariants. Refusal is reported separately from arrival. Exact-input deterministic replay audits are separate from physical success. RGB loss, calibration failure, map refusal and budget exhaustion all remain failures or refusals in their own category, never silently removed.

## Development and held-out runs

First run smoke cases using r1, start [-0.50,-2.55], yaw 0 on open/slalom/narrow. Keep every version's failures. Commit source before each development version; do not change source during a run.

After development, freeze a final source SHA and run all paired conditions for these held-out cases: r1 at [-0.48,-2.50], yaw +12 degrees, and r3 at [-0.54,-2.59], yaw -12 degrees. Both cases run open and slalom maps under map and direct conditions (8 runs). Both run narrow with map (2 refusal checks). Each has the same 240 decision budget. No tuning using held-out outcomes; any further fix requires a newly labeled candidate and complete rerun of the final matrix.

Record all outcomes, source/map SHA, environment, exact actor RGB/actions/history, observer video, output-only referee, elapsed simulation/wall time, decision count and model cost (0: no model calls). Raw images/video stay locally under outputs; the experiment report must state precisely which artifacts, if any, are published remotely. Physically review representative motion and raw actor camera frames, including a failure.

Use `scripts/ugrp_session.py run <unique-name> -- ...` and clean up only owned children. Create a stacked PR on #34; do not merge main without explicit user approval.
