# Existing skills vs Show-Harness-style semantic steps: pilot

Status: preregistered pilot, implementation under test; no outcome claim here.
User request: run a head-to-head after discussing existing action-selecting control.

## Comparison

- **skill**: a VLM selects `approach`, `pick`, `check_grip`, `wait`. Existing
  `LLMTransportSkill` / `VisualBoxSkill` execute the selected skill. Their camera
  geometry and own-issued-command kinematics are preserved.
- **semantic**: same VLM selects 1 cm end-effector translations, 5 degree pitch
  changes, grip open/close, short base motions, or wait. A static robot-specific
  interpreter uses only its own issued command ledger and calibrated arm geometry.
  It never reads measured joints or task coordinates. Unreachable commands are
  rejected without substituting a different movement.
- This is a **systems pilot of a MasterPi adaptation**, not reproduction of the
  complete Show-Harness implementation or published score. No LoRA, tactile/
  proprioception plugin, state-derived recovery, automatic action chunks, or
  scene-derived target is provided to the semantic interpreter.
- The previous proposed 14 servo-button experiment is not used.

## Matched environment and input

- Existing `camera_team` world, only r1 active, cargo `small_box_01`; seed controls
  the existing arena's placements. Three untouched evaluation seeds: 41, 42, 43.
  Development capture/grounding uses seed 40. No policy tuning on evaluation seeds.
- Two planner images: own `robot_cam` and existing `cctv_top`, unchanged camera
  positions/quaternions/FOV/appearance. No `nav_cam` enters control. The former
  Gemini planner's larger state/navigation prompt is replaced by a matched RGB
  planner in both conditions. This is disclosed adaptation of the baseline.
- Only fixed task/hardware instructions and last four own issued model actions
  supplement RGB. Initial commands, encoder/state fields, skill state, contacts,
  inferred target coordinates, simulator/evaluator feedback are not model inputs.
- The coarse skill privately derives geometry from its own RGB and issued
  commands; this is part of the system being compared, not an extra truth input.
- Same `VisualMacroExecutor`, interpolation and settle phase label `match` on
  both sides. The nav-camera drive guard is absent on both sides; drive duration
  is bounded to <=1 second. This differs from the original transport runner.
- `impratio=10`, `noslip_iterations=0`, all warehouse welds OFF from initialization.
  The runner never calls a grasp-attachment API. Geometry, camera and initial-state
  hashes are recorded for equality checks within each seed.

## Budget and run order

- Model: requested `gemini-3.8-flash`, temperature .2, reasoning `none`, output
  maximum 650 tokens, HTTP timeout 40 s, zero automatic retries. Record returned
  model identity and usage; missing billing information remains unknown.
- Maximum 30 logical model calls per episode and 120 simulated seconds. Stop
  issuing model calls at 180,000 reported input tokens (reported-use stop is not
  a hard provider cap; one final request may overshoot). Maximum 180 logical
  requests for the six evaluation episodes. Development calls, if any, separate.
- Paused physics during synchronous model inference in both conditions. Record
  wall time and model latency separately. Videos show SIM time, not wall latency.
- Order: skill-41, semantic-41, semantic-42, skill-42, skill-43, semantic-43.
  Fixed source commit for all six; record all trials, including failures.
- Completion of a coarse skill may stop further decisions according to its RGB
  controller; failed skills and exhausted budgets also stop issuing decisions.
  Physics continues to fixed 120 s so hold and drop are observed without
  referee-driven stopping. Semantic policy retains its normal wait action.
- Invalid JSON/schema terminates decisions and counts as a policy error. Invalid
  state action or unreachable command issues a fixed wait on either side, logged
  as rejection; no rejection description or state is returned to the model.
- Infrastructure failure is reported separately, with no automatic replacement.
  Any incomplete paired episode prevents a clean outcome comparison for that seed.

## Evaluation

- Physics-only endpoint: box >=4 cm above its initial height, bilateral gripper
  contact, no assistance, sustained for >=2 s in contiguous 0.1 s samples. Initial
  and continuous truth is written only to the referee log. No transport/cooperation
  success is claimed from this single-robot test.
- Inspect every apparent success in actual own/top videos to exclude external
  support or other misleading contact. If evidence is insufficient, leave outcome
  unresolved rather than claiming success. Review failures as well.
- Report per-seed lift/hold, success, calls/tokens, wall and SIM time, command
  rejections, parser/infra errors, and constraints. No statistical superiority
  claim from only three scenes; these are diagnostic outcomes, not benchmark rates.
- Verify wire bodies against saved exact images and actor's own action history.
  Verify within-seed fixture hash equality and no camera changes after initialization.
- Preserve logs/videos locally in outputs, version source/protocol/result summary
  and useful compact evidence in experiments. No Google Drive. Publish code/results
  through a PR; main merge still requires explicit user approval.

## Sources

- https://arxiv.org/html/2609.10522v1 (semantic interpreter §3.2; state plugin
  §3.3; limitations and coordination experiments)
- https://github.com/showlab/Show-Harness (official architecture and embodiments)
- Existing baseline `harness/llm_transport_skill.py`, `harness/visual_box_skill.py`,
  `scripts/probe_visual_acquisition.py`, `scripts/evaluate_gemini_team.py`.
