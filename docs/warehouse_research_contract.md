# Warehouse peer-decision experiment

2026-09-05. The user authorized fixing the three research-validity gaps identified
in the current warehouse implementation. This is an executable experiment on the
existing three-MasterPi MuJoCo platform, not approval of the separate Isaac Lab
proposal or a claim of physical robot calibration.

## Boundary

Each robot chooses its own cargo, endpoint role, destination and peer message
using only its local sensor observation and, in the communication condition, its
inbox. A coordinator delivers messages and runs a barrier; it cannot fill in,
optimize, repair or override the robots' decisions. An incompatible or absent
decision is a failed negotiation round. A new goal/failure starts a new decision
round; stale decisions cannot actuate.

The same atomic contact/transport/release controller and MuJoCo referee are used
in every condition. These remain simulation-only skills with ideal internal
feedback; this experiment measures high-level allocation/communication, not
sensor-based motor control, calibrated dynamics or SIM-to-REAL transfer.

Actor observations use decoded ArUco DICT_4X4_50 markers (11 plank, 12 pipe,
13 crate) and metric depth from each robot's own rendered RGB-D camera. The
10cm visual marker plates have no mass or collision. A bounded physical servo
pan sweep (1500, 1300, 1700, 1100, 1900) restores SEARCH_POSE afterward.
This is a tagged-cargo experiment, not marker-free arbitrary-object recognition.
Depth has reproducible 2mm Gaussian sensor noise. Missing IDs stay missing.
Ground-truth mass,
global robot poses, scene geometry and referee state are not actor observations.
Simulated depth is a sensor assumption, not a claim that the physical MasterPi
currently has a depth camera.

## Conditions and outcomes

- `rule`: robot-local deterministic policy on the same observations/action API.
- `llm_no_comm`: independent LLMs; no peer messages or peer decisions in inputs.
- `llm_peer_comm`: independent LLMs with delivered natural-language messages.
- The old omniscient central warehouse controller is an explicitly named
  `central_baseline`; it is not one of the matched sensor-limited conditions.

Every condition uses the same seed list, initial state, sensor profile, mission,
event schedule, action budget and per-robot round/output budget. Log all attempts,
including invalid output, missing measurements, timeout and physical failures.
Store raw replies, the exact actor input, parsed decisions, message deliveries,
execution evidence, model identity, elapsed time and token usage availability.
Unknown token usage is null, not zero. Scripted completers are labelled fixtures
and cannot be aggregated as live LLM evidence.

No success advantage is assumed. A negative result is a valid experiment result.

## Executable task specification

The operator chooses A→B or A→C, all cargo or one named type. The manifest
contains task identities/colors, never exact geometry. The actor chooses its own
role plus a proposed complete roster for the next cargo. The action repertoire
in this allocation experiment is bounded local head scanning, communication,
and one atomic cooperative cargo transfer; it does not yet include autonomous
map exploration. Each robot retains its own previously decoded identities within
the episode. Remembered entries have `current_view=false` and no old camera-relative
range/bearing; they cannot masquerade as a current measurement. Memory is never
shared with a no-comm peer, and reset/external actions invalidate the episode.

`normal` starts all robots with working grippers. `gripper_failure` declares r1's
gripper unavailable to r1 only before negotiation: r1 may scout but cannot be a
carrier. This is a simulated capability fault, not physical hardware damage.
`goal_revision` changes destination after the first completed cargo and requires
new decisions for the revised goal. It is a between-actions revision, not proof
of arbitrary mid-grasp replanning.

Success requires every selected cargo to pass the common delivery/stability
referee. The existing fixture tolerances remain the same for all conditions:
position error ≤0.22m, symmetric yaw error ≤25°, height error ≤0.025m,
linear velocity ≤0.01m/s and angular velocity ≤0.03rad/s. These are engineering
pilot tolerances, not calibrated real-world performance specifications.
Default limits: 12 rounds, 6 physical attempts, 512 generated tokens per call,
60s decision-batch timeout. Unavailable token usage stays null. Rule calls use
zero LLM tokens. No-comm receives generic action/referee success/failure and the
operator's remaining task manifest; it receives no peer messages, roles or
private fault details. LLM peer inboxes contain sender ID and natural language
only. Typed measurement receipts are kept outside actor inputs for validation.

## Run and inspect

```sh
.venv-sim/bin/python -m scripts.evaluate_warehouse_research \
  --seeds 11,12 --scenarios normal,gripper_failure \
  --record-first-peer --output outputs/warehouse_research/my-pilot
```

This uses the already configured local Gemini proxy and creates three separate
completers per episode. `--conditions rule` performs no LLM calls. The output
directory must be new: prior evidence is never overwritten. Each episode uses
a fresh MuJoCo world and records a source hash, model, sensor/controller profile,
actor inputs, raw replies, messages, actions and failures. The recorder camera
is for the reviewer only. Rule-vs-LLM comparisons include policy differences;
the peer-vs-no-comm comparison isolates the addition of the specified message
channel under this fixed tagged-cargo task and common skill controller.
Runs with video recording are excluded from comparative elapsed-time averages.

The early `pilot-20260905` directory is explicitly invalidated in its
`disposition.json`: its colour/backing heuristic confused yellow robot parts
with a crate. It is development evidence only. Experiments with different
sensor profiles/source hashes must not be pooled.

TEAM uses the same request/runtime boundary through `/api/warehouse/decision`
on each robot backend. `UGRP_WAREHOUSE_MODE=central_baseline` explicitly selects
the old omniscient demonstration. Startup now accepts `UGRP_PYTHON` so a system
Python older than 3.10 cannot accidentally launch the new harness.
