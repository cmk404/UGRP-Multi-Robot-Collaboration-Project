# Three-robot dispatch E2E pilot

## Scope

Connect the new research arena's live common plan to actual robot commands.
Each robot first issues the same isolated 0.6-second identification drive. Its
own before/after OWN and TOP frames and fallible image-motion anchor enter its
planning request. The agents negotiate cargo assignment, common dock, route and
job dependency. No robot ID is mapped to a ground-truth spawn by the controller.

Robot-local RGB actors then attempt APPROACH, GRASP, LIFT, TRANSIT, LOWER and
RELEASE. This is an experimental raw-action adapter, not a claimed transfer of
the trained fixed-lane skills. No fixed successful action sequence is supplied.
Approach can overlap; full-job dependencies gate GRASP onward. Pair stages need
fresh image-based readiness from the assigned participants. Shared transit and
apron permissions remain held through release, and remain held after revocation.

Actual model requests and response bodies are saved without secret headers.
Allowed input: own RGB, common fixed TOP RGB, authored static map/calibration,
self-issued command history, peer claims and protocol permission feedback.
No live positions, measured joints, contact or evaluator verdicts enter actors
or stage transitions. Cameras, robot/cargo appearance and weld OFF are preserved.

## First diagnostic

- shared_crossing, seed 11; same scene as the environment planning cohort.
- 3 independent Gemini 3.8 Flash clients, 60-second request timeout.
- At most 8 planning rounds, 24 local-decision rounds, 1200 seconds wall time,
  500,000 reported input tokens; token budget checked between batches.
- APPROACH drive intents last at most 1 second; joint commands and arm/look
  interpolation have 0.2-second leases. No renewal without a new model reply.
- Stop after 4 consecutive rounds without a non-wait action, a blocked/replan
  claim, completion, or a budget limit. Retain every diagnostic including errors.
- No automatic plan replacement while carrying or occupying a resource.
- SIM pauses during parallel inference. Per-job permissions are independent,
  but real-time asynchronous distributed latency is outside this pilot.

## Verdict

The output-only referee samples at 10 Hz and evaluates both cargo separately:
lift at least 15 mm from the settled initial height without floor contact;
all geometry corners inside the selected slot; supported on the floor and no
robot contact; stable within 5 mm over at least 1 second. Both must pass, weld
must remain OFF. Protocol completion and physical success are separate fields.
The video includes all three robots in the unchanged observer view. Identification
probe motion is explicitly labeled and is not counted as cargo work.

Runtime source must be committed before each trial and unchanged while running.
Results and raw hashes will be added after verification. Raw media/logs stay in
local ignored outputs; no Drive upload or remote raw-backup claim.
