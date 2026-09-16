# Visible-goal E2E local execution curriculum

User request: make E2E transport physically succeed. Prior raw-LLM runs at
`e706f15` never reached PREPARE READY. This candidate tests the missing local
execution, with an explicit scope narrower than adaptive LLM collaboration.

Start: r1/r3 60 cm behind the demonstrated grasp station, folded arms. Both
robots must drive physically to the beam. No runtime base relocation, IK,
joint measurements, contact feedback, or output-only referee feedback.
Weld OFF; fixed camera placement/FOV, robot/payload geometry and material.
Rendering 960x720 matches the previously trained local skills (raw LLM pilot
used 640x480). Camera pose/FOV are unchanged.

Candidate: image-based coarse approach -> existing learned near alignment ->
demonstrated arm deployment with RGB recovery -> demonstrated close/lift ->
RGB goal servoing with the existing paired carry barrier -> demonstrated
lower/open/retract. Teacher grasp-station initialization is not applied to
the runtime robot bases. Arm playback remains labeled as playback.

Only the final output evaluator reads physical truth. Require all existing
transport gates at 50 +/-3 cm, continuous sampled bilateral loaded carry,
2 s grasp hold, ground support and separation after release, no approach
payload collisions, no welds, immutable geometry/cameras, stable green-goal
placement. False stage completions cannot substitute for those gates.

Development starts: (0.60,0.60) m. Record every failed candidate and its SHA.
After a development success, freeze final code and validate starts
(0.60,0.60), (0.55,0.63), (0.65,0.57) m, plus a stationary-image replay check
and fresh-clone/offline tests. These three physical cases are engineering
coverage, not a statistical generalization or communication-gain experiment.
No terrain claim. Raw RGB/videos/logs remain local under outputs/; records,
source, asset hashes, selected visual evidence go to Git. No Drive use.

## Added LLM skill interface after local development success

Local development attempts: `46ea4d7` stopped after 42 cm when the goal's
green pixels were occluded; `b1efa6e` remembered the goal but prolonged loaded
creeping ended with slip. `754a267` used the already demonstrated loaded
wheel range .04..10 and succeeded, 48.1858 cm final displacement. No physical
parameters or success thresholds changed. The remaining finite grasp hold
margin must not be described as solved slip recovery.

Next candidate adds two actual independent Gemini 3.8 Flash actors at each
declared skill boundary. Each sees own/TOP current and previous RGB, its own
issued command history, and peer visual claims. Both must choose the offered
skill with confidence >=.8; HOLD/schema failure blocks actuation. At most three
fresh visual attempts and three inference retries each. This is a constrained
skill-permission workflow with a fixed order, not free-form LLM planning or
raw-action autonomy. Paused SIM during inference is explicit.

Validate the three predeclared starts with local execution and with the LLM
interface on the same frozen source. Keep all attempts including denied
permissions. Reconstruct actual model wires and pure RGB wheel decisions from
saved inputs; evaluate physical success separately after all control ends.
