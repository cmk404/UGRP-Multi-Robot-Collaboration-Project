# Jev direct motion comparison — 2026-09-21

Issue #78. Single r2 robot approaches the existing cyan box in the open dispatch arena. Jev/Gemini choose actual 0.2-second mecanum commands from seven discrete candidates. No pretrained approach policy, grasp, carry, or new task negotiation runs. Rule, Gemini and Jev receive the same RGB-derived state and action set. This is a new direct-motion experiment, not a model-only replication of the earlier five-task failure.

## Frozen final protocol

- Final cases: `straight`, `left_offset`, `right_offset`; each policy once per case, 9 episodes. `dev` and `dev_yaw` are separate development poses. One fixed map, box position, three limited start poses; not broad generalization or statistical superiority.
- Model versions: `jev-1.13.0`, `gemini-3.8-flash`. Policies run rule → Gemini → Jev within each case. Fixed order and single repetitions limit wall-time comparisons.
- Shared setup: six 0.2s forward identity probes, stopped settling. Own identity/front inferred from RGB displacement; no initial true pose in policy input. Full wheel envelope estimates current heading; issued turns never update it. Each command has a 0.2s port lease and 0.05s stopped dwell.
- Shared observation: unchanged own RGB and fixed TOP RGB archived. TOP color/shape detects one cyan box and four-wheel envelope. Nominal camera/feature-plane calibration produces relative range/bearing. Own RGB contributes cyan pixel fraction only; this is classical perception plus a text-state policy, not Jev/Gemini raw image understanding. The observer has a limited ±18-degree wheel heading domain and holds on lost/ambiguous evidence.
- Vision goal: range 0.27–0.28m and bearing ±3 degrees, three fresh stopped confirmations. This fixed common completion gate supplies no action direction; models choose all other post-probe motion. Model confidence is recorded, not interpreted as physical safety or used as an actuation threshold. Thus the previous raw pilot's 0.8 confidence filter is not used in any of these matched arms.
- Output-only physical success: terminal stable tail ≥0.35s within 0.26–0.30m and ±6 degrees; RGB completion claim; no cargo/obstacle/peer contacts, weld use or camera/geometry changes. Approach alignment does not certify a grasp-ready arm pose. Evaluator never supplies policy observations, commands or termination.
- Inference pauses SIM, including API latency. No real-time hardware/distributed-control claim. Local command expiry remains active for each SIM step. 70 post-probe steps, 180k input tokens (5k reserved before each call), 360 wall seconds per episode; request timeout 30s, no automatic retries. Raw provider body/request and model version saved. HTTP errors or invalid responses stop that episode and preserve failure; later independent episodes still run.
- Jev's live serialized probabilities can sum to 0.99/1.01. Validator allows only the accumulated half-unit rounding error for seven two-decimal values, while checking bounds, candidate set and maximal choice; raw values stay unchanged.
- Development failures are preserved separately: optical-flow coverage, wheel-corner support, premature RGB completion and weak probe displacement. No final pose is used for tuning. Development API attempt stopped on rounded probabilities; Gemini development completed. Source is committed before each run and fixed throughout each cohort.

## Run

Use the existing simulation environment. From this checkout:

```sh
python scripts/ugrp_session.py run jev-direct-motion -- /absolute/path/to/mjpython scripts/run_jev_motion.py --execute --policies rule gemini jev --cases straight left_offset right_offset --prompt-key --output /new/local/output
```

Key comes from hidden input or TYPESAFE_API_KEY; it is never saved. Requests go to the official TypeSafe endpoint with redirects disabled. Raw evidence is local under `/Users/changmin/projects/ugrp/outputs/jev-direct-motion-20260921-*`; Git records are not a video backup. Main is not merged without user approval.

API: https://docs.typesafe.ai/api and https://docs.typesafe.ai/primitives/choice (2026-09-21).
