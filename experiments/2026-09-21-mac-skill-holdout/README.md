# Native Mac skill comparison

User authorized serial local simulation after the bounded native OpenGL probe.
This is a new Mac cohort; do not combine its denominator or timings with the
earlier GPU/Colab cohort. Existing actor, cameras, RGB mask, skill choices, model
prompts, limits and referee are unchanged.

- 12 fixed holdout cases × 3 resets × rule/Jev/Gemini = 108 prespecified trials.
- Frozen order from `run_jev_skill_cohort.plan('holdout')`, shuffle seed 210922.
- One isolated native `mjpython` process per trial, one at a time.
- Paused simulation during model calls; 90 simulation seconds, 1200 wall seconds,
  400 calls, 1,000,000 input tokens per trial. Process cleanup allowance 45 seconds.
- Whole cohort deadline: four hours. Unstarted trials stay pending; interrupted
  trial is an infrastructure failure. Do not imply that pending trials failed.
- No trial retries, selection or failure deletion. Transport errors and worker
  crashes are distinguished from task failures and do not cancel subsequent trials.
- Same layouts repeated with fresh model responses; these are not independent maps.
- Scope: RGB approach, route selection and recovery, not grasp/carry or physical robots.
- Own RGB, shared top RGB, own issued commands and authored static map only; weld off.

`scripts/run_mac_skill_cohort.py` requires a clean source commit and saves its SHA,
all enumerated jobs, environment and limits in `protocol.json` before simulation.
It verifies six transport/schema preflight requests, then uses direct API requests
through the existing local Gemini proxy and macOS Keychain credential helper.
Credentials travel only through anonymous stdin pipes, never arguments or records.
The cohort actor retains its original single HTTP attempt policy.

Raw output: `outputs/mac-skill-holdout-20260921`. Each completed attempt retains
RGB, requests, responses, output-only referee, video and an independently verified
immutable checkpoint ZIP. `status.json` records active trial and separate failure
counts. `report.json` appears when the finite cohort stops. Raw data are local,
not a remote backup. No Google Drive is used.

Verification: focused supervisor tests cover continuing after task failure and
worker crash, deadline pending counts, source changes, preserving malformed output,
forbidding overwrites and passing credentials without command-line exposure.
Native execution and final cohort results must be checked separately. No success
rate or completed experiment is claimed by this protocol declaration.
