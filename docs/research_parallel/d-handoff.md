# D handoff — R4/R6 pilot manifest and offline evaluation

Status: implementation prepared on `codex/r4-r6-rgb-communication-evaluation`; no simulator, model, remote job, or physical robot was run. Base main is `120cc821b6a1d5c104aab8c8ef2260cf7f8c9a7b`. Source/test implementation commit is `d8fa21b2280f9d38716c5e0e975084002a0e7937`; B-snapshot compatibility hardening is in `632a7415c30eecc0147396aca2364cda4a494145`.

## Scope and files

- `harness/rgb_communication_evaluation.py`: finite six-run manifest, execution admission, event/evaluator validation, raw-row and aggregate evaluation.
- `scripts/evaluate_rgb_communication.py`: offline `prepare`, `validate`, `schedule`, `admit`, and `evaluate` commands. It contains no execution or job-submission path.
- `tests/fixtures/rgb_communication_evaluation/`: provisional protocol and synthetic outcome matrix only.
- `tests/test_rgb_communication_evaluation.py`: success, failure, timeout, API error, aborted, unrun, missing artifact, invalid artifact, denominator, metric, and no-go tests.

The implementation does not modify A's protocol/audit files, B's execution port, C's runtime, the TensorBoard exporter/viewer, or CI lists.

## Frozen provisional pilot interface

The manifest contains one randomized finite block: two scenarios × `none`/`structured`/`natural` = six planned runs. Each row records order, run/scenario/condition IDs, seed, repeat, and required artifact paths. Duplicate rows or an incomplete 2×3 block are rejected.

Current candidate scenarios are:

1. `normal-supported-mission`: open seed-11 beam-one + box-one dispatch to `dock_a`, subject to current RGB runtime verification.
2. `local-visual-misalignment-recovery`: candidate/no-go. Existing saved images do not establish a designated private observer. Admission requires a fixed setup hash and saved allowed inputs that identify the actual observer set. If no information difference exists, it must be labeled a public-recovery case rather than private information.

The provisional per-run caps are 900 wall seconds, 180 SIM seconds, 72 model calls, 600,000 input tokens, 55,296 output tokens, 6,000 issued-command records, 72 messages, and 216,000 message bytes. The six-run cohort caps are 6,900 remote-job wall seconds including setup/recovery reserve, 432 model calls, 3,931,776 model tokens, and 36,000 issued-command records. The message-token cap is intentionally unresolved, so execution remains blocked.

The manifest separately records model, physics, map, calibration, and integrated source hashes. Current hashes are unresolved. `weld=false` and no camera/FOV change are explicit mission fields.

## Runtime and evaluator contracts

C and D selected runtime schema `rgb-communication-event.v1`. Every JSONL event requires:

`schema_version`, `event_id`, `run_id`, `condition`, `robot_id`, `event_type`, `sim_time_s`, `wall_time_s`, `related_ids`, and `payload`.

`event_id` must be unique within a run, `robot_id` is `r1`/`r2`/`r3` or null, wall time is non-decreasing in JSONL order, SIM time may be null, and `related_ids` carries observation/status/request/decision/message/action correlations without evaluator truth. There must be exactly one `run_started` and one `run_finished`. The terminal outcome is one of `success`, `failure`, `timeout`, `api_error`, or `aborted`.

Runtime `action_submitted` counts high-level actions, not actual commands. `planner_requested` counts all model-call attempts, including calls without a response; `planner_responded` carries optional tokens/latency, and `message_sent` carries communication bytes/tokens. `run_started.payload.measured_metrics` distinguishes an actual zero from an unmeasured value. A declared metric with only partial provider usage remains `partially_measured`/null rather than becoming zero; an invalid value still invalidates the artifact.

B's detached `evaluation_snapshot()` is stored without actor access under `evaluator.json.source_snapshot`. D requires snapshot schema `ugrp.rgb_evaluation_snapshot.v1`, the exact evaluation-only boundary declaration, empty active/pending task lists from a post-close snapshot, and final `external_evaluation` fields for `mission_complete`, the manifest's exact object IDs with per-object stage booleans, and recovery required/reached/succeeded. A SIM-clock snapshot may not predate the runtime terminal event. Runtime success must agree with independent `mission_complete`; disagreement is `invalid_artifact`. Actual issued-command count comes from unique, provenance-complete `LOCAL_COMMAND.command_id` rows in B's `coordination_audit`. Those rows prove command issuance, not movement or physical success.

The evaluator wrapper matches `run_id` and condition to the schedule. A missing run directory is `unrun`; a present but incomplete directory is `missing_artifact`; malformed, duplicated, mismatched, or contradictory evidence is `invalid_artifact`. None can silently become success.

## Analysis boundary

All six planned runs remain in the mission-completion denominator. Per-object stage and recovery counts report both evaluator-measured and planned denominators. Command/message/call/token/latency cost summaries include all outcomes with measured/unmeasured counts. Failure durations remain in raw rows but are excluded from the explicitly labeled successful-run-only wall/SIM speed summaries, so a quick failure cannot appear as a speed benefit.

Raw rows are grouped by scenario/seed/repeat into matched condition triplets for later paired analysis. This one-repeat connection pilot has no uncertainty or superiority claim; the report marks uncertainty analysis as not applicable and preserves the rows needed for predeclared repeated blocks later.

Synthetic fixtures are tests, not research results. They must not be added to the shared TensorBoard.

## Go/no-go and next commands

Create a new manifest from the committed integrated source:

```sh
python3 scripts/evaluate_rgb_communication.py prepare \
  --protocol tests/fixtures/rgb_communication_evaluation/pilot_protocol.json \
  --output /new/path/manifest.json
python3 scripts/evaluate_rgb_communication.py admit --manifest /new/path/manifest.json
```

`admit` exits 3 while any blocker remains. A future single named submitter may obtain the ordered schedule only after every blocker is resolved:

```sh
python3 scripts/evaluate_rgb_communication.py schedule \
  --manifest /fixed/path/manifest.json --require-ready
```

Required gates are R0 design approval, R1 information-boundary audit, R2 common-runtime verification, R3 three-condition integration, event-schema agreement, a single named submitter, a clean committed source SHA, four fixed-input SHA-256 values, and every finite per-run/cohort budget. This PR intentionally has no runner; B/C integration must supply the one-run command before any schedule consumer or Colab submission is added.

After artifacts are recovered, offline evaluation is:

```sh
python3 scripts/evaluate_rgb_communication.py evaluate \
  --manifest /fixed/path/manifest.json \
  --artifacts /recovered/pilot-root \
  --output /new/path/evaluation-report.json
```

## TensorBoard handoff

The current exporter accepts one completed source directory with `result.json`. It can display generic success, wall/SIM time, command/call/token scalars, but it does not consume this cohort report, runtime JSONL, evaluator snapshot, run ID/condition, message metrics, or paired rows. Before real pilot results are published, the TensorBoard owner should add a small communication-run adapter that:

1. reads each manifest-listed runtime/evaluator pair and verifies the same hashes/contracts as this evaluator;
2. exports one HParams run per planned trial with run ID, condition, scenario, source/setup hashes, terminal outcome, and measured/unmeasured provenance;
3. adds messages/bytes/message tokens/model latency while retaining the current success and command/call/token tags;
4. records unrun/missing/invalid states without manufacturing zero durations or success;
5. checks the existing snapshot manifest for duplicate source hashes before creating a new snapshot.

Only newly recovered real results should be exported, together with the needed matched baseline. Then verify actual TensorBoard loading, fixed metrics/links, HParams columns from `outputs/tensorboard-view.json`, and the visible dashboard. No dashboard was started here because there is no new real result.

## Verification and remaining integration

Local source-only verification at the time of handoff:

- `tests/test_rgb_communication_evaluation.py`: 8 passed.
- Existing offline regression runner: 1,173 passed, 3 skipped, 184 subtests passed.
- C commit `97a86d3c60b6e1a55c01f96a14029cfbe75c7619` exact six-event `tests/fixtures/rgb_communication/trace.jsonl` (SHA-256 `1d91f754ba7be21339d70247530dd9f48f40eeff0c2a0c8e52579ac37d489927`) was consumed directly by D: terminal `aborted`; high-level actions/messages/model calls each 1; message bytes 189; token fields stayed unmeasured. C's own runtime suite passed 9 tests when run in C's worktree.
- B PR #95 branch HEAD `da3f192816610ad6bfb56550bba58f5c0310866e` exact post-close `evaluation_snapshot()` was wrapped without alteration and accepted by D: snapshot schema/boundary matched, active/pending lists were empty, and zero `LOCAL_COMMAND` rows remained a measured command count of zero.
- Python compilation: passed.
- `git diff --check`: passed.
- CLI prepare/validate: produced a six-run manifest and listed blockers.
- CLI `schedule --require-ready`: blocked as designed.
- CLI `admit`: exited 3 with `go=false` as designed.

CI ownership remains separate. `scripts/run_ci_tests.py` must add `tests/test_rgb_communication_evaluation.py` after review; no existing CI file was edited in this branch. C's committed exact trace and B PR #95's exact detached snapshot were consumed read-only as recorded above. The final integrated branch must keep those producer SHAs or rerun compatibility against their replacements. These compatibility checks do not authorize live execution.

Latest implementation code commit: `632a7415c30eecc0147396aca2364cda4a494145` (initial source/test commit `d8fa21b2280f9d38716c5e0e975084002a0e7937`).
