# Varied-start development evidence record

- Evidence date: 2026-09-10
- Final development source SHA: `b4c9535d24efc6747ed4f6df244b902976efb00f`
- Scope: completed axes, teacher, calibration, training, development, and grasp-tolerance evidence. Heldout and retention outputs were intentionally not inspected or archived.
- Auxiliary grasp constraint: OFF for these physical tests, as recorded by the source reports.

## Result progression

- Axis characterization completed for four trials: forward, left, turn, and r1_forward.
- Teacher smoke v1 failed with `RuntimeError: yaw teacher budget exhausted`; smoke v2 passed. Teacher pilot v1 completed all six cases and replayed every grasp.
- Calibration completed 483/483 static reset-state cases. These are nonphysical calibration samples and are not execution successes.
- The merged training set contains 4,760 actor rows and 4,760 teacher-label rows.
- Development v1 passed 1/8 at source `5ae9589548a1c178a0c143d8e8f502e8b121926d`.
- Development v2 passed 6/8 at source `81793ec1f2f19fd3663e2e461af22ab7fbd70509`.
- Development v3 passed 8/8 across two four-case shards at source `b4c9535d24efc6747ed4f6df244b902976efb00f`.
- The tolerance probe passed audit for 8/8 cases and contains 256 exact command replays. Its source SHA is `d45d3d04a86b9fc4648f9b7165fb10df9691b763`.

## Model metadata correction

The v2 training report's selected-sample metadata says 600 samples per robot/stage despite the merged input containing all 4,760 rows. V3 regenerated this metadata from the full merged input. `outputs/varied-start-model-v2-v3-comparison.json` verifies that the six fitted geometry payloads are identical between v2 and v3; v3 adds corrected selection metadata and `moving_target_fraction` settings.

## Representative raw evidence

The archive keeps all RGB frames and execution trace for development case 03 in v2 (failure) and v3 shard 0 (success). It excludes MP4 files and generally excludes other large execution traces. Reports, case inputs, results, audits, evaluation JSONL, model metadata, and training metadata are retained.
