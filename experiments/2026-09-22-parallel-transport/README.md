# Open-map 2+1 transport restoration

Explicitly independent north/south routes can overlap pickup and loaded travel. Shared destination access remains exclusive: the solo carrier waits at an RGB-observed staging point while the beam occupies the apron. Original cameras, FOV, physics, calibration and model weights are retained; weld is off. Explicit task dependencies are never removed at runtime. This is a recorded-plan physical diagnostic, not fresh LLM negotiation. Cluttered moving-obstacle rotation remains excluded.

## Completed development references

| Source / revision | Controller | Whole task | Wall seconds | Strict sampled concurrent loaded motion |
|---|---|---:|---:|---:|
| 6baecba / v1 transit admission | RGB | success | 276.773 | 2.8 s |
| 6baecba / v1 transit admission | original ACT | failure: beam outside slot although old visual check claimed complete | 286.131 | 1.6 s |
| 6ff7f6f / v2 grasp admission | RGB | success | 347.316 | 4.8 s |
| 6ff7f6f / v2 grasp admission | original ACT | failure: conservative visual check rejected a physically placed beam | 251.540 | 4.0 s |
| db0a6a7 / v3 supported-contour verification | original ACT | success | 245.267 | 4.0 s |

Each completed run has zero reported obstacle-contact steps and zero sampled robot-to-robot contact frames. Sampling does not establish absence of contacts between samples. Concurrent duration requires both objects lifted and contacted, displacement of both objects and all three robots, and overlapping issued TRANSIT leases. Tiny lease gaps exclude entire sample intervals; the metric is a conservative subset of the overlapping stage duration. V1 original zero concurrency metrics are preserved, with hash-verified post-run corrections in separate audit files.

V1 failure exposed a trimmed tracking core being used as the full beam outline. V2's replacement used minimum-area rectangles and independently inflated eroded masks, introducing empty-corner false negatives. V3 bounds the union of actual RGB contour pixels, adds uncertainty only for missing union extent and one-pixel quantization, and retains the physically outside negative fixture. Four recorded boundary fixtures distinguish true and false completion without passing referee state to controls. Relevant regression tests: 44 passed; v3 complete physical workflow succeeded. These references are development diagnostics, not a population success-rate estimate. Timing across revisions is not a controlled speedup measurement.

`reference-results.json` records source, settings, all outcomes, raw locations and hashes. Raw data is local only. `dashboard-check.json` verifies immutable TensorBoard events and media Range requests. Native UI recheck was blocked by the locked Mac screen; it must not be inferred from event readback.

The finite parallel-data expansion and matched evaluation is recorded separately in `experiments/2026-09-22-parallel-act-study/`. Existing GPU-trained ACT versus a new Mac CPU-trained ACT can compare deployment results but cannot isolate data augmentation from training-backend differences.
