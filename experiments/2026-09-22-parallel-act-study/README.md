# Parallel visual-distribution expansion

The eight original train/development demonstrations all have zero overlap between the beam and box TRANSIT windows. The corrected parallel scene changes the model-visible background. This study holds the 128px/history4 architecture, training seed and 8000-update budget fixed and expands demonstrations instead.

Finite sequence: two new RGB demonstrations, combine the successful pickup-overlap reference and original data with whole-episode train/development separation, train one ACT, and run all nine paired physical evaluations. Test configurations are excluded from the added demonstrations. Every success/failure is retained; no action correction uses teacher or referee information. Results remain local and completed trials are exported to the common TensorBoard.

The saved baseline was trained on a Tesla T4 (PyTorch 2.11 CUDA); the expanded model trains on Mac CPU. Architecture, seed, step budget and deployment cases are controlled, but this is a practical existing-versus-expanded checkpoint comparison, not a causal data-only ablation. Three held-out spawn perturbations in the open map are not evidence of broad unseen-map generalization.

Completed additions: train-plus and development-zero both physically succeeded (0 failures / 2 teacher trials), with strict sampled concurrent loaded motion of 6.0s and 5.8s, respectively. Sampled video review shows concurrent carry and final placement. Dataset admission and image hashes passed; 8 train and 3 development episodes were admitted. Training under db0a6a7 completed all 8000 updates in 886.58s, selected step 7500 using development data, and passed native/cache plus saved-model readback checks.

Evaluation was initially blocked by the 8GiB disk reserve, before any physical trial was attempted. After project cleanup, the resumed preflight found 106.36GiB free and verified all 22 protocol assets. The existing trained checkpoint was reused. The historical blocked `study.json` and `training-complete.json` remain unchanged; `evaluation-continuation.json` beside the raw study records the completed continuation.

## Completed matched evaluation

All nine planned trials ran under frozen source `db0a6a70e29bd1bea021b86036a029d1a5aac813` and protocol SHA-256 `305cbb1535926e51a14ded4726df1edaee7440e09ae291170f83a8b8fe2152e0`. Ordinary failures did not stop subsequent trials. No trial was skipped or timed out. No retraining or controller adjustment occurred within this cohort.

| Condition | Whole-task success | Failure count | Approach failures | Carry entries | Beam failures after carry entry |
| --- | ---: | ---: | ---: | ---: | ---: |
| RGB controller | 1/3 | 2/3 | 2 | 1 | 0/1 |
| ACT-before | 0/3 | 3/3 | 2 | 1 | 1/1 |
| ACT-expanded | 0/3 | 3/3 | 2 | 1 | 1/1 |

`test-minus` and `test-yaw` fail in the shared coarse RGB approach before ACT inference. All six report `own_wheel_heading_unresolved` for model slot r3 (physical robot r1). Read-only inspection of the minus image found a 52.470px short wheel-mask dimension against the 52.4px gate. This identifies a brittle shared entry condition, not six independent ACT carry failures. Yaw has the same reported gate failure; the minus dimensional measurement is not asserted for yaw.

On `test-plus`, both ACTs invoke their models, carry the beam and select stop/release, but the beam remains outside its destination slot. The referee and sampled original video agree with the visual placement rejection. The simultaneous box job is interrupted by this failure; its unfinished outcome is not a separately established box-controller failure. RGB completes both deliveries on that same case.

| `test-plus` condition | Wall seconds | Issued commands | ACT calls | Mean ACT call latency | Sampled three-robot loaded motion |
| --- | ---: | ---: | ---: | ---: | ---: |
| RGB controller | 479.55 | 960 | 0 | n/a | 6.0s |
| ACT-before | 327.87 | 967 | 354 | 87.5ms | 6.0s |
| ACT-expanded | 264.38 | 745 | 206 | 109.7ms | 5.8s |

The shorter failed ACT runs do not establish a completion-time improvement. All nine report zero sampled inter-robot contacts, zero obstacle-contact steps and weld off. The concurrent-motion measure is conservative and sampled, not continuous collision or physical-safety certification. Model calls above count ACT inference only; planning is replayed and external LLM calls are zero. Dollar cost was not measured. This cohort does not compare Jev/Gemini or communication modes.

The scene XML hashes match across conditions for each case. The two ACTs also receive identical first carry-input image hashes on the carry-entry case. Observations, issued commands, model inputs, referee outputs, videos, outcome hashes and failure denominators are retained in `evaluation-complete.json`; `evaluation-diagnosis.json` records post-run findings and sampled video review. These three fixed spawn perturbations have one trial per condition, so failure fractions are descriptive counts, not population failure-rate estimates. No held-out success improvement from the expanded dataset was demonstrated.

All nine completed outcomes were exported once to the shared `outputs/tensorboard/0922-ACT재학습평가`, with manifest hashes and event readback checked. Chrome's native TensorBoard showed 13 rows: nine evaluations plus the original successful reference, two added demonstrations and training. HParams columns and seven pinned Time Series cards matched the saved view configuration. All nine original video links passed HTTP Range 206 readback. Older snapshot groups were preserved under `outputs/tensorboard-archive/pre-parallel-evaluation-20260922`; original experiment data was retained. See `evaluation-dashboard.json` and the common `outputs/tensorboard-view.json` for the fixed dashboard links.

Next work should address the shared RGB heading gate with held-out image regression coverage, then investigate ACT stop-label/terminal-state coverage using training/development evidence. This inspected suite must be treated as a regression suite after diagnosis; a new held-out suite is needed for an unbiased final comparison. Neither issue was silently patched into these reported trials.
