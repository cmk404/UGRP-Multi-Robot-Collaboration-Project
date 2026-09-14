# Independent varied-start cohort review

Scope: read-only review of all results, input audits, and grasp reports in `/Users/changmin/projects/ugrp/outputs/short-transport-varied-20260913-v1`, plus direct inspection of every failed first-carry top/r1-own RGB and selected observer-video frames from failed `heldout-01` and successful `heldout-02`. Frozen source: `1d904c255b9ff551b103e6de98ba22bd7692d86f`. Referee fields and observer video are evaluation evidence, not student inputs.

## Ten-run result

Every run passed the audited seven-stage RGB approach (`approach_ok=true`, two final-alignment checks) and every run passed the referee grasp evaluation. Grasp qualifying holds were 2.7–2.8 s and maximum lifts were 0.078659–0.080169 m. Six runs completed transport and release; four stopped before issuing a carry pulse when r1's first carry-anchor RGB was outside learned support.

| case | approach calls / 7 stages | grasp hold / max lift | result | first-carry decision |
|---|---:|---:|---|---|
| heldout-01 | 316 / pass | 2.8 s / 0.079468 m | fail | r1 OOD |
| heldout-02 | 300 / pass | 2.8 s / 0.078659 m | pass | both in-domain |
| heldout-03 | 324 / pass | 2.8 s / 0.079799 m | pass | both in-domain |
| heldout-04 | 330 / pass | 2.7 s / 0.079175 m | fail | r1 OOD |
| heldout-05 | 282 / pass | 2.8 s / 0.079420 m | fail | r1 OOD |
| heldout-06 | 258 / pass | 2.8 s / 0.079642 m | fail | r1 OOD |
| heldout-07 | 306 / pass | 2.7 s / 0.079218 m | pass | both in-domain |
| heldout-08 | 318 / pass | 2.8 s / 0.079558 m | pass | both in-domain |
| heldout-09 | 280 / pass | 2.8 s / 0.080169 m | pass | both in-domain |
| heldout-10 | 346 / pass | 2.8 s / 0.079758 m | pass | both in-domain |

The r1 model's `own_orange_fraction` support is `[0.664909, 0.838054]`. First-anchor values were 0.655979 (heldout-01), 0.630735 (04), 0.657053 (05), and 0.656412 (06), all below the lower limit. Heldout-04 also had `own_orange_y=0.389949`, below its `[0.399167, 0.538365]` support. All other recorded first-anchor features in those cases are within support. Direct viewing agrees with the feature extraction: each failed r1 own image shows the beam dominating the view but with a larger exposed dark lower band than in-domain training/support images. The shared top anchors look nearly aligned and do not reveal the failing own-camera crop. Thus these are first-carry appearance/crop domain failures after a successful approach and lift, rather than failed approach or failed grasp.

## Observer-video check

In heldout-01, the observer frames show both robots initially separated and angled, converging around the floor-supported beam, then positioned at opposite ends. The beam is visibly elevated in the final frame (overlay z about 0.092 m), but the run immediately enters carry-stop after the first RGB OOD and never transports or places it. This visible sequence supports approach and lift; bilateral contact and the 2.8 s qualifying hold come from referee evaluation, not the camera.

In heldout-02, the observer frames visibly show the initial varied poses, convergence to opposite beam ends, lift (overlay z about 0.068 m), elevated forward carry (about 0.090 m), lowering to the floor (about 0.020 m), and gripper retraction with the beam left behind. The evaluator separately establishes the sampled contact, displacement, grounding, clearance, stability, and weld-off gates. Own RGB remains strongly beam-occluded; top RGB supplies global payload/robot appearance but does not itself prove physical contact.

The frame files and SHA-256 values, raw video hashes, failed anchor hashes, exact features, and per-run audit metrics are recorded in the adjacent JSON.
