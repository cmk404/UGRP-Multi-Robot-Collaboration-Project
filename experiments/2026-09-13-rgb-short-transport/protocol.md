# RGB short transport curriculum

Start from PR #33 (10406542), preserving its approach and grasp models.
The first physical diagnostic isolates the previously demonstrated fixed grasp
station. It uses the existing RGB grasp correction, then a privileged offline
teacher to carry the beam 0.20 m forward, lower, open and retract the arms.
It does not establish navigation or independent RGB transport success.

Keep the same payload, physical properties, fixed own/top cameras and weld OFF.
Store source SHA, exact camera inputs, issued commands, privileged labels and
output-only referee streams separately. Preserve every attempted diagnostic.
No policy may consume trial-specific target renders or evaluator feedback.

Before the student cohort, freeze the trained model and source and register all
cases. Fixed-start student qualification is at least 9/10 complete runs, with
20 cm displacement within 3 cm, bilateral contact and at least 3 cm lift throughout
carry, then supported release without robot contact and at most 5 mm motion over
the final second. Report deterministic repeats as repeatability, not new-condition
generalization. Compare replay separately. Only after this qualification, add
predefined departure delays and varied start poses in a new cohort.

Registered follow-ups are `delay-cases.json` (r1 or r3 requested wheel commands
withheld for the first 1, 2, 3 or 5 carry slices) and `varied-cases.json` (the
first ten previous heldout cases, selected in their original order). These test
cases are not teacher training data. Both conditions use identical setup and
grasp/placement commands. Playback repeats the v2 teacher carry actions;
visual predicts from fresh images and requires two stopped confirmations.
Current issued commands never advance the image-based progress estimate by
themselves. The fixed stop threshold is 19 cm to leave room for passive coasting.

The carried-state classifier is an image estimate, not a contact detector.
In particular, a lowered beam still between closed fingers may resemble a lifted
beam. Lower/open/retract remains a demonstrated command sequence; no learned
release-success claim or referee-driven phase correction is made. Final physical
success is evaluated after actor execution. Contact/lift continuity is sampled
at approximately 10 Hz; weld status is also counted at every referee tick.

Raw evidence is local under the primary project's outputs directory. This UGRP
project does not use Google Drive. Main merges require explicit owner approval.
