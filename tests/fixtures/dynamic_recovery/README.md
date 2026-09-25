# Dynamic recovery back-off TOP frames

Raw TOP RGB frames copied byte-for-byte from the D3 live run
`outputs/dynamic-coordination-20260925/D3-dynamic-inject/rgb/` (source
`claude/dynamic-coordination` c340ef2, bundle v44):

- `after-short-backoff-top.jpg` = `pair-199-coarse-top.jpg`, first coarse frame after one 10-slice back-off.
- `after-long-backoff-top.jpg` = `pair-200-coarse-top.jpg`, first coarse frame after a second 10-slice back-off (20 slices total).

With the start-of-run identity crop centres both carriers show zero wheel
pixels, which is the `coarse RGB model convention unresolved` failure of D3.

- `e2-after-fine-failure-backoff-top.jpg` = `outputs/dynamic-team-recovery-20260925/E2-grasp/rgb/pair-273-recovery-recenter-top.jpg`
  (source `claude/dynamic-team-recovery` 846b4b0, bundle v51; sha256 prefix 259b87b2f4b1c707): the stopped TOP
  frame after one full 20-slice back-off that followed a failed fine alignment. The carriers moved
  about 64 px west of their last tracked crops (x≈218 → ≈151), more than the 55 px crop half-width,
  so r1 kept only 76 wheel pixels (< 80) and the next approach stopped as
  `coarse RGB model convention unresolved`.
