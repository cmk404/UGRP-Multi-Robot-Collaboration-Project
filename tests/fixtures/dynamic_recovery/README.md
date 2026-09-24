# Dynamic recovery back-off TOP frames

Raw TOP RGB frames copied byte-for-byte from the D3 live run
`outputs/dynamic-coordination-20260925/D3-dynamic-inject/rgb/` (source
`claude/dynamic-coordination` c340ef2, bundle v44):

- `after-short-backoff-top.jpg` = `pair-199-coarse-top.jpg`, first coarse frame after one 10-slice back-off.
- `after-long-backoff-top.jpg` = `pair-200-coarse-top.jpg`, first coarse frame after a second 10-slice back-off (20 slices total).

With the start-of-run identity crop centres both carriers show zero wheel
pixels, which is the `coarse RGB model convention unresolved` failure of D3.
