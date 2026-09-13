# Final pair-carry cohort — independent trace and visual review

Reviewed 2026-09-14 from `pair-sync-comparison-v1`, frozen source
`99898d8983315e39635c063e0b6b430c01818b22`. No simulator was run.

## Verdict

The 52 result/trace pairs are internally consistent. Across all 970 carry trace
rows, the recorded applied `actions` match the issued `control.forwards` after
replaying the runner's private fault-window split and midpoint interception
rule, with 2 ms allowed at interval boundaries. There are zero mismatches.

The cohort result is 46/52 physical successes. All 26 sync runs succeed. The six
failures are baseline runs: departure r1/r3 at 0.60 and 1.00 s, and midcarry r3
at 0.35 and 0.75 s. Their audits pass because audit success means the artifact
and input/control replay are consistent; it does not mean the physical task
succeeded. Each failed result stops on invalid RGB and lacks the placement and
release phases, consistent with its physical failure.

## Report blackout boundary

All four report-blackout sync runs behave as specified:

- `report-r1-000-sync` and `report-r3-000-sync`: decisions 0–5, observed at
  0.00 through approximately 1.00 s, omit the affected report, remain epoch-1
  HOLD, and issue `{r1: 0, r3: 0}`. Decision 6 at 1.20 s accepts fresh reports
  from both robots and authorizes `all_participants_ready_to_resume` in epoch 1.
- `report-r1-120-sync` and `report-r3-120-sync`: decisions 7–11, observed at
  1.40 through 2.20 s, omit the affected report, remain epoch-1 HOLD, and issue
  `{r1: 0, r3: 0}`. Decision 12 at 2.40 s accepts fresh reports from both and
  authorizes resume in epoch 1.

The nominal 1.20 s boundary is represented as 1.199999999999335 in the stored
simulation clock, so it precedes the blackout comparison and is delivered; the
first omitted sampled report is at 1.40 s. No applied trace row moves either
robot while a report is omitted.

## Video samples

Extracted frames are under `frames/` and retain the video's overlay timestamps.

- `departure-r1-100-sync`: 14 s shows `grasp_hold`; 16 and 18 s show carry;
  20 s shows `place_lower`, 22 s `place_retract`, and 24 s `release_hold`.
  The sequence agrees with success and the 0.214924 m final displacement.
- `departure-r3-100-sync`: 14 s shows `grasp_hold`; 16, 18, and 20 s show
  carry/recovery; 22 s shows `place_open` and 24 s `release_hold`. The sequence
  agrees with success and 0.201470 m final displacement.
- `midcarry-r3-075-sync`: 14 s shows `grasp_hold`; 16, 18, and 20 s show
  continued bounded carry after the midcarry interception; 22 s shows
  `place_open` and 24 s `release_hold`. It agrees with success and 0.210027 m
  final displacement.
- `report-r3-120-sync`: 14 s shows `grasp_hold`; 16 and 18 s show `carry_stop`
  during/after the blackout pause; 20 s shows `place_open` and 22 s
  `release_hold`. This agrees with HOLD/reacquisition and success.
- Matching failed baselines `departure-r1-100-baseline`,
  `departure-r3-100-baseline`, and `midcarry-r3-075-baseline` show increasing
  asymmetric geometry during the 14–16 s samples and their videos end without
  a lower/open/retract/release sequence. This agrees with invalid-RGB aborts.

No sampled video frame conflicts with the stored physical result.

## Limits

- Applied commands prove the recorded interception, not wheel traction or
  payload response. The sampled RGB/video changes and 10 Hz referee stream are
  the separate physical evidence.
- Video inspection is sampled rather than frame-by-frame continuous proof.
  Contact continuity comes from the referee samples and remains limited by
  their maximum 0.15 s sample gap.
- The policy uses a fixed scene, camera/FOV, beam, grasp start, and local
  deterministic models. These results do not establish real-robot transfer,
  general grasp/resource coordination, distributed consensus, independent LLM
  collaboration, or broad timing/geometry generalization.
- Report blackout baseline success is expected because the baseline does not
  consume the synchronization reports; those runs are not evidence that report
  loss is harmless to a distributed controller.
- The intervention comparison bundles the barrier, stop rule, pixel-skew
  catch-up, and rejoin behavior. It does not isolate a pure READY-message causal
  effect.
