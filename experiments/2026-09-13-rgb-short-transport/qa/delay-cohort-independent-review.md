# Independent delay-cohort evidence review

Scope: read-only review of the complete 16-run cohort at `/Users/changmin/projects/ugrp/outputs/short-transport-delay-20260913-v1`, frozen source `1d904c255b9ff551b103e6de98ba22bd7692d86f`. I inspected every run's recorded result/audit through the cohort report, then directly viewed the last top RGB, both last own RGB images, and an extracted final observer frame for each of the four visual failures. Referee state is diagnostic output only. This review does not establish continuity between sampled ticks.

## Result and cause

The reported scores are internally consistent: visual 4/8 and playback 4/8. Both 0.2 s and 0.4 s delay pairs pass; all four 0.6 s and 1.0 s visual runs stop during carry because one robot's `top_own_y` crosses the learned feature support by a small amount. They therefore never enter lower/open/retract/release-hold, so their evaluator release and endpoint gates fail closed with null final metrics. This is an RGB domain-support stop, not evidence of a dropped payload.

| case | OOD robot / feature | support violation | referee at stop | visible evidence |
|---|---|---|---|---|
| delay-r1-03-visual | r3 `top_own_y` | 0.388530 > 0.387337 | carry progress 0.191658 m; height 0.064625 m; both contacts true; both welds false; yaw r1/r3 -4.46/-4.28 deg | observer/top show an elevated orange beam between a modestly clockwise-skewed pair |
| delay-r3-03-visual | r1 `top_own_y` | 0.609624 < 0.611048 | 0.216478 m; 0.062844 m; both contacts true; welds false; yaw +4.21/+4.40 deg | observer/top show the elevated beam and opposite modest skew |
| delay-r1-05-visual | r3 `top_own_y` | 0.387612 > 0.387337 | 0.208425 m; 0.063604 m; both contacts true; welds false; yaw -6.77/-6.57 deg | observer/top show a larger clockwise diagonal than at 0.6 s |
| delay-r3-05-visual | r1 `top_own_y` | 0.610248 < 0.611048 | 0.157122 m; 0.064627 m; both contacts true; welds false; yaw +6.60/+6.75 deg | observer/top show the corresponding larger opposite diagonal |

The own-camera images in all eight inspected views are almost entirely occluded by the orange beam (recorded orange fractions about 0.79-0.80). They provide no visually resolvable evidence of finger contact or relative yaw. The top/observer views support the claim that the beam remained elevated and that asymmetric starts induced common pair/beam yaw. Contact continuity comes only from the sampled referee fields; camera images do not prove bilateral contact.

The fail-closed support behavior is appropriate for this fixed-scene learner, but the support was learned from synchronized demonstrations. The delayed robot changes the pair geometry, and the non-delayed partner's top-image vertical centroid is what exits support. Issued-command history describes intent and does not measure actual motion, so it cannot resolve the resulting synchronization error.

## Paired playback comparison

Playback completes release but loses distance in proportion to withheld command time. At 0.6 s, final displacement is 0.163474 m (r1 delayed) and 0.163464 m (r3 delayed), respectively 0.036526/0.036536 m short; every gate except endpoint passes. At 1.0 s, final displacement is 0.130730/0.130717 m, 0.069270/0.069283 m short; carry progress is also below 0.15 m, while grounding, clearance, stable release, bilateral sampled carry contact, and weld-off gates pass. This isolates the replay limitation: fixed action playback cannot replace withheld motion.

## Bounded next study

Keep this cohort and current thresholds frozen. Collect a new privileged-teacher curriculum with preregistered asymmetric departures and explicit synchronization/re-alignment segments: pause the leader or issue bounded differential corrections until top-RGB pair/beam symmetry recovers, then resume common forward motion. Train student outputs only from temporal own/top RGB anchors and own issued history, with teacher state used solely for offline labels. Evaluate once on a disjoint, preregistered delay/yaw set. This tests recovery from observable pair geometry rather than widening support on these held-out failures.

All 16 cohort entries report evidence and grasp audit success. The exact image paths and SHA-256 values used in this review are recorded in the adjacent JSON file.
