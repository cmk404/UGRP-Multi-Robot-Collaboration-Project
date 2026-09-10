# Pixel perception findings

The calibrated point is the center of the isolated moving finger regions, not a verified pair of exact tips. Raw camera geometry, field of view, image size, scene, startup commands and weld-OFF setting remain unchanged across these diagnostics.

## Verified corrections

- Open/close probes isolate two finger motion regions in the overhead image. Closed fingers also expose reversible surfaces at the bottom edges of the own view; it is inaccurate to say the own camera never shows any jaw surface. The open tips and metric aperture remain unverified.
- Flow-only displacement can drift relative to a new isolated calibration. Candidate acceptance therefore uses fresh open/close measurements and reverses unmeasurable candidates within a bounded probe budget.
- JPEG-scale bridges can merge the two finger regions. Stronger image-difference thresholds provide consistent separate regions without changing camera inputs.
- In v6 frames194–196, two horizontal finger regions of about71/50 pixels are accompanied by a10-pixel satellite only at threshold15. Giving each connected component equal PCA weight yields a false45-degree axis. Thresholds16–25 consistently retain a horizontal axis. The portable RGB fixtures in tests/fixtures/gripper_motion_satellite preserve this case with source SHA and hashes.
- Independent output-only base records show yaw changes by only about0.015degree during that straight-drive interval. These records corroborate the diagnosis; the controller never receives them.
- Selecting the own endpoint nearest the image center every frame switches ends during a pan. v5 frames57/60 demonstrate this. The own endpoint now follows its previous image location, updated from open views.

These changes address measured perception and acceptance failures. They do not by themselves establish capture, lift, independent LLM cooperation, or task completion. The physical evaluator and complete run results remain the success authority.

## Subsequent negative checks

- Raw-command replay of v7 actions0..112 reproduces the original round113 own and top RGB hashes exactly. At that view, both gripper close1500 and close1000 fail all four isolated calibration pairs. A larger pulse excursion does not resolve this ambiguity. See jaw-amplitude-comparison.json and the two diagnostic result/manifests.
- A short own-view rectangle can change its fitted major axis under a small pan; v7 rounds22/25 have aspect ratios about1.60/1.63. Its fitted endpoints are therefore not reliable physical endpoint correspondence. The controller omits endpoint aiming for aspect<2 and compares matched terms across observations. This is an observability heuristic and does not establish physical endpoint identity for every longer contour.
- Bounded alternate look commands are a recovery action when baseline calibration remains unavailable. They are scored only after fresh isolated finger measurements.

- v8 own-frame fitted rectangles extrapolate endpoints beyond the image bottom: r120/214/226 have y about1.074. Their horizontal error must not enter a camera-observed endpoint objective. The selected offscreen endpoint is omitted rather than silently switching to the other end.
- Read-only independent review of v8 r214 confirms that the top target is on the vertical beam centerline 4.2px inside its lower end, desired opening axis horizontal, and the detected anchor matches the two finger-motion-lobe midpoint. Those frames do not support changing top target semantics or declaring the target physically impossible.
