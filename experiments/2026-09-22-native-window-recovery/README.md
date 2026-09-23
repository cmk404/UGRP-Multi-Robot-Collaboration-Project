# Native viewer reset and instability recovery — 2026-09-22

User reported the native window closed after trying the right-side controls. The exact button was not identified. Original source `357e1f2e66dcae20c54f669711308ed18cf03966` logged a QACC instability at world time 56.862, followed by `sim_time must not move backwards`; cleanup also used the reset clock. The original result retained sim_s=-1.298 and wall_s=69.187.

Two untouched controls (headless and native viewer) reached 90 SIM seconds. This isolates the confirmed clock/reset handling defect without claiming which panel control triggered the instability. Original maps, robot geometry, physics settings, and weld OFF are unchanged.

Runtime fix source: `52e70e3af140e4f965076ecf4d3d3167dfc6a1e7` (clean for the native verification). The API detects backward clocks or new BADQPOS/BADQVEL/BADQACC/BADCTRL warnings before stale command leases advance, cancels wheel commands, blocks further commands/observations, and requires reset. Cleanup preserves the original failure. The native CLI keeps the viewer open, pauses, shows a recovery overlay, and accepts R then Space. Headless/API fails explicitly. The fault is retained in runtime-events.json and result.json even after recovery; recovered fault runs intentionally retain protocol_complete=false and exit 2.

Validation:
- Related tests: 62 passed, 27 subtests.
- Real macOS viewer: explicit clock reset and injected NaN velocity both stayed open while paused, reset through the CLI key callback, resumed, reached 2 SIM seconds in episode 2, and closed normally. Script: `mjpython -m scripts.check_simulation_recovery --output outputs/native-recovery-check-v1` (inside owned ugrp_session).
- Native keyboard callbacks are exercised; exact mouse clicks in the user's right-side panel are not automated.
- Panel model-option changes are not restored by R. Restart the process if those settings repeatedly cause instability.
- Ubuntu CI includes the optional physics regressions and the same real-viewer recovery check under Xvfb.

`verification.json` records raw paths, result hashes, source identities, failures, controls, and native recovery records. All referenced artifact hashes were checked. Five completed records were exported once to the primary checkout's `outputs/tensorboard/0922-native-recovery`, read back via EventAccumulator, and verified in the running TensorBoard UI with four pinned metrics and five HParams columns. Existing snapshots and the other task's server were preserved. No video was produced in these checks. The newly opened interactive viewer is still running and is not marked complete or included in the snapshot.

These are infrastructure diagnostics, not robot task success, communication efficacy, or hardware validation. Raw artifacts remain local; hashes are not a backup.
