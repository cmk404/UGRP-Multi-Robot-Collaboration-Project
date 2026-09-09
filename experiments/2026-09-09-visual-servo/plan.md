# Camera-only visual servo gap repair

User authorized implementing the visible gaps. Preserve physical cameras, scene and weld OFF. No state/coordinate/contact feedback to controller. New independent runner uses current/prior own+overhead RGB and own issued commands. First diagnose r1 alone for 16 rounds (peer stationary), then inspect actual predicted landmarks before choosing any broader cohort. A new source SHA is required after fixes; do not edit source during a cohort.

Implementation acceptance: strict pixel observations, state- and view-conditioned local goal-error prediction, isolated own actions, bounded raw pulse changes, reverse opt-in, explicit visual alignment/closure/verification/recovery, exact wire and controller replay. These are code capabilities, not proof of grasp.

Experiment acceptance: inspect landmark overlays against the actual original input; report missing or wrong landmarks, selected stages, samples and actions. Physical grasp is still the output-only two-robot referee (bilateral contact, 3 cm lift, 2 s hold, no weld). Single-robot trials diagnose one-side alignment/closure and cannot establish dual grasp. If prerequisites fail, fix the evidenced cause before further pair trials. Record all unsuccessful attempts, tokens and raw locations. No Drive use. PR review before merging.

## Evidence-driven revisions before final candidate

- dffdfd2 r1/16: no landmarks; self-ID contradicted isolated pixel motion. Exact input/action replay passed 16 requests.
- 44626d3 r1/24: motion cue corrected identity; same-source overhead crop yielded one tentative jaw observation. First unknown wrist target near neutral pointed the own camera away; repeated-drive suppression also interrupted useful approach. Exact replay passed 24 requests.
- Final candidate explicitly reissues the existing documented SEARCH_POSE command values and stores those issued commands (not measured PWM), physically settles for one second, then limits changes from that history. It allows repeated coarse drive and applies local alignment only near the target. Initial camera/scene geometry is unchanged.
- Final candidate cohort: r1-only 30 decisions and r3-only 30 decisions, same seed11 and fixed source; inspect both. Pair test follows only if observations/controller permit a meaningful check. No claim of general success rate from these trials.

- a3829b6 r1/r3 each30: identity mostly grounded but only a few cm of approach and zero local samples. These attempt folders contain the historical suffix `final`; they are superseded, not successful final evidence. Both exact replay audits passed. Next candidate adds an identity-only probe before uncertain overhead actions, 8 bounded bidirectional arm/look calibration commands, and image-error-scaled coarse drive. Run r1 for48 decisions to test the newly exercised local model before extending the cohort. Camera geometry and truth boundary remain unchanged.
