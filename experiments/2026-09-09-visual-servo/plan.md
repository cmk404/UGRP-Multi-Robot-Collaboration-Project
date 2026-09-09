# Camera-only visual servo gap repair

User authorized implementing the visible gaps. Preserve physical cameras, scene and weld OFF. No state/coordinate/contact feedback to controller. New independent runner uses current/prior own+overhead RGB and own issued commands. First diagnose r1 alone for 16 rounds (peer stationary), then inspect actual predicted landmarks before choosing any broader cohort. A new source SHA is required after fixes; do not edit source during a cohort.

Implementation acceptance: strict pixel observations, state- and view-conditioned local goal-error prediction, isolated own actions, bounded raw pulse changes, reverse opt-in, explicit visual alignment/closure/verification/recovery, exact wire and controller replay. These are code capabilities, not proof of grasp.

Experiment acceptance: inspect landmark overlays against the actual original input; report missing or wrong landmarks, selected stages, samples and actions. Physical grasp is still the output-only two-robot referee (bilateral contact, 3 cm lift, 2 s hold, no weld). Single-robot trials diagnose one-side alignment/closure and cannot establish dual grasp. If prerequisites fail, fix the evidenced cause before further pair trials. Record all unsuccessful attempts, tokens and raw locations. No Drive use. PR review before merging.
