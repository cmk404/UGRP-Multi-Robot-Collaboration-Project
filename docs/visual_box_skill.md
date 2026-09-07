# Deterministic own-RGB single-box primitive

This primitive is an isolated, single-robot manipulation experiment. `r1` finds one known small box from its wrist camera, approaches a visible marker face, lowers and closes the gripper, checks visual co-motion, performs a short transfer, releases, and checks the released object visually. The controller is deterministic: it makes no LLM calls and sends no inter-robot messages.

## Controller boundary

`harness.visual_box_skill.VisualBoxSkill(task="short_transfer", destination_zone="B", robot_id="r1")` exposes one controller method:

```python
action = skill.decide(observation)
```

The observation must contain exactly `robot_id`, `frame_id`, `sim_time`, `image`, `sha256`, `camera`, and `actuator_state`. `image` is the robot's own base64 JPEG from `robot_cam`; `actuator_state.servo_pulses` contains its own servos 1, 3, 4, 5, and 6. Frame IDs and simulation time must be monotonic, and the JPEG hash must match. The returned macro is `drive`, `pose`, `wait`, or `finish`; the runner expands it into bounded camera-port actions.

The controller never receives the MuJoCo world, model, data, contacts, constraints, cargo pose, robot pose, destination coordinates, depth buffer, or evaluation snapshots. `scripts/evaluate_visual_box.py` captures world state only after each decision and writes it to `evaluation-only.jsonl` for offline scoring. `control.jsonl` records the RGB-derived decision path, `commands.jsonl` records applied actuator commands and their source-frame hashes, and `inputs/` preserves the exact JPEG inputs.

## Visual and geometric assumptions

- `harness.monocular_box.CameraBoxTracker(target_id="small_box_01").observe(image)` detects the known printed ArUco marker (IDs 14/15) using its manufactured 75 mm side length and the static raw-fisheye camera profile. It estimates marker pose by monocular PnP. When decoding fails briefly, bounded forward/backward-checked optical flow may propagate previously identified corners; its result is marked as tracked and includes age since ID confirmation.
- Marker pose gives the marker-plane inset point in camera coordinates. The controller converts this through the robot's own arm PWM and fixed camera/arm calibration. The upright box-center height offset is a manufactured-geometry assumption, not simulator truth.
- `harness.visual_box_surface.observe_known_box_top(image, servo_pulses)` uses a known upright box, its 34 by 40 mm top dimensions, the fisheye profile, and own-camera forward kinematics to estimate top height from visible cyan surface geometry. Its horizontal output is only the centroid of the visible surface patch; it is not asserted to be the full box center.
- `harness.visual_attachment.compare_box_comotion(before_image, after_image)` compares broad cyan masks during a controlled camera yaw intervention. It requires both visible components to cover at least 6000 pixels, IoU at least 0.88, centroid displacement at most 8 pixels, and area ratio from 0.90 to 1.10. This is evidence of visual attachment only. Cyan color alone does not identify the box, so the caller must first establish identity from its marker.
- Named destination zones are symbolic goal labels; the current runner obtains the label from the seeded task definition, without passing its coordinates. An unvalidated experimental `destination_zone` mode exists; in that mode, `harness.visual_floor.observe_zone` estimates a visible painted floor-zone point from RGB, own camera calibration, and a floor-plane assumption. The controller is not given a seeded layout or zone coordinates.

The known marker size, box dimensions, upright placement, printed marker mount, static camera intrinsics/distortion, arm kinematics, and PWM calibration supply monocular scale. They are fixed manufactured/calibration inputs. They do not expose the current external positions of the box, robot, obstacles, or zone.

## Execution and controls

Each cohort run must use the isolated fixture built by:

```python
MultiMasterPiProductionV2(
    warehouse_layout="arena",
    seed=SEED,
    render=True,
    warehouse_cargo_ids=("small_box_01",),
)
```

Precompile the isolated controller, runner, recorder, and report path before launching runs:

```bash
/opt/anaconda3/bin/python -m py_compile \
  harness/visual_box_skill.py harness/monocular_box.py harness/visual_arm.py \
  harness/visual_floor.py harness/visual_box_surface.py harness/visual_attachment.py \
  harness/visual_box_evaluation.py scripts/evaluate_visual_box.py \
  scripts/record_visual_box.py scripts/render_visual_box_report.py
```

Use the explicit contact-solver configuration `impratio=10` and `noslip_iterations=0` for every positive and negative-control run. Do not rely on the runner defaults, which are different for `impratio`.

Example short-transfer runs are:

```bash
/opt/anaconda3/bin/python -m scripts.evaluate_visual_box \
  --output artifacts/visual-box-cohort/seed-41 --seed 41 --steps 340 \
  --task short_transfer --impratio 10 --noslip-iterations 0 --record

/opt/anaconda3/bin/python -m scripts.evaluate_visual_box \
  --output artifacts/visual-box-cohort/seed-41-open-control --seed 41 --steps 340 \
  --task short_transfer --impratio 10 --noslip-iterations 0 \
  --negative-control-open-gripper --record
```

Repeat with the predetermined cohort seeds and a fresh, nonexistent output directory per run. The negative control intercepts every requested servo-1 command and applies pulse 2000, keeping the gripper open. It must not produce a visual lift declaration or a completed transfer; it tests false-positive resistance rather than manipulation performance.

After all intended run directories contain `result.json` and `control.jsonl`, build the review report with:

```bash
/opt/anaconda3/bin/python -m scripts.render_visual_box_report \
  artifacts/visual-box-cohort
```

The report writes `index.html` and `summary.json` beside the run directories. Inspect the preserved JPEGs, command logs, evaluation-only logs, and recorded video along with aggregate status. A successful process exit or generated report alone is not evidence that the cohort passed. The completed cohort is recorded below.

## Scope of evidence

This primitive tests a deterministic RGB execution skill for one known box and one active robot in an isolated fixture. It does not test an LLM planner, communication, multi-robot cooperation, unknown-object recognition, general grasping, arbitrary camera calibration, or transfer to physical hardware. The short-transfer task follows bounded forward segments; it does not prove obstacle detection, obstacle avoidance, route planning, or safe navigation through an unknown arena. Offline cargo positions and constraint state are referee evidence only and must never feed back into controller decisions.


## Verified cohort (2026-09-06)

Artifacts: `outputs/warehouse_research/coela-single-box-camera-01/`. Recorded seeds 41, 58, and 73 all passed both the visual release check and offline physical outcome gates (3/3). Maximum lift above the initial box height was 6.82–6.85 cm; final planar displacement was 77.69–77.88 cm. No active cargo attachment constraint was used. The seed-41 open-gripper negative control stopped at `VISUAL_LIFT_UNCONFIRMED`, with no visual lift declaration or physical transfer success.

Every recorded decision JPEG and action matched its independently executed unrecorded counterpart exactly (298, 280, and 241 decisions). All preserved input JPEG hashes were checked. The three 1x videos are 10 fps; pickup, carry, and release frames were visually inspected. This is three selected development seeds, not a held-out generalization or optimality result.

The per-run `source_hash` covers a selected list, not every transitive dependency. `source-manifest.json` and `source-snapshot/` preserve the broader source/calibration snapshot taken during the frozen cohort; `verification.json` records artifact checks. `commands.jsonl` contains requested/applied port commands, but explicit runner `port.stop()` calls are not separate entries; consult the frozen runner for lease expiry and stop behavior.
