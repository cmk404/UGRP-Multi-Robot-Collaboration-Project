# Camera-only CoELA-inspired control

The arena evaluator now defaults to `--controller camera`. The former
`strict_local` implementation is retained only as an explicit
`--controller oracle-baseline` comparator: its text observations and automatic
transport skills are not camera-only control.

## Actor boundary

Each `CameraPlanner` gets its own freshly rendered `robot_cam` JPEG as an
actual multimodal image part. Text contains that frame's identity and hash,
its own commanded PWM/motor values, its own recent decisions/command
acknowledgements, and explicitly delivered messages. The fixed task legend
identifies zone colors and cargo appearance; it contains no coordinates,
seed, route, obstacle layout, object detections, depth, or peer state.

The planner has no world object. Its only execution outputs are bounded
forward/turn wheel commands, camera pan PWM, arm/gripper PWM, and wait.
`CameraRobotPort` translates these directly to the robot's own actuators.
It neither localizes cargo/obstacles/peers nor calls automatic pickup,
formation, navigation, IK, or weld helpers. Servo commands slew through
physical actuators; robot/cargo poses are never assigned during an episode.
An action acknowledgement is not proof of a grasp or delivery.

Three separate policy instances run concurrent inference. Each robot gets
its next image after its own action finishes; there is no team decision
barrier. Drive commands expire within one simulated second. Late responses
are discarded at episode end, and actions from frames older than five
simulated seconds are rejected. Message recipients are explicit; private
reasoning remains private. None/structured/natural modes use the same camera
and actuator interface.

## Physics and evaluation

A fresh world contains no MixedEngine or warehouse crew. The physics owner
advances all robots and uses simulator state to compute forces/contact, as
any simulator must. The actor path never receives that state. Automatic
control entry points and the full world-state accessor are trapped during
the run. The offline evaluator and presentation video can inspect true poses
but cannot issue policy decisions. This is an application interface boundary,
not a process sandbox against arbitrary hostile Python code.

Only physical painted zone geometries are made visible in robot cameras;
other viewer decorations remain hidden. Compact zones stay 0.82 m per side,
robot/cargo dimensions remain unchanged, and seeded corridor obstacles exist
before the episode.

## Reproduction and evidence

```sh
python -m scripts.evaluate_coela_camera \
  --output outputs/warehouse_research/NEW_CAMERA_RUN \
  --seeds 41,58,73 --modes natural --max-calls 24 --timeout 90 --record
```

`inputs/rN-NNN.jpg` preserves every actual policy image. `planner_input`
records its digest, local memory and delivered messages; `decision` preserves
the model reply; `actuator_command` ties the applied command to its input
frame. `evaluation-only.json` and `layout-evaluator-only.json` are supervisor
artifacts and never enter the policy context. Source hashes identify cohorts.

This architecture removes the privileged transport controller; it does not
establish reliable visual pickup or coordinated plank carrying. Camera input
validity, actual command execution, and physical task success must be reported
separately. Missing/error runs remain failures. Probe 01 preserves the initial
image-encoding integration failure; Probe 02 verifies the corrected actual
multimodal client path. Do not use either short probe as a transport benchmark.
