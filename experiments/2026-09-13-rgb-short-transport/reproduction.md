# Reproduction and evidence read-back

Run from the repository root at this PR's checkout. Keep simulation dependencies separate from test dependencies; see `environment.json` for the observed Mac runtime. All output directories below must be new. No external model service is used. Mac commands assume `.venv-sim-worker-mac` is available; in a worktree use the primary checkout's absolute runtime paths. Ubuntu may use its Python under `xvfb-run`, but this cohort's physical results have only been measured on the recorded Mac runtime.

```sh
EXP=experiments/2026-09-13-rgb-short-transport
SIM_PY=/Users/changmin/projects/ugrp/.venv-sim-worker-mac/bin/python
MJ_PY=/Users/changmin/projects/ugrp/.venv-sim-worker-mac/bin/mjpython
python3 "$EXP/restore_evidence.py" outputs/short-transport-restored-NEW
python3 -m zipfile -e experiments/2026-09-10-rgb-varied-start/models.zip outputs/short-transport-prior-models-NEW
```

The first command verifies each archive SHA and each restored file SHA, preserves original relative paths, refuses to overwrite a destination, and does not modify recorded provenance. The old model ZIP has SHA `d6d421c2d35ed448c4492592220b3e3141d346f5e576ebbdf5a7a3c787b7e635`; models are under its `models/grasp`, `models/varied`, and `models/straight` directories.

Historical visual input/physics audits can use restored files:

```sh
"$SIM_PY" scripts/audit_camera_short_transport_student.py \
  --run-dir outputs/short-transport-restored-NEW/short-transport-varied-20260913-v1/heldout-02-visual \
  --transport-model-dir "$EXP/models" \
  --stage-model-dir outputs/short-transport-prior-models-NEW/models/varied \
  --out outputs/short-transport-heldout02-audit-NEW.json
```

Complete RGB needed by this full audit is archived for every fixed/delay run and varied 01/02. The other eight varied runs retain their original passed audit and all carry/grasp inputs, but their approach image sequence requires the local raw directory. Historical playback audits additionally require the teacher report at the absolute `playback_teacher_source.path` recorded in their original result; the auditor checks its SHA. Extraction does not silently rewrite that path or make this requirement portable. A new playback run records the local restored teacher path correctly.

Retrain from the archived teacher RGB and reports:

```sh
"$SIM_PY" scripts/train_camera_short_transport_student.py \
  --teacher-dir outputs/short-transport-restored-NEW/short-transport-teacher-20260913-v2 \
    outputs/short-transport-restored-NEW/short-transport-teacher-20260913-v3 \
    outputs/short-transport-restored-NEW/short-transport-teacher-20260913-v4 \
  --out-dir outputs/short-transport-retrained-NEW
```

The numeric model files should match in the same numerical runtime. Training manifests/report hashes change with the new source SHA and provenance paths. Teacher archives contain every image consumed by this trainer; they do not contain all earlier teacher grasp-loop images and cannot establish a complete camera audit of the teacher's grasp stage.

For a fresh fixed paired cohort, use the committed model rather than replacing it with an unverified retraining. Commit any source/protocol edits first; the cohort rejects dirty source or changed models/HEAD. Paths below can be made absolute when needed:

```sh
"$SIM_PY" scripts/ugrp_session.py run short-transport-fixed-NEW -- \
  "$SIM_PY" scripts/run_camera_short_transport_cohort.py \
  --cases-json "$EXP/fixed-cases.json" \
  --grasp-model-dir outputs/short-transport-prior-models-NEW/models/grasp \
  --transport-model-dir "$EXP/models" \
  --replay-teacher-dir outputs/short-transport-restored-NEW/short-transport-teacher-20260913-v2 \
  --mjpython "$MJ_PY" --out-dir outputs/short-transport-fixed-NEW
```

Only after fixed qualification (at least 9/10 visual full completion), run `delay-cases.json` with both default conditions. For the registered varied cohort, use `varied-cases.json`, add `--stage-model-dir outputs/short-transport-prior-models-NEW/models/varied --conditions visual`, and use a new session/output name. A paired varied replay experiment would be additional scope and was not run here. Runtime commands are served through the existing session wrapper, which cleans its child process group on completion; `scripts/ugrp_session.py status NAME` verifies that specific session is stopped.

Source provenance:

| Attempt | SHA | Outcome |
|---|---|---|
| teacher v1 | `464ff9a` | failed grasp evaluation key lookup before carry; preserved |
| teacher v2 | `45afc85` | passing 0.10-speed demonstration |
| teachers v3/v4 | `b61bb4c` | passing 0.07/0.04-speed demonstrations |
| training, fixed development/cohort | `50eb2746` | frozen learned models; fixed 10/10 visual, 10/10 playback |
| varied development, delay/varied cohorts | `1d904c25` | validation/preflight strengthening; unchanged policy/models |

[Source 50eb CI](https://github.com/kcm0127-dotcom/ugrp/actions/runs/34754559325) and [source 1d CI](https://github.com/kcm0127-dotcom/ugrp/actions/runs/34754953715) passed both `offline-regressions` and `ubuntu-simulation`. Offline source audits are evidence recomputation, not operating-system isolation. A successful audit of a stopped run does not convert it into a transport success.
