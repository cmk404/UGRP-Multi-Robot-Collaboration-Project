#!/bin/bash
# Stage 2c cohort (door v3, pre-registered in README 2c): seeds 831-836 x status channel on/off,
# then the experimenter open-at-lift arm 837 (r2:2) and 838 (r1:0), ON only.
# Two sims at a time, each under ugrp_session, from a FROZEN detached worktree; load average logged.
# usage: run_stage2c_cohort.sh <frozen_worktree> <output_root>
set -u
WT=${1:?frozen worktree}; OUT=${2:?output root}
PY=/Users/changmin/projects/ugrp/.venv-sim-worker-mac/bin/python
SESSION=/Users/changmin/projects/ugrp/scripts/ugrp_session.py
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1 MKL_NUM_THREADS=1
mkdir -p "$OUT"
cd "$WT" || exit 1
[ -z "$(git status --porcelain)" ] || { echo "frozen worktree is dirty"; exit 1; }
SRC=$(git rev-parse --short HEAD)
COMMON="--stage door --approach v2 --door-version v3 --on-failure continue"
run() {  # name, extra args...
  local name=$1; shift
  echo "$(date +%T) $name load $(sysctl -n vm.loadavg) src $SRC" >> "$OUT/cohort.log"
  python3 "$SESSION" run "kiro-m2c-$name" -- "$PY" scripts/run_m2_pair.py $COMMON "$@" \
    --output "$OUT/$name" > "$OUT/$name.log" 2>&1 &
}
succ() {  # arm -> number of successes among the first four seeds (stop rule)
  "$PY" -c "import json,sys,pathlib; print(sum(json.loads(p.read_text())['evaluation_only']['success_gt'] for s in (831,832,833,834) for p in [pathlib.Path('$OUT')/f's{s}-$1'/'result.json'] if p.exists()))"
}
STOP_ON=0; STOP_OFF=0
for seed in 831 832 833 834 835 836; do
  if [ $seed -ge 835 ]; then   # pre-registered stop rule: first 4 of an arm all failed -> stop that arm
    [ "$(succ on)" = 0 ] && STOP_ON=1; [ "$(succ off)" = 0 ] && STOP_OFF=1
  fi
  if [ $STOP_ON = 0 ]; then run "s$seed-on" --seed $seed --status-channel on
  else echo "$(date +%T) s$seed-on NOT RUN: stop rule (first 4 ON failed)" >> "$OUT/cohort.log"; fi
  if [ $STOP_OFF = 0 ]; then run "s$seed-off" --seed $seed --status-channel off
  else echo "$(date +%T) s$seed-off NOT RUN: stop rule (first 4 OFF failed)" >> "$OUT/cohort.log"; fi
  wait
done
run s837-on-openlift --seed 837 --status-channel on --inject-open-at-lift r2:2
run s838-on-openlift --seed 838 --status-channel on --inject-open-at-lift r1:0
wait
echo "$(date +%T) done load $(sysctl -n vm.loadavg)" >> "$OUT/cohort.log"
