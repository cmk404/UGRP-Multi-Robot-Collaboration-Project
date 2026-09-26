#!/bin/bash
# Stage 2b cohort (door v2): seeds 821-826 x status channel on/off, --stage door --approach v2,
# --on-failure continue; two sims at a time, from a FROZEN detached worktree.
# usage: run_stage2b_cohort.sh <frozen_worktree> <output_root>
set -u
WT=${1:?frozen worktree}; OUT=${2:?output root}
PY=/Users/changmin/projects/ugrp/.venv-sim/bin/python
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1 MKL_NUM_THREADS=1
mkdir -p "$OUT"
cd "$WT" || exit 1
[ -z "$(git status --porcelain)" ] || { echo "frozen worktree is dirty"; exit 1; }
for seed in 821 822 823 824 825 826; do
  for arm in on off; do
    echo "$(date +%T) seed $seed $arm load $(sysctl -n vm.loadavg)" >> "$OUT/cohort.log"
    "$PY" scripts/run_m2_pair.py --seed $seed --stage door --approach v2 --door-version v2 --status-channel $arm \
      --on-failure continue --output "$OUT/s$seed-$arm" > "$OUT/s$seed-$arm.log" 2>&1 &
  done
  wait
done
echo "$(date +%T) done" >> "$OUT/cohort.log"
