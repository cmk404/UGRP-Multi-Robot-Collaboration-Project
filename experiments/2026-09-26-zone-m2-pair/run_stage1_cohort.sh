#!/bin/bash
# Stage 1 cohort: seeds 711-718 x status channel on/off, two sims at a time, from a FROZEN detached worktree.
# usage: run_stage1_cohort.sh <frozen_worktree> <output_root>
set -u
WT=${1:?frozen worktree}; OUT=${2:?output root}
PY=/Users/changmin/projects/ugrp/.venv-sim/bin/python
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1 MKL_NUM_THREADS=1
mkdir -p "$OUT"
cd "$WT" || exit 1
[ -z "$(git status --porcelain)" ] || { echo "frozen worktree is dirty"; exit 1; }
for seed in 711 712 713 714 715 716 717 718; do
  for arm in on off; do
    echo "$(date +%T) seed $seed $arm load $(sysctl -n vm.loadavg)" >> "$OUT/cohort.log"
    "$PY" scripts/run_m2_pair.py --seed $seed --status-channel $arm --output "$OUT/s$seed-$arm" > "$OUT/s$seed-$arm.log" 2>&1 &
  done
  wait
done
echo "$(date +%T) done" >> "$OUT/cohort.log"
