#!/bin/bash
# Stage 3 cohort (failure propagation): seeds 721-724 x status channel on/off, --on-failure continue,
# the pre-registered experimenter drop per seed; two sims at a time, from a FROZEN detached worktree.
# usage: run_stage3_cohort.sh <frozen_worktree> <output_root>
set -u
WT=${1:?frozen worktree}; OUT=${2:?output root}
PY=/Users/changmin/projects/ugrp/.venv-sim/bin/python
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1 MKL_NUM_THREADS=1
mkdir -p "$OUT"
cd "$WT" || exit 1
[ -z "$(git status --porcelain)" ] || { echo "frozen worktree is dirty"; exit 1; }
drop_of() { case $1 in 721) echo r1:3.0;; 722) echo r2:3.0;; 723) echo r1:6.0;; 724) echo r2:6.0;; esac; }
for seed in 721 722 723 724; do
  for arm in on off; do
    echo "$(date +%T) seed $seed $arm drop $(drop_of $seed) load $(sysctl -n vm.loadavg)" >> "$OUT/cohort.log"
    "$PY" scripts/run_m2_pair.py --seed $seed --status-channel $arm --on-failure continue \
      --inject-drop "$(drop_of $seed)" --output "$OUT/s$seed-$arm" > "$OUT/s$seed-$arm.log" 2>&1 &
  done
  wait
done
echo "$(date +%T) done" >> "$OUT/cohort.log"
