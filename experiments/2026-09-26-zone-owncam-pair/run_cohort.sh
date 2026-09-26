#!/bin/zsh
# Pre-registered pair feasibility cohort: O (own_only) 611-614, S (stub_approach) 611-612. 2 lanes, thread caps.
set -u
WT=/Users/changmin/projects/ugrp-wt/zone-owncam-pair
cd $WT
SHA=$(git rev-parse --short HEAD)
OUT=/Users/changmin/projects/ugrp/outputs/zone-owncam-pair-20260926/cohort-$SHA
mkdir -p $OUT
LOG=$OUT/cohort.log
source /Users/changmin/projects/ugrp/.venv-sim-worker-mac/bin/activate
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1 MKL_NUM_THREADS=1

run1() {
  local arm=$1 cond=$2 seed=$3
  mkdir -p $OUT/$arm
  echo "$(date -u +%FT%TZ) arm=$arm seed=$seed start load=$(sysctl -n vm.loadavg)" >> $LOG
  python3 scripts/study_owncam_pair_beam.py --seed $seed --condition $cond --contact-profile cargo_noslip_v1 \
    --output $OUT/$arm/$seed > $OUT/$arm/$seed.log 2>&1
  echo "$(date -u +%FT%TZ) arm=$arm seed=$seed end rc=$? load=$(sysctl -n vm.loadavg) $(tail -1 $OUT/$arm/$seed.log | cut -c1-260)" >> $LOG
}

echo "$(date -u +%FT%TZ) pair cohort source=$SHA dirty=$(git status --porcelain | wc -l | tr -d ' ')" >> $LOG
( run1 O own_only 611; run1 O own_only 613; run1 S stub_approach 611 ) &
( run1 O own_only 612; run1 O own_only 614; run1 S stub_approach 612 ) &
wait
echo "$(date -u +%FT%TZ) cohort done" >> $LOG
