#!/bin/zsh
# Pre-registered pair v2 cohort: seeds 621-626, own_only, arms ON/OFF (--status-channel). 2 lanes, thread caps.
set -u
WT=/Users/changmin/projects/ugrp-wt/zone-owncam-pair
cd $WT
SHA=$(git rev-parse --short HEAD)
OUT=/Users/changmin/projects/ugrp/outputs/zone-owncam-pair-20260926/cohort-v2-$SHA
mkdir -p $OUT
LOG=$OUT/cohort.log
source /Users/changmin/projects/ugrp/.venv-sim-worker-mac/bin/activate
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1 MKL_NUM_THREADS=1

run1() {
  local arm=$1 chan=$2 seed=$3
  mkdir -p $OUT/$arm
  echo "$(date -u +%FT%TZ) arm=$arm seed=$seed start load=$(sysctl -n vm.loadavg)" >> $LOG
  python3 scripts/study_owncam_pair_beam.py --seed $seed --condition own_only --status-channel $chan --contact-profile cargo_noslip_v1 \
    --output $OUT/$arm/$seed > $OUT/$arm/$seed.log 2>&1
  echo "$(date -u +%FT%TZ) arm=$arm seed=$seed end rc=$? load=$(sysctl -n vm.loadavg) $(tail -1 $OUT/$arm/$seed.log | cut -c1-260)" >> $LOG
}

echo "$(date -u +%FT%TZ) pair v2 cohort source=$SHA dirty=$(git status --porcelain | wc -l | tr -d ' ')" >> $LOG
( for s in 621 622 623 624 625 626; do run1 ON on $s; done ) &
( for s in 621 622 623 624 625 626; do run1 OFF off $s; done ) &
wait
echo "$(date -u +%FT%TZ) cohort done" >> $LOG
