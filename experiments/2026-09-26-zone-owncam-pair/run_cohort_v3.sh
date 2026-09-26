#!/bin/zsh
# Pre-registered pair v3 cohort: hold check fullframe_v3; arms ON/OFF (--status-channel).
# Carry seeds 631-636 (no injection) + deliberate-drop seeds 641-644 (experimenter opens one gripper). 2 lanes.
set -u
WT=/Users/changmin/projects/ugrp-wt/zone-owncam-pair
cd $WT
SHA=$(git rev-parse --short HEAD)
OUT=/Users/changmin/projects/ugrp/outputs/zone-owncam-pair-20260926/cohort-v3-$SHA
mkdir -p $OUT
LOG=$OUT/cohort.log
source /Users/changmin/projects/ugrp/.venv-sim-worker-mac/bin/activate
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1 MKL_NUM_THREADS=1
typeset -A DROP
DROP=(641 r1:3.0 642 r2:3.0 643 r1:6.0 644 r2:6.0)

run1() {
  local arm=$1 chan=$2 seed=$3
  local extra=()
  [ -n "${DROP[$seed]:-}" ] && extra=(--inject-drop ${DROP[$seed]})
  mkdir -p $OUT/$arm
  echo "$(date -u +%FT%TZ) arm=$arm seed=$seed start load=$(sysctl -n vm.loadavg)" >> $LOG
  python3 scripts/study_owncam_pair_beam.py --seed $seed --condition own_only --status-channel $chan \
    --hold-check fullframe_v3 --contact-profile cargo_noslip_v1 $extra --output $OUT/$arm/$seed > $OUT/$arm/$seed.log 2>&1
  echo "$(date -u +%FT%TZ) arm=$arm seed=$seed end rc=$? load=$(sysctl -n vm.loadavg) $(tail -1 $OUT/$arm/$seed.log | cut -c1-300)" >> $LOG
}

echo "$(date -u +%FT%TZ) pair v3 cohort source=$SHA dirty=$(git status --porcelain | wc -l | tr -d ' ')" >> $LOG
( for s in 631 632 633 634 635 636 641 642 643 644; do run1 ON on $s; done ) &
( for s in 631 632 633 634 635 636 641 642 643 644; do run1 OFF off $s; done ) &
wait
echo "$(date -u +%FT%TZ) cohort done" >> $LOG
