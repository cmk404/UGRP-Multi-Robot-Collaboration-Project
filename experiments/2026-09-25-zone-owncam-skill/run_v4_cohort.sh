#!/bin/zsh
# v4 pre-registered cohort: arm D (drop safety, 531/533 + drop at carry+15 s) first, then arm P 531-540.
# Both cargo_noslip_v1, 2 lanes. Stop rule: any false OWN_RGB_PLACEMENT_IN_SLOT stops the cohort.
set -u
WT=/Users/changmin/projects/ugrp-wt/zone-owncam-skill-v2
cd $WT
SHA=$(git rev-parse --short HEAD)
OUT=/Users/changmin/projects/ugrp/outputs/zone-owncam-skill-20260925/cohort-v4-$SHA
mkdir -p $OUT
LOG=$OUT/cohort.log
source /Users/changmin/projects/ugrp/.venv-sim-worker-mac/bin/activate
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1 MKL_NUM_THREADS=1
STOP=$OUT/STOP

run1() {
  local arm=$1 seed=$2; shift 2
  [ -f $STOP ] && { echo "$(date -u +%FT%TZ) arm=$arm seed=$seed SKIPPED (stop rule)" >> $LOG; return; }
  mkdir -p $OUT/$arm
  echo "$(date -u +%FT%TZ) arm=$arm seed=$seed start load=$(sysctl -n vm.loadavg)" >> $LOG
  python3 scripts/run_zone_owncam_skill.py --profile v4 --seed $seed --contact-profile cargo_noslip_v1 "$@" \
    --output $OUT/$arm/$seed > $OUT/$arm/$seed.log 2>&1
  echo "$(date -u +%FT%TZ) arm=$arm seed=$seed end rc=$? load=$(sysctl -n vm.loadavg) $(tail -1 $OUT/$arm/$seed.log | cut -c1-200)" >> $LOG
  if [ -f $OUT/$arm/$seed/result.json ] && python3 -c "
import json,sys;e=json.load(open('$OUT/$arm/$seed/result.json'))['evaluation_only']
sys.exit(0 if (e['skill_claim_in_slot'] and not e['place_in_slot_gt']) else 1)"; then
    echo "$(date -u +%FT%TZ) STOP RULE: false IN_SLOT arm=$arm seed=$seed" >> $LOG; touch $STOP
  fi
}

echo "$(date -u +%FT%TZ) cohort v4 source=$SHA dirty=$(git status --porcelain | wc -l | tr -d ' ')" >> $LOG
run1 D 531 --inject-drop-after-carry-s 15 &
run1 D 533 --inject-drop-after-carry-s 15 &
wait
( for s in 531 533 535 537 539; do run1 P $s; done ) &
( for s in 532 534 536 538 540; do run1 P $s; done ) &
wait
echo "$(date -u +%FT%TZ) cohort done" >> $LOG
