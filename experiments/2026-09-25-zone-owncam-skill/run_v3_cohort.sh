#!/bin/zsh
# v3 pre-registered cohort driver: arm P (cargo_noslip_v1) then arm S (local_contact_fine), 2 lanes.
# Stop rule: any false OWN_RGB_PLACEMENT_IN_SLOT stops the cohort.
set -u
WT=/Users/changmin/projects/ugrp-wt/zone-owncam-skill-v2
cd $WT
SHA=$(git rev-parse --short HEAD)
OUT=/Users/changmin/projects/ugrp/outputs/zone-owncam-skill-20260925/cohort-v3-$SHA
mkdir -p $OUT
LOG=$OUT/cohort.log
source /Users/changmin/projects/ugrp/.venv-sim-worker-mac/bin/activate
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1 MKL_NUM_THREADS=1
STOP=$OUT/STOP

lane() {
  local arm=$1 profile=$2; shift 2
  for seed in "$@"; do
    [ -f $STOP ] && { echo "$(date -u +%FT%TZ) arm=$arm seed=$seed SKIPPED (stop rule)" >> $LOG; continue; }
    mkdir -p $OUT/$arm
    echo "$(date -u +%FT%TZ) arm=$arm seed=$seed start load=$(sysctl -n vm.loadavg)" >> $LOG
    python3 scripts/run_zone_owncam_skill.py --profile v3 --seed $seed --contact-profile $profile \
      --output $OUT/$arm/$seed > $OUT/$arm/$seed.log 2>&1
    rc=$?
    echo "$(date -u +%FT%TZ) arm=$arm seed=$seed end rc=$rc load=$(sysctl -n vm.loadavg) $(tail -1 $OUT/$arm/$seed.log | cut -c1-200)" >> $LOG
    if [ -f $OUT/$arm/$seed/result.json ] && python3 -c "
import json,sys;e=json.load(open('$OUT/$arm/$seed/result.json'))['evaluation_only']
sys.exit(0 if (e['skill_claim_in_slot'] and not e['place_in_slot_gt']) else 1)"; then
      echo "$(date -u +%FT%TZ) STOP RULE: false IN_SLOT arm=$arm seed=$seed" >> $LOG; touch $STOP
    fi
  done
}

echo "$(date -u +%FT%TZ) cohort v3 source=$SHA dirty=$(git status --porcelain | wc -l | tr -d ' ')" >> $LOG
lane P cargo_noslip_v1 521 523 525 527 529 &
lane P cargo_noslip_v1 522 524 526 528 530 &
wait
lane S local_contact_fine 521 523 525 527 529 &
lane S local_contact_fine 522 524 526 528 530 &
wait
echo "$(date -u +%FT%TZ) cohort done" >> $LOG
