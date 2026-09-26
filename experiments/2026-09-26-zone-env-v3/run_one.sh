#!/bin/bash
# Launch ONE pre-registered environment-v3 episode as an owned session (thread caps, disk check, load log).
# Usage: run_one.sh <loop|m1|loop-plain|m1-plain> <prereg.json> <episode_id> <output root> [runner args]
#   loop / m1           : through observe_top.py (evaluation-only TOP frames under <episode>/eval_only/top/)
#   loop-plain/m1-plain : the unchanged runner directly (determinism check of the wrapper)
set -euo pipefail
kind=$1; prereg=$2; ep=$3; out=$4; shift 4
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
PY=/Users/changmin/projects/ugrp/.venv-sim-worker-mac/bin/python
free=$(df -g /System/Volumes/Data | tail -1 | awk '{print $4}')
mkdir -p "$out"
if [ "$free" -lt 30 ]; then
  echo "$(date -u +%FT%TZ) $ep REFUSED disk free ${free} GiB < 30" | tee -a "$out/launch_load.txt"
  exit 4
fi
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1 MKL_NUM_THREADS=1
echo "$(date -u +%FT%TZ) start $ep kind=$kind free=${free}GiB head=$(git -C "$ROOT" rev-parse --short HEAD) $(uptime)" >> "$out/launch_load.txt"
cd "$ROOT"
case "$kind" in
  loop)       cmd=("$PY" experiments/2026-09-26-zone-env-v3/observe_top.py loop) ;;
  m1)         cmd=("$PY" experiments/2026-09-26-zone-env-v3/observe_top.py m1) ;;
  loop-plain) cmd=("$PY" scripts/run_owncam_closed_loop.py) ;;
  m1-plain)   cmd=("$PY" scripts/run_m1_owncam.py) ;;
  *) echo "unknown kind $kind"; exit 2 ;;
esac
status=0
python3 scripts/ugrp_session.py run "kiro-env3-$ep" -- "${cmd[@]}" --prereg "$prereg" --only "$ep" --output "$out" "$@" \
  > "$out/$ep.log" 2>&1 || status=$?
echo "$(date -u +%FT%TZ) end $ep status=$status free=$(df -g /System/Volumes/Data | tail -1 | awk '{print $4}')GiB $(uptime)" >> "$out/launch_load.txt"
exit $status
