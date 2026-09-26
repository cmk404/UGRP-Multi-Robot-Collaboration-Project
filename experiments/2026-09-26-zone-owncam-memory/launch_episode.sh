#!/bin/bash
# Launch one memory ON/OFF episode under a UGRP session (records the load average and free disk first).
#   launch_episode.sh <attempt> <condition> <episode_id> <split> [frozen_source.json]
# Output: $ROOT_OUT/<attempt>/<condition>/<episode>; log: $ROOT_OUT/logs/<attempt>-<condition>-<seed>.log
set -euo pipefail
ATTEMPT=$1; COND=$2; EP=$3; SPLIT=$4; FROZEN=${5:-}
HERE=$(cd "$(dirname "$0")" && pwd)
WT=$(cd "$HERE/../.." && pwd)
PY=/Users/changmin/projects/ugrp/.venv-sim-worker-mac/bin/python
ROOT_OUT=${ROOT_OUT:-/Users/changmin/projects/ugrp/outputs/owncam-memory-20260926}
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1 MKL_NUM_THREADS=1
mkdir -p "$ROOT_OUT/logs"
FREE=$(df -g "$ROOT_OUT" | awk 'NR==2 {print $4}')
if [ "$FREE" -lt 30 ]; then echo "refused: ${FREE} GiB free (< 30)" >&2; exit 3; fi
echo "$(date '+%F %T') launch $ATTEMPT $COND $EP sha=$(git -C "$WT" rev-parse --short HEAD) load={$(sysctl -n vm.loadavg | tr -d '{}')} free=${FREE}GiB" >> "$ROOT_OUT/logs/launch_load.txt"
ARGS=(--prereg "$HERE/prereg.json" --condition "$COND" --output "$ROOT_OUT/$ATTEMPT" --only "$EP" --split "$SPLIT")
if [ -n "$FROZEN" ]; then ARGS+=(--frozen "$FROZEN"); fi
cd "$WT"
exec python3 scripts/ugrp_session.py run "kiro-mem-$ATTEMPT-$COND-${EP##*-}" -- "$PY" scripts/run_m1_owncam_memory.py "${ARGS[@]}"
