#!/bin/bash
# Launch teacher renders of the tag-free environment v3 as owned sessions.
# Usage: run_render.sh <session-name> <output root> <episode ids (comma-separated)> [run_vl_teacher_render.py args]
# Thread caps, disk >= 30 GiB, machine-wide sim cap (< 6 running sims, re-checked every 60 s), load log.
set -euo pipefail
name=$1; out=$2; eps=$3; shift 3
HERE=$(cd "$(dirname "$0")" && pwd)
PRIMARY=/Users/changmin/projects/ugrp
V3=/Users/changmin/projects/ugrp-wt/kiro-vision-loc-v3src
PY=$PRIMARY/.venv-sim-worker-mac/bin/python
mkdir -p "$out"
count_sims() {
  # The literal pattern of the job rules also matches agent CLIs whose prompt text contains "run_";
  # count only python processes, excluding kiro-cli and the shared TensorBoard server.
  { ps -Ao command | grep "[P]ython.*\(run_\|scripts\.run\|eval_\)" | grep -v -e kiro-cli -e run_tensorboard || true; } | wc -l | tr -d ' '
}
while true; do
  n=$(count_sims)
  if [ "$n" -lt 6 ]; then break; fi
  echo "$(date -u +%FT%TZ) $name WAIT sims=$n $(uptime)" >> "$out/launch_load.txt"
  sleep 60
done
free=$(df -g /System/Volumes/Data | tail -1 | awk '{print $4}')
if [ "$free" -lt 30 ]; then
  echo "$(date -u +%FT%TZ) $name REFUSED disk free ${free} GiB < 30" | tee -a "$out/launch_load.txt"
  exit 4
fi
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1 MKL_NUM_THREADS=1
export UGRP_V3_SOURCE=$V3
echo "$(date -u +%FT%TZ) start $name eps=$eps sims=$(count_sims) free=${free}GiB own=$(git -C "$HERE" rev-parse --short HEAD) v3=$(git -C "$V3" rev-parse --short HEAD) $(uptime)" >> "$out/launch_load.txt"
status=0
cd "$V3"
python3 "$PRIMARY/scripts/ugrp_session.py" run "$name" -- "$PY" "$HERE/run_vl_teacher_render.py" --only "$eps" --output "$out" "$@" \
  > "$out/$name.log" 2>&1 || status=$?
echo "$(date -u +%FT%TZ) end $name status=$status free=$(df -g /System/Volumes/Data | tail -1 | awk '{print $4}')GiB $(uptime)" >> "$out/launch_load.txt"
exit $status
