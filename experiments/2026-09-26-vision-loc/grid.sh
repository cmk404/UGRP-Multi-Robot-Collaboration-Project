#!/bin/bash
# Offline PF grid (no simulation): localize + score one config over episodes, at most N configs in parallel.
# Usage: grid.sh <out root> <calibration> <obs dir|-> <oracle obs dir|-> <filters> "<episodes>" <cfg1.json> [cfg2.json ...]
set -euo pipefail
out=$1; cal=$2; obs=$3; orc=$4; filters=$5; eps=$6; shift 6
HERE=$(cd "$(dirname "$0")" && pwd)
PY=${VL_PY:-/Users/changmin/projects/ugrp/.venv-sim-worker-mac/bin/python}
MAXJ=${VL_JOBS:-4}
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1 MKL_NUM_THREADS=1
mkdir -p "$out"
run() {
  cfg=$1; name=$(basename "$cfg" .json); d="$out/$name"
  extra=()
  [ "$obs" != "-" ] && extra+=(--obs "$obs")
  [ "$orc" != "-" ] && extra+=(--oracle-obs "$orc")
  echo "$(date -u +%FT%TZ) start $name $(uptime)" >> "$out/grid_load.txt"
  "$PY" "$HERE/vision_loc_cli.py" localize --episodes $eps --calibration "$cal" --config "$cfg" --filters "$filters" \
    "${extra[@]}" --output "$d" > "$d.log" 2>&1
  "$PY" "$HERE/vision_loc_cli.py" score --episodes $eps --estimates "$d" "${extra[@]}" --output "$d/metrics.json" > "$d.score" 2>&1
  echo "$(date -u +%FT%TZ) end $name $(uptime)" >> "$out/grid_load.txt"
}
for cfg in "$@"; do
  while [ "$(jobs -rp | wc -l)" -ge "$MAXJ" ]; do sleep 5; done
  run "$cfg" &
done
wait
