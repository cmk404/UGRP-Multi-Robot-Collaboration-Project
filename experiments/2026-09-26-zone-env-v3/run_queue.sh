#!/bin/bash
# Run pre-registered episodes one after another (one lane). Two lanes at most run at a time.
# Usage: run_queue.sh <kind> <prereg.json> <output root> [--wait-session NAME] ep1 ep2 ...
set -uo pipefail
kind=$1; prereg=$2; out=$3; shift 3
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/../.." && pwd)
if [ "${1:-}" = "--wait-session" ]; then
  wait_for=$2; shift 2
  while python3 "$ROOT/scripts/ugrp_session.py" status "$wait_for" | grep -q running; do sleep 15; done
fi
for ep in "$@"; do
  "$HERE/run_one.sh" "$kind" "$prereg" "$ep" "$out"
  status=$?
  if [ $status -eq 4 ]; then
    echo "$(date -u +%FT%TZ) queue stopped: disk below 30 GiB before $ep" >> "$out/launch_load.txt"
    exit 4
  fi
done
