#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
export UGRP_UI_MODE=real
export UGRP_UI_EXECUTE_DEFAULT="${UGRP_UI_EXECUTE_DEFAULT:-1}"
export UGRP_CAMERA_SELF_MASK="${UGRP_CAMERA_SELF_MASK:-masterpi_eye_in_hand}"
export UGRP_PLANNER_IMAGE_MAX_EDGE="${UGRP_PLANNER_IMAGE_MAX_EDGE:-320}"
export UGRP_STRUCTURED_RED_FASTPATH="${UGRP_STRUCTURED_RED_FASTPATH:-0}"
export UGRP_MULTI_LLM_AGENTS="${UGRP_MULTI_LLM_AGENTS:-1}"
export GROQ_RATE_LIMIT_RETRIES="${GROQ_RATE_LIMIT_RETRIES:-0}"
export GROQ_PLANNER_CALL_BUDGET="${GROQ_PLANNER_CALL_BUDGET:-20}"
export GROQ_TPM_MATRIX_RETRIES="${GROQ_TPM_MATRIX_RETRIES:-1}"
export GROQ_TPM_MATRIX_MAX_WAIT="${GROQ_TPM_MATRIX_MAX_WAIT:-15}"
export GROQ_REQUEST_TIMEOUT="${GROQ_REQUEST_TIMEOUT:-8}"
export GROQ_SHARED_KEY_ROTATION="${GROQ_SHARED_KEY_ROTATION:-1}"
export GROQ_ROBOT_MODEL_SHARDING="${GROQ_ROBOT_MODEL_SHARDING:-1}"
export UGRP_REAL_TRACE_ENABLE="${UGRP_REAL_TRACE_ENABLE:-1}"
export UGRP_REAL_TRACE_ALL_FRAMES="${UGRP_REAL_TRACE_ALL_FRAMES:-1}"
export UGRP_REAL_TRACE_JPEG_QUALITY="${UGRP_REAL_TRACE_JPEG_QUALITY:-55}"
export UGRP_REAL_TRACE_FRAME_INTERVAL_S="${UGRP_REAL_TRACE_FRAME_INTERVAL_S:-0.25}"

if [[ "${UGRP_MULTI_ROBOT_CHILD:-0}" == "1" ]]; then
  RID="${UGRP_ROBOT_ID:?UGRP_ROBOT_ID required}"
  ROBOT_HOST="${UGRP_ROBOT_HOST:?UGRP_ROBOT_HOST required}"
  PORT="${PORT:?PORT required}"
  export UGRP_ROBOT_USER="${UGRP_ROBOT_USER:-ugrp1}"

  run_harness() {
    exec python3 -m harness --chat --backend groq --host 127.0.0.1 --port "$PORT" \
      "$@" --robot "$ROBOT_HOST" --robot-id "$RID" \
      --actions scripts/robot_actions.py --max-steps 16
  }

  # A disconnected REAL slot still gets a live chat/status process, but no
  # actuator execution permission. This prevents a configured-but-offline slot
  # from being treated as a valid command target.
  if [[ "${UGRP_REAL_PLACEHOLDER:-0}" == "1" ]]; then
    run_harness --no-camera
  elif python3 - "$ROBOT_HOST" <<'PYPROBE' >/dev/null 2>&1
import sys
from harness.pi_camera import grab_snapshot
grab_snapshot(host=sys.argv[1])
PYPROBE
  then
    run_harness --execute
  elif [[ "${UGRP_REAL_FORCE_EXECUTE:-0}" == "1" ]]; then
    run_harness --execute --no-camera
  else
    run_harness --no-camera
  fi
fi

# REAL robot identities are provided by the service environment. Keep unknown
# slots disabled rather than inventing a hostname: an accidental alias match
# could turn a UI click into motion on the wrong machine.
r1_host="${UGRP_REAL_ROBOT_1:-ugrp1}"
r2_host="${UGRP_REAL_ROBOT_2:-}"
r3_host="${UGRP_REAL_ROBOT_3:-}"
r1_enabled="${UGRP_REAL_R1_ENABLED:-1}"
r2_enabled="${UGRP_REAL_R2_ENABLED:-0}"
r3_enabled="${UGRP_REAL_R3_ENABLED:-0}"
default_robot_user="${UGRP_ROBOT_USER:-ugrp1}"
r1_user="${UGRP_REAL_ROBOT_1_USER:-$default_robot_user}"
r2_user="${UGRP_REAL_ROBOT_2_USER:-$default_robot_user}"
r3_user="${UGRP_REAL_ROBOT_3_USER:-$default_robot_user}"
pids=()
start_child() {
  local rid="$1" host="$2" port="$3" enabled="$4" robot_user="$5"
  if [[ "$enabled" != "1" || -z "$host" ]]; then
    rid_upper="$(printf '%s' "$rid" | tr '[:lower:]' '[:upper:]')"
    printf 'REAL %s offline placeholder (configure host + enable flag for hardware)\n' "$rid_upper" >&2
    # .invalid can never resolve to a physical robot. The child also receives
    # UGRP_REAL_PLACEHOLDER=1, which prevents --execute from being enabled.
    env UGRP_MULTI_ROBOT_CHILD=1 UGRP_REAL_PLACEHOLDER=1 UGRP_ROBOT_ID="$rid" \
      UGRP_ROBOT_HOST="offline-${rid}.invalid" UGRP_ROBOT_USER="$robot_user" PORT="$port" "$0" &
    pids+=("$!")
    return 0
  fi
  env UGRP_MULTI_ROBOT_CHILD=1 UGRP_ROBOT_ID="$rid" UGRP_ROBOT_HOST="$host" \
    UGRP_ROBOT_USER="$robot_user" PORT="$port" "$0" &
  pids+=("$!")
}
cleanup() {
  trap - TERM INT EXIT
  ((${#pids[@]})) && kill "${pids[@]}" 2>/dev/null || true
  wait 2>/dev/null || true
}
trap cleanup TERM INT EXIT
start_child r1 "$r1_host" "${UGRP_REAL_R1_PORT:-${PORT:-8083}}" "$r1_enabled" "$r1_user"
start_child r2 "$r2_host" "${UGRP_REAL_R2_PORT:-8086}" "$r2_enabled" "$r2_user"
start_child r3 "$r3_host" "${UGRP_REAL_R3_PORT:-8087}" "$r3_enabled" "$r3_user"
if ((${#pids[@]} == 0)); then
  printf 'No REAL robot slots enabled.\n' >&2
  exit 2
fi
# `wait -n` requires Bash 4.3+, while macOS still ships Bash 3.2.
# Poll child liveness portably and reap the first child that exits.
while :; do
  for pid in "${pids[@]}"; do
    if ! kill -0 "$pid" 2>/dev/null; then
      if wait "$pid"; then status=0; else status=$?; fi
      cleanup
      exit "$status"
    fi
  done
  sleep 1
done
