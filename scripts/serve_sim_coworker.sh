#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
PYTHON="${UGRP_PYTHON:-$ROOT/.venv-sim/bin/python}"
[[ -x "$PYTHON" ]] || PYTHON=python3
export UGRP_SIM_BRIDGE_LOCAL="${UGRP_SIM_BRIDGE_LOCAL:-http://127.0.0.1:8091}"
export UGRP_UI_EXECUTE_DEFAULT="${UGRP_UI_EXECUTE_DEFAULT:-1}"
export UGRP_SIM_SPEED_CONTROL="1"
export UGRP_CAMERA_SELF_MASK="masterpi_eye_in_hand"
export UGRP_PLANNER_IMAGE_MAX_EDGE="${UGRP_PLANNER_IMAGE_MAX_EDGE:-320}"
export UGRP_LLM_BACKEND="${UGRP_LLM_BACKEND:-gemini}"
export UGRP_LLM_MODEL="${UGRP_LLM_MODEL:-gemini-3.7-flash}"
export GEMINI_PROXY_REQUEST_TIMEOUT="${GEMINI_PROXY_REQUEST_TIMEOUT:-45}"
# SIM and REAL must present the same planner loop to the LLM.
# Keep the deterministic structured executor opt-in only for explicit tests/debug.
export UGRP_STRUCTURED_RED_FASTPATH="${UGRP_STRUCTURED_RED_FASTPATH:-0}"
export UGRP_MULTI_LLM_AGENTS="${UGRP_MULTI_LLM_AGENTS:-1}"
export UGRP_WAREHOUSE_MODE="${UGRP_WAREHOUSE_MODE:-llm_peer_comm}"
export UGRP_WAREHOUSE_ARCHITECTURE="${UGRP_WAREHOUSE_ARCHITECTURE:-coela}"
export UGRP_WAREHOUSE_LAYOUT="${UGRP_WAREHOUSE_LAYOUT:-mixed}"
export UGRP_SIM_SPEED_FILE="${UGRP_SIM_SPEED_FILE:-/tmp/ugrp_sim_speed}"
# One completion already rotates across every configured key and then the
# fallback model. Repeating whole provider rounds can turn a quota miss into a
# multi-minute robot turn without adding new physical evidence.
export GROQ_RATE_LIMIT_RETRIES="${GROQ_RATE_LIMIT_RETRIES:-0}"
export GROQ_PLANNER_CALL_BUDGET="${GROQ_PLANNER_CALL_BUDGET:-20}"
export GROQ_TPM_MATRIX_RETRIES="${GROQ_TPM_MATRIX_RETRIES:-1}"
export GROQ_TPM_MATRIX_MAX_WAIT="${GROQ_TPM_MATRIX_MAX_WAIT:-15}"
export GROQ_REQUEST_TIMEOUT="${GROQ_REQUEST_TIMEOUT:-8}"
export GROQ_SHARED_KEY_ROTATION="${GROQ_SHARED_KEY_ROTATION:-1}"
export GROQ_ROBOT_MODEL_SHARDING="${GROQ_ROBOT_MODEL_SHARDING:-1}"
[[ -f "$UGRP_SIM_SPEED_FILE" ]] || printf "1" > "$UGRP_SIM_SPEED_FILE"

# Child mode is one independent agent/chat state pointed at one camera namespace
# of the same shared MuJoCo world. R1 retains the historical public port 8082.
if [[ "${UGRP_MULTI_ROBOT_CHILD:-0}" == "1" ]]; then
  RID="${UGRP_ROBOT_ID:-r1}"
  PORT="${PORT:?PORT required for SIM child}"
  export UGRP_UI_MODE=sim
  CAMERA_URL="$UGRP_SIM_BRIDGE_LOCAL/robot/$RID/snapshot"
  # Backward-compatible R1 camera while the authoritative GPU is still the
  # single-robot worker. R2/R3 remain offline until the 3x worker publishes
  # namespaced frames; do not fake them from R1.
  if [[ "$RID" == "r1" ]] && ! curl -fsS --max-time 1 "$CAMERA_URL" -o /dev/null 2>/dev/null; then
    CAMERA_URL="$UGRP_SIM_BRIDGE_LOCAL/snapshot"
  fi
  exec "$PYTHON" -m harness --chat --backend "$UGRP_LLM_BACKEND" --model "$UGRP_LLM_MODEL" --host 127.0.0.1 --port "$PORT" --execute \
    --robot-id "$RID" \
    --camera-url "$CAMERA_URL" \
    --actions "$ROOT/scripts/sim_actions.py" --max-steps 16
fi

# If a remote GPU is already authoritative, wait only for R1's first namespaced
# frame. R2/R3 children can start immediately and will surface offline previews
# until the same worker publishes their round-robin frames.
if curl -fsS --max-time 1 "$UGRP_SIM_BRIDGE_LOCAL/health" 2>/dev/null | "$PYTHON" -c 'import json,sys; o=json.load(sys.stdin); raise SystemExit(0 if o.get("remote_authoritative") else 1)' 2>/dev/null; then
  for _ in $(seq 1 40); do
    if curl -fsS --max-time 1 "$UGRP_SIM_BRIDGE_LOCAL/robot/r1/snapshot" -o /dev/null; then break; fi
    sleep 0.25
  done
fi

pids=()
start_child() {
  local rid="$1" port="$2"
  env UGRP_MULTI_ROBOT_CHILD=1 UGRP_ROBOT_ID="$rid" PORT="$port" "$0" &
  pids+=("$!")
}
cleanup() {
  trap - TERM INT EXIT
  ((${#pids[@]})) && kill "${pids[@]}" 2>/dev/null || true
  wait 2>/dev/null || true
}
trap cleanup TERM INT EXIT
start_child r1 "${UGRP_SIM_R1_PORT:-8082}"
start_child r2 "${UGRP_SIM_R2_PORT:-8084}"
start_child r3 "${UGRP_SIM_R3_PORT:-8085}"
# macOS still ships Bash 3.2, which has no `wait -n`. Poll the three child
# PIDs so the supervisor remains portable while still terminating the sibling
# agents if any one backend exits.
while true; do
  for pid in "${pids[@]}"; do
    if ! kill -0 "$pid" 2>/dev/null; then
      wait "$pid"
      status=$?
      cleanup
      exit "$status"
    fi
  done
  sleep 0.25
done
