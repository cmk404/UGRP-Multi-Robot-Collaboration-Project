#!/usr/bin/env bash
# Activate Oracle-side REAL spatial stack only when MasterPi actuator ownership is free.
# It never kills a competing controller and never sends motor/servo/STOP commands.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ROBOT="${UGRP_ROBOT:-ugrp1}"
ROBOT_HOST="${UGRP_ROBOT_HOST:-100.119.44.65}"
ROBOT_USER="${UGRP_ROBOT_USER:-ugrp1}"
ROBOT_HOSTKEY_ALIAS="${UGRP_ROBOT_HOSTKEY_ALIAS:-ugrp1.local}"
CHECK_ONLY=0
if [[ "${1:-}" == "--check-only" ]]; then CHECK_ONLY=1; fi

owner_json="$(ssh \
  -o BatchMode=yes \
  -o ConnectTimeout=5 \
  -o StrictHostKeyChecking=accept-new \
  -o User="$ROBOT_USER" \
  -o HostName="$ROBOT_HOST" \
  -o HostKeyAlias="$ROBOT_HOSTKEY_ALIAS" \
  "$ROBOT" '
python3 - <<"PY"
import fcntl,json,os
p="/tmp/ugrp-masterpi-actuator.lock"
h=open(p,"a+")
try:
    fcntl.flock(h.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
except BlockingIOError:
    h.seek(0); print(h.read().strip() or "{\"busy\":true}")
    raise SystemExit(3)
else:
    print("{\"free\":true}")
finally:
    h.close()
PY
' 2>/dev/null)" || rc=$?
rc="${rc:-0}"
if [[ "$rc" -eq 3 ]]; then
  echo "REAL spatial activation deferred: MasterPi actuator lease is busy: $owner_json" >&2
  exit 3
elif [[ "$rc" -ne 0 ]]; then
  echo "REAL spatial activation deferred: could not verify MasterPi actuator lease" >&2
  exit "$rc"
fi

echo "MasterPi actuator lease: free"
if [[ "$CHECK_ONLY" -eq 1 ]]; then exit 0; fi
cd "$ROOT"

# Preserve only conservative carry evidence across this deliberate harness
# restart. Losing PROBABLE_HELD here could authorize a second acquisition while
# the physical gripper still contains a block. Vision/planner state is not
# persisted. The new REAL process consumes this file exactly once.
RESUME_GRASP_PATH="${UGRP_REAL_RESUME_GRASP_PATH:-/tmp/ugrp-real-resume-grasp.json}"
rm -f "$RESUME_GRASP_PATH"
if status_json="$(curl -fsS --max-time 1 http://127.0.0.1:8083/api/status 2>/dev/null)"; then
  carry_json="$(./.venv-sim/bin/python -m harness.real_carry --host "$ROBOT_HOST" 2>/dev/null || true)"
  STATUS_JSON="$status_json" CARRY_JSON="$carry_json" RESUME_GRASP_PATH="$RESUME_GRASP_PATH" python3 - <<'PY'
import json, os
from pathlib import Path
try:
    status=json.loads(os.environ.get("STATUS_JSON", "{}"))
except ValueError:
    status={}
try:
    carry=json.loads(os.environ.get("CARRY_JSON", "{}"))
except ValueError:
    carry={}
grasp=((status.get("world_state") or {}).get("grasp") or {})
state=str(grasp.get("state") or "").upper()
color=str(grasp.get("held_object_color") or "").lower()
# Positive HELD can survive a reload. Weak PROBABLE_HELD survives only while
# its Pi-side causal precision-pick handoff is still fresh and same-colour.
preserve = state == "HELD" or (
    state == "PROBABLE_HELD"
    and carry.get("known") is True
    and carry.get("valid") is True
    and str(carry.get("target_color") or "").lower() == color
)
if preserve and color in {"red", "blue", "yellow"}:
    payload={
        "state": state,
        "held_object_color": color,
        "confidence": grasp.get("confidence"),
        "visual_support": grasp.get("visual_support"),
        "held_streak": grasp.get("held_streak"),
    }
    path=Path(os.environ["RESUME_GRASP_PATH"])
    path.write_text(json.dumps(payload), encoding="utf-8")
    path.chmod(0o600)
PY
fi

# Oracle service restart only. Do not modify or restart Pi-side camera/controller services here.
systemctl --user restart ugrp-real.service
# Startup includes camera/SSH initialization and can legitimately take several
# seconds. Wait for the actual HTTP readiness condition instead of assuming a
# fixed two-second boot time.
ready=0
for _ in $(seq 1 20); do
  if curl -fsS --max-time 1 http://127.0.0.1:8083/api/status >/dev/null 2>&1; then
    ready=1
    break
  fi
  sleep 0.5
done
if [[ "$ready" -ne 1 ]]; then
  rm -f "$RESUME_GRASP_PATH"
  echo "REAL spatial activation failed: harness did not become HTTP-ready" >&2
  exit 4
fi
rm -f "$RESUME_GRASP_PATH"
echo "REAL spatial stack activated on Oracle; no MasterPi actuator command sent"
