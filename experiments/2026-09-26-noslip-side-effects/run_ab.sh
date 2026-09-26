#!/bin/bash
# 2026-09-26 cargo_noslip_v1 부작용 감사 A/B 드라이버.
# 동기 SIM, 스레드 1, 레인 2개(동시 SIM 2개 이하). 실행마다 부하 평균은 result.json에 들어간다.
# 사용: bash experiments/2026-09-26-noslip-side-effects/run_ab.sh <출력 루트>
set -u
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
OUT="${1:-outputs/noslip-audit/$(git -C "$ROOT" rev-parse --short HEAD)}"
PY="$ROOT/.venv-sim-worker-mac/bin/python"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1 MKL_NUM_THREADS=1
export PYTHONPATH="$ROOT"
mkdir -p "$ROOT/$OUT"
LOG="$ROOT/$OUT/driver.log"

# 싼 것부터, 비싼 유지 시나리오는 뒤로. 레인은 홀짝으로 나눈다.
SCENARIOS="drive_commands wall_push arm_sweep idle_settle solo_carry solo_hold_load pair_beam_hold"
SEEDS="11 12 13"
JOBS=()
for s in $SCENARIOS; do
  for seed in $SEEDS; do
    for p in local_contact_fine cargo_noslip_v1; do
      JOBS+=("$s|$p|$seed")
    done
  done
done

run_lane() {
  local lane="$1"
  local i=0
  for job in "${JOBS[@]}"; do
    if [ $((i % 2)) -eq "$lane" ]; then
      IFS='|' read -r sc pr sd <<< "$job"
      local dir="$OUT/$sc-$pr-$sd"
      if [ -f "$ROOT/$dir/result.json" ]; then
        echo "[lane$lane] skip $sc $pr $sd (already done)" >> "$LOG"
      else
        echo "[lane$lane] $(date -u +%FT%TZ) start $sc $pr $sd load=$(uptime | sed 's/.*averages: //')" >> "$LOG"
        (cd "$ROOT" && "$PY" -m scripts.audit_contact_profiles --scenario "$sc" \
            --contact-profile "$pr" --seed "$sd" --output "$dir" >> "$LOG" 2>&1)
        echo "[lane$lane] $(date -u +%FT%TZ) exit=$? $sc $pr $sd load=$(uptime | sed 's/.*averages: //')" >> "$LOG"
      fi
    fi
    i=$((i + 1))
  done
}

echo "=== $(date -u +%FT%TZ) audit A/B start: ${#JOBS[@]} runs -> $OUT" >> "$LOG"
run_lane 0 &
L0=$!
run_lane 1 &
L1=$!
wait "$L0" "$L1"
echo "=== $(date -u +%FT%TZ) audit A/B done" >> "$LOG"
ls "$ROOT/$OUT" | wc -l
