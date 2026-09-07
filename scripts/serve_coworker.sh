#!/usr/bin/env bash
# Coworker window: scene + allowlisted skills.
# Groq 키가 있으면 Groq 비전, 없으면 로컬 mlx Qwen3-VL.
# Default is dry-run. Pass --execute to actually run robot skills.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
if [[ -f "$ROOT/.groq_keys" || -n "${GROQ_API_KEY:-}" || -n "${GROQ_API_KEYS:-}" ]]; then
  PY="${UGRP_PYTHON:-python3}"
  exec "$PY" -m harness --chat --backend groq --host 127.0.0.1 --port "${PORT:-8080}" "$@"
fi
PY="${MLX_PYTHON:-$HOME/anaconda3/envs/mlx_env/bin/python}"
if [[ ! -x "$PY" ]]; then
  PY="$(conda info --base)/envs/mlx_env/bin/python"
fi
exec "$PY" -m harness --chat --backend mlx --host 127.0.0.1 --port "${PORT:-8080}" "$@"
