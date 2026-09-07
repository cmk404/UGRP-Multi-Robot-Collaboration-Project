#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
python -m pip install -q -r requirements.txt
if [[ -e /dev/nvidia0 ]] || command -v nvidia-smi >/dev/null 2>&1; then
  export MUJOCO_GL=egl
else
  # CPU Studio can render headlessly through Mesa.
  export MUJOCO_GL=osmesa
fi
exec python server.py --host 0.0.0.0 --port "${PORT:-7860}" --seed "${UGRP_SIM_SEED:-11}"
