#!/bin/bash
# macOS: Finder double-click. Ubuntu/WSL: bash scripts/open_simulation.command
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "$0")" && pwd)"
project_dir="$(dirname -- "$script_dir")"
git_dir="$(git -C "$project_dir" rev-parse --path-format=absolute --git-common-dir)"
primary_dir="$(dirname -- "$git_dir")"
sim_python="${UGRP_SIM_PYTHON:-}"
if [[ -z "$sim_python" ]]; then
  for candidate in "$project_dir/.venv-sim-worker-mac/bin/python" "$project_dir/.venv-dev/bin/python" "$primary_dir/.venv-sim-worker-mac/bin/python" "$primary_dir/.venv-dev/bin/python"; do
    if [[ -x "$candidate" ]]; then sim_python="$candidate"; break; fi
  done
fi
if [[ ! -x "$sim_python" ]]; then
  echo '시뮬레이션 Python 환경이 없습니다. docs/local_simulation.md의 최초 설치를 진행하세요.' >&2
  echo '기존 환경은 UGRP_SIM_PYTHON=/absolute/path/bin/python으로 지정할 수 있습니다.' >&2
  exit 1
fi
if [[ "$(uname -s)" == Linux ]]; then
  export MUJOCO_GL="${MUJOCO_GL:-osmesa}"
  if [[ "$MUJOCO_GL" == osmesa ]]; then export PYOPENGL_PLATFORM=osmesa; fi
fi
cd "$project_dir"
if ! "$sim_python" -c 'import mujoco, numpy, cv2, PIL, websockets' >/dev/null; then
  echo '의존성을 확인하세요: 선택한 Python으로 -m pip install -r requirements-sim.txt' >&2
  exit 1
fi
exec "$sim_python" scripts/ugrp_session.py run simulation-live -- \
  "$sim_python" -m scripts.sim_live "$@"
