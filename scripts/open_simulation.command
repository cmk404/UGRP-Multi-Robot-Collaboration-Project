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
cd "$project_dir"
# No arguments opens terminal mode selection plus the native research scene.
if [[ $# -eq 0 ]]; then set -- start; fi
runner="$sim_python"
if [[ "$(uname -s)" == Darwin && ( "$1" == run || "$1" == console || "$1" == start || "$1" == dispatch || "$1" == replay ) ]]; then
  headless=false
  isolated_viewer=false
  for arg in "$@"; do
    [[ "$arg" == --headless ]] && headless=true
    [[ "$1" == dispatch && "$arg" == --realtime-control ]] && isolated_viewer=true
  done
  # Realtime dispatch starts its own mjpython observer. Its physics owner must
  # stay in ordinary Python instead of creating a second Cocoa application.
  if [[ "$headless" == false && "$isolated_viewer" == false ]]; then
    runner="$(dirname -- "$sim_python")/mjpython"
    if [[ ! -x "$runner" ]]; then
      echo 'MuJoCo macOS viewer needs mjpython next to the selected Python. Install requirements-sim.txt.' >&2
      exit 1
    fi
  fi
fi
exec "$sim_python" scripts/ugrp_session.py run "simulation-native-$$" -- \
  "$runner" -m scripts.sim_cli "$@"
