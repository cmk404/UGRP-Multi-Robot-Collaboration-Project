#!/bin/bash
# Double-click from Finder; stop with Ctrl-C. No background service is installed.
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "$0")" && pwd)"
git_dir="$(git -C "$script_dir" rev-parse --path-format=absolute --git-common-dir)"
project_dir="$(dirname -- "$git_dir")"
review_python="${UGRP_REVIEW_PYTHON:-$project_dir/.venv-sim-worker-mac/bin/python}"
review_logdir="${UGRP_TENSORBOARD_LOGDIR:-$project_dir/outputs/tensorboard}"
if [[ ! -x "$review_python" ]]; then
  review_python="$project_dir/.venv-dev/bin/python"
fi
if [[ ! -x "$review_python" ]]; then
  echo 'Python 환경을 찾지 못했습니다. UGRP_REVIEW_PYTHON을 지정하세요.'
  exit 1
fi
exec "$review_python" "$script_dir/ugrp_session.py" run tensorboard-review -- \
  "$review_python" "$script_dir/run_tensorboard.py" \
  --logdir "$review_logdir" --prefer-latest-collection
