#!/bin/zsh
set -euo pipefail

ROOT_DIR="${0:A:h:h}"

exec python3 "$ROOT_DIR/dashboard/server.py" "$@"
