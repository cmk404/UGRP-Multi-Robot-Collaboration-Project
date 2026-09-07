#!/usr/bin/env bash
# Open WebUI in front of local Ollama. No Docker.
set -euo pipefail
export OLLAMA_BASE_URL="${OLLAMA_BASE_URL:-http://127.0.0.1:11434}"
export DATA_DIR="${DATA_DIR:-$HOME/.open-webui}"
PORT="${PORT:-3000}"
exec uvx --python 3.11 open-webui@latest serve --host 127.0.0.1 --port "$PORT"
