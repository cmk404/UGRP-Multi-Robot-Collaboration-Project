#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
[[ -s "$ROOT/.sim_remote_url" ]] || { echo "missing .sim_remote_url; deploy Lightning sim first" >&2; exit 2; }
mkdir -p "$HOME/.config/systemd/user"
cp "$ROOT/scripts/systemd/ugrp-sim-remote-proxy.service" "$HOME/.config/systemd/user/"
systemctl --user daemon-reload
systemctl --user enable --now ugrp-sim-remote-proxy.service
# Switch only the simulation coworker. Physical robot UI (:8080) is untouched.
sed -i 's#UGRP_SIM_BRIDGE_LOCAL:-http://127.0.0.1:8091#UGRP_SIM_BRIDGE_LOCAL:-http://127.0.0.1:8093#' "$ROOT/scripts/serve_sim_coworker.sh"
systemctl --user restart ugrp-sim-coworker.service
sleep 1
curl -fsS http://127.0.0.1:8093/health >/dev/null
echo "Lightning simulation connected to Oracle coworker (:8082)."
