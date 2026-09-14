#!/usr/bin/env bash
# Verify the documented Ubuntu bootstrap without loading credentials or calling a model.
set -euo pipefail
ugrp_repo=$(cd "$(dirname "$0")/.." && pwd)
ugrp_tmp=$(mktemp -d)
ugrp_session="proxy-smoke-$$"
export UGRP_SESSION_DIR="$ugrp_tmp/sessions"
cleanup() {
  python3 "$ugrp_repo/scripts/ugrp_session.py" stop "$ugrp_session" || true
  if [ -n "${ugrp_pid:-}" ]; then wait "$ugrp_pid" || true; fi
  rm -rf "$ugrp_tmp"
}
trap cleanup EXIT
cd "$ugrp_tmp"
release='https://github.com/router-for-me/CLIProxyAPI/releases/download/v7.2.155'
archive='CLIProxyAPI_7.2.155_linux_amd64.tar.gz'
curl -fL "$release/$archive" -o "$archive"
curl -fL "$release/checksums.txt" -o checksums.txt
grep " $archive$" checksums.txt | sha256sum --check --strict -
tar -xzf "$archive"
test -x ./cli-proxy-api
mkdir auth
# Preserve the example's settings except for isolated auth storage and test port.
python3 - "$ugrp_repo/configs/gemini-proxy.example.yaml" "$ugrp_tmp" <<'PY'
from pathlib import Path
import sys
root = Path(sys.argv[2])
s = Path(sys.argv[1]).read_text()
s = s.replace('port: 8391', 'port: 18397')
s = s.replace('~/.local/share/ugrp/gemini-proxy/auth', str(root / 'auth'))
(root / 'config.yaml').write_text(s)
PY
# Do not inherit a Home-mode or management credential from the runner environment.
env -u HOME_JWT -u MANAGEMENT_PASSWORD python3 "$ugrp_repo/scripts/ugrp_session.py" run "$ugrp_session" -- \
  "$ugrp_tmp/cli-proxy-api" --config "$ugrp_tmp/config.yaml" >server.log 2>&1 &
ugrp_pid=$!
for attempt in $(seq 1 30); do
  if curl -fsS http://127.0.0.1:18397/v1/models >models.json 2>/dev/null; then break; fi
  if ! kill -0 "$ugrp_pid" 2>/dev/null; then cat server.log; exit 1; fi
  sleep 1
done
python3 - <<'PY'
import json
from pathlib import Path
assert json.loads(Path('models.json').read_text()) == {'data': [], 'object': 'list'}
print('PASS: loopback /v1/models works without credentials or API key; zero registered models')
PY
python3 "$ugrp_repo/scripts/ugrp_session.py" stop "$ugrp_session"
wait "$ugrp_pid" || true
ugrp_pid=''
test ! -e "$UGRP_SESSION_DIR/$ugrp_session.json"
if curl -fsS --max-time 2 http://127.0.0.1:18397/v1/models >/dev/null 2>&1; then
  echo 'FAIL: listener remains after session stop' >&2
  exit 1
fi
printf '%s\n' 'PASS: session and listener stopped'
