#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

if [[ ! -x .venv-lightning/bin/lightning || ! -x .venv-lightning/bin/python ]]; then
  echo "Lightning SDK/CLI is not installed in .venv-lightning." >&2
  exit 2
fi

printf 'Lightning API key (input hidden): ' >&2
IFS= read -r -s KEY
printf '\n' >&2
if [[ -z "$KEY" || "$KEY" == *$'\n'* || "$KEY" == *$'\r'* ]]; then
  echo "Invalid empty/multiline API key." >&2
  exit 2
fi

umask 077
AUTH_TMP=".env.gpu.auth.$$"
printf "LIGHTNING_API_KEY='%s'\n" "$KEY" > "$AUTH_TMP"
chmod 600 "$AUTH_TMP"
unset KEY

set -a
. "$AUTH_TMP"
set +a

IDENTITY="$(.venv-lightning/bin/lightning auth whoami --json 2>/dev/null || true)"
if [[ -z "$IDENTITY" ]]; then
  rm -f "$AUTH_TMP"
  echo "Lightning authentication failed. Check the API key and retry." >&2
  exit 3
fi

USERNAME="$(printf '%s' "$IDENTITY" | python3 -c 'import json,sys; print(json.load(sys.stdin).get("username") or "")')"
if [[ -z "$USERNAME" ]]; then
  rm -f "$AUTH_TMP"
  echo "Lightning authentication succeeded, but username discovery failed." >&2
  exit 3
fi

TEAMSPACE="$(LIGHTNING_USERNAME="$USERNAME" .venv-lightning/bin/python - <<'PY'
from lightning_sdk import User
import os
u = User(os.environ["LIGHTNING_USERNAME"])
choices = []
for org in u.organizations:
    for ts in org.teamspaces:
        choices.append(f"{org.name}/{ts.name}")
if not choices:
    for ts in u.teamspaces:
        owner = getattr(ts.owner, "name", None)
        if owner:
            choices.append(f"{owner}/{ts.name}")
print(choices[0] if choices else "")
PY
)"

if [[ -z "$TEAMSPACE" ]]; then
  rm -f "$AUTH_TMP"
  echo "Lightning authentication succeeded, but no accessible teamspace was found." >&2
  exit 3
fi

FINAL_TMP=".env.gpu.tmp.$$"
printf "LIGHTNING_API_KEY='%s'\nUGRP_LIGHTNING_TEAMSPACE='%s'\n" "$LIGHTNING_API_KEY" "$TEAMSPACE" > "$FINAL_TMP"
chmod 600 "$FINAL_TMP"
mv "$FINAL_TMP" .env.gpu
rm -f "$AUTH_TMP"
unset LIGHTNING_API_KEY IDENTITY USERNAME TEAMSPACE

echo "Lightning authentication OK. Teamspace configured automatically."
echo "GPU watchdog will pick it up automatically."
