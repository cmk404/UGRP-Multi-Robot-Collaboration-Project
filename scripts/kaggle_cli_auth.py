"""Refresh existing Kaggle OAuth credentials before the SDK's late-expiry window.

Never prints tokens, creates credentials, or changes the authorized account.
"""
from datetime import datetime, timedelta, timezone
import json
import os
from pathlib import Path
import shutil
import subprocess


def refresh_needed(metadata, now=None):
    if not metadata.get('refresh_token'):
        return False
    expires = metadata.get('access_token_expiration')
    if not expires or not metadata.get('access_token'):
        return True
    stamp = datetime.fromisoformat(expires)
    if stamp.tzinfo is None:
        stamp = stamp.replace(tzinfo=timezone.utc)
    return stamp <= (now or datetime.now(timezone.utc)) + timedelta(minutes=5)


def refresh_existing_oauth():
    if any(os.environ.get(k) for k in ('KAGGLE_API_TOKEN','KAGGLE_KEY')):
        return
    credentials = Path.home()/'.kaggle/credentials.json'
    if not credentials.is_file() or not refresh_needed(json.loads(credentials.read_text())):
        return
    # Kaggle lives in its own installed Python environment on macOS. Use that
    # entrypoint's interpreter, without passing secret values to the process.
    executable = shutil.which('kaggle')
    if not executable:
        raise RuntimeError('Kaggle CLI is unavailable')
    first = Path(executable).read_text().splitlines()[0]
    interpreter = first.removeprefix('#!').strip()
    if not first.startswith('#!') or not Path(interpreter).is_file():
        raise RuntimeError('Cannot locate Kaggle Python; refresh existing OAuth with its SDK')
    code = """from kagglesdk.kaggle_client import KaggleClient
from kagglesdk.kaggle_creds import KaggleCredentials
import sys
try:
 with KaggleClient() as client:
  creds=KaggleCredentials.load(client)
  if creds is None:raise RuntimeError('OAuth credentials absent')
  creds.refresh_access_token()
except Exception as exc:
 print(type(exc).__name__,file=sys.stderr)
 sys.exit(1)
"""
    result = subprocess.run([interpreter,'-c',code],capture_output=True,text=True,timeout=60)
    if result.returncode or refresh_needed(json.loads(credentials.read_text())):
        raise RuntimeError('Kaggle OAuth refresh failed; saved credentials were retained')
