"""Explicit Mac-only, fixed-budget real Gemini pilot; default is readiness only.

No account switching, proxy restart, background service or automatic rerun.
A readiness TCP check does not establish authentication/model/quota availability.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import os
from pathlib import Path
import platform
import socket
import subprocess
import sys
from urllib.parse import urlsplit

from scripts.verify_gemini_budget_run import verify_run


def readiness():
    modules = {name: importlib.util.find_spec(name) is not None
               for name in ('mujoco', 'cv2', 'numpy', 'PIL')}
    reachable = False
    # Read only the already-configured endpoint; never print its URL or secrets.
    try:
        endpoint = urlsplit(os.environ.get('GEMINI_PROXY_URL', 'http://127.0.0.1:8391/v1/chat/completions'))
        if endpoint.scheme in ('http', 'https') and endpoint.hostname:
            with socket.create_connection((endpoint.hostname, endpoint.port or
                                           (443 if endpoint.scheme == 'https' else 80)), timeout=2):
                reachable = True
    except (OSError, ValueError):
        pass
    return {'host_os': platform.system(), 'python': platform.python_version(),
            'modules_available': modules, 'configured_proxy_tcp_reachable': reachable,
            'model_auth_and_quota_verified': False, 'gemini_completion_calls_in_check': 0,
            'ready_for_attempt': platform.system() == 'Darwin' and all(modules.values()) and reachable}


def pilot_command(output):
    return [sys.executable, '-m', 'scripts.evaluate_gemini_team', '--output', str(output),
            '--seed', '41', '--robots', '1', '--seconds', '300', '--model', 'gemini-3.8-flash',
            '--max-calls', '30', '--max-input-tokens', '60000', '--input-request-estimate', '6000',
            '--impratio', '10', '--noslip-iterations', '3', '--communication', 'none',
            '--request-timeout', '30', '--max-transient-failures', '2', '--record']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true', help='explicitly consume actual Gemini quota on Mac')
    parser.add_argument('--output', type=Path, help='fresh output directory, required for --execute')
    args = parser.parse_args()
    report = readiness()
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    if not args.execute:
        return 0 if report['ready_for_attempt'] else 2
    if not args.output:
        parser.error('--execute requires --output')
    if not report['ready_for_attempt']:
        print('LIVE_PILOT_NOT_STARTED: Mac/runtime/proxy readiness failed.', flush=True)
        return 2
    output = args.output.resolve()
    log_path = output.with_name(output.name + '.log')
    if output.exists() or log_path.exists():
        parser.error('output or log already exists; previous trials will not be overwritten')
    output.parent.mkdir(parents=True, exist_ok=True)
    command = pilot_command(output)
    with log_path.open('x', encoding='utf-8') as log:
        completed = subprocess.run(command, cwd=Path(__file__).resolve().parents[1],
                                   stdout=log, stderr=subprocess.STDOUT, check=False)
    verification = verify_run(output)
    verification['runner_exit_code'] = completed.returncode
    if completed.returncode:
        verification['automated_checks_passed'] = False
        verification['status'] = 'NOT_VERIFIED'
    if output.is_dir():
        (output / 'budget-verification.json').write_text(json.dumps(verification, indent=2), encoding='utf-8')
    print(json.dumps(verification, ensure_ascii=False, indent=2))
    return 0 if verification['automated_checks_passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
