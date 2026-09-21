"""Bounded collection of one existing Kaggle job; never submits or retries a job."""
import argparse
import contextlib
import io
import json
from pathlib import Path
import sys
import subprocess
import time
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from scripts.cloud_collection import RemoteHealth, write_json
from scripts.kaggle_simulation_cli import status, collect
from scripts.cloud_progress import summarize_log


def progress(output):
    kernel=json.loads((output/'job.json').read_text())['kernel']
    try:
        response=subprocess.run(['kaggle','kernels','logs',kernel,'--follow'],capture_output=True,timeout=10)
        raw=response.stdout
    except subprocess.TimeoutExpired as exc:raw=exc.stdout or b''
    text=raw.decode(errors='replace')[-2_000_000:]
    saved=output/'live-progress.log'
    if text:saved.write_text(text)
    elif saved.exists():text=saved.read_text()
    return summarize_log(text)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--seconds', type=float, default=14400)
    args=p.parse_args()
    if not 0 < args.seconds <= 15300: p.error('invalid finite deadline')
    health=RemoteHealth(); state={'started_unix':time.time(), 'complete':False, 'verified':False}
    deadline=time.monotonic()+args.seconds
    terminal={'complete','error','failed','cancelled','cancelacknowledged'}
    while time.monotonic() < deadline:
        current=None
        try:
            with contextlib.redirect_stdout(io.StringIO()): current=status(args.output)
            if current=='unknown': raise ValueError('unrecognized remote status')
            state['status_last_seen_unix']=time.time()
            if current not in terminal:
                try:state['progress']=progress(args.output)
                except Exception as exc:state['progress']={'visibility':'unavailable','error_type':type(exc).__name__}
            if current in terminal:
                state['remote_terminal']=True
                with contextlib.redirect_stdout(io.StringIO()): code=collect(args.output)
                state.update(collection_exit_code=code, verified=(args.output/'verified-result.json').exists())
                state['complete']=state['verified']
                state['ended_unix']=time.time()
            health.success(current)
            state.update(health.state)
            state['collection_degraded']=False
            write_json(args.output/'collection-status.json',state)
            if state['complete']: return 0
        except Exception as exc:
            exhausted=health.failure(exc)
            state.update(health.state)
            state['collection_degraded']=exhausted
            integrity=isinstance(exc,ValueError) and current in terminal
            if integrity:state.update(stop_reason='checkpoint_integrity_failed',ended_unix=time.time())
            write_json(args.output/'collection-status.json',state)
            if integrity:return 1
        time.sleep(30)
    state.update(stop_reason='collection_deadline',ended_unix=time.time())
    write_json(args.output/'collection-status.json',state)
    return 1

if __name__=='__main__': raise SystemExit(main())
