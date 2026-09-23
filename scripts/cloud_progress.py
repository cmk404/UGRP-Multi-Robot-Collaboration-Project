"""Structured live progress across nested logs, without forwarding request bodies."""
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

PREFIX = 'UGRP_PROGRESS '
FIELDS = {'schema','unix','event','phase','name','planned','attempted','successes','failures',
          'pending','trial_id','exit_code','elapsed_s','timed_out','ready','checks'}


def emit(event, **fields):
    row = {'schema':'ugrp.progress.v1','unix':time.time(),'event':event,**fields}
    if set(row)-FIELDS:raise ValueError('unapproved progress fields')
    print(PREFIX+json.dumps(row,allow_nan=False),flush=True)
    return row


def parse_line(line):
    if not line.startswith(PREFIX) or len(line)>8192:return None
    try:row=json.loads(line[len(PREFIX):])
    except ValueError:return None
    if (not isinstance(row,dict) or row.get('schema')!='ugrp.progress.v1'
            or set(row)-FIELDS or not isinstance(row.get('unix'),(int,float))):return None
    return row


def copy_output(pipe, log):
    for line in iter(pipe.readline,''):
        log.write(line);log.flush()
        if parse_line(line) is not None:print(line.rstrip('\n'),flush=True)


def run_logged(command, log_path, *, name, cwd=None, env=None, timeout=None, heartbeat_s=30,
               stdin_text=None, termination_grace_s=10):
    """Own one child group, preserve all logs and forward only structured progress."""
    start=time.monotonic();timed_out=False
    env={**(os.environ if env is None else env),'PYTHONUNBUFFERED':'1'}
    emit('process_start',name=name)
    with Path(log_path).open('w') as log:
        child=subprocess.Popen(list(map(str,command)),cwd=cwd,env=env,stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT,text=True,start_new_session=True,
                               stdin=subprocess.PIPE if stdin_text is not None else subprocess.DEVNULL)
        reader=threading.Thread(target=copy_output,args=(child.stdout,log),daemon=True);reader.start()
        try:
            if child.stdin is not None:
                try:child.stdin.write(stdin_text)
                except BrokenPipeError:pass
                finally:child.stdin.close()
            while child.poll() is None:
                remaining=None if timeout is None else timeout-(time.monotonic()-start)
                if remaining is not None and remaining<=0:
                    timed_out=True;break
                try:child.wait(timeout=min(heartbeat_s,remaining) if remaining is not None else heartbeat_s)
                except subprocess.TimeoutExpired:
                    emit('process_running',name=name,elapsed_s=time.monotonic()-start)
        finally:
            if child.poll() is None:
                try:os.killpg(child.pid,signal.SIGTERM)
                except ProcessLookupError:pass
                try:child.wait(timeout=termination_grace_s)
                except subprocess.TimeoutExpired:
                    try:os.killpg(child.pid,signal.SIGKILL)
                    except ProcessLookupError:pass
                    child.wait()
            reader.join(timeout=5)
            if reader.is_alive():raise RuntimeError('child output did not close')
            child.stdout.close()
    result={'exit_code':child.returncode,'timed_out':timed_out,'wall_s':time.monotonic()-start}
    emit('process_exit',name=name,exit_code=child.returncode,timed_out=timed_out,elapsed_s=result['wall_s'])
    return result


def summarize_log(text, *, now=None):
    rows=[r for line in text.splitlines() if (r:=parse_line(line)) is not None]
    if not rows:return {'visibility':'unavailable','attempted':None,'planned':None}
    now=time.time() if now is None else now
    last=rows[-1];summary={'visibility':'live' if now-last['unix']<=120 else 'stale',
                         'last_progress_unix':last['unix'],'latest_event':last}
    counts=next((r for r in reversed(rows) if 'attempted' in r and 'planned' in r),None)
    if counts:summary.update({k:counts[k] for k in ('planned','attempted','successes','failures','pending') if k in counts})
    return summary
