#!/usr/bin/env python3
"""Finite, four-slot Colab relay with persistent independent transfer pipelines.

No new endpoint, credential upload or robot input changes. Explicit temporary
HTTP rejection recovery is bounded and preserves every first attempt failure.
"""
from __future__ import annotations
import argparse
import concurrent.futures
import json
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from harness.model_mailbox import ENDPOINTS,MAX_BYTES
from harness.model_http import post_with_recovery
from scripts.colab_relay_pipeline import Pipeline,atomic_json,drained_seen


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--session',required=True);p.add_argument('--remote',required=True)
    p.add_argument('--keychain-helper',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--resume-relay',type=Path,help='Stopped, fully drained predecessor audit directory')
    p.add_argument('--seconds',type=float,default=14400)
    p.add_argument('--max-calls',type=int,default=8000)
    a=p.parse_args()
    if not a.remote.startswith('/content/') or '..' in Path(a.remote).parts:p.error('invalid remote root')
    if not 0<a.seconds<=14400 or not 0<a.max_calls<=50000:p.error('invalid finite budget')
    from scripts.colab_live_contents import LiveContentsClient
    from scripts.colab_pooled_contents import pooled_factory
    from colab_cli.state import StateStore
    session=StateStore().get(a.session)
    def factory():return LiveContentsClient(session,factory=pooled_factory)
    client=factory()
    key=subprocess.check_output([str(a.keychain_helper.resolve())],stderr=subprocess.DEVNULL,timeout=15).decode()
    if not key:raise ValueError('empty Keychain credential')
    seen=drained_seen(a.resume_relay) if a.resume_relay else set()
    a.output.mkdir(parents=True,exist_ok=False)
    deadline=time.time()+a.seconds;pending={};lock=threading.RLock();draining=threading.Event()
    stats={'calls':0,'api_calls':0,'delivered':0,'errors':0,'started_unix':time.time(),'complete':False,
           'transport_version':'pooled_parallel_v1','inherited_requests':len(seen),
           'retry_policy':'explicit_http_rejection_v1',
           'predecessor':str(a.resume_relay) if a.resume_relay else None}
    def save():
        with lock:atomic_json(a.output/'status.json',stats)
    def event(name,**values):
        with lock:
            field={'model_start':'calls','delivered':'delivered','expired':'expired_deliveries','io_error':'errors'}[name]
            stats[field]=stats.get(field,0)+1
            if values.get('error_type'):stats['last_error_type']=values['error_type']
            save()
    def execute(row,until):
        remaining=until-time.time()
        if remaining<=0:return {'status':'transport_error','error_type':'ExpiredRequest','latency_s':0}
        def admit():
            with lock:
                if stats['api_calls']>=a.max_calls:return False
                stats['api_calls']+=1;save();return True
        result=post_with_recovery(row['body'],ENDPOINTS[row['provider']],key if row['provider']=='jev' else None,min(30,remaining),admit=admit)
        with lock:
            for field,value in [('first_attempt_failures',int(result['first_attempt_failed'])),
                                ('recovered_requests',int(result['recovered'])),('retry_calls',result['retry_count'])]:
                stats[field]=stats.get(field,0)+value
            save()
        return result
    pipeline=Pipeline(factory,execute,a.remote,a.output,deadline,event=event)
    signal.signal(signal.SIGTERM,lambda *_:draining.set())
    signal.signal(signal.SIGINT,lambda *_:draining.set())
    save()
    try:
        with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
            while time.time()<deadline or pending:
                for identity,future in list(pending.items()):
                    if not future.done():continue
                    result=future.result();del pending[identity]
                    if result.get('retry'):seen.discard(identity)
                with lock:
                    stats.update(pending=len(pending),draining=draining.is_set(),last_observed_unix=time.time())
                    save()
                exhausted=stats['api_calls']>=a.max_calls or time.time()>=deadline or draining.is_set()
                if exhausted:
                    if not pending:break
                    time.sleep(.05);continue
                if len(pending)>=4:time.sleep(.05);continue
                try:
                    listing=client.list_dir(a.remote+'/requests').get('content',[])
                    for entry in listing:
                        name=entry.get('name','')
                        if not name.endswith('.json'):continue
                        identity=name[:-5]
                        # Include reservations so up to four downloads cannot overshoot the limit.
                        if identity in seen or len(pending)>=4 or stats['calls']+len(pending)>=a.max_calls:continue
                        if len(identity)!=32 or any(c not in '0123456789abcdef' for c in identity):continue
                        if entry.get('size',0)>MAX_BYTES:continue
                        seen.add(identity);pending[identity]=pool.submit(pipeline.handle,identity)
                except FileNotFoundError:pass
                except Exception as exc:event('io_error',error_type=type(exc).__name__)
                time.sleep(.05)
    finally:
        pipeline.close();client.close()
        with lock:
            stats.update(complete=True,ended_unix=time.time(),pending=len(pending));save()
    return 0

if __name__=='__main__':raise SystemExit(main())
