#!/usr/bin/env python3
"""Finite Colab Contents relay. No listener, credential upload, or endpoint forwarding."""
from __future__ import annotations
import argparse
import base64
import concurrent.futures
import json
from pathlib import Path
import subprocess
import sys
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from harness.model_mailbox import ENDPOINTS,MAX_BYTES,validate_request
from harness.model_http import post


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--session',required=True);p.add_argument('--remote',required=True)
    p.add_argument('--keychain-helper',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--seconds',type=float,default=14400)
    p.add_argument('--max-calls',type=int,default=8000)
    a=p.parse_args()
    if not a.remote.startswith('/content/') or '..' in Path(a.remote).parts:p.error('invalid remote root')
    if not 0<a.seconds<=14400 or not 0<a.max_calls<=50000:p.error('invalid finite budget')
    import requests
    from colab_cli.contents import ContentsClient
    from colab_cli.state import StateStore
    original=requests.request
    def bounded(*args,**kwargs):
        kwargs.setdefault('timeout',(5,10));return original(*args,**kwargs)
    requests.request=bounded
    client=ContentsClient(StateStore().get(a.session))
    key=subprocess.check_output([str(a.keychain_helper.resolve())],stderr=subprocess.DEVNULL,timeout=15).decode()
    if not key:raise ValueError('empty Keychain credential')
    a.output.mkdir(parents=True,exist_ok=False)
    deadline=time.monotonic()+a.seconds
    seen=set();pending={};expirations={};stats={'calls':0,'delivered':0,'errors':0,'started_unix':time.time(),'complete':False}
    def save():
        tmp=a.output/'status.tmp';tmp.write_text(json.dumps(stats,indent=2)+'\n');tmp.replace(a.output/'status.json')
    def execute(row):
        remaining=row['expires_unix']-time.time()
        if remaining<=0:return {'status':'transport_error','error_type':'ExpiredRequest','latency_s':0}
        return post(row['body'],ENDPOINTS[row['provider']],key if row['provider']=='jev' else None,min(30,remaining))
    save()
    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
        try:
            while time.monotonic()<deadline:
                for identity,future in list(pending.items()):
                    if not future.done():continue
                    response=future.result()
                    value={'id':identity,'response':response}
                    payload=json.dumps(value,allow_nan=False)
                    (a.output/f'{identity}-response.json').write_text(payload+'\n')
                    path=a.remote+'/responses/'+identity+'.json'
                    if time.time() >= expirations[identity]:
                        stats['expired_deliveries']=stats.get('expired_deliveries',0)+1
                        del pending[identity];save();continue
                    try:
                        client._request('PUT',path,json_data={'type':'file','format':'text','content':payload})
                    except Exception as exc:
                        stats['last_error_type']=type(exc).__name__;stats['errors']+=1
                        # Retry delivery only; never repeat the billable model call.
                        continue
                    stats['delivered']+=1;del pending[identity];save()
                if len(seen)>=a.max_calls and not pending:break
                if len(pending)>=4:time.sleep(.1);continue
                try:
                    listing=client.list_dir(a.remote+'/requests').get('content',[])
                    for entry in listing:
                        name=entry.get('name','')
                        if not name.endswith('.json'):continue
                        identity=name[:-5]
                        if identity in seen or len(pending)>=4 or len(seen)>=a.max_calls:continue
                        if len(identity)!=32 or any(c not in '0123456789abcdef' for c in identity):continue
                        if entry.get('size',0)>MAX_BYTES:continue
                        item=client._request('GET',a.remote+'/requests/'+name,params={'content':'1'})
                        raw=item['content']
                        if item.get('format')=='base64':raw=base64.b64decode(raw).decode()
                        row=validate_request(json.loads(raw))
                        if row['id']!=identity:raise ValueError('identity mismatch')
                        (a.output/(identity+'-request.json')).write_text(json.dumps(row)+'\n')
                        seen.add(identity);expirations[identity]=row['expires_unix'];pending[identity]=pool.submit(execute,row);stats['calls']+=1;save()
                except FileNotFoundError:pass
                except Exception as exc:
                    stats['last_error_type']=type(exc).__name__;stats['errors']+=1;save()
                time.sleep(.2)
        finally:
            stats.update(complete=True,ended_unix=time.time(),pending=len(pending));save()
    return 0

if __name__=='__main__':raise SystemExit(main())
