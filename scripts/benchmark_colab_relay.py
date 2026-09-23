"""Bounded four-message transport benchmark; no model or robot calls."""
import concurrent.futures
import hashlib
import json
from pathlib import Path
import statistics
import subprocess
import time
import uuid

from scripts.colab_live_contents import LiveContentsClient
from scripts.colab_pooled_contents import pooled_factory
from scripts.cloud_collection import write_json


def benchmark(session, output):
    # Fixed synthetic payloads, isolated from the production mailbox.
    remote = '/content/relay-benchmark-'+uuid.uuid4().hex
    payload = json.dumps({'fixture': 'x'*8192})
    response = json.dumps({'fixture_response': 'y'*1024})
    old = LiveContentsClient(session)
    fresh = LiveContentsClient(session, factory=pooled_factory)
    workers = [LiveContentsClient(session, factory=pooled_factory) for _ in range(4)]
    rows = []
    deadline = time.monotonic()+180
    import requests
    original = requests.request
    def bounded(*args, **kwargs):
        kwargs.setdefault('timeout', (5, 10))
        return original(*args, **kwargs)
    requests.request = bounded
    def roundtrip(client, index, mode):
        if time.monotonic()>=deadline:raise TimeoutError('benchmark deadline')
        item = client._request('GET', remote+f'/request-{index}.json', params={'content':'1'})
        if item['content'] != payload:
            raise ValueError('benchmark request content mismatch')
        client._request('PUT', remote+f'/{mode}-{index}.json',
                        json_data={'type':'file','format':'text','content':response})
    try:
        fresh._request('PUT', remote, json_data={'type':'directory'})
        for index in range(4):
            fresh._request('PUT', remote+f'/request-{index}.json',
                json_data={'type':'file','format':'text','content':payload})
        old.refresh()
        for client in workers:
            client.refresh()
            client.list_dir(remote)  # Warm pooled connections outside measured rounds.
        with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
            for pair in range(4):
                order = ('sequential','pooled_parallel') if pair%2 == 0 else ('pooled_parallel','sequential')
                for mode in order:
                    if time.monotonic()>=deadline:raise TimeoutError('benchmark deadline')
                    began = time.monotonic()
                    (old if mode == 'sequential' else fresh).list_dir(remote)
                    if mode == 'sequential':
                        for index in range(4):roundtrip(old,index,mode)
                    else:
                        futures = [pool.submit(roundtrip,workers[index],index,mode) for index in range(4)]
                        for future in futures:future.result()
                    rows.append({'pair':pair,'mode':mode,'wall_s':time.monotonic()-began})
        for mode in ('sequential','pooled_parallel'):
            for index in range(4):
                item = fresh._request('GET', remote+f'/{mode}-{index}.json',params={'content':'1'})
                if item['content'] != response:raise ValueError('benchmark response content mismatch')
        medians = {mode:statistics.median(r['wall_s'] for r in rows if r['mode']==mode)
                   for mode in ('sequential','pooled_parallel')}
        result = {'scope':'Contents API only; no model or robot execution', 'model_calls':0,
                  'request_bytes':len(payload.encode()),'response_bytes':len(response.encode()),
                  'request_sha256':hashlib.sha256(payload.encode()).hexdigest(),
                  'response_sha256':hashlib.sha256(response.encode()).hexdigest(),
                  'rows':rows,'median_batch_s':medians,
                  'ratio':medians['sequential']/medians['pooled_parallel'],
                  'verified':True,'finished_unix':time.time(),
                  'source_sha':subprocess.check_output(['git','rev-parse','HEAD'],cwd=Path(__file__).resolve().parents[1],text=True).strip()}
        write_json(output,result)
        return result
    finally:
        requests.request = original
        for client in [old,fresh,*workers]:client.close()
