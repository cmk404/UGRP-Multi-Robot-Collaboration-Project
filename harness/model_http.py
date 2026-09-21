"""Dependency-free model HTTP transport shared by local and remote runners."""
import json
import time
import urllib.request
import urllib.error

class NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, *args, **kwargs): return None


def post(body, url, key=None, timeout=30):
    headers = {'Content-Type':'application/json'}
    if key: headers['Authorization'] = 'Bearer ' + key
    req = urllib.request.Request(url, json.dumps(body, allow_nan=False).encode(), headers)
    start = time.perf_counter()
    try:
        with urllib.request.build_opener(NoRedirect).open(req, timeout=timeout) as response:
            status, raw = response.status, response.read(2_000_000).decode()
            retry_after = response.headers.get('Retry-After')
    except urllib.error.HTTPError as exc:
        status, raw = exc.code, exc.read(2_000_000).decode(errors='replace')
        retry_after = exc.headers.get('Retry-After') if exc.headers else None
    except (OSError, urllib.error.URLError) as exc:
        return {'status':'transport_error','error_type':type(exc).__name__, 'latency_s':time.perf_counter()-start}
    if key: raw = raw.replace(key,'[REDACTED]')
    result = {'http_status':status,'raw_response':raw,'latency_s':time.perf_counter()-start,'status':'http_error'}
    try:
        if retry_after is not None:result['retry_after_s']=max(0,min(60,float(retry_after)))
    except (ValueError,TypeError):pass
    if status == 200:
        try: result['body'] = json.loads(raw); result['status'] = 'ok'
        except ValueError: result['status'] = 'invalid_json'
    return result


def post_with_recovery(body, url, key=None, timeout=30, *, max_attempts=3,
                       call=post, clock=time.monotonic, sleep=time.sleep, admit=lambda:True):
    """Retry explicit temporary HTTP rejection only; keep every original outcome.

    Never replay an uncertain network timeout, invalid response, or client error.
    One total deadline covers provider calls and backoff. Actor failures already
    recorded by earlier cohorts are not replaced by this new transport policy.
    """
    if not 1<=max_attempts<=3 or not 0<timeout<=120:raise ValueError('invalid recovery budget')
    began=clock();deadline=began+timeout;attempts=[]
    result={'status':'transport_error','error_type':'ModelBudgetExhausted','latency_s':0}
    for index in range(max_attempts):
        remaining=deadline-clock()
        if remaining<=0 or not admit():break
        result=call(body,url,key,remaining)
        attempts.append(dict(result))
        if result.get('status')!='http_error' or result.get('http_status') not in (429,503,529):break
        delay=max(.5*(2**index),result.get('retry_after_s',0))
        if index+1>=max_attempts or clock()+delay+.1>=deadline:break
        sleep(delay)
    return {**result,'latency_s':clock()-began,'http_attempts':attempts,
            'http_attempt_count':len(attempts),'retry_count':max(0,len(attempts)-1),
            'first_attempt_failed':bool(attempts and attempts[0].get('status')!='ok'),
            'recovered':bool(len(attempts)>1 and result.get('status')=='ok'),
            'retry_policy':'explicit_http_rejection_v1'}
