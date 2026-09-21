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
    except urllib.error.HTTPError as exc:
        status, raw = exc.code, exc.read(2_000_000).decode(errors='replace')
    except (OSError, urllib.error.URLError) as exc:
        return {'status':'transport_error','error_type':type(exc).__name__, 'latency_s':time.perf_counter()-start}
    if key: raw = raw.replace(key,'[REDACTED]')
    result = {'http_status':status,'raw_response':raw,'latency_s':time.perf_counter()-start,'status':'http_error'}
    if status == 200:
        try: result['body'] = json.loads(raw); result['status'] = 'ok'
        except ValueError: result['status'] = 'invalid_json'
    return result

