"""Private file transport; no credentials or arbitrary network endpoints on workers."""
from __future__ import annotations
import json
from pathlib import Path
import time
import uuid

ENDPOINTS = {'jev':'https://api.typesafe.ai/v1/systemone',
             'gemini':'http://127.0.0.1:8391/v1/chat/completions'}
MODELS = {'jev':'jev-1.13.0','gemini':'gemini-3.8-flash'}
MAX_BYTES = 128_000


def validate_request(data, now=None):
    now = time.time() if now is None else now
    if set(data) != {'id','provider','body','expires_unix'}:
        raise ValueError('unexpected relay fields')
    if len(data['id']) != 32 or any(c not in '0123456789abcdef' for c in data['id']):
        raise ValueError('invalid request identity')
    if data['provider'] not in MODELS or data['body'].get('model') != MODELS[data['provider']]:
        raise ValueError('unapproved provider/model')
    if not now < data['expires_unix'] <= now + 125:
        raise ValueError('expired or excessive deadline')
    if len(json.dumps(data,allow_nan=False).encode()) > MAX_BYTES:
        raise ValueError('oversized request')
    return data


class MailboxTransport:
    def __init__(self, root, *, transport_grace_s=90):
        if not 0 <= transport_grace_s <= 90: raise ValueError("invalid transport grace")
        self.transport_grace_s = transport_grace_s
        self.root = Path(root)
        for name in ('requests','responses'):
            (self.root/name).mkdir(parents=True,exist_ok=True)

    def __call__(self, body, url, key=None, timeout=30):
        if key:
            raise ValueError('credentials must remain on the relay host')
        provider = next((p for p,u in ENDPOINTS.items() if u == url), None)
        start = time.monotonic()
        identity = uuid.uuid4().hex
        budget = min(timeout,30) + self.transport_grace_s
        request = validate_request({'id':identity,'provider':provider,'body':body,
                                    'expires_unix':time.time()+budget})
        path = self.root/'requests'/f'{identity}.json'
        tmp = path.with_suffix('.tmp')
        tmp.write_text(json.dumps(request,allow_nan=False));tmp.replace(path)
        result_path = self.root/'responses'/path.name
        try:
            while time.monotonic()-start < budget:
                if result_path.exists():
                    # Contents API may expose a file during upload; wait for complete JSON.
                    try: reply = json.loads(result_path.read_text())
                    except (ValueError,OSError): time.sleep(.05);continue
                    if reply.get('id') != identity: raise ValueError('response identity mismatch')
                    result = reply['response']
                    result['provider_latency_s'] = result.get('latency_s')
                    result['latency_s'] = time.monotonic()-start
                    result['transport'] = 'colab_private_mailbox'
                    return result
                time.sleep(.05)
            return {'status':'transport_error','error_type':'MailboxTimeout',
                    'transport':'colab_private_mailbox','latency_s':time.monotonic()-start}
        finally:
            path.unlink(missing_ok=True)
