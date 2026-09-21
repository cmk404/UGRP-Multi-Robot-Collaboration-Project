"""Independent pipelines; delivery retries never repeat the provider policy."""
import base64
import json
from pathlib import Path
import threading
import time
from harness.model_mailbox import validate_request


def atomic_json(path, value):
    temp = path.with_suffix('.tmp')
    temp.write_text(json.dumps(value, allow_nan=False)+'\n')
    temp.replace(path)


def drained_seen(output):
    """Call only after the predecessor process has been stopped at a drained boundary."""
    output = Path(output)
    state = json.loads((output/'status.json').read_text())
    requests = {p.name.removesuffix('-request.json') for p in output.glob('*-request.json')}
    responses = {p.name.removesuffix('-response.json') for p in output.glob('*-response.json')}
    if (requests != responses or state['calls'] != len(requests)
            or state['calls'] != state['delivered'] or state.get('pending', 0)):
        raise ValueError('predecessor has outstanding or uncertain model calls')
    for identity in requests:
        if len(identity) != 32 or any(c not in '0123456789abcdef' for c in identity):
            raise ValueError('invalid prior request identity')
        if json.loads((output/(identity+'-response.json')).read_text())['id'] != identity:
            raise ValueError('prior response identity mismatch')
    return requests


class Pipeline:
    def __init__(self, client_factory, execute, remote, output, deadline,
                 *, event=lambda *a, **k: None, clock=time.time, sleep=time.sleep):
        self.client_factory, self.execute = client_factory, execute
        self.remote, self.output, self.deadline = remote, Path(output), deadline
        self.event, self.clock, self.sleep = event, clock, sleep
        self.local = threading.local()
        self.clients = []
        self.lock = threading.Lock()

    def client(self):
        if not hasattr(self.local, 'client'):
            self.local.client = self.client_factory()
            with self.lock:
                self.clients.append(self.local.client)
        return self.local.client

    def close(self):
        for client in self.clients:
            client.close()

    def handle(self, identity):
        started = self.clock()
        client = self.client()
        timings = {'id': identity, 'started_unix': started}
        # Contents GET retries are safe: no model call has happened yet.
        for attempt in range(3):
            try:
                item = client._request('GET', self.remote+'/requests/'+identity+'.json', params={'content': '1'})
                raw = item['content']
                if item.get('format') == 'base64':
                    raw = base64.b64decode(raw).decode()
                row = validate_request(json.loads(raw), now=self.clock())
                if row['id'] != identity:
                    raise ValueError('identity mismatch')
                break
            except (ValueError, FileNotFoundError):
                return {'id': identity, 'admitted': False, 'retry': False}
            except Exception as exc:
                self.event('io_error', error_type=type(exc).__name__)
                if attempt == 2 or self.clock() >= self.deadline:
                    return {'id': identity, 'admitted': False, 'retry': True}
                self.sleep(.1)
        timings['download_s'] = self.clock()-started
        atomic_json(self.output/(identity+'-request.json'), row)
        self.event('model_start')
        began = self.clock()
        # Execute the bounded provider policy once. Upload retries below never
        # invoke it again; uncertain provider timeouts are not replayed by it.
        try:
            response = self.execute(row, min(row['expires_unix'], self.deadline))
        except Exception as exc:
            response = {'status': 'transport_error', 'error_type': type(exc).__name__,
                        'latency_s': self.clock()-began}
        timings['provider_s'] = self.clock()-began
        value = {'id': identity, 'response': response}
        atomic_json(self.output/(identity+'-response.json'), value)
        began = self.clock()
        delivered = False
        while self.clock() < min(row['expires_unix'], self.deadline):
            try:
                client._request('PUT', self.remote+'/responses/'+identity+'.json',
                                json_data={'type': 'file', 'format': 'text', 'content': json.dumps(value, allow_nan=False)})
                delivered = True
                self.event('delivered')
                break
            except Exception as exc:
                self.event('io_error', error_type=type(exc).__name__)
                self.sleep(.1)
        timings.update(delivery_s=self.clock()-began, delivered=delivered, finished_unix=self.clock())
        atomic_json(self.output/(identity+'-timing.json'), timings)
        if not delivered:
            self.event('expired')
        return {'id': identity, 'admitted': True, 'delivered': delivered}
