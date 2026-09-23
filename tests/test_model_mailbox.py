import concurrent.futures
import json
from pathlib import Path
import tempfile
import time
import unittest
from harness.model_mailbox import ENDPOINTS,MailboxTransport,validate_request

class MailboxTests(unittest.TestCase):
    def sample(self):
        return {'id':'a'*32,'provider':'jev','body':{'model':'jev-1.13.0'},'expires_unix':time.time()+10}

    def test_allowlist_rejects_arbitrary_endpoints_models_expiry_and_size(self):
        changes=[{'url':'https://evil.invalid'}, {'provider':'other'}, {'body':{'model':'other'}},
                 {'expires_unix':0}, {'expires_unix':time.time()+3600}, {'id':'../secret'},
                 {'body':{'model':'jev-1.13.0','x':'a'*128000}}]
        for change in changes:
            with self.subTest(change=list(change)):
                with self.assertRaises(ValueError):validate_request({**self.sample(),**change})

    def test_credentials_never_written(self):
        with tempfile.TemporaryDirectory() as d:
            relay=MailboxTransport(d)
            with self.assertRaises(ValueError):relay({'model':'jev-1.13.0'},ENDPOINTS['jev'],'secret')
            self.assertEqual(list((Path(d)/'requests').iterdir()),[])

    def test_round_trip_preserves_response_and_separates_latency(self):
        with tempfile.TemporaryDirectory() as d, concurrent.futures.ThreadPoolExecutor(1) as pool:
            relay=MailboxTransport(d)
            future=pool.submit(relay,{'model':'jev-1.13.0'},ENDPOINTS['jev'],None,2)
            until=time.monotonic()+1
            while not list((Path(d)/'requests').glob('*.json')) and time.monotonic()<until:time.sleep(.01)
            path=next((Path(d)/'requests').glob('*.json'));row=json.loads(path.read_text())
            self.assertGreater(row['expires_unix']-time.time(),80)
            response={'status':'ok','body':{'answer':'x'},'latency_s':.001}
            (Path(d)/'responses'/path.name).write_text(json.dumps({'id':row['id'],'response':response}))
            result=future.result()
            self.assertEqual(result['body'],response['body']);self.assertEqual(result['provider_latency_s'],.001)
            self.assertGreater(result['latency_s'],.001);self.assertFalse(path.exists())

    def test_timeout_cleans_pending_request_without_fallback(self):
        with tempfile.TemporaryDirectory() as d:
            result=MailboxTransport(d,transport_grace_s=0)({'model':'jev-1.13.0'},ENDPOINTS['jev'],None,.03)
            self.assertEqual(result['error_type'],'MailboxTimeout')
            self.assertEqual(list((Path(d)/'requests').iterdir()),[])
