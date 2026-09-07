import io
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

class ProxySourceRegressionTests(unittest.TestCase):
    def test_sse_and_mjpeg_have_different_read_paths(self):
        text=(ROOT/'harness/web.py').read_text()
        self.assertIn('if "text/event-stream" in ctype:', text)
        self.assertIn('line = resp.readline()', text)
        self.assertIn('elif "multipart/" in ctype:', text)
        self.assertIn('reader = getattr(resp, "read1", None)', text)
        sse=text.split('if "text/event-stream" in ctype:',1)[1].split('elif "multipart/" in ctype:',1)[0]
        self.assertNotIn('read1(', sse)

if __name__=='__main__': unittest.main()
