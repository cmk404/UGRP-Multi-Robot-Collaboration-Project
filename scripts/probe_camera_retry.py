"""One injected timeout followed by the real model, through the real runner.

This diagnostic is excluded from the normal seed cohort. It proves runtime
recovery/fresh captures, not the availability of an external provider.
"""
import sys
from pathlib import Path
import json
from harness.gemini_proxy import GeminiProxyCompleter, GeminiProxyError
from scripts import evaluate_gemini_team as runner


class TimeoutOnce(GeminiProxyCompleter):
    def complete(self, *args, **kwargs):
        if not getattr(self, '_injected', False):
            self._injected = True
            raise GeminiProxyError('diagnostic injected timeout', error_kind='timeout', retryable=True, latency_ms=0)
        return super().complete(*args, **kwargs)


def main():
    out = Path('outputs/warehouse_research/camera-retry-probe-01')
    runner.GeminiProxyCompleter = TimeoutOnce
    sys.argv = ['evaluate_gemini_team', '--output', str(out), '--seed','41',
                '--robots','1','--seconds','10','--max-calls','3','--noslip-iterations','3']
    runner.main()
    rows = [json.loads(line) for line in (out/'llm-decisions.jsonl').read_text().splitlines()]
    requests = [r for r in rows if r['event']=='llm_request']
    results = [r for r in rows if r['event']=='llm_result']
    verified = (len(requests)>=2 and results[0]['disposition']=='inference_error'
                and results[0]['inference_error']['retry_scheduled']
                and requests[1]['time']>requests[0]['time']
                and requests[0]['images'][0]['path']!=requests[1]['images'][0]['path']
                and any(r['disposition']=='accepted' and r.get('audit',{}).get('response_model')=='gemini-3.8-flash' for r in results[1:]))
    (out/'retry-verification.json').write_text(json.dumps({'verified':verified,
        'diagnostic_only':True,'injected_timeouts':1,'real_model_after_retry':True if verified else None,
        'request_times':[r['time'] for r in requests]},indent=2))
    if not verified:
        raise RuntimeError('retry integration not confirmed')


if __name__ == '__main__':
    main()
