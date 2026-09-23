"""Finite real-model connectivity/schema checks; no robot observations or commands."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time
from urllib.parse import urlparse

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from harness.model_http import post_with_recovery
from harness.model_mailbox import ENDPOINTS,MODELS
from scripts.cloud_collection import write_json


def check(key, gemini_url, *, samples=3, call=post_with_recovery):
    parsed=urlparse(gemini_url)
    if parsed.scheme!='http' or parsed.hostname not in ('127.0.0.1','localhost'):
        raise ValueError('existing local Gemini proxy required')
    if not 1<=samples<=3:raise ValueError('invalid finite sample count')
    bodies={'jev':{'model':MODELS['jev'],'state':{'purpose':'transport-only connectivity check'},
                  'questions':{'health':{'type':'choice','instructions':'Choose ready.',
                              'criteria':{'ready':'The service can answer this request.','unavailable':'The service cannot answer.'}}}},
            'gemini':{'model':MODELS['gemini'],'messages':[{'role':'user','content':'Reply with exactly {"ready":true}. This is a connectivity check.'}],
                      'temperature':0,'max_tokens':64,'reasoning_effort':'none','response_format':{'type':'json_object'}}}
    rows=[];started=time.time()
    for index in range(samples):
        for provider in ('jev','gemini'):
            response=call(bodies[provider],ENDPOINTS['jev'] if provider=='jev' else gemini_url,
                          key if provider=='jev' else None,30)
            valid=False
            if response.get('status')=='ok':
                try:
                    body=response['body']
                    if provider=='jev':valid=body['answers']['health']['choice']=='ready'
                    else:valid=json.loads(body['choices'][0]['message']['content'])=={'ready':True}
                except (ValueError,KeyError,TypeError,IndexError):pass
            rows.append({'provider':provider,'sample':index,'valid':valid,'response':response})
    return {'scope':'Model connectivity/schema preflight only; no simulation or task-success selection',
            'models':MODELS,'gemini_url':gemini_url,
            'started_unix':started,'finished_unix':time.time(),'ready':all(r['valid'] for r in rows),
            'logical_requests':len(rows),'http_attempts':sum(r['response']['http_attempt_count'] for r in rows),
            'first_attempt_failures':sum(r['response']['first_attempt_failed'] for r in rows),'rows':rows}


def validate_report(report, *, now=None):
    now=time.time() if now is None else now
    rows=report.get('rows',[])
    return (report.get('ready') is True and report.get('models')==MODELS
            and report.get('logical_requests')==6 and len(rows)==6
            and 0<=now-report.get('finished_unix',0)<=300
            and all(sum(row.get('provider')==provider and row.get('valid') is True
                        for row in rows)==3 for provider in MODELS))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--keychain-helper',type=Path,required=True)
    p.add_argument('--gemini-url',required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    key=subprocess.check_output([str(a.keychain_helper.resolve())],stderr=subprocess.DEVNULL,timeout=15).decode().strip()
    if not key:raise ValueError('missing model credential')
    report=check(key,a.gemini_url);write_json(a.output,report)
    print(json.dumps({k:report[k] for k in ('ready','logical_requests','http_attempts','first_attempt_failures')}))
    return 0 if report['ready'] else 1


if __name__=='__main__':raise SystemExit(main())
