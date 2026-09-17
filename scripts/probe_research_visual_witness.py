#!/usr/bin/env python3
"""Offline image-only witness check on explicitly labelled archive fixtures.

Labels/source descriptions are evaluation-only. No simulator or actions run.
Teacher-origin positive images test witness sensitivity, not student autonomy.
"""
import argparse
import base64
import hashlib
import io
import json
from pathlib import Path
import subprocess
import sys
from urllib.request import urlopen

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from harness.gemini_proxy import GeminiProxyCompleter, _to_gemini_multi_image_messages
from harness.research_execution_recovery import request_with_recovery
from harness.research_visual_evidence import build_review_request, validate_review, supports_claim


def run(manifest, output):
    if subprocess.check_output(['git','status','--porcelain'],cwd=ROOT,text=True).strip():
        raise RuntimeError('commit source before probing')
    output.mkdir(parents=True,exist_ok=False)
    sha=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
    def save(path,value):path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')
    cases=json.loads(manifest.read_text())
    results=[]
    for index,case in enumerate(cases):
        folder=output/f'case-{index:02d}';folder.mkdir()
        def packet(name):
            images={}
            for label,relative in case[name].items():
                source=(manifest.parent/relative).resolve();raw=source.read_bytes()
                images[label]={'jpeg_base64':base64.b64encode(raw).decode(),
                    'ref':str(source),'sha256':hashlib.sha256(raw).hexdigest()}
            return {'images':images}
        arguments=dict(phase=case['phase'],roles=case['roles'],camera=packet('current'),previous=packet('previous'))
        wire_count=0;requests=[];calls=[]
        def http_open(req,*,timeout):
            nonlocal wire_count
            wire_count+=1
            (folder/f'wire-{wire_count}.json').write_bytes(req.data)
            with urlopen(req,timeout=timeout) as response:raw=response.read()
            (folder/f'wire-{wire_count}-response.json').write_bytes(raw)
            return io.BytesIO(raw)
        completer=GeminiProxyCompleter(model='gemini-3.8-flash',max_tokens=900,timeout=30.,
                                      reasoning_effort='none',http_open=http_open)
        def make_request(attempt,retry):
            request_id=f'witness-probe-{index}-a{attempt}'
            inputs={**arguments,'request_id':request_id,'retry':retry}
            request=build_review_request(case['robot_id'],**inputs)
            save(folder/f'inputs-{attempt}.json',inputs);save(folder/f'request-{attempt}.json',request)
            requests.append(request)
            return {**request,'request_id':request_id}
        answer,stop=request_with_recovery(completer,make_request,validate_review,calls.append)
        for i,request in enumerate(requests,1):
            wire=json.loads((folder/f'wire-{i}.json').read_text())
            assert wire=={'model':'gemini-3.8-flash','messages':_to_gemini_multi_image_messages(request['messages'],request['images']),
                          'temperature':.2,'max_tokens':900,'reasoning_effort':'none'}
        result={'case':case,'review':answer,'supported':supports_claim(answer,case['phase']),
                'stop_reason':stop,'calls':calls,'exact_wire_audited':True}
        save(folder/'result.json',result);results.append(result)
        print(json.dumps({k:result[k] for k in ('supported','review','stop_reason')},ensure_ascii=False),flush=True)
    report={'source_sha':sha,'manifest_sha256':hashlib.sha256(manifest.read_bytes()).hexdigest(),
            'scope':'archived image classification only; teacher-origin positives are not autonomous execution',
            'cases':results,'files':{str(p.relative_to(output)):hashlib.sha256(p.read_bytes()).hexdigest()
                                    for p in output.rglob('*') if p.is_file()}}
    save(output/'result.json',report)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifest',type=Path);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();run(args.manifest.resolve(),args.output.resolve())
