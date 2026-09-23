"""Append immutable post-run measurement evidence without changing raw results."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from harness.dispatch_evaluation import concurrent_transport


def audit(source):
    source=Path(source)
    names=('result.json','issued-commands.json','referee-only.jsonl')
    files={name:(source/name).read_bytes() for name in names}
    result=json.loads(files['result.json']);history=json.loads(files['issued-commands.json'])
    samples=[json.loads(line) for line in files['referee-only.jsonl'].splitlines()]
    record={'evaluator_source_sha':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
            'source_files_sha256':{n:hashlib.sha256(v).hexdigest() for n,v in files.items()},
            'execution_source_sha':result.get('source_sha'),
            'original_reported':result.get('evaluation',{}).get('concurrent_transport'),
            'concurrent_transport':concurrent_transport(samples,history,result.get('plan')),
            'reason':'Merge adjoining per-robot TRANSIT leases before comparing the common referee sample clock. Raw physics and original result remain unchanged.'}
    if any((source/n).read_bytes()!=v for n,v in files.items()):raise RuntimeError('source changed during audit')
    output=source/'concurrency-audit.json'
    with output.open('x') as file:json.dump(record,file,indent=2);file.write('\n')
    return record


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source',type=Path,required=True)
    print(json.dumps(audit(p.parse_args().source),indent=2))
