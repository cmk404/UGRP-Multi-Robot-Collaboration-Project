"""Audit recorded requests/wires against original permitted RGB and history."""
import base64
import hashlib
import json
from pathlib import Path
import sys


def audit(root):
    root=Path(root)
    raw={}
    for f in (root/'rgb').glob('*.jpg'):
        raw.setdefault(hashlib.sha256(f.read_bytes()).hexdigest(),[]).append(f.name)
    requests={}
    errors=[]
    images_checked=0
    for f in sorted((root/'team').glob('*/*-request.json')):
        req=json.loads(f.read_text());rid=f.parent.name
        context=json.loads(req['messages'][1]['content'])
        reqid=req['request_id']
        if any(k in json.dumps(context) for k in ('setup_only','unexpected_obstacles','spawns','qpos','evaluation-only')):
            errors.append(f'{reqid}: forbidden context key')
        if 'north_blocked' in reqid or any(k in reqid for k in ('seed','shared_crossing')):
            errors.append(f'{reqid}: identifier condition leak')
        expected=[]
        for image in req['images']:
            data=base64.b64decode(image['image'].split(',',1)[1]);sha=hashlib.sha256(data).hexdigest()
            names=raw.get(sha,[]);label=image['label'].lower();images_checked+=1
            own=('own' in label and 'top' not in label)
            if not names or (own and not any(n.endswith('-'+rid+'.jpg') for n in names)) or (
                    not own and not any(n.endswith('-top.jpg') for n in names)):
                errors.append(f'{reqid}: non-owned/non-original image {label}')
            expected.append(sha)
        requests[reqid]={'sha':expected,'context':context,'robot':rid,'file':str(f.relative_to(root))}
    wires=0
    for f in sorted((root/'team').glob('*/wire-*.json')):
        if f.stem.endswith('-response'):continue
        body=json.loads(f.read_text());content=body['messages'][-1]['content']
        context=json.loads(content[0]['text']);reqid=context['request_id']
        wire_images=[hashlib.sha256(base64.b64decode(x['image_url']['url'].split(',',1)[1])).hexdigest()
            for x in content if x.get('type')=='image_url']
        if reqid not in requests or context!=requests[reqid]['context'] or wire_images!=requests[reqid]['sha']:
            errors.append(f'{f}: wire mismatch')
        wires+=1
    result=json.loads((root/'result.json').read_text())
    history=json.loads((root/'issued-commands.json').read_text())
    for reqid,request in requests.items():
        rid=request['robot'];context=request['context']
        for command in context.get('own_issued_commands',[]):
            if command not in history[rid]:errors.append(reqid+': history not owned or mutated')
        if '-execute-' in reqid:
            n=int(reqid.rsplit('-',1)[1]);turn=result['turns'][n]
            if any(c.get('issued_at_s',-1)>turn['at_s'] for c in context['own_issued_commands']):
                errors.append(reqid+': future command history')
    return {'run':str(root.resolve()),'source_sha':result['source_sha'],'requests':len(requests),
        'actual_wires':wires,'original_images_checked':images_checked,
        'passed':not errors,'errors':errors,
        'scope':'all saved request and wire image bytes, allowed context, own command ownership and temporal ordering; no claim of correct visual interpretation'}

if __name__=='__main__':
    print(json.dumps([audit(p) for p in sys.argv[1:]],indent=2))
