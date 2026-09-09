"""Verify recorded wire bodies contain only static task text and exact owned RGB."""
from pathlib import Path
import base64
import hashlib
import json
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

def audit(root):
    from harness.camera_pair_policy import static_task, _USER_TEXT
    metadata = root / 'result.json'
    if not metadata.exists():
        metadata = root / 'progress.json'
    task = json.loads(metadata.read_text()).get('config', {}).get('task', 'carry')
    result=[]
    for rid in ('r1','r3'):
        for path in sorted((root/rid).glob('wire-*.json')):
            index=int(path.stem.split('-')[1])-1
            payload=json.loads(path.read_text())
            assert set(payload)=={'model','messages','temperature','max_tokens','reasoning_effort'}
            messages=payload['messages'];assert len(messages)==2
            assert messages[0]=={'role':'system','content':static_task(rid,task)}
            content=messages[1]['content']
            assert messages[1]['role']=='user' and set(messages[1])=={'role','content'}
            assert len(content)==5
            assert content[0]=={'type':'text','text':_USER_TEXT}
            hashes={}
            for k,label,name in [(1,'OWN_VIEW','own'),(3,'OVERHEAD','overhead')]:
                assert content[k]=={'type':'text','text':label}
                image=content[k+1];assert set(image)=={'type','image_url'} and image['type']=='image_url'
                assert set(image['image_url'])=={'url'}
                uri=image['image_url']['url'];assert uri.startswith('data:image/jpeg;base64,')
                data=base64.b64decode(uri.split(',',1)[1],validate=True)
                assert data==(root/rid/f'{index:03d}-{name}.jpg').read_bytes()
                hashes[name]=hashlib.sha256(data).hexdigest()
            result.append({'robot_id':rid,'round':index,'wire':str(path.relative_to(root)),**hashes})
    assert result, 'no audited requests'
    for row in result:
        peer=next((x for x in result if x['robot_id']!=row['robot_id'] and x['round']==row['round']),None)
        if peer:assert peer['overhead']==row['overhead']
    return {'ok':True,'requests':len(result),'audit':result,
            'checked':'exact static text + exactly two original JPEGs, correct own view and identical overhead per round; no other observation fields'}
if __name__=='__main__':
    root=Path(sys.argv[1]).resolve();result=audit(root)
    (root/'input-audit.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='audit'}))
