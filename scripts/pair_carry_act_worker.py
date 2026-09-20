"""Strict two-image plus authored/issued-context subprocess protocol."""
import argparse,base64,json,math,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))

def decode_request(r):
    if not isinstance(r,dict) or set(r)!={'own_rgb','top_rgb','context'}:raise ValueError('unapproved worker field')
    if not isinstance(r['context'],list) or len(r['context'])!=8 or not all(type(v) in (int,float) and math.isfinite(v) and abs(v)<=2 for v in r['context']):raise ValueError('invalid context')
    if any(not isinstance(r[k],str) or len(r[k])>8000000 for k in ('own_rgb','top_rgb')):raise ValueError('invalid image')
    return base64.b64decode(r['own_rgb'],validate=True),base64.b64decode(r['top_rgb'],validate=True),r['context']

def main():
    p=argparse.ArgumentParser();p.add_argument('--model-dir',type=Path,required=True);a=p.parse_args()
    import torch
    from harness.pair_carry_act import CarryAct
    torch.set_num_threads(2);actor=CarryAct.load(a.model_dir)
    print(json.dumps({'ready':True,'runtime_inputs':['own_rgb','top_rgb','context']}),flush=True)
    for line in sys.stdin:
        try:result=actor.predict(*decode_request(json.loads(line)))
        except Exception as e:
            print(json.dumps({'error':str(e)}),flush=True);return 1
        print(json.dumps(result,allow_nan=False),flush=True)
    return 0
if __name__=='__main__':raise SystemExit(main())
