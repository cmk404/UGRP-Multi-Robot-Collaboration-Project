"""Explicit fixed solo seed cohort; freeze policy, retain every outcome."""
from __future__ import annotations
import argparse,hashlib,json,subprocess,sys
from pathlib import Path
from scripts.evaluate_gemini_cohort import run_cohort


def policy_hashes():
    return {str(p):hashlib.sha256(p.read_bytes()).hexdigest()
            for base in ('harness','sim','calibration') for p in sorted(Path(base).rglob('*'))
            if p.is_file() and p.suffix in ('.py','.xml','.json','.png','.yaml','.yml')
            and '__pycache__' not in p.parts}


def command(output,seed,reasoning_effort='none',request_timeout=30):
    return [sys.executable,'-m','scripts.evaluate_gemini_team','--output',str(output),
        '--seed',str(seed),'--robots','1','--seconds','300','--model','gemini-3.8-flash',
        '--max-calls','30','--max-input-tokens','120000','--input-request-estimate','6000',
        '--impratio','10','--noslip-iterations','3','--communication','none',
        '--request-timeout',str(request_timeout),'--max-transient-failures','2',
        '--reasoning-effort',reasoning_effort,'--record']


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--seeds',type=int,nargs='+',default=[42,43,44,45,46])
    ap.add_argument('--execute',action='store_true')
    ap.add_argument('--reasoning-effort',choices=('none','low','medium','high'),default='none')
    ap.add_argument('--request-timeout',type=float,default=30)
    args=ap.parse_args()
    if not 0 < args.request_timeout <= 300:ap.error('request timeout must be in (0, 300]')
    if not args.execute:ap.error('--execute is required for actual Gemini quota use')
    if len(set(args.seeds))!=len(args.seeds):ap.error('seeds must be unique')
    args.output.mkdir(parents=True,exist_ok=False)
    frozen=policy_hashes()
    runs=[{'path':f'solo-{s}','seed':s,'communication':'none',
           'reasoning_effort':args.reasoning_effort,'request_timeout':args.request_timeout} for s in args.seeds]
    manifest={'purpose':'fixed single-robot seed cohort, no within-cohort tuning or retries',
      'seeds':args.seeds,'robots':1,'max_calls':30,'max_input_tokens':120000,'seconds':300,
      'model':'gemini-3.8-flash','reasoning_effort':args.reasoning_effort,
      'request_timeout':args.request_timeout,
      'sequential':True,'policy_source':frozen,'runs':runs}
    (args.output/'validation-manifest.json').write_text(json.dumps(manifest,indent=2))
    def execute(run):
        if policy_hashes()!=frozen:
            print('POLICY_CHANGED_STOP',flush=True);return 3
        print(json.dumps({'event':'start','seed':run['seed']}),flush=True)
        with (args.output/(run['path']+'.log')).open('x') as log:
            code=subprocess.run(command(args.output/run['path'],run['seed'],run['reasoning_effort'],run['request_timeout']),stdout=log,stderr=subprocess.STDOUT).returncode
        path=args.output/run['path']/'result.json'
        if path.exists():
            r=json.loads(path.read_text());summary={'event':'finished','seed':run['seed'],
              'success':r.get('success'),'reason':r.get('outcomes',{}).get('r1',{}).get('reason'),
              'calls':r.get('llm_calls',{}).get('r1'),'input':r.get('input_usage',{}).get('r1',{}).get('reported_prompt_tokens')}
            print(json.dumps(summary),flush=True)
        return code
    state=run_cohort(args.output,runs,jobs=1,execute=execute)
    print(json.dumps(state),flush=True)
    return 2 if state['blocking_reason'] else 0

if __name__=='__main__':main()
