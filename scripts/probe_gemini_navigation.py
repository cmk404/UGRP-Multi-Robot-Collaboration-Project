"""Explicit frozen-camera Gemini probe. No robot commands or delivery claims."""
from __future__ import annotations
import argparse, base64, json
from pathlib import Path
from harness.gemini_transport_policy import GeminiTransportPlanner
from harness.gemini_proxy import GeminiProxyCompleter
from harness.visual_drive_guard import validate_visual_drive


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--execute',action='store_true')
    args=ap.parse_args()
    if not args.execute: ap.error('--execute is required to consume actual Gemini quota')
    args.output.mkdir(parents=True,exist_ok=False)
    rows=[json.loads(l) for l in (args.run/'llm-decisions.jsonl').read_text().splitlines()]
    req=[r for r in rows if r.get('event')=='llm_request']
    res={r['call_id']:r for r in rows if r.get('event')=='llm_result'}
    provider=GeminiProxyCompleter(model='gemini-3.8-flash',max_tokens=800,timeout=30)
    total=0; records=[]
    for target in (7,11,12):
        if total+7000>21000: break
        class Switch:
            model_name='gemini-3.8-flash'
            last_model=None
            last_usage=None
            real=False
            reply=None
            def complete(self,messages,*,images):
                if self.real:
                    answer=provider.complete(messages,images=images)
                    self.last_usage=provider.last_usage;self.last_model=provider.last_model
                    return answer
                return self.reply
        switch=Switch();planner=GeminiTransportPlanner('r1',switch)
        for index,r in enumerate(req[:target],1):
            obs=[]
            for e in r['images']:
                raw=(args.run/e['path']).read_bytes()
                obs.append({'robot_id':'r1','camera':'robot_cam' if e['camera']=='wrist' else 'nav_cam',
                    'frame_id':index,'sim_time':r['time'],'image':base64.b64encode(raw).decode(),
                    'sha256':e['sha256'],'actuator_state':{}})
            switch.real=index==target
            switch.reply=res[r['call_id']]['audit']['raw_text']
            decision=planner.decide(*obs,r['memory'],**r['task'],skill_state=r['skill_state'],
                                    budget=r['budget'],execution_feedback=r['execution_feedback'])
            if switch.real:
                usage=planner.last_audit.get('usage') or {}
                tokens=usage.get('prompt_tokens')
                record={'frozen_source_run':str(args.run.resolve()),'call_id':r['call_id'],
                    'decision':decision,'guard':validate_visual_drive(obs[1],decision['action']),
                    'audit':planner.last_audit,'executed':False}
                records.append(record)
                (args.output/f'probe-{target:02d}.json').write_text(json.dumps(record,ensure_ascii=False,indent=2))
                print(json.dumps({'call':target,'decision':decision,'guard_allowed':record['guard']['allowed'],
                                  'tokens':tokens},ensure_ascii=False),flush=True)
                if not isinstance(tokens,int):raise RuntimeError('UNKNOWN_USAGE_STOP')
                total+=tokens
    (args.output/'summary.json').write_text(json.dumps({'actual_calls':len(records),'input_tokens':total,
       'all_proposed_actions_pass_current_guard':all(r['guard']['allowed'] for r in records),
       'physical_commands':0,'delivery_verified':False},indent=2))

if __name__=='__main__':main()
