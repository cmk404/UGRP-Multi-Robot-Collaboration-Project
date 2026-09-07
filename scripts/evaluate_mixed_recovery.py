"""Matched mixed-task recovery pilot; retain failures and missing scenario triggers."""
import argparse
from pathlib import Path
import json
import time

from harness.gemini_proxy import GeminiProxyCompleter
from harness.mixed_warehouse_runtime import MixedLLMPolicy,MixedRulePolicy,run_mixed_episode
from harness.warehouse_runtime import LocalEnvironment
from sim.multi_masterpi_production import MultiMasterPiProductionV2
from scripts.evaluate_warehouse_research import source_signature


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--seeds',default='11')
    p.add_argument('--conditions',default='rule,llm_no_comm,llm_peer_comm')
    p.add_argument('--scenarios',default='normal,joint_obstacle')
    p.add_argument('--output',required=True)
    args=p.parse_args();out=Path(args.output);out.mkdir(parents=True,exist_ok=False)
    signature=source_signature();design={'seeds':[int(v) for v in args.seeds.split(',')],
       'conditions':args.conditions.split(','),'scenarios':args.scenarios.split(','),
       'max_calls_per_robot':8,'timeout_s':150.,'response_tokens':384,
       'source_signature':signature,'obstacle_source':'controlled_scenario_event; not claimed as camera perception',
       'controller':'ideal_sim_geometry; calibrated sensor-only control is not available'}
    (out/'design.json').write_text(json.dumps(design,indent=2));results=[]
    for seed in design['seeds']:
        for scenario in design['scenarios']:
            for condition in design['conditions']:
                w=MultiMasterPiProductionV2(seed=seed,warehouse_layout='mixed',render=True)
                injected=[];started=time.monotonic()
                def hook(env,eid,status):
                    if scenario!='joint_obstacle' or injected:return
                    a=next((a for a in status.get('active',[]) if len(a['participants'])==2),None)
                    if a and status.get('activities') and w.data.time>14:
                        event=env.request('r1',{'operation':'mixed_obstacle','episode_id':eid,
                                              'assignment_id':a['assignment_id'],'position_xy':[.72,-.64]})
                        if not event.get('event_id'):raise RuntimeError('SCENARIO_INJECTION_FAILED')
                        injected.append(event)
                try:
                    policies={r:MixedRulePolicy(r) if condition=='rule' else MixedLLMPolicy(r,GeminiProxyCompleter(max_tokens=384,timeout=30)) for r in w.robot_ids}
                    result=run_mixed_episode(LocalEnvironment(w),policies,condition=condition,
                        max_calls_per_robot=8,timeout_s=150,journal_path=out/f'{seed}-{scenario}-{condition}.jsonl',scenario_hook=hook)
                    result.update(seed=seed,scenario=scenario,scenario_triggered=bool(injected),
                                  event_recovered=any(e.get('event')=='recovered' for e in w._mixed_engine.recovery_history))
                    if scenario=='joint_obstacle' and (not injected or not result['event_recovered']):
                        result['success']=False;result['reason']='RECOVERY_NOT_VERIFIED'
                except Exception as exc:
                    result={'condition':condition,'seed':seed,'scenario':scenario,'success':False,'reason':str(exc),'elapsed_s':time.monotonic()-started}
                finally:w.close()
                results.append(result)
                if source_signature()['sha256']!=signature['sha256']:raise RuntimeError('SOURCE_CHANGED_DURING_COMPARISON')
                summary={c:{'successes':sum(r['success'] for r in results if r['condition']==c),
                            'denominator':len(design['seeds'])*len(design['scenarios']),
                            'recorded':sum(r['condition']==c for r in results)} for c in design['conditions']}
                (out/'summary.json').write_text(json.dumps({'results':results,'summary':summary},ensure_ascii=False,indent=2))
                print(seed,scenario,condition,result['success'],result['reason'],flush=True)

if __name__=='__main__':main()
