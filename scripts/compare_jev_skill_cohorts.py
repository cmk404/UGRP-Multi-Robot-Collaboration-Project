"""Aggregate all frozen jobs and matched ablations; no model/physics access."""
import argparse,collections,json,statistics
from pathlib import Path

def avg(xs):return round(statistics.mean(xs),3) if xs else None
def summarize(rs):
 return {'n':len(rs),'successes':sum(r['success'] for r in rs),
   'calls':sum(r['model_calls'] for r in rs),'mean_calls':avg([r['model_calls'] for r in rs]),
   'mean_wall_s':avg([r['wall_s'] for r in rs]),'mean_sim_s':avg([r['sim_s'] for r in rs]),
   'mean_commands':avg([r['commands'] for r in rs]),
   'input_tokens':sum(r['input_tokens'] for r in rs),'output_tokens':sum(r['output_tokens'] for r in rs),
   'contacts':sum(r.get(k,0) for r in rs for k in ['cargo_contact_steps','peer_contact_steps','obstacle_contact_steps']),
   'weld_steps':sum(r.get('weld_steps',0) for r in rs),'cost_usd':None,
   'stop_reasons':dict(collections.Counter(r['stop_reason'] for r in rs))}

def main():
 p=argparse.ArgumentParser();p.add_argument('--root',type=Path,action='append',required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
 results=[];protocols={};groups={};families={};failures=[];incomplete=[];archive={}
 for root in a.root:
  protocol=json.loads((root/'protocol.json').read_text());phase=protocol['phase'];protocols[phase]=protocol
  for job in protocol['jobs']:
   d=root/job['trial_id'];f=d/'result.json'
   if not f.exists():incomplete.append([phase,job['trial_id']]);continue
   r=json.loads(f.read_text());r['partition']=phase;r['trial_id']=job['trial_id'];results.append(r)
   rows=json.loads((d/'turns.json').read_text());archive[(phase,job['trial_id'])]=rows
   key='/'.join([phase,r['policy'],r['arm']]);groups.setdefault(key,[]).append(r)
   if phase=='holdout':families.setdefault('/'.join([r['case'].rstrip('0123456789'),r['policy']]),[]).append(r)
   if not r['success']:
    valid=[x for x in rows if x.get('observation',{}).get('valid')];last=valid[-1] if valid else {}
    requests=[x for x in rows if 'request' in x]
    failures.append({'partition':phase,'trial_id':job['trial_id'],'stop_reason':r['stop_reason'],'error':r['error'],
      'final_evaluation':r.get('final_evaluation'),'last_RGB':{k:last.get('observation',{}).get(k) for k in ['range_m','bearing_deg','quality']},
      'last_state':last.get('state'),'recent_choices':[x.get('answers') for x in requests[-5:]],
      'execution_counts':dict(collections.Counter(x.get('execution_source',x.get('decision_source')) for x in rows)),
      'invalid_RGB':dict(collections.Counter(x['observation']['reason'] for x in rows if x.get('observation',{}).get('valid') is False)),
      'contacts':{k:r.get(k) for k in ['cargo_contact_steps','peer_contact_steps','obstacle_contact_steps']}})
 full={(r['case'],r.get('repeat',0),r['policy']):r for r in results if r['partition']=='holdout'}
 ablation=[r for r in results if r['partition']=='ablation']
 always={(r['case'],r.get('repeat',0),r['policy']):r for r in ablation if r['arm']=='always'}
 matched={}
 for arm in ['always','single','no_confidence','primitive']:
  for policy in ['jev','gemini']:
   pairs=[]
   for r in ablation:
    if r['arm']!=arm or r['policy']!=policy:continue
    base=(always if arm=='primitive' else full).get((r['case'],r.get('repeat',0),policy))
    if base:pairs.append((base,r))
   if not pairs:continue
   matched[policy+'/'+arm]={'baseline':'always-skill' if arm=='primitive' else 'full-event-skill',
      'baseline_summary':summarize([x for x,y in pairs]),'ablation_summary':summarize([y for x,y in pairs]),
      'both_success':sum(x['success'] and y['success'] for x,y in pairs),
      'baseline_only_success':sum(x['success'] and not y['success'] for x,y in pairs),
      'ablation_only_success':sum(y['success'] and not x['success'] for x,y in pairs),
      'pairs':[[x['trial_id'],y['trial_id']] for x,y in pairs]}
 contribution={}
 for key,rs in groups.items():
  rows=[row for r in rs for row in archive[(r['partition'],r['trial_id'])]]
  requests=[r for r in rows if 'request' in r]
  choices=collections.Counter(r.get('chosen_skill') for r in rows if 'chosen_skill' in r)
  optioncounts=collections.Counter(sum(k not in ('hold_and_observe','stop') for k in r['state']['candidates']) for r in requests)
  contribution[key]={'chosen_skills':dict(choices),'moving_candidate_keys_per_request':dict(optioncounts),
      'execution_counts':dict(collections.Counter(r.get('execution_source',r.get('decision_source')) for r in rows)),
      'discarded_responses':sum('discard_reason' in r for r in rows),
      'identical_consecutive_request_pairs':sum(x['request']==y['request'] for r in rs for seq in [[x for x in archive[(r['partition'],r['trial_id'])] if 'request' in x]] for x,y in zip(seq,seq[1:]))}
 report={'protocols':protocols,'complete':not incomplete,'missing':incomplete,'results':results,
    'summary':{k:summarize(v) for k,v in groups.items()},'families':{k:summarize(v) for k,v in families.items()},
    'matched_ablation':matched,'contribution':contribution,'failures':failures}
 a.output.mkdir(parents=True,exist_ok=True);(a.output/'comparison.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
 print(json.dumps({'complete':not incomplete,'trials':len(results),'failures':len(failures),'summary':report['summary']},ensure_ascii=False))
if __name__=='__main__':main()
