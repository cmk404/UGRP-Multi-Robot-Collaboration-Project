"""Audit saved skill trials without model calls, actuation or referee feedback."""
from __future__ import annotations
import argparse,collections,gzip,hashlib,json,math,statistics,sys
from pathlib import Path
import numpy as np
SOURCE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(SOURCE))
from harness.jev_skill_motion import SkillObserver,request,parse_answer,request_compatible
from scripts.run_jev_motion import physical_goal_confirmed
from sim.camera_robot_port import CameraRobotPort

from harness.jev_skill_motion import SkillController, bounded_command, reference_choice

def replay_control(rows,setup,job):
    """Replay the controller from saved RGB estimates and actual model choices."""
    c=SkillController(setup['static_map'],always_query=job['arm']=='always',primitive=job['arm']=='primitive')
    verified=0;pending=None
    for row in rows:
        o=row['observation']
        if row.get('execution_source')=='inference_brake':
            if o.get('valid'):c.done=c.gate.update(o)
            else:c.gate.count=0;c.done=False
            assert row['command']==bounded_command()
            verified+=1
            continue
        fresh='request_origin_tick' in row
        if pending and not fresh:
            assert 'discard_reason' in pending
            c.active=None;c.path=[];pending=None
        if not o.get('valid'):
            c.active=None;c.path=[];c.gate.count=0
            assert row['command']==bounded_command()
            verified+=1
            continue
        if not fresh:c.observe_progress(o)
        state=c.state(o)
        assert state==row['state'],(row['tick'],'state mismatch')
        if c.done:
            if fresh:
                assert pending and pending['tick']==row['request_origin_tick']
                assert pending.get('response_unused_reason')=='RGB_completion'
                pending=None
            assert row['command']==bounded_command()
            verified+=1
            continue
        inside=.27<=o['range_m']<=.28 and abs(o['bearing_deg'])<=3
        reason=None if inside or fresh else c.need_query(state)
        if row.get('execution_source')=='request_only':
            assert reason==row['query_reason'];pending=row
            continue
        if fresh:
            assert pending and pending['tick']==row['request_origin_tick']
            pending=None
        elif reason:assert reason==row['query_reason']
        if 'command' not in row:
            assert job.get('error'),(row['tick'],'missing command without an error')
            continue
        if reason or fresh:
            answers=row.get('answers',{})
            if job['policy']=='rule':chosen=reference_choice(state)
            else:
                chosen=answers['action']
                if answers.get('evidence')=='observe_again' and job['arm']!='primitive':chosen='hold_and_observe'
            assert row['chosen_skill']==chosen
            c.select(chosen,o,row.get('confidence') if job['arm']!='no_confidence' else None)
            if answers.get('progress')=='reconsider':c.cautious=True
        command,execution=c.command(o)
        assert command==row['command'],(row['tick'],'command mismatch')
        assert execution==row['execution_source'],(row['tick'],'execution mismatch')
        assert c.active==row['active_skill'],(row['tick'],'active skill mismatch')
        verified+=1
    return verified


def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def write(path,value):path.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n')

def audit(root,*,replay=True):
 protocol=json.loads((root/'protocol.json').read_text());trials=[];manifest={};errors=[];turns_out=[]
 port=object.__new__(CameraRobotPort);port._allow_reverse=True;port._allow_mecanum=True
 for job in protocol['jobs']:
  p=root/job['trial_id'];rf=p/'result.json'
  if not rf.exists():errors.append({'trial':job['trial_id'],'error':'missing_result'});continue
  result=json.loads(rf.read_text());rows=json.loads((p/'turns.json').read_text())
  try:
   assert result['source_sha']==protocol['source_sha']
   assert result['case']==job['case'] and result['policy']==job['policy'] and result['arm']==job['arm']
   setup=json.loads((p/'setup-only.json').read_text());ident=json.loads((p/'identity.json').read_text())
   observer=SkillObserver(setup['static_map'],(p/'rgb/probe-before-top.jpg').read_bytes(),
      (p/'rgb/probe-after-top.jpg').read_bytes(),np.array(ident['claim']['center'])*[959,719]) if replay else None
   calls=0;latencies=[];tokens_in=tokens_out=0;replayed=0;hashed=0;rejected=0
   previous_o=None;request_rows={};candidate_counts=[];discarded=0
   for row in rows:
    for item in row['images'].values():
     path=p/item['path'];assert digest(path)==item['sha256'];hashed+=1
    o=row['observation']
    if replay:
     if 'request_origin_tick' in row:
      assert o==previous_o
     else:
      try:
       rebuilt=observer.observe((p/row['images']['own_rgb']['path']).read_bytes(),(p/row['images']['shared_top_rgb']['path']).read_bytes())
       assert o.get('valid') is True
       assert rebuilt==o,(row['tick'],'RGB reconstruction mismatch')
       replayed+=1;previous_o=rebuilt
      except ValueError as exc:
       assert o.get('valid') is False and str(exc)==o['reason']
       observer.recent.clear();previous_o=None
    if 'command' in row:port.validate_bounded(row['command'],.2)
    if 'state' in row:
     state=row['state'];assert set(state)=={'task','observation','active_skill','skill_age_ticks','no_progress_ticks','persistent_stall','detour_note','recent_issued','candidates','input_boundary'}
     assert set(state['observation'])=={'distance','target_side','alignment','range_trend','quality','goal_distance_edge','goal_alignment_edge','stationary_evidence'}
     candidate_counts.append(sum(k not in ('hold_and_observe','stop') for k in state['candidates']))
    if 'request' in row:
     calls+=1;expected=request(row['state'],job['policy'],decomposed=job['arm']!='single')
     assert row['request']==expected
     request_rows[row['tick']]=row
     if row.get('response'):
      response=row['response'];latencies.append(response['latency_s'])
      if response['status']=='ok' and not row.get('response_unused_reason'):
       try:
        answers,confidence=parse_answer(response['body'],job['policy'],row['state'],decomposed=job['arm']!='single')
        assert row.get('answers',answers)==answers
       except (ValueError,KeyError,TypeError) as exc:
        assert result['stop_reason']=='error' and result['error']['type']==type(exc).__name__
        rejected+=1
      body=response.get('body') or {};usage=body.get('usage',{}) if isinstance(body,dict) else {}
      tokens_in+=usage.get('input_tokens',usage.get('prompt_tokens',0));tokens_out+=usage.get('output_tokens',usage.get('completion_tokens',0))
     if 'discard_reason' in row:discarded+=1
    if 'request_origin_tick' in row:
     origin=request_rows[row['request_origin_tick']]
     if row.get('execution_source')=='RGB_completion':
      assert origin.get('response_unused_reason')=='RGB_completion'
      assert row['command']==bounded_command()
     else:
      assert request_compatible(origin['state'],row['state'])
      assert row['response_age_sim_s']<=4.
   assert calls==len(list(p.glob('*-request.json')))
   controlled=replay_control(rows,setup,{**job,'error':result['error']})
   assert calls==result['model_calls'],('calls',calls,result['model_calls'])
   assert tokens_in==result['input_tokens'] and tokens_out==result['output_tokens']
   referee=json.loads((p/'referee-only.json').read_text())
   expected_success=bool(result['stop_reason']=='RGB_goal_confirmed' and physical_goal_confirmed(referee)
      and not any(result[k] for k in ('weld_steps','cargo_contact_steps','peer_contact_steps','obstacle_contact_steps')) and result['camera_geometry_unchanged'])
   assert result['success']==expected_success
   result={**result,'audit':{'replayed_rgb_observations':replayed,'verified_input_image_hashes':hashed,
       'replayed_controller_commands':controlled,'schema_rejected_responses':rejected,'request_count':calls,'stale_responses_discarded':discarded,'candidate_count_histogram':dict(collections.Counter(candidate_counts)),
       'api_latencies_s':latencies,'passed':True}}
  except Exception as exc:
   errors.append({'trial':job['trial_id'],'error':type(exc).__name__+': '+str(exc)})
   result={**result,'audit':{'passed':False}}
  trials.append(result);turns_out.append({'trial_id':job['trial_id'],'turns':rows})
  for path in p.rglob('*'):
   if path.is_file():manifest[str(path.relative_to(root))]={'sha256':digest(path),'bytes':path.stat().st_size}
  print(json.dumps({'audited':len(trials),'planned':len(protocol['jobs']),'errors':len(errors),'trial':job['trial_id']}),flush=True)
 initial={}
 for job in protocol['jobs']:
  p=root/job['trial_id']/ 'rgb'
  paths=[p/'probe-before-r2.jpg',p/'probe-before-top.jpg',p/'probe-after-r2.jpg',p/'probe-after-top.jpg']
  if all(x.exists() for x in paths):initial.setdefault(job['case'],set()).add(tuple(digest(x) for x in paths))
 mismatches=[case for case,values in initial.items() if len(values)!=1]
 if mismatches:errors.append({'error':'paired_initial_RGB_mismatch','cases':mismatches})
 groups={}
 for r in trials:groups.setdefault(r['policy']+'-'+r['arm'],[]).append(r)
 summary={}
 for key,rs in groups.items():
  lat=[v for r in rs for v in r['audit'].get('api_latencies_s',[])]
  successes=[r for r in rs if r['success']]
  summary[key]={'trials':len(rs),'successes':len(successes),'stop_reasons':dict(collections.Counter(r['stop_reason'] for r in rs)),
      'calls':sum(r['model_calls'] for r in rs),'mean_calls':statistics.mean(r['model_calls'] for r in rs),
      'mean_wall_s':statistics.mean(r['wall_s'] for r in rs),'mean_sim_s':statistics.mean(r['sim_s'] for r in rs),
      'success_median_calls':statistics.median(r['model_calls'] for r in successes) if successes else None,
      'api_p50_s':statistics.median(lat) if lat else None,'api_p95_s':float(np.quantile(lat,.95)) if lat else None,
      'input_tokens':sum(r['input_tokens'] for r in rs),'output_tokens':sum(r['output_tokens'] for r in rs),'cost_usd':None}
 return {'protocol':protocol,'summary':summary,'trials':trials,'audit_errors':errors,'paired_initial_RGB_cases':len(initial),'paired_initial_RGB_mismatches':mismatches,'complete':len(trials)==len(protocol['jobs'])},manifest,turns_out

def main():
 p=argparse.ArgumentParser();p.add_argument('root',type=Path);p.add_argument('--output',type=Path,required=True);p.add_argument('--skip-rgb-replay',action='store_true');args=p.parse_args()
 args.output.mkdir(parents=True,exist_ok=True)
 report,manifest,turns=audit(args.root,replay=not args.skip_rgb_replay)
 write(args.output/'report.json',report)
 for name,data in [('raw-manifest.json.gz',manifest),('turns.json.gz',turns)]:
  with gzip.open(args.output/name,'wt') as f:json.dump(data,f,ensure_ascii=False,separators=(',',':'))
 print(json.dumps({'complete':report['complete'],'errors':report['audit_errors'],'summary':report['summary']}),flush=True)
 return int(bool(report['audit_errors']) or not report['complete'])
if __name__=='__main__':raise SystemExit(main())
