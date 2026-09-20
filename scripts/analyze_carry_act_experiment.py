"""Post-run measurements only; never policy inputs."""
import argparse,hashlib,json,math
from pathlib import Path
import numpy as np
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--out',type=Path,required=True)
OUT=parser.parse_args().out.resolve()
def read(p):return json.loads(p.read_text())
def samples(p):return [json.loads(l) for l in p.open()]
def nearest(ss,t):return min(ss,key=lambda s:abs(s['sim_time_s']-t))
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
audit=read(OUT/'audit.json');data=read(OUT/'dataset.json');report=read(OUT/'report.json');rows=[]
for run in report['runs']:
 if run['phase']!='final':continue
 p=Path(run['output']);r=read(p/'result.json');ss=samples(p/'referee-only.jsonl');setup=read(p/'episode-setup-only.json');plan=r['plan'];slot=setup['static_map']['docks'][plan['dock']]['slots']['beam'];cargo=ss[-1]['cargo']['beam'];corners=np.array(cargo['corners'])[:,:2];margin=np.array(slot['half_extents_m'])-np.abs(corners-slot['center_m']);act=[v for v in read(p/'pair-decisions.json') if v['kind']=='act_carry'];cmd=read(p/'issued-commands.json')
 row={'condition':run['condition'],'case':run['case']['id'],'minimum_beam_slot_margin_m':float(margin.min()),'final_beam_center_error_m':(np.array(cargo['position'][:2])-slot['center_m']).tolist(),'cost_usd':r['cost_usd'],'external_llm_calls':r['llm_calls'],'commands_all_robots':sum(map(len,cmd.values())),'all_robots_carry_commands':sum(c['stage']=='TRANSIT' for cc in cmd.values() for c in cc),'pair_carry_commands':sum(c['stage']=='TRANSIT' for rid in r['bindings']['pair_model_slots'].values() for c in cmd[rid]),'wall_s':r['wall_s']}
 if act:
  row.update({'decision_rounds':len(act),'one_robot_done_rounds':sum(sum(d['done'] for d in a['decisions'].values())==1 for a in act),'both_done_rounds':sum(all(d['done'] for d in a['decisions'].values()) for a in act),'any_pause_rounds':sum(any(d['done'] for d in a['decisions'].values()) for a in act),'last_permission':act[-1]['permission']})
 source=next(e for e in data['train'] if Path(e['root']).name==run['case']['teacher_case']);tp=Path(source['root']);tr=read(tp/'result.json');ts=samples(tp/'referee-only.jsonl');tw=tr['evaluation']['cargo']['beam']['carry_clearance']['window_s'];rw=r['evaluation']['cargo']['beam']['carry_clearance']['window_s']
 if rw and tw:
  before=nearest(ts,tw[0]);after=nearest(ss,rw[0]);row['carry_entry_beam_difference_from_training_m']=float(np.linalg.norm(np.array(after['cargo']['beam']['position'])-before['cargo']['beam']['position']));row['carry_entry_robot_differences_from_training_m']={rid:float(np.linalg.norm(np.array(after['robots'][rid])-before['robots'][rid])) for rid in after['robots']}
 row['source_train_case']=str(tp);rows.append(row)
result={'scope':'Post-run referee analysis only; never actor input. Negative slot margin means corners outside target. Initial offsets do not imply equally large loaded-state changes.','rows':rows,'report_sha256':sha(OUT/'report.json'),'audit_sha256':sha(OUT/'audit.json')}
(OUT/'diagnostics.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
print(json.dumps(result,indent=2))
