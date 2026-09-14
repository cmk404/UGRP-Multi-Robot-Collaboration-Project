"""Output-only evidence packaging. Does not run physics or change an actor."""
from pathlib import Path
import csv
import gzip
import hashlib
import json
import subprocess
import sys
from PIL import Image, ImageDraw, ImageFont

ROOT=Path(__file__).resolve().parents[2]
RAW=Path(sys.argv[1]).resolve()
DEST=Path(sys.argv[2]).resolve();DEST.mkdir(parents=True,exist_ok=True)
GRASP=Path('/Users/changmin/projects/ugrp-worktrees/pair-loaded-navigation/outputs/pair-navigation/runtime/models/grasp')
sys.path.insert(0,str(ROOT))
from scripts.audit_pair_navigation import audit
from scripts.evaluate_pair_navigation import evaluate_grasp_stability
import math

def write(path,data): path.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n')
def sha(path): return hashlib.sha256(path.read_bytes()).hexdigest()

catalog=json.loads((ROOT/'maps/pair_navigation/catalog.json').read_text())
cohort=json.loads((RAW/'cohort.json').read_text())
close_pulse=cohort.get('close_pulse',1700)
catalog['entries']=[e for e in catalog['entries'] if e['id'] in [t['id'] for t in cohort['trials']]]
(RAW/'analysis').mkdir(exist_ok=True)
media=DEST/'media';media.mkdir(exist_ok=True)
results=[]; selection=[]
font='/System/Library/Fonts/AppleSDGothicNeo.ttc'
overview=Image.new('RGB',(1440,860),'#101b2b')
viewdraw=ImageDraw.Draw(overview)
for k,e in enumerate(catalog['entries']):
    p=RAW/e['id']; report=json.loads((p/'result.json').read_text())
    assert report['source_sha']==cohort['source_sha']
    if (p/'input-audit.json').exists(): checked=json.loads((p/'input-audit.json').read_text())
    else:
        checked=audit(p,GRASP);write(p/'input-audit.json',checked)
    assert checked['passed']
    final=report['steps'][-1]['decisions']
    safe_gates=['grasp_stable_before_departure','bilateral_every_sample','lifted_every_sample',
                'continuous_samples','level_payload','wall_contact_free','other_collision_free',
                'within_authored_bounds','weld_off','cameras_geometry_unchanged','full_grasp_samples','grasp_spacing_preserved','grasp_bilateral_through_lift_hold','grasp_hold_lifted_every_sample','carry_spacing_preserved']
    expected_stop=bool(not e['expected_geometric_route'] and report.get('refused_no_route') and
                       not report['error'] and all(report['evaluation']['gates'][g] for g in safe_gates))
    r={key:val for key,val in report.items() if key not in ['steps','sync_events','spacing_steps','spacing_sync_events']}
    r.update(id=e['id'],title_ko=e['title_ko'],expected_geometric_route=e['expected_geometric_route'],
             expected_blocked_stop_passed=expected_stop,decision_rounds=len(report['steps']),
             robot_decision_commands=2*(len(report['steps'])+len(report['spacing_steps'])),spacing_rounds=len(report['spacing_steps']),final_decisions=final,input_audit=checked,
             raw_directory=str(p),raw_result_sha256=sha(p/'result.json'))
    eval_rows=[json.loads(x) for x in (p/'evaluation-only.jsonl').read_text().splitlines()]
    anchor=r['grasp_stability']['anchor_spacing_m']
    r['max_carry_spacing_change_m']=max(abs(math.dist(x['bases']['r1'][:2],x['bases']['r3'][:2])-anchor) for x in eval_rows if x['phase'] in ('carry','carry_stop'))
    statuses=[row['decisions']['r1']['status'] for row in report['steps']]
    r['vision_hold_rounds']=statuses.count('vision_hold')
    r['vision_hold_sim_seconds']=r['vision_hold_rounds']*.2
    r['vision_recovery_episodes']=sum(status=='vision_hold' and (i==0 or statuses[i-1]!='vision_hold') for i,status in enumerate(statuses))
    r['vision_recovery_resumes']=sum(status!='vision_hold' and report['steps'][i]['decisions']['r1']['ready'] and i>0 and statuses[i-1]=='vision_hold' for i,status in enumerate(statuses))
    r['memory_wheel_observations','spacing_rounds','max_carry_spacing_change_m']=sum(obs.get('appearance_source')=='memory' for row in report['steps'] for obs in row['decisions']['r1'].get('observations',{}).values())
    results.append(r)
    status=('완주 성공' if r['success'] else '차단 판단·공동 정지' if expected_stop else
            {'vision_stop':'영상 재획득 한도 도달','payload_decoupled':'짐 정렬 이탈 판정','formation_abort':'로봇 대형 이탈','track':'판단 예산 소진'}.get(final['r1']['status'],'완주 실패'))
    title=Image.new('RGB',(960,84),'#101b2b');draw=ImageDraw.Draw(title)
    draw.text((18,7),f"{k+1}. {e['title_ko']} — {status}",font=ImageFont.truetype(font,27),fill='white')
    draw.text((18,45),f'실제 MuJoCo 기록 4배속 | 간격 유지 · 닫힘 {close_pulse} PWM · weld OFF',font=ImageFont.truetype(font,20),fill='#b7cce6')
    titlepath=RAW/'analysis'/f"{e['id']}-title.png";title.save(titlepath)
    clip=media/(e['id']+'.mp4')
    cmd=['ffmpeg','-y','-v','error','-i',str(p/'motion.mp4'),'-i',str(titlepath),'-filter_complex',
         '[0:v]setpts=PTS/4,pad=960:804:0:84:color=0x101b2b[base];[base][1:v]overlay=0:0,setsar=1,fps=16[v]',
         '-map','[v]','-an','-c:v','libx264','-preset','veryfast','-crf','22','-pix_fmt','yuv420p','-movflags','+faststart',str(clip)]
    subprocess.run(cmd,check=True)
    for stage,idx in [('start',0),('middle',len(report['steps'])//2),('stop',len(report['steps'])-1)]:
        row=report['steps'][idx]; top=p/row['images']['r1']['top']['path']
        destination=media/f"{e['id']}-{stage}-top.jpg"
        destination.write_bytes(top.read_bytes())
        selection.append({'id':e['id'],'stage':stage,'decision_index':idx,'sim_time_s':row['sim_time_s'],
                          'source':str(top),'sha256':sha(top),'image':destination.relative_to(DEST).as_posix()})
    stop=Image.open(media/f"{e['id']}-stop-top.jpg").resize((480,360))
    x=(k%3)*480;y=(k//3)*430
    overview.paste(stop,(x,y+70))
    viewdraw.text((x+12,y+8),f"{k+1}. {e['title_ko']}",font=ImageFont.truetype(font,23),fill='white')
    viewdraw.text((x+12,y+39),status,font=ImageFont.truetype(font,18),fill='#ffb4a7' if not r['success'] and not expected_stop else '#9fe1bf')
    print('PACKAGED '+e['id'],flush=True)
overview.save(media/'overview.jpg',quality=92)
concat=RAW/'analysis/concat.txt'
concat.write_text(''.join("file '"+str(media/(e['id']+'.mp4'))+"'\n" for e in catalog['entries']))
subprocess.run(['ffmpeg','-y','-v','error','-f','concat','-safe','0','-i',str(concat),'-c','copy','-movflags','+faststart',str(media/'grasp-spacing-transport-demo.mp4')],check=True)
write(DEST/'results.json',{'execution_sha':cohort['source_sha'],'scope':f'One fixed-start trial per terrain with whole-grasp and carry-spacing gates; impratio=10; close {close_pulse} PWM; zero LLM calls',
    'feasible_successes':sum(r['success'] for r in results if r['expected_geometric_route']),
    'feasible_trials':sum(r['expected_geometric_route'] for r in results),'blocked_stop_passed':any(r['expected_blocked_stop_passed'] for r in results),'runs':results})
write(DEST/'cohort.json',cohort)
write(DEST/'visual-selection.json',selection)
write(DEST/'environment.json',results[0]['environment'])
fields=['id','success','expected_blocked_stop_passed','final_status','decision_rounds','navigation_sim_seconds','min_lift_m','wall_contact_ticks','unexpected_contact_ticks','goal_position_error_m','goal_yaw_error_deg','wall_seconds','vision_hold_rounds','vision_recovery_episodes','vision_recovery_resumes','memory_wheel_observations','spacing_rounds','max_carry_spacing_change_m']
with (DEST/'results.csv').open('w') as f:
    w=csv.DictWriter(f,fieldnames=fields,lineterminator='\n');w.writeheader()
    for r in results:
        w.writerow({k:(r['final_decisions']['r1']['status'] if k=='final_status' else r.get(k,r['evaluation'].get(k))) for k in fields})
manifest=[]
for e in catalog['entries']:
    for p in sorted((RAW/e['id']).rglob('*')):
        if p.is_file(): manifest.append({'path':p.relative_to(RAW).as_posix(),'bytes':p.stat().st_size,'sha256':sha(p)})
hashes=DEST/'raw-hashes.jsonl.gz'
with gzip.open(hashes,'wt') as f:
    for item in manifest: f.write(json.dumps(item)+'\n')
write(DEST/'raw-manifest.json',{'raw_root':str(RAW),'raw_remote_backup':False,'files':len(manifest),
     'bytes':sum(r['bytes'] for r in manifest),'hash_index_sha256':sha(hashes),
     'hash_index':'raw-hashes.jsonl.gz','selected_media':[{ 'path':p.relative_to(DEST).as_posix(),'bytes':p.stat().st_size,'sha256':sha(p)} for p in sorted(media.iterdir())]})
baseline=json.loads((ROOT/'experiments/2026-09-14-pair-transport-robustness/results.json').read_text())
comparison=[]
for new in results:
    old=next(r for r in baseline['runs'] if r['id']==new['id'])
    assert old['id']==new['id']
    old_rows=[json.loads(x) for x in (Path(old['raw_directory'])/'evaluation-only.jsonl').read_text().splitlines()]
    old_grasp=evaluate_grasp_stability(old_rows)
    comparison.append({'baseline_whole_grasp':old_grasp,'candidate_whole_grasp':new['grasp_stability'],'candidate_max_carry_spacing_change_m':new['max_carry_spacing_change_m'],'id':new['id'],'baseline_source_sha':old['source_sha'],'candidate_source_sha':new['source_sha'],
        'baseline_success':old['success'],'candidate_success':new['success'],
        'baseline_decisions':old['decision_rounds'],'candidate_decisions':new['decision_rounds'],
        'baseline_final_status':old['final_decisions']['r1']['status'],'candidate_final_status':new['final_decisions']['r1']['status'],
        'baseline_sim_seconds':old['evaluation']['navigation_sim_seconds'],'candidate_sim_seconds':new['evaluation']['navigation_sim_seconds'],
        'candidate_recovery_episodes':new['vision_recovery_episodes'],'candidate_recovery_resumes':new['vision_recovery_resumes'],
        'candidate_wall_contact_ticks':new['wall_contact_ticks'],'candidate_other_contact_ticks':new['unexpected_contact_ticks']})
write(DEST/'comparison.json',comparison)
print('ALL EVIDENCE PACKAGED',flush=True)
