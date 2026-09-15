"""Output-only metrics/archive/media. This module is never imported by actors."""
from __future__ import annotations

import argparse
import bisect
import csv
import gzip
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess

import cv2
import numpy as np


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def read(path): return json.loads(path.read_text())
def lines(path): return [json.loads(s) for s in path.read_text().splitlines()]
def write(path, value): path.write_text(json.dumps(value, indent=2, ensure_ascii=False)+'\n')
def sha(path): return hashlib.sha256(path.read_bytes()).hexdigest()


def metrics(folder):
    result = read(folder/'result.json'); rows = lines(folder/'actor-decisions.jsonl')
    samples = lines(folder/'evaluation-only.jsonl')
    observations = {s['observation_sequence']:s for s in samples if 'observation_sequence' in s}
    errors = [math.dist(r['reports'][u]['position_m'], observations[r['sequence']]['robots'][u]['xyz_m'][:2])
              for r in rows for u in ('r1','r3') if r['reports'][u]['visual_ok']]
    wait, maxima = {u:0. for u in ('r1','r3')}, {u:0. for u in ('r1','r3')}
    for row in rows:
        dt = max(a['duration_s'] for a in row['issued_actions'].values())
        for u in wait:
            waiting = row['permissions'][u]['phase'] == 'HOLD' and not row['reports'][u]['arrived']
            wait[u] = wait[u] + dt if waiting else 0.
            maxima[u] = max(maxima[u], wait[u])
    checks = {'rgb_error_within_declared_uncertainty': max(errors) <= .035,
              'all_input_audits_pass': read(folder/'input-audit.json')['success']}
    detail = {}
    if result['scenario'] == 'report_gap':
        missing = [r for r in rows if r['missing_reports']]
        checks['missing_reports_stop_both'] = all(all(not any(abs(a[k]) > 1e-12 for k in ('forward','left','turn'))
            for a in r['issued_actions'].values()) for r in missing)
        first, last = missing[0]['sequence'], missing[-1]['sequence'] + 1
        displacement = {u: max(math.dist(observations[i]['robots'][u]['xyz_m'][:2],
                        observations[first]['robots'][u]['xyz_m'][:2]) for i in range(first, last+1)) for u in ('r1','r3')}
        detail['report_gap_stop_displacement_m'] = displacement
        checks['gap_stop_within_margin'] = max(displacement.values()) <= .08
        detail['first_fresh_resume_sequence'] = next(r['sequence'] for r in rows[last:] if
            r['permissions']['r1']['phase'] == 'GO')
    if result['scenario'] == 'restart':
        before, after = rows[23]['reservation_state'], rows[24]['reservation_state']
        checks['restart_retains_generation'] = before['reservations']['r1']['generation'] == after['reservations']['r1']['generation']
        checks['restart_increments_epoch'] = before['epoch']+1 == after['epoch']
        checks['restart_waiter_not_granted'] = rows[24]['permissions']['r3']['phase'] == 'HOLD'
    if result['scenario'] == 'delayed_owner':
        checks['delay_never_reassigns_owner_corridor'] = all(rows[i]['permissions']['r3']['phase']=='HOLD' for i in range(24,36))
    if result['scenario'] == 'blocked_exit':
        checks['blocked_exit_r1_never_moves'] = all(not any(abs(r['issued_actions']['r1'][k])>1e-12
            for k in ('forward','left','turn')) for r in rows)
        checks['blocked_exit_safe_wait'] = result['evaluation']['collision_steps']==0 and checks['blocked_exit_r1_never_moves']
    return {**result, 'max_rgb_position_error_m':max(errors), 'max_contiguous_wait_s':maxima,
            'behavior_checks':checks, 'details':detail}


def panel(folder, row, label, width=640):
    frame = cv2.imread(str(folder/row['images']['r1']['top']['path']))
    frame = cv2.resize(frame,(width,round(width*.75)))
    panel = np.zeros((frame.shape[0]+76,width,3),np.uint8); panel[76:] = frame
    cv2.putText(panel,label,(12,23),cv2.FONT_HERSHEY_SIMPLEX,.56,(240,240,240),1,cv2.LINE_AA)
    cv2.putText(panel,f"{row['clock_s']:.1f} SIM s | r1: {row['permissions']['r1']['phase']} | r3: {row['permissions']['r3']['phase']}",
                (12,47),cv2.FONT_HERSHEY_SIMPLEX,.45,(190,220,240),1,cv2.LINE_AA)
    cv2.putText(panel,'Saved top RGB | review text only',(12,66),cv2.FONT_HERSHEY_SIMPLEX,.40,(170,170,170),1,cv2.LINE_AA)
    return panel


def movie(folders, labels, output, *, speed):
    rows = [lines(p/'actor-decisions.jsonl') for p in folders]
    clocks = [[r['clock_s'] for r in group] for group in rows]
    duration = max(c[-1] for c in clocks)
    frame0 = np.hstack([panel(p, group[0], label) for p, group, label in zip(folders,rows,labels)])
    h,w = frame0.shape[:2]
    process = subprocess.Popen(['ffmpeg','-y','-loglevel','error','-f','rawvideo','-pix_fmt','bgr24',
        '-s',f'{w}x{h}','-r',str(4*speed),'-i','-','-an','-c:v','libx264','-preset','veryfast',
        '-crf','18','-pix_fmt','yuv420p','-movflags','+faststart',str(output)],stdin=subprocess.PIPE)
    try:
        for now in np.arange(0,duration+.251,.25):
            images=[]
            for p, group, times, label in zip(folders,rows,clocks,labels):
                index=max(0,bisect.bisect_right(times,now)-1)
                images.append(panel(p,group[index],label+f' | {speed}x'))
            process.stdin.write(np.hstack(images).tobytes())
        process.stdin.close()
        if process.wait(timeout=30): raise RuntimeError('review movie failed')
    finally:
        if process.poll() is None: process.terminate(); process.wait(timeout=5)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--raw',type=Path,required=True)
    args=parser.parse_args(); raw=args.raw.resolve()
    records=HERE/'records'; validation=HERE/'validation'; media=HERE/'media'
    for p in (records,validation,media): p.mkdir(parents=True,exist_ok=True)
    cases=read(raw/'cohort.json')['results']
    results=[{'label':r['label'],**metrics(raw/r['label'])} for r in cases]
    write(records/'summary.json',results)
    shutil.copy2(raw/'cohort.json',records/'cohort.json')
    with (records/'comparison.csv').open('w') as stream:
        writer=csv.writer(stream); writer.writerow(['case','arrival_success','sim_s','wall_s','decisions',
            'min_circle_clearance_m','max_rgb_error_m','max_wait_s'])
        for r in results:
            writer.writerow([r['label'],r['evaluation']['success'],r['evaluation']['sim_elapsed_s'],r['wall_elapsed_s'],
                r['decision_rounds'],r['evaluation']['sampled_min_circle_clearance_m'],r['max_rgb_position_error_m'],
                max(r['max_contiguous_wait_s'].values())])
    pilots=[ROOT/'outputs'/name for name in ('traffic-pilot-crossing-v2','traffic-pilot-head-on','traffic-pilot-head-on-v2')]
    manifest=[]
    for source in [raw/r['label'] for r in cases]+pilots:
        destination=records/source.name; destination.mkdir(exist_ok=True)
        for name in ('run.json','result.json','actor-maps.json','manifest.json','invariants-before.json',
                     'invariants-after.json','traffic-events.json','contact-events.json','input-audit.json','scene.xml'):
            if (source/name).exists(): shutil.copy2(source/name,destination/name)
        for name in ('actor-decisions.jsonl','evaluation-only.jsonl'):
            with gzip.GzipFile(filename=str(destination/(name+'.gz')),mode='wb',mtime=0) as stream:
                stream.write((source/name).read_bytes())
        manifest.append({'raw_directory':str(source),'source_sha':read(source/'result.json')['source_sha'],
            'raw_manifest_sha256':sha(source/'manifest.json'),'raw_rgb_and_original_video':'local only',
            'archive_directory':str(destination.relative_to(HERE))})
    for logfile in raw.glob('*.log.txt'): shutil.copy2(logfile,records/logfile.name)
    write(records/'raw-locations.json',manifest)
    write(validation/'behavior-verification.json',{'all_checks_pass':all(all(r['behavior_checks'].values()) for r in results),
        'conditions':{r['label']:r['behavior_checks'] for r in results}})
    movie([raw/'crossing-independent',raw/'crossing-reserved'],['Independent RGB: stopped before arrival','Reserved RGB: yield, then both arrive'],
          media/'crossing-comparison.mp4',speed=2)
    movie([raw/'head_on-reserved'],['Narrow passage: retain old route, extend, yield'],media/'head-on-yield.mp4',speed=4)
    for number in range(4):
        groups=[]
        for result in results[2*number:2*number+2]:
            folder=raw/result['label']; rows=lines(folder/'actor-decisions.jsonl')
            picks=[0,len(rows)//3,2*len(rows)//3,len(rows)-1]
            groups.append(np.hstack([panel(folder,rows[i],result['label'],width=320) for i in picks]))
        cv2.imwrite(str(media/f'review-{number+1}.jpg'),np.vstack(groups))
    selected = {}
    for name in ('head_on-reserved','report_gap-reserved','blocked_exit-reserved'):
        folder=raw/name; rows=lines(folder/'actor-decisions.jsonl')
        index=24 if name=='report_gap-reserved' else len(rows)//2
        selected[name]=index
        for u in ('r1','r3'):
            image=cv2.imread(str(folder/rows[index]['images'][u]['own']['path']))
            cv2.imwrite(str(media/f'{name}-{u}-own.jpg'),image)
    write(validation/'visual-review-selection.json',{'review_contact_sheets':'4 samples per final condition',
        'own_rgb_selected_sequences':selected,'videos':'top RGB sampled every .25 command-clock seconds; 2x/4x playback',
        'overlays':'review video text only; never actor inputs','all_frames_manually_reviewed':False})
    print(json.dumps({'conditions':len(results),'arrival_successes':sum(r['evaluation']['success'] for r in results),
                      'all_behavior_checks_pass':all(all(r['behavior_checks'].values()) for r in results)}))


if __name__=='__main__':main()
