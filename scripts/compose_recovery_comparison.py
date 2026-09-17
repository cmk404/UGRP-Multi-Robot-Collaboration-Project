"""Align existing observer recordings by their printed simulation timestamps."""
import argparse
import bisect
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT=Path(__file__).resolve().parents[1]
FONT='/System/Library/Fonts/Supplemental/Arial Unicode.ttf'
large=ImageFont.truetype(FONT,30)
small=ImageFont.truetype(FONT,24)


def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def load_video(folder,cache):
    p=folder/'motion.mp4';report=json.loads((folder/'result.json').read_text())
    cap=cv2.VideoCapture(str(p));frames=[];times=[];ocr=[]
    known=json.loads(cache.read_text()) if cache.exists() else None
    if known and known['sha256']!=digest(p):raise ValueError('source changed')
    while True:
        ok,bgr=cap.read()
        if not ok:break
        if known:
            stamp=known['timestamps'][len(frames)];line=known['ocr'][len(frames)]
        else:
            ok,buf=cv2.imencode('.png',bgr[:54]);assert ok
            result=subprocess.run(['tesseract','stdin','stdout','--psm','7'],input=buf.tobytes(),
                stdout=subprocess.PIPE,stderr=subprocess.PIPE,check=True,env={**os.environ,'OMP_THREAD_LIMIT':'1'})
            line=result.stdout.decode().strip()
            match=re.search(r't\s*=\s*(\d+\.\d+)\s*s',line)
            if not match:raise ValueError(f'OCR missing timestamp {p} frame{len(frames)}: {line}')
            stamp=float(match.group(1))
        if times and not 0<=stamp-times[-1]<=1.0:raise ValueError('nonmonotonic or implausible frame time')
        frames.append(Image.fromarray(cv2.cvtColor(bgr,cv2.COLOR_BGR2RGB)));times.append(stamp);ocr.append(line)
    cap.release()
    if not frames:raise ValueError('empty video')
    if abs(times[0]-report['initial_state']['sim_time_s'])>.02:raise ValueError('initial timestamp mismatch')
    if abs(times[-1]-report['final_state']['sim_time_s'])>.6:raise ValueError('final timestamp mismatch')
    record={'path':str(p.resolve()),'sha256':digest(p),'frames':len(frames),'timestamps':times,'ocr':ocr,
            'success':report['success'],'physical_grasp_success':report['evaluation']['grasp_success'],'approach_sim_s':report['approach_sim_s'],
            'result_sha256':digest(folder/'result.json')}
    cache.write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n')
    return frames,times,record


def main():
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True)
    p.add_argument('--spec',type=Path,required=True);p.add_argument('--seed',choices=['16','17'],default='16');p.add_argument('--cases',nargs='+',default=['nominal-01','yaw_lateral-01','far_yaw-01','overshoot_yaw-01'])
    a=p.parse_args();out=a.out.resolve();out.mkdir(parents=True,exist_ok=True)
    conditions=json.loads(a.spec.read_text())['conditions']
    specs=[('정상 시연 ACT','nominal'+a.seed),('복구 시연 ACT','recovery'+a.seed),('학생 상태 추가 ACT','aggregated'+a.seed),('복구 교사','recovery_teacher')]
    manifest={'alignment':'OCR original observer header t; validated monotonic and against initial/final physics timestamps. 10fps sample-and-hold, no interpolation. Simulation time, not wall-time.', 'cases':[]}
    cv2.setNumThreads(1)
    for case in a.cases:
        sources=[load_video(Path(conditions[condition]['cohort'])/case,out/(case+'-'+condition+'-timestamps.json')) for _,condition in specs]
        first=sources[0][1][0]
        if any(s[1][0]!=first for s in sources):raise ValueError('different start timestamps')
        end=max(s[1][-1] for s in sources);nframes=int(np.ceil((end-first)*10))+11
        dest=out/(case+'-comparison.mp4')
        proc=subprocess.Popen(['ffmpeg','-y','-v','error','-f','rawvideo','-pix_fmt','rgb24','-s','1920x1720','-r','10','-i','-',
            '-an','-c:v','libx264','-threads','2','-preset','veryfast','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(dest)],stdin=subprocess.PIPE)
        base=Image.new('RGB',(1920,1720),'#101820');draw=ImageDraw.Draw(base)
        draw.text((22,14),f'{case}  |  학습 seed {a.seed}  |  같은 시작 조건 · SIM 시간 정렬',font=large,fill='white')
        for i,(title,_) in enumerate(specs):
            x=(i%2)*960;y=70+(i//2)*790
            info=sources[i][2];result='전체 성공' if info['success'] else ('실패·들기만 성공' if info['physical_grasp_success'] else '실패')
            draw.text((x+18,y),f'{title}  |  {result}  |  접근 {info["approach_sim_s"]:.1f}s',font=large,fill=(['#80c8ff','#e4c78a','#69e1bb','#ffcf78'][i] if info['success'] else '#ff7777'))
        try:
            for n in range(nframes):
                now=first+n/10;frame=base.copy()
                for i,(images,times,info) in enumerate(sources):
                    index=max(0,bisect.bisect_right(times,now)-1);index=min(index,len(images)-1)
                    x=(i%2)*960;y=120+(i//2)*790
                    frame.paste(images[index],(x,y))
                    if now>times[-1]+.01:
                        ImageDraw.Draw(frame).text((x+20,y+650),'기록 종료 · 마지막 화면 유지',font=small,fill='#ffcf78',stroke_width=2,stroke_fill='black')
                draw=ImageDraw.Draw(frame)
                draw.text((22,1655),f'SIM 경과 {min(now,end)-first:.1f}s · 원본 약4fps의 기록 시각으로 정렬 · 프레임 사이에는 이전 화면 유지',font=small,fill='white')
                draw.text((22,1690),'전체 성공: 정렬·안정·접촉·파지 기준 통과 | ACT 두 RGB 입력 · 교사 정답 상태 사용 | 실패 시 접근 시간은 성공 시간이 아님',font=small,fill='#b4bfcc')
                proc.stdin.write(np.asarray(frame).tobytes())
                if n in (0,40,110,nframes-1):frame.resize((960,860)).save(out/(case+f'-qa-{n:03d}.jpg'))
        finally:
            proc.stdin.close()
        if proc.wait(timeout=60):raise RuntimeError('ffmpeg failed')
        subprocess.run(['ffmpeg','-v','error','-i',str(dest),'-f','null','-'],check=True)
        manifest['cases'].append({'case':case,'sources':[s[2] for s in sources],'output':str(dest),'sha256':digest(dest),'frames':nframes,'duration_s':nframes/10})
        (out/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
        print(json.dumps({'case':case,'video':str(dest),'duration_s':nframes/10}),flush=True)


if __name__=='__main__':main()
